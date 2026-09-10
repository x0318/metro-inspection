import math
import os
import queue
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Protocol, Set

import cv2
import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from .route_config import RouteCatalog
from .camera_profiles import camera_config


class StatusSink(Protocol):

    def update_ros_graph(self, external_node_count: int, graph_error: bool = False) -> None:
        ...

    def update_odom(self, data: Dict[str, object]) -> None:
        ...

    def update_navigation(self, data: Dict[str, object]) -> None:
        ...

    def update_safety(self, data: Dict[str, object]) -> None:
        ...

    def update_camera(self, camera_name: str, data: Dict[str, object]) -> None:
        ...


class PatrolControlNode(Node):
    """Own Nav2 goals and expose low-rate robot state to the web process."""

    ACTIVE_STATES = {'starting', 'running', 'canceling'}

    def __init__(
        self,
        status_sink: StatusSink,
        command_queue: 'queue.Queue[Dict[str, object]]',
        routes: RouteCatalog,
        camera_directory: Path,
        behavior_tree: Optional[Path] = None,
        **node_kwargs: Any,
    ) -> None:
        super().__init__('metro_navigation_demo', **node_kwargs)
        self.camera_topics = {camera['id']: camera['topic'] for camera in camera_config()}
        self._status_sink = status_sink
        self._command_queue = command_queue
        self._routes = routes
        self._camera_directory = camera_directory
        self._camera_directory.mkdir(parents=True, exist_ok=True)
        self._behavior_tree = behavior_tree
        self._own_fully_qualified_name = self.get_fully_qualified_name()

        self._navigation_state = 'idle'
        self._navigation: Dict[str, object] = {}
        self._mode: Optional[str] = None
        self._route_name: Optional[str] = None
        self._destination: Optional[str] = None
        self._stage: Optional[str] = None
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_reason: Optional[str] = None
        self._active_waypoints: List[Dict[str, float]] = []
        self._resume_waypoints: List[Dict[str, float]] = []
        self._last_poses_remaining = 0
        self._pending_send_deadline = 0.0
        self._segment_distance = 0.0
        self._segment_start_mileage = 0.0
        self._progress_base = 0.0
        self._progress_span = 100.0
        self._progress = 0.0
        self._pose_xy: Optional[tuple[float, float]] = None
        self._previous_odom_xy: Optional[tuple[float, float]] = None
        self._mileage_m = 0.0
        self._last_odom_status = 0.0
        self._estop_active = False
        self._heartbeat_timeout = float(
            os.environ.get('PLATFORM_HEARTBEAT_TIMEOUT_SECONDS', '3.0')
        )
        if self._heartbeat_timeout <= 0.0:
            raise ValueError('PLATFORM_HEARTBEAT_TIMEOUT_SECONDS must be positive')
        self._last_heartbeat = time.monotonic()
        self._camera_last_frame = {name: 0.0 for name in self.camera_topics}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._odom_subscription = self.create_subscription(
            Odometry, '/odom', self._handle_odom, sensor_qos
        )
        self._camera_subscriptions = []
        self._selected_camera = next(iter(self.camera_topics))
        for camera_name, topic in self.camera_topics.items():
            subscription = self.create_subscription(
                CompressedImage,
                topic,
                lambda message, camera=camera_name, source=topic: self._handle_camera(
                    camera, source, message
                ),
                sensor_qos,
            )
            self._camera_subscriptions.append(subscription)
            self._status_sink.update_camera(camera_name, {
                'available': False,
                'streaming': True,
                'topic': topic,
            })

        safety_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._estop_publisher = self.create_publisher(
            Bool, '/safety/estop_active', safety_qos
        )
        self._navigation_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses'
        )
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._command_timer = self.create_timer(0.05, self._process_commands, clock=self._steady_clock)
        self._graph_timer = self.create_timer(1.0, self._refresh_ros_graph, clock=self._steady_clock)
        self._safety_timer = self.create_timer(0.1, self._check_safety, clock=self._steady_clock)

        self._publish_estop(False)
        self._set_navigation(state='idle', message='等待任务')
        self._refresh_ros_graph()
        self.get_logger().info('Web navigation and safety bridge initialized')

    @staticmethod
    def _fully_qualified_name(name: str, namespace: str) -> str:
        clean_namespace = namespace.rstrip('/')
        return (clean_namespace + '/' + name) if clean_namespace else ('/' + name)

    def _refresh_ros_graph(self) -> None:
        try:
            external_nodes: Set[str] = {
                self._fully_qualified_name(name, namespace)
                for name, namespace in self.get_node_names_and_namespaces()
                if self._fully_qualified_name(name, namespace)
                != self._own_fully_qualified_name
            }
            self._status_sink.update_ros_graph(len(external_nodes))
        except Exception as error:
            self._status_sink.update_ros_graph(0, graph_error=True)
            self.get_logger().warning(f'ROS graph check failed: {error}')

    @staticmethod
    def _yaw_from_odometry(message: Odometry) -> float:
        q = message.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _handle_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        xy = (float(position.x), float(position.y))
        if self._previous_odom_xy is not None:
            delta = math.dist(self._previous_odom_xy, xy)
            if delta < 2.0:
                self._mileage_m += delta
        self._previous_odom_xy = xy
        self._pose_xy = xy

        now = time.monotonic()
        if now - self._last_odom_status < 0.25:
            return
        self._last_odom_status = now
        twist = message.twist.twist
        self._status_sink.update_odom({
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
            'pose': {
                'x': round(xy[0], 3),
                'y': round(xy[1], 3),
                'yaw': round(self._yaw_from_odometry(message), 4),
            },
            'velocity': {
                'linear': round(math.hypot(twist.linear.x, twist.linear.y), 3),
                'angular': round(float(twist.angular.z), 3),
            },
            'mileage_m': round(self._mileage_m, 3),
        })

    def _handle_camera(self, name: str, topic: str, message: CompressedImage) -> None:
        now = time.monotonic()

        # Limit each web preview stream to 2 FPS.
        if now - self._camera_last_frame[name] < 0.5:
            return
        self._camera_last_frame[name] = now

        try:
            compressed = np.frombuffer(message.data, dtype=np.uint8)
            if compressed.size == 0:
                raise ValueError('compressed camera message is empty')

            image = cv2.imdecode(compressed, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('failed to decode compressed camera frame')

            preview = cv2.resize(
                image,
                (640, 360),
                interpolation=cv2.INTER_AREA,
            )

            success, encoded = cv2.imencode(
                '.jpg',
                preview,
                [cv2.IMWRITE_JPEG_QUALITY, 55],
            )
            if not success:
                raise ValueError('failed to encode camera preview')

            frame = encoded.tobytes()

            output_path = self._camera_directory / f'{name}.jpg'
            temporary_path = self._camera_directory / f'.{name}.tmp'
            with temporary_path.open('wb') as stream:
                stream.write(frame)
            os.replace(temporary_path, output_path)

            self._status_sink.update_camera(name, {
                'available': True,
                'streaming': True,
                'topic': topic,
                'width': 640,
                'height': 360,
                'updated_at': datetime.now(timezone.utc).isoformat(),
            })
        except Exception as error:
            self.get_logger().warning(
                f'Camera {name} frame conversion failed: {error}',
                throttle_duration_sec=5.0,
            )

    def _process_commands(self) -> None:
        for _ in range(20):
            try:
                command = self._command_queue.get_nowait()
            except queue.Empty:
                break
            self._last_heartbeat = time.monotonic()
            try:
                self._handle_command(command)
            except Exception as error:
                self.get_logger().error(f'Command failed: {error}')
                self._set_navigation(state='failed', message=str(error))

        if (
            self._navigation_state == 'starting'
            and self._send_future is None
            and self._active_waypoints
        ):
            self._try_send_goal()

    def _handle_command(self, command: Dict[str, object]) -> None:
        name = command.get('command')
        if name == 'heartbeat':
            return
        if name == 'start':
            self._start_navigation(command)
        elif name == 'junction_choice':
            self._choose_junction(str(command.get('choice', '')))
        elif name == 'pause':
            self._pause_navigation()
        elif name == 'resume':
            self._resume_navigation()
        elif name == 'cancel':
            self._cancel_navigation()
        elif name == 'estop':
            self._activate_estop('平台触发紧急制动')
        elif name == 'reset_estop':
            self._reset_estop()
        elif name == 'select_camera':
            self._select_camera(str(command.get('camera_name', '')))
        else:
            raise ValueError(f'unknown command: {name!r}')

    def _select_camera(self, camera_name: str) -> None:
        if camera_name not in self.camera_topics:
            raise ValueError('unknown camera')
        self._selected_camera = camera_name

    def _start_navigation(self, command: Dict[str, object]) -> None:
        if self._estop_active:
            raise ValueError('急停尚未复位，不能启动任务')
        if self._navigation_state in self.ACTIVE_STATES or self._goal_handle is not None:
            raise ValueError('已有导航任务正在执行')

        mode = str(command.get('mode', ''))
        self._mode = mode
        self._destination = None
        self._route_name = None
        if mode == 'destination':
            destination = str(command.get('destination', ''))
            route_name, label, waypoints = self._routes.destination_route(destination)
            self._destination = destination
            self._route_name = route_name
            self._stage = 'route'
            self._prepare_goal(waypoints, label, 0.0, 100.0)
        elif mode == 'junction':
            label, waypoints = self._routes.junction_common_route()
            self._stage = 'junction_common'
            self._prepare_goal(waypoints, label, 0.0, 45.0)
        else:
            raise ValueError('导航模式必须是 destination 或 junction')

    def _choose_junction(self, choice: str) -> None:
        if self._estop_active:
            raise ValueError('急停尚未复位，不能选择路线')
        if self._navigation_state != 'waiting_choice':
            raise ValueError('小车当前不在等待岔路选择')
        route_name, label, waypoints = self._routes.junction_choice_route(choice)
        self._route_name = route_name
        self._destination = choice
        self._stage = 'route'
        self._prepare_goal(waypoints, label, 45.0, 55.0)

    def _prepare_goal(
        self,
        waypoints: List[Dict[str, float]],
        label: str,
        progress_base: float,
        progress_span: float,
    ) -> None:
        self._active_waypoints = list(waypoints)
        self._resume_waypoints = list(waypoints)
        self._last_poses_remaining = len(waypoints)
        self._progress_base = progress_base
        self._progress_span = progress_span
        self._progress = progress_base
        self._segment_distance = self._route_distance(waypoints)
        self._segment_start_mileage = self._mileage_m
        self._pending_send_deadline = time.monotonic() + 15.0
        self._cancel_reason = None
        self._send_future = None
        self._goal_handle = None
        self._set_navigation(
            state='starting',
            label=label,
            progress_percent=round(self._progress, 1),
            poses_remaining=len(waypoints),
            distance_remaining_m=round(self._segment_distance, 2),
            message='正在连接 Nav2 导航服务器',
        )

    def _route_distance(self, waypoints: List[Dict[str, float]]) -> float:
        points = [(item['x'], item['y']) for item in waypoints]
        if self._pose_xy is not None:
            points.insert(0, self._pose_xy)
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    def _try_send_goal(self) -> None:
        if self._estop_active:
            return
        if not self._navigation_client.server_is_ready():
            if time.monotonic() >= self._pending_send_deadline:
                self._active_waypoints = []
                self._set_navigation(
                    state='failed', message='15 秒内未发现 Nav2 导航服务器'
                )
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._create_pose(item) for item in self._active_waypoints]
        if self._behavior_tree is not None:
            goal.behavior_tree = str(self._behavior_tree)
        self._send_future = self._navigation_client.send_goal_async(
            goal, feedback_callback=self._handle_feedback
        )
        self._send_future.add_done_callback(self._handle_goal_response)
        self._set_navigation(state='starting', message='导航任务已提交，等待 Nav2 接受')

    def _create_pose(self, waypoint: Dict[str, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._routes.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint['x']
        pose.pose.position.y = waypoint['y']
        pose.pose.orientation.z = math.sin(waypoint['yaw'] / 2.0)
        pose.pose.orientation.w = math.cos(waypoint['yaw'] / 2.0)
        return pose

    def _handle_goal_response(self, future: Any) -> None:
        self._send_future = None
        try:
            goal_handle = future.result()
        except Exception as error:
            self._set_navigation(state='failed', message=f'提交导航任务失败：{error}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self._set_navigation(state='failed', message='Nav2 拒绝了导航任务')
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._handle_result)
        if self._estop_active:
            self._cancel_reason = 'estop'
            goal_handle.cancel_goal_async()
            return
        self._set_navigation(state='running', message='正在执行导航路线')

    def _handle_feedback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        distance = float(getattr(feedback, 'distance_remaining', math.nan))
        poses_remaining = max(
            0, int(getattr(feedback, 'number_of_poses_remaining', 0))
        )
        self._last_poses_remaining = poses_remaining
        if poses_remaining > 0:
            self._resume_waypoints = self._active_waypoints[-poses_remaining:]

        # Nav2 emits an all-zero feedback sample before its first path exists.
        # Completion is handled by the Action result, so a zero distance is
        # never useful for in-flight progress.
        if poses_remaining == 0 or (math.isfinite(distance) and distance <= 0.01):
            return

        travelled = max(0.0, self._mileage_m - self._segment_start_mileage)
        fraction = min(
            travelled / max(self._segment_distance, 0.001),
            1.0,
        )
        self._progress = max(
            self._progress,
            self._progress_base + self._progress_span * fraction,
        )
        self._set_navigation(
            state='running',
            progress_percent=round(min(self._progress, 99.9), 1),
            distance_remaining_m=round(distance, 2) if math.isfinite(distance) else None,
            poses_remaining=poses_remaining,
            message='正在执行导航路线',
        )

    def _handle_result(self, future: Any) -> None:
        self._goal_handle = None
        self._result_future = None
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self._set_navigation(state='failed', message=f'读取导航结果失败：{error}')
            return

        reason = self._cancel_reason
        self._cancel_reason = None
        if reason == 'estop' or self._estop_active:
            self._set_navigation(state='estop', message='急停锁定，任务已取消')
            return
        if reason == 'pause':
            self._set_navigation(state='paused', message='任务已暂停，可从剩余路线继续')
            return
        if reason == 'cancel':
            self._clear_task()
            self._set_navigation(state='idle', progress_percent=0.0, message='任务已取消')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._stage == 'junction_common':
                self._progress = 45.0
                self._active_waypoints = []
                self._set_navigation(
                    state='waiting_choice',
                    progress_percent=45.0,
                    distance_remaining_m=0.0,
                    poses_remaining=0,
                    message='已到达预设岔路等待点，请选择前进方向',
                )
            else:
                self._progress = 100.0
                self._active_waypoints = []
                self._set_navigation(
                    state='completed',
                    progress_percent=100.0,
                    distance_remaining_m=0.0,
                    poses_remaining=0,
                    message='巡检路线已完成',
                )
            return

        error_code = getattr(wrapped_result.result, 'error_code', status)
        self._set_navigation(
            state='failed',
            message=f'导航未完成，Nav2 状态/错误码：{status}/{error_code}',
        )

    def _pause_navigation(self) -> None:
        if self._navigation_state not in {'starting', 'running'}:
            raise ValueError('当前任务不能暂停')
        if not self._resume_waypoints:
            self._resume_waypoints = list(self._active_waypoints)
        if self._goal_handle is None:
            self._active_waypoints = []
            self._set_navigation(state='paused', message='任务已暂停，可继续')
            return
        self._cancel_reason = 'pause'
        self._set_navigation(state='canceling', message='正在暂停并保存剩余路线')
        self._goal_handle.cancel_goal_async()

    def _resume_navigation(self) -> None:
        if self._estop_active:
            raise ValueError('急停尚未复位，不能继续任务')
        if self._navigation_state != 'paused' or not self._resume_waypoints:
            raise ValueError('没有可继续的暂停任务')
        base = self._progress
        span = max(0.0, self._progress_base + self._progress_span - base)
        self._prepare_goal(
            list(self._resume_waypoints),
            str(self._navigation.get('label') or '继续巡检'),
            base,
            span,
        )

    def _cancel_navigation(self) -> None:
        if self._navigation_state in {'idle', 'completed', 'failed'}:
            self._clear_task()
            self._set_navigation(state='idle', progress_percent=0.0, message='等待任务')
            return
        if self._goal_handle is None:
            self._clear_task()
            self._set_navigation(state='idle', progress_percent=0.0, message='任务已取消')
            return
        self._cancel_reason = 'cancel'
        self._set_navigation(state='canceling', message='正在取消导航任务')
        self._goal_handle.cancel_goal_async()

    def _activate_estop(self, message: str) -> None:
        self._estop_active = True
        self._publish_estop(True, message)
        self._cancel_reason = 'estop'
        self._active_waypoints = []
        self._resume_waypoints = []
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._set_navigation(state='estop', message=message)

    def _reset_estop(self) -> None:
        if not self._estop_active:
            return
        self._estop_active = False
        self._publish_estop(False, '急停已复位，保持停车，等待新任务')
        self._clear_task()
        self._set_navigation(
            state='idle', progress_percent=0.0, message='急停已复位，等待新任务'
        )

    def _publish_estop(self, active: bool, message: Optional[str] = None) -> None:
        output = Bool()
        output.data = active
        self._estop_publisher.publish(output)
        self._status_sink.update_safety({
            'estop_active': active,
            'heartbeat_timeout_seconds': self._heartbeat_timeout,
            'message': message or ('急停锁定' if active else '安全许可正常'),
        })

    def _check_safety(self) -> None:
        if self._navigation_state not in self.ACTIVE_STATES:
            return
        heartbeat_age = time.monotonic() - self._last_heartbeat
        if heartbeat_age > self._heartbeat_timeout and not self._estop_active:
            self.get_logger().error(
                f'Platform heartbeat timed out after {heartbeat_age:.2f}s; activating e-stop'
            )
            self._activate_estop('平台连接中断，已自动急停')

    def _clear_task(self) -> None:
        self._mode = None
        self._route_name = None
        self._destination = None
        self._stage = None
        self._active_waypoints = []
        self._resume_waypoints = []
        self._last_poses_remaining = 0
        self._progress = 0.0
        self._progress_base = 0.0
        self._progress_span = 100.0
        self._send_future = None
        self._navigation.update({
            'label': None,
            'distance_remaining_m': None,
            'poses_remaining': 0,
        })

    def _set_navigation(self, state: Optional[str] = None, **updates: object) -> None:
        if state is not None:
            self._navigation_state = state
        defaults = {
            'state': self._navigation_state,
            'mode': self._mode,
            'route': self._route_name,
            'destination': self._destination,
            'progress_percent': round(self._progress, 1),
            'can_pause': self._navigation_state in {'starting', 'running'},
            'can_resume': self._navigation_state == 'paused' and bool(self._resume_waypoints),
            'can_cancel': self._navigation_state not in {'idle', 'completed', 'estop'},
            'can_choose_junction': self._navigation_state == 'waiting_choice',
        }
        defaults.update(updates)
        self._navigation.update(defaults)
        self._status_sink.update_navigation(dict(self._navigation))

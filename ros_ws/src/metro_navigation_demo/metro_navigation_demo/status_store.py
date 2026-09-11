import copy
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Dict, Optional


class StatusStore:
    """Thread-safe snapshot shared by the ROS worker and FastAPI."""

    def __init__(
        self,
        odom_timeout_seconds: float = 3.0,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if odom_timeout_seconds <= 0.0:
            raise ValueError('odom_timeout_seconds must be positive')

        self._lock = threading.Lock()
        self._clock = monotonic_clock
        self._odom_timeout_seconds = odom_timeout_seconds
        self._external_node_count = 0
        self._graph_checked = False
        self._graph_error = False
        self._last_odom_at: Optional[float] = None
        self._odom = {
            'topic': '/odom',
            'frame_id': None,
            'child_frame_id': None,
            'pose': {'x': None, 'y': None, 'yaw': None},
            'velocity': {'linear': 0.0, 'angular': 0.0},
            'mileage_m': 0.0,
        }
        self._navigation = {
            'state': 'idle',
            'mode': None,
            'route': None,
            'destination': None,
            'label': None,
            'progress_percent': 0.0,
            'distance_remaining_m': None,
            'poses_remaining': 0,
            'message': '等待任务',
            'can_pause': False,
            'can_resume': False,
            'can_cancel': False,
            'can_choose_junction': False,
        }
        self._safety = {
            'estop_active': False,
            'heartbeat_timeout_seconds': 3.0,
            'message': '安全许可正常',
        }
        self._cameras: Dict[str, object] = {}

    def update_ros_graph(
        self,
        external_node_count: int,
        graph_error: bool = False,
    ) -> None:
        with self._lock:
            self._external_node_count = max(0, int(external_node_count))
            self._graph_checked = True
            self._graph_error = bool(graph_error)

    def mark_odom_received(self, data: Optional[Dict[str, object]] = None) -> None:
        with self._lock:
            self._last_odom_at = self._clock()
            if data:
                self._odom.update(copy.deepcopy(data))

    def update_navigation(self, data: Dict[str, object]) -> None:
        with self._lock:
            self._navigation.update(copy.deepcopy(data))

    def update_safety(self, data: Dict[str, object]) -> None:
        with self._lock:
            self._safety.update(copy.deepcopy(data))

    def update_camera(self, camera_name: str, data: Dict[str, object]) -> None:
        with self._lock:
            self._cameras[camera_name] = copy.deepcopy(data)

    def apply_ros_update(self, update: Dict[str, object]) -> None:
        update_type = update.get('type')
        if update_type == 'graph':
            self.update_ros_graph(
                int(update.get('external_node_count', 0)),
                graph_error=bool(update.get('graph_error', False)),
            )
        elif update_type == 'odom':
            self.mark_odom_received(update.get('data'))
        elif update_type == 'navigation':
            self.update_navigation(update.get('data', {}))
        elif update_type == 'safety':
            self.update_safety(update.get('data', {}))
        elif update_type == 'camera':
            camera_name = update.get('camera_name')
            if isinstance(camera_name, str):
                self.update_camera(camera_name, update.get('data', {}))

    def snapshot(self) -> Dict[str, object]:
        now = self._clock()
        with self._lock:
            external_node_count = self._external_node_count
            graph_checked = self._graph_checked
            graph_error = self._graph_error
            last_odom_at = self._last_odom_at
            odom = copy.deepcopy(self._odom)
            navigation = copy.deepcopy(self._navigation)
            safety = copy.deepcopy(self._safety)
            cameras = copy.deepcopy(self._cameras)

        odom_age = None
        if last_odom_at is not None:
            odom_age = max(0.0, now - last_odom_at)
        odom_fresh = (
            odom_age is not None
            and odom_age <= self._odom_timeout_seconds
        )
        odom.update({
            'received': last_odom_at is not None,
            'fresh': odom_fresh,
            'age_seconds': round(odom_age, 3) if odom_age is not None else None,
            'timeout_seconds': self._odom_timeout_seconds,
        })

        ros_online = graph_checked and not graph_error and external_node_count > 0
        return {
            'service': 'metro_navigation_demo',
            'backend': {'online': True, 'status': 'online'},
            'ros': {
                'online': ros_online,
                'status': 'online' if ros_online else 'offline',
                'graph_checked': graph_checked,
                'graph_error': graph_error,
                'external_node_count': external_node_count,
            },
            'odom': odom,
            'navigation': navigation,
            'safety': safety,
            'cameras': cameras,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        }

"""三维定位算法 主流程"""

import cv2
import message_filters
import numpy as np
import rclpy
import signal
from copy import deepcopy
from dataclasses import dataclass
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from metro_inspection_interfaces.msg import DefectEvent
from pathlib import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray
from visualization_msgs.msg import Marker

from .cloud_projector import (
    dbscan_median_point,
    pointcloud2_to_xyz,
    project_camera_points,
    robust_median_point,
    select_bbox_points,
    select_mask_points,
)
from .geometry_utils import transform_point, transform_points
from .semantic_geometry import Point3D, TunnelSemanticProjector
from .spatial_event_tracking import SpatialEventTracker


@dataclass
class DetectionLocalization:
    detection: Detection2D
    index: int
    drawable: bool = False
    roi_uv: object = None
    estimated_uv: object = None
    global_point: object = None
    roi_count: int = 0
    inlier_count: int = 0
    status: str = ""


class DamageLocalizer(Node):
    """Fuse time-synchronized 2D detections and lidar points into 3D positions."""

    def __init__(self):
        super().__init__("damage_localizer")
        self.declare_parameter("detections_topic", "/damage_detections")
        self.declare_parameter("cloud_topic", "/lidar/points")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("mask_topic", "/damage_mask")
        self.declare_parameter("use_mask", True)
        self.declare_parameter("mask_time_tolerance_sec", 0.20)
        self.declare_parameter("mask_min_value", 1)
        self.declare_parameter("mask_fallback_to_bbox", True)
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_point_topic", "/damage_point_camera")
        self.declare_parameter("global_point_topic", "/damage_point_global")
        self.declare_parameter(
            "debug_image_topic", "/localization/debug_projection"
        )
        self.declare_parameter("marker_topic", "/localization/estimated_marker")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("sync_queue_size", 20)
        self.declare_parameter("sync_slop_sec", 0.08)
        self.declare_parameter("roi_scale", 0.75)
        self.declare_parameter("min_roi_points", 4)
        self.declare_parameter("max_depth_deviation_m", 0.08)
        self.declare_parameter("estimator_method", "dbscan_median")
        self.declare_parameter("dbscan_eps_m", 0.15)
        self.declare_parameter("dbscan_min_samples", 3)
        self.declare_parameter("max_debug_points", 1800)
        self.declare_parameter("publish_events", False)
        self.declare_parameter("event_topic", "/localized/defect_events")
        self.declare_parameter("camera_name", "odin1")
        self.declare_parameter("model_name", "")
        self.declare_parameter("event_confirmation_hits", 3)
        self.declare_parameter("event_dedup_distance_m", 0.5)
        self.declare_parameter("event_republish_period_sec", 2.0)
        self.declare_parameter("chainage_start_m", 12000.0)
        self.declare_parameter("chainage_axis", "x")
        self.declare_parameter("chainage_sign", 1.0)
        self.declare_parameter("segment_start_id", 1000)
        self.declare_parameter("segment_length_m", 1.2)
        self.declare_parameter("segment_name", "仿真环号")
        self.declare_parameter("clock_center_y_m", 0.0)
        self.declare_parameter("clock_center_z_m", 1.75)

        self.bridge = CvBridge()
        self.camera_info = None
        self.last_status_log_ns = 0
        self.sync_count = 0
        self.nonempty_detection_count = 0
        self.empty_detection_count = 0
        self.projection_failure_count = 0
        self.roi_failure_count = 0
        self.mask_used_count = 0
        self.mask_missing_count = 0
        self.mask_fallback_count = 0
        self.camera_point_count = 0
        self.global_tf_failure_count = 0
        self.global_point_count = 0
        self.event_count = 0
        self.latest_mask_msg = None
        self.last_frame_key = None
        self.marker_ids = set()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.mask_sub = None
        if bool(self.get_parameter("use_mask").value):
            self.mask_sub = self.create_subscription(
                Image,
                self.get_parameter("mask_topic").value,
                self.on_mask,
                qos_profile_sensor_data,
            )
        self.camera_point_pub = self.create_publisher(
            PointStamped, self.get_parameter("camera_point_topic").value, 10
        )
        self.global_point_pub = self.create_publisher(
            PointStamped, self.get_parameter("global_point_topic").value, 10
        )
        # RViz Image displays request Reliable QoS by default.  A reliable
        # publisher also remains compatible with best-effort subscribers.
        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter("debug_image_topic").value,
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter("marker_topic").value, 10
        )
        self.event_pub = None
        self.event_tracker = None
        self.semantic_projector = None
        if bool(self.get_parameter("publish_events").value):
            event_qos = QoSProfile(depth=50)
            event_qos.reliability = ReliabilityPolicy.RELIABLE
            self.event_pub = self.create_publisher(
                DefectEvent,
                str(self.get_parameter("event_topic").value),
                event_qos,
            )
            self.event_tracker = SpatialEventTracker(
                distance_threshold_m=float(
                    self.get_parameter("event_dedup_distance_m").value
                ),
                confirmation_hits=int(
                    self.get_parameter("event_confirmation_hits").value
                ),
                republish_period_sec=float(
                    self.get_parameter("event_republish_period_sec").value
                ),
                event_prefix=str(self.get_parameter("camera_name").value),
            )
            self.semantic_projector = TunnelSemanticProjector(
                chainage_start_m=float(
                    self.get_parameter("chainage_start_m").value
                ),
                chainage_axis=str(self.get_parameter("chainage_axis").value),
                chainage_sign=float(self.get_parameter("chainage_sign").value),
                segment_start_id=int(
                    self.get_parameter("segment_start_id").value
                ),
                segment_length_m=float(
                    self.get_parameter("segment_length_m").value
                ),
                segment_name=str(self.get_parameter("segment_name").value),
                clock_center_y_m=float(
                    self.get_parameter("clock_center_y_m").value
                ),
                clock_center_z_m=float(
                    self.get_parameter("clock_center_z_m").value
                ),
            )

        detections_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            self.get_parameter("detections_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        cloud_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        image_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("image_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [detections_sub, cloud_sub, image_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.on_synced_data)
        self.status_timer = self.create_timer(2.0, self.log_pipeline_status)
        self.get_logger().info(
            "Damage localizer started: Detection2DArray + optional mask + "
            "PointCloud2 -> camera/global 3D point."
        )

    def on_camera_info(self, msg: CameraInfo):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def on_mask(self, msg: Image):
        self.latest_mask_msg = msg

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def get_matching_mask(self, image_msg: Image):
        if not bool(self.get_parameter("use_mask").value):
            return None, "disabled"
        mask_msg = self.latest_mask_msg
        if mask_msg is None:
            self.mask_missing_count += 1
            return None, "missing"
        dt = abs(
            self.stamp_to_sec(mask_msg.header.stamp)
            - self.stamp_to_sec(image_msg.header.stamp)
        )
        tolerance = float(self.get_parameter("mask_time_tolerance_sec").value)
        if dt > tolerance:
            self.mask_missing_count += 1
            return None, f"stale dt={dt:.3f}s"
        return mask_msg, "ok"

    def mask_msg_to_array(self, mask_msg: Image, image_width: int, image_height: int):
        mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="passthrough")
        mask = np.asarray(mask)
        if mask.ndim == 3:
            if mask.shape[2] == 4:
                mask = cv2.cvtColor(mask, cv2.COLOR_BGRA2GRAY)
            else:
                mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        if mask.dtype != np.uint8:
            mask = np.nan_to_num(mask.astype(np.float32), nan=0.0)
            if mask.max(initial=0.0) <= 1.0:
                mask = mask * 255.0
            mask = np.clip(mask, 0, 255).astype(np.uint8)
        if mask.shape[0] != int(image_height) or mask.shape[1] != int(image_width):
            mask = cv2.resize(
                mask,
                (int(image_width), int(image_height)),
                interpolation=cv2.INTER_NEAREST,
            )
        return mask

    @staticmethod
    def detection_score(detection: Detection2D):
        return max(
            (float(result.hypothesis.score) for result in detection.results),
            default=0.0,
        )

    def lookup_transform(self, target_frame, source_frame, stamp):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(seconds=0.25),
        )

    def warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_status_log_ns > 2_000_000_000:
            self.get_logger().warn(message)
            self.last_status_log_ns = now_ns

    def estimate_damage_point(self, roi_points):
        """Select the 3D point estimator according to ROS parameters."""
        method = str(self.get_parameter("estimator_method").value).strip().lower()
        if method in ("dbscan", "dbscan_median"):
            estimated, inliers = dbscan_median_point(
                roi_points,
                eps=float(self.get_parameter("dbscan_eps_m").value),
                min_samples=int(self.get_parameter("dbscan_min_samples").value),
            )
            return estimated, inliers, "dbscan_median"

        if method not in ("mad", "mad_median", "robust_median"):
            self.warn_throttled(
                f"Unknown estimator_method='{method}', fallback to mad_median."
            )
        estimated, inliers = robust_median_point(
            roi_points,
            max_depth_deviation=float(
                self.get_parameter("max_depth_deviation_m").value
            ),
        )
        return estimated, inliers, "mad_median"

    def log_pipeline_status(self):
        self.get_logger().info(
            "Pipeline status: "
            f"synced={self.sync_count}, "
            "detections(nonempty/empty)="
            f"{self.nonempty_detection_count}/{self.empty_detection_count}, "
            f"projection_failed={self.projection_failure_count}, "
            f"roi_rejected={self.roi_failure_count}, "
            "mask(used/missing/fallback)="
            f"{self.mask_used_count}/{self.mask_missing_count}/{self.mask_fallback_count}, "
            "published(camera/global)="
            f"{self.camera_point_count}/{self.global_point_count}, "
            f"global_tf_failed={self.global_tf_failure_count}"
            f", platform_events={self.event_count}"
        )

    def publish_debug_image(
        self,
        image_msg,
        projected_uv,
        targets=(),
        mask_image=None,
        status_text="",
    ):
        try:
            debug = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8").copy()
        except Exception as exc:
            self.warn_throttled(f"Cannot create debug projection image: {exc}")
            return

        if mask_image is not None:
            mask = np.asarray(mask_image)
            if mask.ndim == 3:
                mask = mask[..., 0]
            if mask.shape[:2] == debug.shape[:2]:
                overlay = debug.copy()
                overlay[mask > 0] = (0, 0, 255)
                debug = cv2.addWeighted(overlay, 0.25, debug, 0.75, 0)

        max_points = max(1, int(self.get_parameter("max_debug_points").value))
        if len(projected_uv) > max_points:
            step = max(1, len(projected_uv) // max_points)
            draw_uv = projected_uv[::step]
        else:
            draw_uv = projected_uv
        for u, v in draw_uv:
            cv2.circle(debug, (int(round(u)), int(round(v))), 1, (0, 210, 0), -1)

        for target in targets:
            if not target.drawable:
                continue
            detection = target.detection
            bbox = detection.bbox
            cx = float(bbox.center.position.x)
            cy = float(bbox.center.position.y)
            half_w = float(bbox.size_x) / 2.0
            half_h = float(bbox.size_y) / 2.0
            p1 = (int(round(cx - half_w)), int(round(cy - half_h)))
            p2 = (int(round(cx + half_w)), int(round(cy + half_h)))
            color = self.target_color(target.index)
            cv2.rectangle(debug, p1, p2, color, 2)
            label = f"#{target.index + 1} {target.status}"
            if detection.results:
                best_result = max(
                    detection.results,
                    key=lambda result: float(result.hypothesis.score),
                )
                label = (
                    f"#{target.index + 1} {best_result.hypothesis.class_id} "
                    f"{float(best_result.hypothesis.score):.2f} {target.status}"
                )
            label_y = max(48, p1[1] - 8)
            cv2.putText(
                debug, label, (max(0, p1[0]), label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
            )

            if target.roi_uv is not None:
                for u, v in target.roi_uv:
                    cv2.circle(debug, (int(round(u)), int(round(v))), 2, color, -1)

            if target.estimated_uv is not None:
                cv2.drawMarker(
                    debug,
                    tuple(int(round(value)) for value in target.estimated_uv),
                    color, cv2.MARKER_CROSS, 16, 2,
                )

        if status_text:
            cv2.putText(
                debug,
                status_text,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (20, 240, 240),
                2,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_pub.publish(debug_msg)

    def publish_point(self, publisher, xyz, frame_id, stamp):
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        publisher.publish(msg)
        return msg

    @staticmethod
    def target_color(index):
        return ((255, 80, 0), (0, 200, 255), (200, 50, 220), (40, 220, 80))[index % 4]

    def clear_estimated_markers(self, header):
        for marker_id in sorted(self.marker_ids):
            marker = Marker()
            marker.header = header
            marker.ns = "damage_localization"
            marker.id = marker_id
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
        self.marker_ids.clear()

    def publish_estimated_marker(self, point_msg, index, class_name):
        sphere = Marker()
        sphere.header = point_msg.header
        sphere.ns = "damage_localization"
        sphere.id = 2 * index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = point_msg.point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.14
        sphere.scale.y = 0.14
        sphere.scale.z = 0.14
        blue, green, red = self.target_color(index)
        sphere.color.r = red / 255.0
        sphere.color.g = green / 255.0
        sphere.color.b = blue / 255.0
        sphere.color.a = 1.0
        sphere.lifetime = Duration(seconds=2.0).to_msg()
        self.marker_pub.publish(sphere)

        label = Marker()
        label.header = point_msg.header
        label.ns = "damage_localization"
        label.id = 2 * index + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = deepcopy(point_msg.point)
        label.pose.position.z += 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color.r = 1.0
        label.color.g = 0.9
        label.color.b = 0.1
        label.color.a = 1.0
        label.lifetime = sphere.lifetime
        label.text = (
            f"#{index + 1} {class_name}  x={point_msg.point.x:.2f} "
            f"y={point_msg.point.y:.2f} z={point_msg.point.z:.2f} m"
        )
        self.marker_pub.publish(label)
        self.marker_ids.update((sphere.id, label.id))

    @staticmethod
    def best_hypothesis(detection: Detection2D):
        if not detection.results:
            return None
        return max(
            detection.results,
            key=lambda result: float(result.hypothesis.score),
        ).hypothesis

    @staticmethod
    def localization_quality(inlier_count: int, roi_count: int) -> float:
        """Return a bounded geometric support score, not a learned probability."""

        if inlier_count <= 0 or roi_count <= 0:
            return 0.0
        inlier_ratio = min(1.0, float(inlier_count) / float(roi_count))
        point_support = min(1.0, float(inlier_count) / 12.0)
        return float(np.sqrt(inlier_ratio * point_support))

    def publish_defect_event(
        self,
        *,
        detections: Detection2DArray,
        detection: Detection2D,
        image: Image,
        global_point: PointStamped,
        inlier_count: int,
        roi_count: int,
        track,
    ) -> None:
        if self.event_pub is None or self.event_tracker is None:
            return

        hypothesis = self.best_hypothesis(detection)
        if hypothesis is None:
            return
        stamp = detections.header.stamp
        event = DefectEvent()
        event.header = detections.header
        event.event_id = track.event_id
        event.detection_id = (
            f"{track.event_id}:{int(stamp.sec)}.{int(stamp.nanosec):09d}"
        )
        event.camera_name = str(self.get_parameter("camera_name").value)
        event.class_name = str(hypothesis.class_id)
        event.confidence = float(hypothesis.score)
        event.severity = DefectEvent.SEVERITY_UNKNOWN
        event.bbox = detection.bbox
        event.image_width = int(image.width)
        event.image_height = int(image.height)

        event.has_3d_position = True
        event.position.header = global_point.header
        event.position.point.x = track.position[0]
        event.position.point.y = track.position[1]
        event.position.point.z = track.position[2]
        event.localization_method = DefectEvent.LOCALIZATION_CURRENT_CLOUD
        event.localization_confidence = self.localization_quality(
            inlier_count, roi_count
        )
        configured_model = str(self.get_parameter("model_name").value).strip()
        event.model_name = Path(configured_model).name if configured_model else ""
        event.snapshot_uri = ""

        semantic = self.semantic_projector.project(
            Point3D(
                x=track.position[0],
                y=track.position[1],
                z=track.position[2],
            )
        )
        event.has_semantic_location = True
        event.chainage_m = semantic.chainage_m
        event.chainage = semantic.chainage
        event.segment_name = semantic.segment_name
        event.segment_id = semantic.segment_id
        event.segment_offset_m = semantic.segment_offset_m
        event.clock_position_hours = semantic.clock_position_hours
        event.structure_area = semantic.structure_area
        self.event_pub.publish(event)
        self.event_count += 1

    def on_synced_data(self, detections, cloud, image):
        self.sync_count += 1
        stamp = detections.header.stamp
        frame_key = (int(stamp.sec), int(stamp.nanosec))
        if self.last_frame_key is not None and frame_key <= self.last_frame_key:
            return
        self.last_frame_key = frame_key
        self.clear_estimated_markers(cloud.header)
        if not detections.detections:
            self.empty_detection_count += 1
            self.publish_debug_image(image, (), status_text="detections=0")
            return
        if self.camera_info is None:
            self.warn_throttled("Waiting for CameraInfo before projecting lidar points.")
            return
        if not cloud.header.frame_id:
            self.warn_throttled("PointCloud2 has an empty frame_id.")
            return

        camera_frame = self.camera_info.header.frame_id or image.header.frame_id
        try:
            cloud_to_camera = self.lookup_transform(
                camera_frame, cloud.header.frame_id, cloud.header.stamp
            )
            points_lidar = pointcloud2_to_xyz(cloud)
            points_camera_all = transform_points(points_lidar, cloud_to_camera)
            points_camera, projected_uv, _ = project_camera_points(
                points_camera_all,
                self.camera_info.k,
                image.width,
                image.height,
            )
        except (TransformException, ValueError) as exc:
            self.projection_failure_count += 1
            self.warn_throttled(f"Projection skipped: {exc}")
            return

        self.nonempty_detection_count += 1
        mask_image = None
        mask_msg, mask_status = self.get_matching_mask(image)
        if mask_msg is not None:
            try:
                mask_image = self.mask_msg_to_array(mask_msg, image.width, image.height)
            except Exception as exc:
                mask_status = f"bad mask: {exc}"
                self.warn_throttled(mask_status)

        global_frame = str(self.get_parameter("global_frame").value)
        camera_to_global = None
        try:
            camera_to_global = self.lookup_transform(
                global_frame, camera_frame, cloud.header.stamp
            )
        except TransformException as exc:
            self.global_tf_failure_count += 1
            self.warn_throttled(f"Camera points available, but global TF failed: {exc}")

        targets = []
        event_targets = []
        for index, detection in enumerate(detections.detections):
            target = DetectionLocalization(detection=detection, index=index)
            targets.append(target)
            try:
                self.localize_detection(
                    target, points_camera, projected_uv, mask_image,
                    mask_msg is not None, camera_frame, global_frame,
                    camera_to_global, cloud.header.stamp,
                )
                if target.global_point is not None:
                    hypothesis = self.best_hypothesis(detection)
                    class_name = str(hypothesis.class_id).strip() if hypothesis else ""
                    self.publish_estimated_marker(
                        target.global_point, index, class_name or "damage"
                    )
                    if class_name and np.isfinite(hypothesis.score):
                        event_targets.append(target)
            except Exception as exc:
                self.roi_failure_count += 1
                target.status = "failed"
                self.warn_throttled(f"Detection #{index + 1} localization failed: {exc}")

        if self.event_tracker is not None:
            observations = []
            for target in event_targets:
                point = target.global_point.point
                observations.append((
                    self.best_hypothesis(target.detection).class_id,
                    (point.x, point.y, point.z),
                ))
            associations = self.event_tracker.observe_frame(
                observations=observations, frame_key=frame_key,
                stamp_sec=self.stamp_to_sec(stamp),
            )
            for target, (track, should_publish) in zip(event_targets, associations):
                if not should_publish:
                    continue
                try:
                    self.publish_defect_event(
                        detections=detections, detection=target.detection,
                        image=image, global_point=target.global_point,
                        inlier_count=target.inlier_count, roi_count=target.roi_count,
                        track=track,
                    )
                except Exception as exc:
                    self.warn_throttled(f"Event {track.event_id} publication failed: {exc}")

        self.publish_debug_image(
            image, projected_uv, targets=targets, mask_image=mask_image,
            status_text=(
                f"detections={len(targets)} "
                f"localized={sum(t.global_point is not None for t in targets)} "
                f"mask={mask_status}"
            ),
        )

    def localize_detection(
        self, target, points_camera, projected_uv, mask_image, mask_present,
        camera_frame, global_frame, camera_to_global, stamp,
    ):
        detection = target.detection
        bbox = detection.bbox
        dimensions = (bbox.center.position.x, bbox.center.position.y,
                      bbox.size_x, bbox.size_y)
        if not all(np.isfinite(value) for value in dimensions):
            raise ValueError("bbox must be finite")
        if bbox.size_x <= 0.0 or bbox.size_y <= 0.0:
            raise ValueError("bbox dimensions must be positive")
        target.drawable = True
        roi_scale = float(self.get_parameter("roi_scale").value)
        min_points = max(1, int(self.get_parameter("min_roi_points").value))
        selection_mode = "bbox"
        if mask_present:
            roi_points = np.empty((0, 3), dtype=np.float64)
            roi_uv = np.empty((0, 2), dtype=np.float64)
            selection_mode = "mask"
            if mask_image is not None:
                roi_points, roi_uv = select_mask_points(
                    points_camera, projected_uv, mask_image,
                    bbox=bbox, roi_scale=roi_scale,
                    min_value=int(self.get_parameter("mask_min_value").value),
                )
                self.mask_used_count += 1
            if len(roi_points) < min_points and bool(
                self.get_parameter("mask_fallback_to_bbox").value
            ):
                self.mask_fallback_count += 1
                roi_points, roi_uv = select_bbox_points(
                    points_camera, projected_uv, bbox, roi_scale=roi_scale,
                )
                selection_mode = "bbox_fallback"
        else:
            roi_points, roi_uv = select_bbox_points(
                points_camera, projected_uv, bbox, roi_scale=roi_scale,
            )

        target.roi_uv = roi_uv
        target.roi_count = len(roi_points)
        if len(roi_points) < min_points:
            self.roi_failure_count += 1
            target.status = f"{selection_mode} ROI {len(roi_points)}/{min_points}"
            self.warn_throttled(
                f"Detection #{target.index + 1}: {target.status}"
            )
            return

        estimated_camera, inlier_points, estimator_method = self.estimate_damage_point(
            roi_points
        )
        if not np.all(np.isfinite(estimated_camera)) or estimated_camera[2] <= 0.0:
            raise ValueError("Estimated camera point must be finite with positive depth")
        target.inlier_count = len(inlier_points)
        self.publish_point(
            self.camera_point_pub,
            estimated_camera,
            camera_frame,
            stamp,
        )
        self.camera_point_count += 1

        fx = float(self.camera_info.k[0])
        skew = float(self.camera_info.k[1])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        target.estimated_uv = (
            (
                fx * estimated_camera[0] + skew * estimated_camera[1]
            ) / estimated_camera[2] + cx,
            fy * estimated_camera[1] / estimated_camera[2] + cy,
        )
        target.status = (
            f"{estimator_method} {selection_mode} "
            f"ROI={len(roi_points)} inliers={len(inlier_points)}"
        )
        if camera_to_global is None:
            target.status += " no global TF"
            return
        estimated_global = transform_point(estimated_camera, camera_to_global)
        if not np.all(np.isfinite(estimated_global)):
            raise ValueError("Estimated global point must be finite")
        target.global_point = self.publish_point(
            self.global_point_pub,
            estimated_global,
            global_frame,
            stamp,
        )
        self.global_point_count += 1


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = DamageLocalizer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Simulation-only model annotation. Does not run or publish YOLO detections."""

from functools import partial
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from metro_inspection_interfaces.msg import DefectEvent
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .model_annotation_geometry import load_tunnel_regions, project_region
from .semantic_geometry import Point3D, TunnelSemanticProjector


CAMERAS = {
    "odin1": "/odin1/rgb",
    "xj1": "/subway_v2/xj1",
    "xj2": "/subway_v2/xj2",
    "xj3": "/subway_v2/xj3",
    "xj4": "/subway_v2/xj4",
    "pitch_camera": "/subway_v2/pitch_camera",
}


def transform_matrix(transform):
    result = np.eye(4)
    q = transform.rotation
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    p = transform.translation
    result[:3, 3] = [p.x, p.y, p.z]
    return result


class ModelAnnotationNode(Node):
    def __init__(self):
        super().__init__("model_annotation")
        self.declare_parameter("mesh_path", "")
        self.declare_parameter("event_topic", "/model_annotation/defect_events")
        self.declare_parameter("max_distance_m", 12.0)
        self.declare_parameter("camera_fps", 2.0)
        self.bridge = CvBridge()
        self.tf = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.tf, self)
        self.marker_frame_broadcaster = TransformBroadcaster(self)
        self.last_marker_frame_stamp = -1
        self.world_pose = Buffer(cache_time=Duration(seconds=20))
        self.infos, self.pending, self.last_frame, self.last_event = {}, {}, {}, {}
        self.create_subscription(Odometry, "/wheel/odom_raw", self.on_world_pose, qos_profile_sensor_data)
        self.publishers_by_camera = {}
        for camera, topic in CAMERAS.items():
            self.create_subscription(CameraInfo, topic + "/camera_info", partial(self.on_info, camera), qos_profile_sensor_data)
            self.create_subscription(CompressedImage, topic + "/image_raw/compressed", partial(self.on_image, camera), qos_profile_sensor_data)
            self.publishers_by_camera[camera] = self.create_publisher(
                CompressedImage, f"/model_annotation/{camera}/image/compressed", qos_profile_sensor_data)
        self.events = self.create_publisher(DefectEvent, self.get_parameter("event_topic").value, 20)
        self.semantic = TunnelSemanticProjector(
            chainage_start_m=12000, chainage_axis="x", chainage_sign=1,
            segment_start_id=1000, segment_length_m=1.2, segment_name="仿真环号",
            clock_center_y_m=0, clock_center_z_m=1.75)
        self.get_logger().info("Loading model surfaces for MODEL ANNOTATION DEMO")
        self.mesh, self.regions = load_tunnel_regions(Path(self.get_parameter("mesh_path").value))
        self.get_logger().info(f"Loaded {len(self.regions)} reference regions; no YOLO inference")
        self.create_timer(0.05, self.process_pending)
        self.last_warning = 0.0

    def on_world_pose(self, message):
        # gazebo_ros_diff_drive publishes the simulation WorldPose on wheel/odom_raw.
        # Keep it in a private buffer, separate from the EKF's relative odom frame.
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "model_demo_base"
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.world_pose.set_transform(transform, "gazebo_world_pose")

    def on_info(self, camera, message):
        self.infos[camera] = message

    def on_image(self, camera, message):
        self.pending[camera] = message

    def process_pending(self):
        now = time.monotonic()
        interval = 1.0 / max(0.1, float(self.get_parameter("camera_fps").value))
        for camera, message in list(self.pending.items()):
            if now - self.last_frame.get(camera, -interval) < interval:
                continue
            info = self.infos.get(camera)
            if info is None:
                continue
            stamp = Time.from_msg(message.header.stamp)
            try:
                base_camera = self.tf.lookup_transform("base_footprint", info.header.frame_id, stamp)
                world_base = self.world_pose.lookup_transform("world", "model_demo_base", stamp)
            except TransformException as error:
                if now - self.last_warning > 5:
                    self.get_logger().warning(f"Waiting for timestamped model projection pose: {error}")
                    self.last_warning = now
                continue
            self.pending.pop(camera, None)
            self.last_frame[camera] = now
            image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                continue
            height, width = image.shape[:2]
            if (width, height) != (info.width, info.height):
                continue
            pose = transform_matrix(world_base.transform) @ transform_matrix(base_camera.transform)
            self.update_marker_frame(stamp, world_base)
            k = np.array(info.k).reshape(3, 3)
            distortion = np.asarray(info.d) if info.d else np.zeros(5)
            for region in self.regions:
                box = project_region(region, pose, k, distortion, width, height, self.mesh,
                                     float(self.get_parameter("max_distance_m").value))
                if box is None:
                    continue
                x1, y1, x2, y2 = map(int, box)
                color = {
                    "water_leakage": (50, 210, 245),
                    "crack": (245, 170, 70),
                    "foreign_object": (100, 230, 80),
                    "segment_damage": (200, 100, 245),
                    "fastener_loose": (70, 150, 255),
                    "fastener_missing": (220, 80, 220),
                    "fastener_broken": (90, 90, 255),
                    "bracket_loose": (255, 150, 60),
                }[region.class_name]
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
                label = f"MODEL {region.site_id} {region.class_name}"
                tx = min(x1, max(0, width - cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)[0][0] - 8))
                cv2.putText(image, label, (tx, max(55, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                if now - self.last_event.get(region.site_id, -10) >= 2:
                    self.publish_event(region, camera, message.header, box, width, height)
                    self.last_event[region.site_id] = now
            cv2.rectangle(image, (0, 0), (width, 38), (25, 25, 25), -1)
            cv2.putText(image, "MODEL ANNOTATION DEMO | NO YOLO", (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 210, 245), 2)
            output = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")
            output.header = message.header
            self.publishers_by_camera[camera].publish(output)

    def update_marker_frame(self, stamp, world_base):
        if stamp.nanoseconds <= self.last_marker_frame_stamp:
            return
        try:
            odom_base = self.tf.lookup_transform("odom", "base_footprint", stamp)
        except TransformException:
            return
        # WorldPose and EKF odom have different origins. Align the display-only
        # world frame using the same robot pose/time, without changing odom TF.
        odom_world = transform_matrix(odom_base.transform) @ np.linalg.inv(
            transform_matrix(world_base.transform))
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "model_annotation_world"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, odom_world[:3, 3])
        q = Rotation.from_matrix(odom_world[:3, :3]).as_quat()
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
        self.marker_frame_broadcaster.sendTransform(transform)
        self.last_marker_frame_stamp = stamp.nanoseconds

    def publish_event(self, region, camera, header, box, width, height):
        event = DefectEvent()
        event.header = header
        event.event_id = f"model-demo:{region.site_id}"
        event.detection_id = f"{event.event_id}:{camera}:{header.stamp.sec}.{header.stamp.nanosec:09d}"
        event.camera_name, event.class_name = camera, region.class_name
        event.confidence = 0.0
        x1, y1, x2, y2 = box
        event.bbox.center.position.x = float((x1 + x2) / 2)
        event.bbox.center.position.y = float((y1 + y2) / 2)
        event.bbox.size_x, event.bbox.size_y = float(x2 - x1), float(y2 - y1)
        event.image_width, event.image_height = width, height
        event.has_3d_position = True
        event.position.header.stamp = header.stamp
        event.position.header.frame_id = "world"
        event.position.point.x, event.position.point.y, event.position.point.z = map(float, region.reference)
        event.localization_method = DefectEvent.LOCALIZATION_MODEL_REFERENCE
        event.model_name = "subway_tunnel_v2 [MODEL ANNOTATION DEMO; no YOLO]"
        semantic = self.semantic.project(Point3D(*region.reference))
        event.has_semantic_location = True
        for field in ("chainage_m", "chainage", "segment_name", "segment_id", "segment_offset_m", "clock_position_hours", "structure_area"):
            setattr(event, field, getattr(semantic, field))
        self.events.publish(event)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = ModelAnnotationNode()
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

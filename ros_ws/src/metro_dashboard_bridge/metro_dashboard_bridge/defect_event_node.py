import threading
from functools import partial
from typing import Dict, List

import rclpy
from metro_inspection_interfaces.msg import DefectEvent
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage

from .camera_store import CameraDefinition, CameraFrameStore
from .camera_stream import CameraStreamConfig
from .defect_store import DefectStore
from .event_conversion import defect_event_to_record


DEFAULT_CAMERAS = (
    ("xj1", "XJ1", "/subway_v2/xj1/image_raw/compressed"),
    ("xj2", "XJ2", "/subway_v2/xj2/image_raw/compressed"),
    ("xj3", "XJ3", "/subway_v2/xj3/image_raw/compressed"),
    ("xj4", "XJ4", "/subway_v2/xj4/image_raw/compressed"),
    (
        "pitch_camera",
        "Pitch",
        "/subway_v2/pitch_camera/image_raw/compressed",
    ),
)


class DefectEventBridgeNode(Node):
    """Validate ROS defect events and expose them through a shared store."""

    def __init__(self) -> None:
        super().__init__("defect_event_bridge")
        self.declare_parameter(
            "defect_topic",
            "defect_events",
            ParameterDescriptor(
                description="DefectEvent topic consumed by the dashboard bridge.",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "maximum_records",
            500,
            ParameterDescriptor(
                description="Maximum number of defect events retained in memory.",
                read_only=True,
                integer_range=[IntegerRange(from_value=1, to_value=100000, step=1)],
            ),
        )
        for camera_id, _, topic in DEFAULT_CAMERAS:
            self.declare_parameter(
                f"camera_topics.{camera_id}",
                topic,
                ParameterDescriptor(
                    description=f"CompressedImage topic for camera {camera_id}.",
                    read_only=True,
                ),
            )
        self.declare_parameter(
            "camera_timeout_seconds",
            3.0,
            ParameterDescriptor(
                description="Age after which a camera is reported offline.",
                read_only=True,
                floating_point_range=[
                    FloatingPointRange(from_value=0.5, to_value=30.0, step=0.1)
                ],
            ),
        )
        self.declare_parameter(
            "camera_preview_width",
            640,
            ParameterDescriptor(
                description="Maximum browser preview width in pixels.",
                read_only=True,
                integer_range=[IntegerRange(from_value=160, to_value=1920, step=1)],
            ),
        )
        self.declare_parameter(
            "camera_preview_height",
            360,
            ParameterDescriptor(
                description="Maximum browser preview height in pixels.",
                read_only=True,
                integer_range=[IntegerRange(from_value=90, to_value=1080, step=1)],
            ),
        )
        self.declare_parameter(
            "camera_preview_jpeg_quality",
            55,
            ParameterDescriptor(
                description="JPEG quality used for browser camera previews.",
                read_only=True,
                integer_range=[IntegerRange(from_value=20, to_value=95, step=1)],
            ),
        )
        self.declare_parameter(
            "camera_stream_default_fps",
            6.0,
            ParameterDescriptor(
                description="Default browser camera stream frame rate.",
                read_only=True,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=30.0, step=0.5)
                ],
            ),
        )
        self.declare_parameter(
            "camera_stream_max_fps",
            10.0,
            ParameterDescriptor(
                description="Maximum browser-requested camera stream frame rate.",
                read_only=True,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=30.0, step=0.5)
                ],
            ),
        )

        topic = str(self.get_parameter("defect_topic").value).strip()
        if not topic:
            raise ValueError("defect_topic must not be empty")
        maximum_records = int(self.get_parameter("maximum_records").value)
        self.store = DefectStore(maximum_records=maximum_records)
        self._statistics_lock = threading.Lock()
        self._accepted_count = 0
        self._rejected_count = 0
        camera_definitions = [
            CameraDefinition(
                camera_id=camera_id,
                label=label,
                topic=str(
                    self.get_parameter(f"camera_topics.{camera_id}").value
                ).strip(),
            )
            for camera_id, label, _ in DEFAULT_CAMERAS
        ]
        self.camera_timeout_seconds = float(
            self.get_parameter("camera_timeout_seconds").value
        )
        self.camera_stream_config = CameraStreamConfig(
            max_width=int(self.get_parameter("camera_preview_width").value),
            max_height=int(self.get_parameter("camera_preview_height").value),
            jpeg_quality=int(
                self.get_parameter("camera_preview_jpeg_quality").value
            ),
            default_fps=float(
                self.get_parameter("camera_stream_default_fps").value
            ),
            max_fps=float(self.get_parameter("camera_stream_max_fps").value),
        )
        self.camera_store = CameraFrameStore(camera_definitions)

        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            DefectEvent,
            topic,
            self._on_defect_event,
            event_qos,
        )
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._camera_subscriptions: List[object] = []
        for definition in self.camera_store.definitions:
            self._camera_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    definition.topic,
                    partial(self._on_camera_frame, definition.camera_id),
                    camera_qos,
                )
            )
        self.get_logger().info(
            f"Defect event bridge listening on '{topic}' with reliable QoS"
        )
        self.get_logger().info(
            "Camera bridge listening on: "
            + ", ".join(
                f"{definition.camera_id}={definition.topic}"
                for definition in self.camera_store.definitions
            )
        )

    def _on_defect_event(self, message: DefectEvent) -> None:
        try:
            record = defect_event_to_record(message)
            self.store.upsert(record)
        except ValueError as error:
            with self._statistics_lock:
                self._rejected_count += 1
            self.get_logger().warning(f"Rejected invalid DefectEvent: {error}")
            return

        with self._statistics_lock:
            self._accepted_count += 1
            accepted_count = self._accepted_count
        if accepted_count == 1 or accepted_count % 20 == 0:
            self.get_logger().info(
                f"Accepted {accepted_count} DefectEvent messages; "
                f"stored={self.store.count()}"
            )

    def _on_camera_frame(self, camera_id: str, message: CompressedImage) -> None:
        source_timestamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1_000_000_000.0
        )
        try:
            self.camera_store.update(
                camera_id=camera_id,
                data=message.data,
                image_format=message.format,
                frame_id=message.header.frame_id,
                source_timestamp=source_timestamp,
            )
        except ValueError as error:
            self.get_logger().warning(
                f"Rejected invalid frame from camera '{camera_id}': {error}"
            )

    def statistics(self) -> Dict[str, int]:
        with self._statistics_lock:
            return {
                "accepted_messages": self._accepted_count,
                "rejected_messages": self._rejected_count,
                "stored_events": self.store.count(),
            }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DefectEventBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

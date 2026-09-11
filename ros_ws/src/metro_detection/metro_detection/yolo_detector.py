"""ROS 2 image detector backed by an Ultralytics YOLO checkpoint."""

import os
import signal
import time
import warnings
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Mapping, Sequence

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
import cv2
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

from .detection_conversion import (
    DetectionBox,
    display_class_names,
    normalize_class_names,
    to_detection_array,
)


def image_qos_profile(reliability: str) -> QoSProfile:
    """Create an input QoS that can wake Gazebo's demand-driven cameras."""
    normalized = reliability.strip().lower()
    policies = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }
    if normalized not in policies:
        raise ValueError(
            "image_reliability must be 'reliable' or 'best_effort', "
            f"found {reliability!r}"
        )
    return QoSProfile(depth=1, reliability=policies[normalized])


def to_compressed_image(bgr_image, header, jpeg_quality: int) -> CompressedImage:
    """Encode one annotated OpenCV image for browser-friendly transport."""
    quality = int(jpeg_quality)
    if not 20 <= quality <= 95:
        raise ValueError("annotated_jpeg_quality must be between 20 and 95")
    success, encoded = cv2.imencode(
        ".jpg",
        bgr_image,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not success:
        raise RuntimeError("OpenCV could not encode the annotated image")

    message = CompressedImage()
    message.header = header
    message.format = "jpeg"
    message.data = encoded.tobytes()
    return message


@dataclass
class CameraStream:
    """ROS interfaces and counters for one camera using the shared model."""

    name: str
    image_topic: str
    detections_topic: str
    annotated_image_topic: str
    detection_pub: object = None
    annotated_pub: object = None
    annotated_compressed_pub: object = None
    image_sub: object = None
    last_inference_monotonic: float = 0.0
    received_frames: int = 0
    processed_frames: int = 0
    dropped_frames: int = 0
    published_boxes: int = 0
    inference_seconds: float = 0.0


class YoloDetector(Node):
    """Run one shared YOLO model on one or more camera image streams."""

    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/odin1/rgb/image_raw")
        self.declare_parameter("detections_topic", "/damage_detections")
        self.declare_parameter(
            "annotated_image_topic", "/damage_detection/annotated_image"
        )
        self.declare_parameter("camera_names", [""])
        self.declare_parameter("image_topics", [""])
        self.declare_parameter("detections_topics", [""])
        self.declare_parameter("annotated_image_topics", [""])
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("use_half_precision", False)
        self.declare_parameter("fallback_to_cpu", True)
        self.declare_parameter("max_inference_rate_hz", 10.0)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("annotated_jpeg_quality", 75)
        self.declare_parameter("status_period_sec", 5.0)
        self.declare_parameter("image_reliability", "reliable")
        self.declare_parameter("readiness_topic", "/yolo/ready")

        model_path = Path(
            os.path.expanduser(str(self.get_parameter("model_path").value))
        )
        if not model_path.is_file():
            raise RuntimeError(
                "YOLO model does not exist: "
                f"{model_path}. Set model_path or METRO_YOLO_MODEL_PATH."
            )

        self.bridge = CvBridge()
        self.image_qos = image_qos_profile(
            str(self.get_parameter("image_reliability").value)
        )
        self.streams = self.configure_streams()
        self.ready = False
        readiness_qos = QoSProfile(depth=1)
        readiness_qos.reliability = ReliabilityPolicy.RELIABLE
        readiness_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.readiness_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("readiness_topic").value),
            readiness_qos,
        )
        for stream in self.streams:
            stream.detection_pub = self.create_publisher(
                Detection2DArray,
                stream.detections_topic,
                qos_profile_sensor_data,
            )
            stream.annotated_pub = self.create_publisher(
                Image,
                stream.annotated_image_topic,
                qos_profile_sensor_data,
            )
            stream.annotated_compressed_pub = self.create_publisher(
                CompressedImage,
                stream.annotated_image_topic + "/compressed",
                qos_profile_sensor_data,
            )
            stream.image_sub = self.create_subscription(
                Image,
                stream.image_topic,
                partial(self.on_image, stream),
                self.image_qos,
            )

        try:
            warnings.filterwarnings(
                "ignore",
                message="Unable to import Axes3D.*",
                category=UserWarning,
            )
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics/PyTorch is unavailable. Run "
                "./scripts/setup_yolo_environment.sh and start this node "
                "through ./scripts/open_yolo_detector.sh."
            ) from exc

        self.torch = torch
        self.model = YOLO(str(model_path))
        raw_names = self.model.names
        normalized_names = normalize_class_names(raw_names)
        # Rendering and backend initialization can replace model label metadata.
        self.class_names = normalize_class_names(raw_names)
        self.model.model.names = normalized_names
        self.annotation_names = display_class_names(normalized_names)
        self.device = self.resolve_device(
            str(self.get_parameter("device").value)
        )
        self.cpu_fallback_used = False
        status_period = max(
            1.0, float(self.get_parameter("status_period_sec").value)
        )
        self.status_timer = self.create_timer(status_period, self.log_status)

        names = self.model.names
        labels = names.values() if isinstance(names, Mapping) else names
        class_labels = ", ".join(str(name) for name in labels)
        self.get_logger().info(f"Loaded YOLO model: {model_path}")
        if normalized_names != raw_names:
            self.get_logger().info(
                f"Normalized checkpoint classes: {raw_names} -> "
                f"{normalized_names}"
            )
        self.get_logger().info(
            f"Device={self.device}; image_reliability="
            f"{self.image_qos.reliability.name}; classes=[{class_labels}]"
        )
        for stream in self.streams:
            self.get_logger().info(
                f"Pipeline[{stream.name}]: {stream.image_topic} -> "
                f"{stream.detections_topic}; annotated="
                f"{stream.annotated_image_topic}[/compressed]"
            )

    @staticmethod
    def _nonempty_strings(values: Sequence[object]) -> list[str]:
        return [str(value).strip() for value in values if str(value).strip()]

    def configure_streams(self) -> list[CameraStream]:
        """Use array parameters when configured, otherwise retain legacy mode."""
        names = self._nonempty_strings(
            self.get_parameter("camera_names").value
        )
        if not names:
            return [
                CameraStream(
                    name="camera",
                    image_topic=str(
                        self.get_parameter("image_topic").value
                    ).strip(),
                    detections_topic=str(
                        self.get_parameter("detections_topic").value
                    ).strip(),
                    annotated_image_topic=str(
                        self.get_parameter("annotated_image_topic").value
                    ).strip(),
                )
            ]

        parameter_names = (
            "image_topics",
            "detections_topics",
            "annotated_image_topics",
        )
        values = {
            parameter_name: self._nonempty_strings(
                self.get_parameter(parameter_name).value
            )
            for parameter_name in parameter_names
        }
        for parameter_name, configured in values.items():
            if len(configured) != len(names):
                raise ValueError(
                    f"camera_names has {len(names)} entries, but "
                    f"{parameter_name} has {len(configured)}"
                )
        if len(set(names)) != len(names):
            raise ValueError("camera_names must be unique")
        if len(set(values["image_topics"])) != len(names):
            raise ValueError("image_topics must be unique")
        if len(set(values["detections_topics"])) != len(names):
            raise ValueError("detections_topics must be unique")

        return [
            CameraStream(
                name=name,
                image_topic=values["image_topics"][index],
                detections_topic=values["detections_topics"][index],
                annotated_image_topic=values["annotated_image_topics"][index],
            )
            for index, name in enumerate(names)
        ]

    def resolve_device(self, requested: str) -> str:
        """Resolve `auto` and retain explicit Ultralytics device syntax."""
        normalized = requested.strip().lower()
        if normalized in ("", "auto"):
            return "0" if self.torch.cuda.is_available() else "cpu"
        if normalized.startswith("cuda:"):
            normalized = normalized.split(":", 1)[1]
        if normalized != "cpu" and not self.torch.cuda.is_available():
            if bool(self.get_parameter("fallback_to_cpu").value):
                self.get_logger().warning(
                    f"CUDA device '{requested}' is unavailable; using CPU."
                )
                return "cpu"
            raise RuntimeError(
                f"Requested CUDA device '{requested}' is unavailable."
            )
        return normalized

    def should_process_frame(self, stream: CameraStream) -> bool:
        """Limit each camera independently without queuing old images."""
        max_rate = float(self.get_parameter("max_inference_rate_hz").value)
        now = time.monotonic()
        if max_rate > 0.0 and stream.last_inference_monotonic > 0.0:
            if now - stream.last_inference_monotonic < 1.0 / max_rate:
                return False
        stream.last_inference_monotonic = now
        return True

    def predict(self, bgr_image):
        """Run inference and retry once on CPU after a CUDA runtime failure."""
        kwargs = {
            "source": bgr_image,
            "conf": float(self.get_parameter("confidence_threshold").value),
            "iou": float(self.get_parameter("iou_threshold").value),
            "imgsz": int(self.get_parameter("image_size").value),
            "device": self.device,
            "verbose": False,
        }
        if (
            bool(self.get_parameter("use_half_precision").value)
            and self.device != "cpu"
        ):
            kwargs["half"] = True
        try:
            return self.model.predict(**kwargs)[0]
        except Exception as exc:
            can_fallback = (
                self.device != "cpu"
                and bool(self.get_parameter("fallback_to_cpu").value)
                and not self.cpu_fallback_used
            )
            if not can_fallback:
                raise
            self.cpu_fallback_used = True
            self.device = "cpu"
            kwargs["device"] = "cpu"
            kwargs.pop("half", None)
            self.get_logger().warning(
                "CUDA inference failed; retrying on CPU once. "
                f"Original error: {exc}"
            )
            return self.model.predict(**kwargs)[0]

    @staticmethod
    def result_boxes(result):
        """Detach Ultralytics tensors before creating ROS messages."""
        if result.boxes is None or len(result.boxes) == 0:
            return []
        xyxy = result.boxes.xyxy.detach().cpu().tolist()
        confidence = result.boxes.conf.detach().cpu().tolist()
        class_indices = result.boxes.cls.detach().cpu().tolist()
        return [
            DetectionBox(
                x1=coords[0],
                y1=coords[1],
                x2=coords[2],
                y2=coords[3],
                confidence=score,
                class_index=int(class_index),
            )
            for coords, score, class_index in zip(
                xyxy, confidence, class_indices
            )
        ]

    def publish_empty(self, stream: CameraStream, image: Image):
        stream.detection_pub.publish(
            to_detection_array(image.header, [], self.class_names)
        )

    def on_image(self, stream: CameraStream, image: Image):
        stream.received_frames += 1
        if not self.should_process_frame(stream):
            stream.dropped_frames += 1
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(
                f"Cannot convert input image from {stream.name}: {exc}"
            )
            self.publish_empty(stream, image)
            return

        started = time.perf_counter()
        try:
            result = self.predict(bgr)
        except Exception as exc:
            self.get_logger().error(
                f"YOLO inference failed for {stream.name}: {exc}"
            )
            self.publish_empty(stream, image)
            return
        elapsed = time.perf_counter() - started

        boxes = self.result_boxes(result)
        output = to_detection_array(image.header, boxes, self.class_names)
        stream.detection_pub.publish(output)
        stream.processed_frames += 1
        stream.published_boxes += len(output.detections)
        stream.inference_seconds += elapsed
        if not self.ready and all(
            configured.processed_frames > 0 for configured in self.streams
        ):
            self.ready = True
            ready_message = Bool()
            ready_message.data = True
            self.readiness_pub.publish(ready_message)
            self.get_logger().info(
                "YOLO is ready: every configured camera has completed inference"
            )

        if bool(self.get_parameter("publish_annotated_image").value):
            try:
                result.names = self.annotation_names
                rendered = result.plot()
                annotated = self.bridge.cv2_to_imgmsg(
                    rendered, encoding="bgr8"
                )
                annotated.header = image.header
                stream.annotated_pub.publish(annotated)
                stream.annotated_compressed_pub.publish(
                    to_compressed_image(
                        rendered,
                        image.header,
                        int(
                            self.get_parameter(
                                "annotated_jpeg_quality"
                            ).value
                        ),
                    )
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"Cannot publish {stream.name} annotated image: {exc}"
                )

    def log_status(self):
        summaries = []
        for stream in self.streams:
            average_ms = (
                1000.0 * stream.inference_seconds / stream.processed_frames
                if stream.processed_frames
                else 0.0
            )
            summaries.append(
                f"{stream.name}:received={stream.received_frames},"
                f"processed={stream.processed_frames},"
                f"rate_limited={stream.dropped_frames},"
                f"boxes={stream.published_boxes},"
                f"average={average_ms:.1f}ms"
            )
        self.get_logger().info(
            f"YOLO status ({self.device}): " + "; ".join(summaries)
        )


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = None
    try:
        node = YoloDetector()
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

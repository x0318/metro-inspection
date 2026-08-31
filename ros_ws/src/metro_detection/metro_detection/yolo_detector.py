"""ROS 2 image detector backed by an Ultralytics YOLO checkpoint."""

import os
import time
import warnings
from pathlib import Path
from typing import Mapping

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from .detection_conversion import DetectionBox, to_detection_array


class YoloDetector(Node):
    """Run YOLO on the newest available camera frame and publish 2D boxes."""

    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/odin1/rgb/image_raw")
        self.declare_parameter("detections_topic", "/damage_detections")
        self.declare_parameter(
            "annotated_image_topic", "/damage_detection/annotated_image"
        )
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("use_half_precision", False)
        self.declare_parameter("fallback_to_cpu", True)
        self.declare_parameter("max_inference_rate_hz", 10.0)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("status_period_sec", 5.0)

        model_path = Path(
            os.path.expanduser(str(self.get_parameter("model_path").value))
        )
        if not model_path.is_file():
            raise RuntimeError(
                "YOLO model does not exist: "
                f"{model_path}. Set model_path or METRO_YOLO_MODEL_PATH."
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
        self.device = self.resolve_device(
            str(self.get_parameter("device").value)
        )
        self.cpu_fallback_used = False
        self.bridge = CvBridge()
        self.last_inference_monotonic = 0.0
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.published_boxes = 0
        self.inference_seconds = 0.0

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter("detections_topic").value),
            qos_profile_sensor_data,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            str(self.get_parameter("annotated_image_topic").value),
            qos_profile_sensor_data,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )
        status_period = max(
            1.0, float(self.get_parameter("status_period_sec").value)
        )
        self.status_timer = self.create_timer(status_period, self.log_status)

        names = self.model.names
        labels = names.values() if isinstance(names, Mapping) else names
        class_labels = ", ".join(str(name) for name in labels)
        self.get_logger().info(f"Loaded YOLO model: {model_path}")
        self.get_logger().info(
            f"Device={self.device}; classes=[{class_labels}]"
        )
        self.get_logger().info(
            "Pipeline: "
            f"{self.get_parameter('image_topic').value} -> "
            f"{self.get_parameter('detections_topic').value}"
        )

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

    def should_process_frame(self) -> bool:
        """Limit inference rate without queuing old high-resolution images."""

        max_rate = float(self.get_parameter("max_inference_rate_hz").value)
        now = time.monotonic()
        if max_rate > 0.0 and self.last_inference_monotonic > 0.0:
            if now - self.last_inference_monotonic < 1.0 / max_rate:
                return False
        self.last_inference_monotonic = now
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

    def publish_empty(self, image: Image):
        self.detection_pub.publish(
            to_detection_array(image.header, [], self.model.names)
        )

    def on_image(self, image: Image):
        self.received_frames += 1
        if not self.should_process_frame():
            self.dropped_frames += 1
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Cannot convert input image: {exc}")
            self.publish_empty(image)
            return

        started = time.perf_counter()
        try:
            result = self.predict(bgr)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            self.publish_empty(image)
            return
        elapsed = time.perf_counter() - started

        boxes = self.result_boxes(result)
        output = to_detection_array(image.header, boxes, self.model.names)
        self.detection_pub.publish(output)
        self.processed_frames += 1
        self.published_boxes += len(output.detections)
        self.inference_seconds += elapsed

        if bool(self.get_parameter("publish_annotated_image").value):
            try:
                annotated = self.bridge.cv2_to_imgmsg(
                    result.plot(), encoding="bgr8"
                )
                annotated.header = image.header
                self.annotated_pub.publish(annotated)
            except Exception as exc:
                self.get_logger().warning(
                    f"Cannot publish annotated image: {exc}"
                )

    def log_status(self):
        average_ms = (
            1000.0 * self.inference_seconds / self.processed_frames
            if self.processed_frames
            else 0.0
        )
        self.get_logger().info(
            "YOLO status: "
            f"received={self.received_frames}, "
            f"processed={self.processed_frames}, "
            f"rate_limited={self.dropped_frames}, "
            f"boxes={self.published_boxes}, "
            f"average_inference={average_ms:.1f} ms, "
            f"device={self.device}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

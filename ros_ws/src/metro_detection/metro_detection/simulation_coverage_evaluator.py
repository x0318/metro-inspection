"""Evaluate ten-defect simulation coverage from real YOLO output messages."""

import json
import signal
from functools import partial

import rclpy
from metro_inspection_interfaces.msg import DefectEvent
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from .coverage_tracking import (
    CoverageTracker,
    DefectSite,
    DetectionObservation,
    SiteConfirmation,
)


class SimulationCoverageEvaluator(Node):
    """Confirm simulation sites only after actual YOLO boxes are received."""

    def __init__(self) -> None:
        super().__init__("simulation_coverage_evaluator")
        self.declare_parameter("odometry_topic", "/wheel/odom_raw")
        self.declare_parameter("camera_names", ["xj1"])
        self.declare_parameter("detection_topics", ["/damage_detections/xj1"])
        self.declare_parameter("camera_offsets_m", [0.0])
        self.declare_parameter("image_widths", [1440])
        self.declare_parameter("image_heights", [1080])
        self.declare_parameter("defect_ids", ["sim_defect_01"])
        self.declare_parameter("defect_chainages_m", [0.0])
        self.declare_parameter("match_tolerance_m", 0.9)
        self.declare_parameter("confirmation_hits", 3)
        self.declare_parameter(
            "coverage_topic", "/simulation/defect_coverage"
        )
        self.declare_parameter("event_topic", "/simulation/defect_events")
        self.declare_parameter("publish_events", True)
        self.declare_parameter("event_republish_period_sec", 5.0)
        self.declare_parameter("status_period_sec", 2.0)
        self.declare_parameter(
            "model_name", "yolov8n_sim_demo_best(1).pt"
        )

        camera_names = self._strings("camera_names")
        detection_topics = self._strings("detection_topics")
        camera_offsets = self._floats("camera_offsets_m")
        image_widths = self._integers("image_widths")
        image_heights = self._integers("image_heights")
        self._same_length(
            camera_names,
            detection_topics=detection_topics,
            camera_offsets_m=camera_offsets,
            image_widths=image_widths,
            image_heights=image_heights,
        )
        if len(set(camera_names)) != len(camera_names):
            raise ValueError("camera_names must be unique")

        defect_ids = self._strings("defect_ids")
        defect_chainages = self._floats("defect_chainages_m")
        self._same_length(defect_ids, defect_chainages_m=defect_chainages)
        sites = [
            DefectSite(site_id=site_id, chainage_m=chainage)
            for site_id, chainage in zip(defect_ids, defect_chainages)
        ]
        offsets_by_camera = dict(zip(camera_names, camera_offsets))
        self.tracker = CoverageTracker(
            sites=sites,
            camera_offsets_m=offsets_by_camera,
            match_tolerance_m=float(
                self.get_parameter("match_tolerance_m").value
            ),
            confirmation_hits=int(
                self.get_parameter("confirmation_hits").value
            ),
        )
        self.image_sizes = {
            name: (width, height)
            for name, width, height in zip(
                camera_names, image_widths, image_heights
            )
        }
        self.model_name = str(self.get_parameter("model_name").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)
        self.confirmed_events = {}
        self.robot_chainage_m = None

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.coverage_pub = self.create_publisher(
            String,
            str(self.get_parameter("coverage_topic").value),
            status_qos,
        )
        self.event_pub = self.create_publisher(
            DefectEvent,
            str(self.get_parameter("event_topic").value),
            20,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.detection_subs = [
            self.create_subscription(
                Detection2DArray,
                topic,
                partial(self.on_detections, camera_name),
                qos_profile_sensor_data,
            )
            for camera_name, topic in zip(camera_names, detection_topics)
        ]
        status_period = max(
            0.5, float(self.get_parameter("status_period_sec").value)
        )
        self.status_timer = self.create_timer(status_period, self.publish_status)
        event_republish_period = float(
            self.get_parameter("event_republish_period_sec").value
        )
        self.event_timer = None
        if self.publish_events and event_republish_period > 0.0:
            self.event_timer = self.create_timer(
                event_republish_period, self.republish_events
            )
        self.publish_status()
        self.get_logger().info(
            f"Truth-assisted coverage evaluator ready for {len(sites)} sites. "
            "Reference positions validate real YOLO boxes; they never create boxes."
        )

    def _strings(self, parameter_name: str) -> list[str]:
        values = [
            str(value).strip()
            for value in self.get_parameter(parameter_name).value
        ]
        if not values or any(not value for value in values):
            raise ValueError(f"{parameter_name} must contain nonempty values")
        return values

    def _floats(self, parameter_name: str) -> list[float]:
        return [
            float(value) for value in self.get_parameter(parameter_name).value
        ]

    def _integers(self, parameter_name: str) -> list[int]:
        values = [
            int(value) for value in self.get_parameter(parameter_name).value
        ]
        if any(value <= 0 for value in values):
            raise ValueError(f"{parameter_name} values must be positive")
        return values

    @staticmethod
    def _same_length(reference: list, **named_values: list) -> None:
        if not reference:
            raise ValueError("parameter arrays must not be empty")
        for name, values in named_values.items():
            if len(values) != len(reference):
                raise ValueError(
                    f"expected {len(reference)} {name} values, got {len(values)}"
                )

    def on_odometry(self, message: Odometry) -> None:
        self.robot_chainage_m = float(message.pose.pose.position.x)

    @staticmethod
    def observations(
        camera_name: str, message: Detection2DArray
    ) -> list[DetectionObservation]:
        output = []
        for detection in message.detections:
            if not detection.results:
                continue
            hypothesis = max(
                detection.results,
                key=lambda result: result.hypothesis.score,
            ).hypothesis
            output.append(
                DetectionObservation(
                    camera_name=camera_name,
                    stamp_sec=int(message.header.stamp.sec),
                    stamp_nanosec=int(message.header.stamp.nanosec),
                    frame_id=message.header.frame_id,
                    class_name=hypothesis.class_id,
                    confidence=float(hypothesis.score),
                    center_x=float(detection.bbox.center.position.x),
                    center_y=float(detection.bbox.center.position.y),
                    size_x=float(detection.bbox.size_x),
                    size_y=float(detection.bbox.size_y),
                )
            )
        return output

    def on_detections(
        self, camera_name: str, message: Detection2DArray
    ) -> None:
        if self.robot_chainage_m is None:
            return
        confirmation = self.tracker.observe(
            robot_chainage_m=self.robot_chainage_m,
            camera_name=camera_name,
            observations=self.observations(camera_name, message),
        )
        if confirmation is None:
            return

        self.get_logger().info(
            f"Confirmed {confirmation.site.site_id} at "
            f"x={confirmation.site.chainage_m:.3f} m from real YOLO frames: "
            f"class={confirmation.observation.class_name}, "
            f"camera={confirmation.observation.camera_name}, "
            f"hits={confirmation.hit_count}"
        )
        if self.publish_events:
            event = self.to_event(confirmation)
            self.confirmed_events[event.event_id] = event
            self.event_pub.publish(event)
        self.publish_status()

    def republish_events(self) -> None:
        """Let a dashboard started later receive the stable confirmed set."""
        for event in self.confirmed_events.values():
            self.event_pub.publish(event)

    def to_event(self, confirmation: SiteConfirmation) -> DefectEvent:
        observation = confirmation.observation
        event = DefectEvent()
        event.header.stamp.sec = observation.stamp_sec
        event.header.stamp.nanosec = observation.stamp_nanosec
        event.header.frame_id = observation.frame_id
        event.event_id = confirmation.site.site_id
        event.detection_id = (
            f"{confirmation.site.site_id}:{observation.camera_name}:"
            f"{observation.stamp_sec}.{observation.stamp_nanosec:09d}"
        )
        event.camera_name = observation.camera_name
        event.class_name = observation.class_name
        event.confidence = observation.confidence
        event.severity = DefectEvent.SEVERITY_UNKNOWN
        event.bbox.center.position.x = observation.center_x
        event.bbox.center.position.y = observation.center_y
        event.bbox.center.theta = 0.0
        event.bbox.size_x = observation.size_x
        event.bbox.size_y = observation.size_y
        event.image_width, event.image_height = self.image_sizes[
            observation.camera_name
        ]
        event.has_3d_position = False
        event.localization_method = DefectEvent.LOCALIZATION_NONE
        event.localization_confidence = 0.0
        event.model_name = self.model_name + " [truth-assisted simulation coverage]"
        event.snapshot_uri = ""
        event.has_semantic_location = False
        return event

    def publish_status(self) -> None:
        sites = self.tracker.status()
        confirmed = sum(site["confirmed"] for site in sites)
        message = String()
        message.data = json.dumps(
            {
                "truth_assisted": True,
                "confirmed": confirmed,
                "total": len(sites),
                "coverage_percent": round(100.0 * confirmed / len(sites), 1),
                "robot_chainage_m": self.robot_chainage_m,
                "sites": sites,
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.coverage_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = None
    try:
        node = SimulationCoverageEvaluator()
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

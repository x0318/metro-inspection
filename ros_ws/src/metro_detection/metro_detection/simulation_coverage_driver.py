"""Drive one bounded simulation pass and stop at 10/10 or the route end."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from metro_detection.odometry_guard import (
    OdometryGuard,
    OdometryGuardState,
)


class SimulationCoverageDriver(Node):
    """Publish a bounded test velocity without allowing route overrun."""

    def __init__(self) -> None:
        super().__init__("simulation_coverage_driver")
        self.declare_parameter("odometry_topic", "/wheel/odom_raw")
        self.declare_parameter("coverage_topic", "/simulation/defect_coverage")
        self.declare_parameter("command_topic", "/cmd_vel_safe")
        self.declare_parameter("detector_readiness_topic", "/yolo/ready")
        self.declare_parameter("speed_mps", 0.2)
        self.declare_parameter("target_chainage_m", 31.0)
        self.declare_parameter("startup_delay_sec", 2.0)
        self.declare_parameter("timeout_sec", 240.0)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("odometry_timeout_sec", 1.0)
        self.declare_parameter("odometry_max_age_sec", 0.5)
        self.declare_parameter("odometry_future_tolerance_sec", 0.05)
        self.declare_parameter(
            "safety_state_topic", "/simulation/coverage_driver_state"
        )

        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.target_chainage_m = float(
            self.get_parameter("target_chainage_m").value
        )
        self.startup_delay_sec = float(
            self.get_parameter("startup_delay_sec").value
        )
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        command_rate_hz = float(
            self.get_parameter("command_rate_hz").value
        )
        odometry_timeout_sec = float(
            self.get_parameter("odometry_timeout_sec").value
        )
        odometry_max_age_sec = float(
            self.get_parameter("odometry_max_age_sec").value
        )
        odometry_future_tolerance_sec = float(
            self.get_parameter("odometry_future_tolerance_sec").value
        )
        if self.speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive for this coverage pass")
        if command_rate_hz <= 0.0 or self.timeout_sec <= 0.0:
            raise ValueError("command rate and timeout must be positive")

        steady_now = time.monotonic()
        self.odometry_guard = OdometryGuard(
            timeout_sec=odometry_timeout_sec,
            max_age_sec=odometry_max_age_sec,
            future_tolerance_sec=odometry_future_tolerance_sec,
            start_steady_sec=steady_now,
        )
        self.robot_chainage_m = None
        self.confirmed_count = 0
        self.total_count = 0
        self.first_odom_monotonic = None
        self.drive_started_monotonic = None
        self.stop_cycles_remaining = 0
        self.finished = False
        self.detector_ready = False
        self.last_fault_log_reason = None
        self.last_status_payload = None

        self.command_pub = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.coverage_sub = self.create_subscription(
            String,
            str(self.get_parameter("coverage_topic").value),
            self.on_coverage,
            10,
        )
        readiness_qos = QoSProfile(depth=1)
        readiness_qos.reliability = ReliabilityPolicy.RELIABLE
        readiness_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.safety_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("safety_state_topic").value),
            readiness_qos,
        )
        self.readiness_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("detector_readiness_topic").value),
            self.on_detector_readiness,
            readiness_qos,
        )
        self.rearm_service = self.create_service(
            Trigger, "~/rearm", self.on_rearm
        )
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / command_rate_hz,
            self.on_timer,
            clock=self.safety_clock,
        )
        self.publish_state(
            "waiting_for_odometry", "waiting_for_first_odometry", force=True
        )
        self.get_logger().info(
            f"Automatic coverage pass armed: speed={self.speed_mps:.2f} m/s, "
            f"stop x={self.target_chainage_m:.2f} m; odometry timeout="
            f"{odometry_timeout_sec:.2f} s; waiting for YOLO readiness"
        )

    def on_odometry(self, message: Odometry) -> None:
        now = time.monotonic()
        pose = message.pose.pose
        twist = message.twist.twist
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        numeric_values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
            *message.pose.covariance,
            *message.twist.covariance,
        )
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        previous_state = self.odometry_guard.state
        accepted = self.odometry_guard.observe(
            stamp_ns=stamp_ns,
            ros_now_ns=self.get_clock().now().nanoseconds,
            steady_now_sec=now,
            numeric_values=numeric_values,
            orientation_xyzw=orientation,
        )
        if not accepted or self.odometry_guard.fault_latched:
            self.handle_odometry_fault()
            return

        self.robot_chainage_m = float(pose.position.x)
        if (
            previous_state != OdometryGuardState.MONITORING
            and self.odometry_guard.state == OdometryGuardState.MONITORING
        ):
            self.first_odom_monotonic = now
            self.drive_started_monotonic = None
            self.last_fault_log_reason = None
            self.get_logger().info(
                "Fresh odometry accepted; startup delay restarted"
            )

    def on_coverage(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            self.confirmed_count = int(status["confirmed"])
            self.total_count = int(status["total"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid coverage status: {exc}")

    def on_detector_readiness(self, message: Bool) -> None:
        was_ready = self.detector_ready
        self.detector_ready = bool(message.data)
        if self.detector_ready and not was_ready:
            self.get_logger().info("YOLO readiness received; drive can start")

    def on_rearm(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self.finished:
            response.success = False
            response.message = "coverage_driver_is_terminally_stopped"
            return response

        success, reason = self.odometry_guard.rearm(
            steady_now_sec=time.monotonic(),
            ros_now_ns=self.get_clock().now().nanoseconds,
        )
        response.success = success
        response.message = reason
        self.publish_stop()
        if success:
            self.first_odom_monotonic = None
            self.drive_started_monotonic = None
            self.last_fault_log_reason = None
            self.publish_state("waiting_for_new_odometry", reason, force=True)
            self.get_logger().warning(
                "Odometry fault rearmed; a newer valid message is required "
                "before motion can resume"
            )
        else:
            self.get_logger().warning(f"Odometry rearm rejected: {reason}")
        return response

    def publish_stop(self) -> None:
        self.command_pub.publish(Twist())

    def publish_state(
        self, state: str, reason: str, *, force: bool = False
    ) -> None:
        payload = json.dumps(
            {
                "state": state,
                "reason": reason,
                "motion_authorized": state == "driving",
                "odometry_guard_state": self.odometry_guard.state.value,
            },
            sort_keys=True,
        )
        if not force and payload == self.last_status_payload:
            return
        message = String()
        message.data = payload
        self.safety_state_pub.publish(message)
        self.last_status_payload = payload

    def handle_odometry_fault(self) -> None:
        self.publish_stop()
        reason = self.odometry_guard.reason
        self.first_odom_monotonic = None
        self.drive_started_monotonic = None
        self.publish_state("fault_stop", reason)
        if reason != self.last_fault_log_reason:
            self.get_logger().error(
                f"Odometry safety fault latched: {reason}; forcing stop. "
                "Restore odometry, then call ~/rearm"
            )
            self.last_fault_log_reason = reason

    def begin_stop(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.stop_cycles_remaining = 10
        self.publish_stop()
        self.publish_state("terminal_stop", reason, force=True)
        self.get_logger().info(reason + "; publishing stop command")

    def on_timer(self) -> None:
        if self.finished:
            if self.stop_cycles_remaining > 0:
                self.publish_stop()
                self.stop_cycles_remaining -= 1
                return
            self.timer.cancel()
            self.get_logger().info("Coverage driver is stopped and inactive")
            return

        now = time.monotonic()
        if not self.odometry_guard.check_freshness(now):
            if self.odometry_guard.fault_latched:
                self.handle_odometry_fault()
            else:
                self.publish_stop()
                self.publish_state(
                    self.odometry_guard.state.value,
                    self.odometry_guard.reason,
                )
            return
        if self.robot_chainage_m is None or self.first_odom_monotonic is None:
            self.publish_stop()
            self.publish_state(
                "waiting_for_odometry", "waiting_for_fresh_odometry"
            )
            return
        if not self.detector_ready:
            self.publish_stop()
            self.publish_state(
                "waiting_for_detector", "detector_readiness_not_received"
            )
            return
        if now - self.first_odom_monotonic < self.startup_delay_sec:
            self.publish_stop()
            self.publish_state("startup_delay", "startup_delay_active")
            return
        if self.total_count > 0 and self.confirmed_count >= self.total_count:
            self.begin_stop(
                f"Coverage complete ({self.confirmed_count}/{self.total_count})"
            )
            return
        if self.robot_chainage_m >= self.target_chainage_m:
            self.begin_stop(
                f"Route end reached at x={self.robot_chainage_m:.3f} m "
                f"with coverage {self.confirmed_count}/{self.total_count}"
            )
            return
        if self.drive_started_monotonic is None:
            self.drive_started_monotonic = now
            self.get_logger().info(
                f"Coverage pass started at x={self.robot_chainage_m:.3f} m"
            )
        elif now - self.drive_started_monotonic >= self.timeout_sec:
            self.begin_stop(
                f"Coverage pass timed out with {self.confirmed_count}/"
                f"{self.total_count} confirmed"
            )
            return

        command = Twist()
        command.linear.x = self.speed_mps
        self.command_pub.publish(command)
        self.publish_state("driving", "automatic_coverage_pass_active")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SimulationCoverageDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

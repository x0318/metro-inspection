"""Drive one bounded simulation pass and stop at 10/10 or the route end."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String


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
        if self.speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive for this coverage pass")
        if command_rate_hz <= 0.0 or self.timeout_sec <= 0.0:
            raise ValueError("command rate and timeout must be positive")

        self.robot_chainage_m = None
        self.confirmed_count = 0
        self.total_count = 0
        self.first_odom_monotonic = None
        self.drive_started_monotonic = None
        self.stop_cycles_remaining = 0
        self.finished = False
        self.detector_ready = False

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
        self.readiness_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("detector_readiness_topic").value),
            self.on_detector_readiness,
            readiness_qos,
        )
        self.timer = self.create_timer(1.0 / command_rate_hz, self.on_timer)
        self.get_logger().info(
            f"Automatic coverage pass armed: speed={self.speed_mps:.2f} m/s, "
            f"stop x={self.target_chainage_m:.2f} m; waiting for YOLO readiness"
        )

    def on_odometry(self, message: Odometry) -> None:
        self.robot_chainage_m = float(message.pose.pose.position.x)
        if self.first_odom_monotonic is None:
            self.first_odom_monotonic = time.monotonic()

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

    def begin_stop(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.stop_cycles_remaining = 10
        self.get_logger().info(reason + "; publishing stop command")

    def on_timer(self) -> None:
        if self.finished:
            if self.stop_cycles_remaining > 0:
                self.command_pub.publish(Twist())
                self.stop_cycles_remaining -= 1
                return
            self.timer.cancel()
            self.get_logger().info("Coverage driver is stopped and inactive")
            return

        if self.robot_chainage_m is None or self.first_odom_monotonic is None:
            return
        if not self.detector_ready:
            self.command_pub.publish(Twist())
            return
        now = time.monotonic()
        if now - self.first_odom_monotonic < self.startup_delay_sec:
            self.command_pub.publish(Twist())
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

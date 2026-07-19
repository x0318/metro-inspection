#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class TunnelObstacleGuard(Node):
    """Fail-safe longitudinal command gate for a rail-constrained patrol robot."""

    def __init__(self) -> None:
        super().__init__("tunnel_obstacle_guard")

        self.declare_parameter("stop_distance", 2.0)
        self.declare_parameter("resume_distance", 2.5)
        self.declare_parameter("front_half_angle", 0.35)
        self.declare_parameter("report_after", 10.0)
        self.declare_parameter("scan_timeout", 2.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_forward_speed", 0.35)

        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.resume_distance = float(self.get_parameter("resume_distance").value)
        self.front_half_angle = float(self.get_parameter("front_half_angle").value)
        self.report_after = float(self.get_parameter("report_after").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)

        if self.resume_distance <= self.stop_distance:
            raise ValueError("resume_distance must be greater than stop_distance")

        self.requested_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_scan_time = 0.0
        self.front_distance = math.inf
        self.blocked = True
        self.blocked_since = time.monotonic()
        self.report_sent = False
        self.block_reason = "waiting_for_scan"

        self.safe_cmd_pub = self.create_publisher(Twist, "/cmd_vel_safe", 1)
        self.blocked_pub = self.create_publisher(Bool, "/navigation/obstacle_blocked", 1)
        self.error_pub = self.create_publisher(String, "/navigation/obstacle_error", 1)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, 10)
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_timer(0.05, self._control_tick)

        self.get_logger().info(
            "Tunnel guard active: straight-only, stop at %.2f m, report after %.1f s"
            % (self.stop_distance, self.report_after)
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self.requested_cmd = msg
        self.last_cmd_time = time.monotonic()

    def _scan_callback(self, msg: LaserScan) -> None:
        nearest = math.inf
        angle = msg.angle_min
        for distance in msg.ranges:
            normalized_angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(normalized_angle) <= self.front_half_angle:
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    nearest = min(nearest, distance)
            angle += msg.angle_increment

        self.front_distance = nearest
        self.last_scan_time = time.monotonic()

        if not self.blocked and nearest <= self.stop_distance:
            self._set_blocked(True, "front_obstacle")
        elif self.blocked and self.block_reason == "front_obstacle":
            if nearest >= self.resume_distance:
                self._set_blocked(False, "clear")
        elif self.blocked and self.block_reason in ("waiting_for_scan", "scan_stale"):
            if nearest > self.resume_distance:
                self._set_blocked(False, "clear")
            elif nearest <= self.stop_distance:
                self._set_blocked(True, "front_obstacle")

    def _set_blocked(self, blocked: bool, reason: str) -> None:
        state_changed = blocked != self.blocked or reason != self.block_reason
        self.blocked = blocked
        self.block_reason = reason
        if blocked:
            if state_changed:
                self.blocked_since = time.monotonic()
                self.report_sent = False
                self.get_logger().warn(
                    f"Motion blocked: {reason}, front_distance={self.front_distance:.3f} m"
                )
        else:
            if state_changed:
                self.report_sent = False
                self.get_logger().info("Obstacle cleared; straight motion permitted")

        if state_changed:
            state = Bool()
            state.data = blocked
            self.blocked_pub.publish(state)

    def _control_tick(self) -> None:
        now = time.monotonic()

        if self.last_scan_time == 0.0:
            self._set_blocked(True, "waiting_for_scan")
        elif (
            now - self.last_scan_time > self.scan_timeout
            and self.block_reason != "front_obstacle"
        ):
            self._set_blocked(True, "scan_stale")

        output = Twist()
        command_is_fresh = now - self.last_cmd_time <= self.command_timeout
        if not self.blocked and command_is_fresh:
            # Rail-constrained motion: forward only, with no steering or backup.
            output.linear.x = min(
                self.max_forward_speed, max(0.0, self.requested_cmd.linear.x)
            )

        self.safe_cmd_pub.publish(output)

        if self.blocked and not self.report_sent:
            blocked_duration = now - self.blocked_since
            if blocked_duration >= self.report_after:
                report = String()
                report.data = (
                    "NAVIGATION_BLOCKED: reason=%s duration=%.1fs front_distance=%.3fm"
                    % (self.block_reason, blocked_duration, self.front_distance)
                )
                self.error_pub.publish(report)
                self.get_logger().error(report.data)
                self.report_sent = True


def main() -> None:
    rclpy.init()
    node = TunnelObstacleGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            zero = Twist()
            for _ in range(3):
                node.safe_cmd_pub.publish(zero)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

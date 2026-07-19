#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("output_topic", "/cmd_vel_drive")
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("output_rate", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.timeout = float(self.get_parameter("timeout").value)
        output_rate = float(self.get_parameter("output_rate").value)
        if self.timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")
        if output_rate <= 0.0:
            raise ValueError("output_rate must be greater than zero")

        self.last_command = Twist()
        self.last_command_time = time.monotonic()
        self.have_command = False
        self.timed_out = True

        self.command_sub = self.create_subscription(
            Twist, input_topic, self._command_callback, 10
        )
        self.command_pub = self.create_publisher(Twist, output_topic, 10)
        self.timer = self.create_timer(1.0 / output_rate, self._publish_command)

        self.get_logger().info(
            "Drive watchdog active: %s -> %s, timeout %.2f s"
            % (input_topic, output_topic, self.timeout)
        )

    def _command_callback(self, msg: Twist) -> None:
        self.last_command = msg
        self.last_command_time = time.monotonic()
        self.have_command = True
        if self.timed_out:
            self.get_logger().info("Safe command stream restored")
            self.timed_out = False

    def _publish_command(self) -> None:
        command_age = time.monotonic() - self.last_command_time
        if self.have_command and command_age <= self.timeout:
            self.command_pub.publish(self.last_command)
            return

        self.command_pub.publish(Twist())
        if not self.timed_out:
            self.get_logger().error(
                "Safe command stream timed out after %.2f s; forcing stop"
                % command_age
            )
            self.timed_out = True


def main() -> None:
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

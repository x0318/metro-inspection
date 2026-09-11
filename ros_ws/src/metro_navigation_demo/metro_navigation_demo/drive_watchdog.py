#!/usr/bin/env python3

import time
import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("output_topic", "/cmd_vel_drive")
        self.declare_parameter("estop_topic", "/safety/estop_active")
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("output_rate", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
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
        self.estop_active = False

        self.command_sub = self.create_subscription(
            Twist, input_topic, self._command_callback, 10
        )
        self.command_pub = self.create_publisher(Twist, output_topic, 10)
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.estop_sub = self.create_subscription(
            Bool, estop_topic, self._estop_callback, safety_qos
        )
        self.timer = self.create_timer(1.0 / output_rate, self._publish_command)

        self.get_logger().info(
            "Drive watchdog active: %s -> %s, e-stop %s, timeout %.2f s"
            % (input_topic, output_topic, estop_topic, self.timeout)
        )

    def _command_callback(self, msg: Twist) -> None:
        if self.estop_active:
            return
        self.last_command = msg
        self.last_command_time = time.monotonic()
        self.have_command = True
        if self.timed_out:
            self.get_logger().info("Safe command stream restored")
            self.timed_out = False

    def _estop_callback(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active == self.estop_active:
            return
        self.estop_active = active
        self.last_command = Twist()
        self.have_command = False
        self.timed_out = True
        if active:
            self.get_logger().error("Emergency stop active; forcing continuous zero velocity")
        else:
            self.get_logger().info("Emergency stop reset; waiting for a fresh command")

    def _publish_command(self) -> None:
        if self.estop_active:
            self.command_pub.publish(Twist())
            return
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
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = CmdVelWatchdog()
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

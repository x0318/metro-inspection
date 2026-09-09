#!/usr/bin/env python3

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from cmd_vel_watchdog_core import CommandWatchdog, WatchdogState


class CmdVelWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("output_topic", "/cmd_vel_drive")
        self.declare_parameter("estop_topic", "/safety/estop_active")
        self.declare_parameter(
            "status_topic", "/safety/cmd_vel_watchdog_state"
        )
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("output_rate", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.timeout = float(self.get_parameter("timeout").value)
        output_rate = float(self.get_parameter("output_rate").value)
        if self.timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")
        if output_rate <= 0.0:
            raise ValueError("output_rate must be greater than zero")

        self.safety = CommandWatchdog(self.timeout)
        self.last_command = Twist()
        self.last_status_payload = None

        self.command_pub = self.create_publisher(Twist, output_topic, 10)
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, status_topic, safety_qos
        )
        self.command_sub = self.create_subscription(
            Twist, input_topic, self._command_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, estop_topic, self._estop_callback, safety_qos
        )
        self.rearm_service = self.create_service(
            Trigger, "~/rearm", self._rearm_callback
        )
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / output_rate,
            self._publish_command,
            clock=self.safety_clock,
        )
        self._publish_state(force=True)

        self.get_logger().info(
            "Drive watchdog active: %s -> %s, e-stop %s, timeout %.2f s"
            % (input_topic, output_topic, estop_topic, self.timeout)
        )

    def _command_callback(self, msg: Twist) -> None:
        previous_state = self.safety.state
        if not self.safety.receive_command(time.monotonic()):
            return
        self.last_command = msg
        if previous_state != WatchdogState.ACTIVE:
            self.get_logger().info("Safe command stream restored")
        self._publish_state()

    def _estop_callback(self, msg: Bool) -> None:
        active = bool(msg.data)
        if not self.safety.set_estop(active):
            return

        self.last_command = Twist()
        self.command_pub.publish(Twist())
        if active:
            self.get_logger().error(
                "Emergency stop active; forcing continuous zero velocity"
            )
        else:
            self.get_logger().warning(
                "Emergency stop reset; pre-stop command discarded. "
                "Call ~/rearm to authorize motion again"
            )
        self._publish_state(force=True)

    def _rearm_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        success, reason = self.safety.rearm()
        response.success = success
        response.message = reason
        self.last_command = Twist()
        self.command_pub.publish(Twist())
        if success:
            self.get_logger().warning(
                "Drive watchdog rearmed; waiting for a command received "
                "after rearm"
            )
        else:
            self.get_logger().warning(f"Drive watchdog rearm rejected: {reason}")
        self._publish_state(force=True)
        return response

    def _publish_command(self) -> None:
        previous_state = self.safety.state
        if self.safety.may_forward(time.monotonic()):
            self.command_pub.publish(self.last_command)
            self._publish_state()
            return

        self.last_command = Twist()
        self.command_pub.publish(Twist())
        if (
            self.safety.state == WatchdogState.COMMAND_TIMEOUT
            and previous_state != WatchdogState.COMMAND_TIMEOUT
        ):
            self.get_logger().error(
                "Safe command stream timed out after %.2f s; forcing stop "
                "and discarding the command" % self.timeout
            )
        self._publish_state()

    def _publish_state(self, *, force: bool = False) -> None:
        payload = json.dumps(
            {
                "state": self.safety.state.value,
                "estop_active": self.safety.estop_active,
                "rearm_required": self.safety.rearm_required,
                "motion_authorized": (
                    self.safety.state == WatchdogState.ACTIVE
                ),
            },
            sort_keys=True,
        )
        if not force and payload == self.last_status_payload:
            return
        message = String()
        message.data = payload
        self.status_pub.publish(message)
        self.last_status_payload = payload


def main() -> None:
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

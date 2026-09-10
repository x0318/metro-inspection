#!/usr/bin/env python3

import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from cmd_vel_watchdog_core import CommandWatchdog, InputFreshness, WatchdogState


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
        self.declare_parameter("required_sensor_topic", "")
        self.declare_parameter("required_sensor_timeout", 1.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.timeout = float(self.get_parameter("timeout").value)
        output_rate = float(self.get_parameter("output_rate").value)
        self.required_sensor_topic = str(
            self.get_parameter("required_sensor_topic").value
        ).strip()
        required_sensor_timeout = float(
            self.get_parameter("required_sensor_timeout").value
        )
        if self.timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")
        if output_rate <= 0.0:
            raise ValueError("output_rate must be greater than zero")
        if required_sensor_timeout <= 0.0:
            raise ValueError("required_sensor_timeout must be greater than zero")

        self.safety = CommandWatchdog(self.timeout)
        self.sensor_freshness = (
            InputFreshness(required_sensor_timeout)
            if self.required_sensor_topic
            else None
        )
        self.sensor_was_fresh = self.sensor_freshness is None
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
        self.required_sensor_sub = None
        if self.required_sensor_topic:
            self.required_sensor_sub = self.create_subscription(
                PointCloud2,
                self.required_sensor_topic,
                self._required_sensor_callback,
                qos_profile_sensor_data,
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
            "Drive watchdog active: %s -> %s, e-stop %s, timeout %.2f s, "
            "required sensor %s"
            % (
                input_topic,
                output_topic,
                estop_topic,
                self.timeout,
                self.required_sensor_topic or "disabled",
            )
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

    def _required_sensor_callback(self, msg: PointCloud2) -> None:
        del msg
        if self.sensor_freshness is not None:
            self.sensor_freshness.observe(time.monotonic())

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
        now = time.monotonic()
        sensor_is_fresh = self._required_sensor_is_fresh(now)
        if sensor_is_fresh != self.sensor_was_fresh:
            self.sensor_was_fresh = sensor_is_fresh
            if sensor_is_fresh:
                self.get_logger().info("Required drive sensor stream restored")
            else:
                self.get_logger().error(
                    "Required drive sensor stream is missing or stale; forcing stop"
                )
        if sensor_is_fresh and self.safety.may_forward(now):
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

    def _required_sensor_is_fresh(self, now: float) -> bool:
        if self.sensor_freshness is None:
            return True
        return self.sensor_freshness.is_fresh(now)

    def _publish_state(self, *, force: bool = False) -> None:
        sensor_is_fresh = self._required_sensor_is_fresh(time.monotonic())
        state = (
            self.safety.state.value
            if sensor_is_fresh
            else "required_sensor_stale"
        )
        payload = json.dumps(
            {
                "state": state,
                "estop_active": self.safety.estop_active,
                "rearm_required": self.safety.rearm_required,
                "required_sensor_topic": self.required_sensor_topic,
                "required_sensor_fresh": sensor_is_fresh,
                "motion_authorized": (
                    sensor_is_fresh
                    and self.safety.state == WatchdogState.ACTIVE
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
    # Finish the executor and publish the final stop before destroying the ROS
    # context. The default signal handler can shut it down during spin_once.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = None
    try:
        node = CmdVelWatchdog()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.command_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

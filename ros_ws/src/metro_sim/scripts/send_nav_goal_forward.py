#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger


class RelativeNavGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("relative_nav_goal_client")
        self.odom = None
        self.last_feedback_log = 0.0
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.nav2_active_client = self.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def _feedback_callback(self, feedback_msg) -> None:
        now = time.monotonic()
        if now - self.last_feedback_log >= 1.0:
            remaining = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f"Distance remaining: {remaining:.3f} m")
            self.last_feedback_log = now

    def wait_for_odom(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def wait_for_nav2(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if not self.nav2_active_client.wait_for_service(timeout_sec=0.5):
                continue
            future = self.nav2_active_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done() and future.result() is not None:
                if future.result().success:
                    return True
            time.sleep(0.2)
        return False

    def navigate_forward(self, distance: float, timeout_sec: float) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/navigate_to_pose action server is unavailable")
            return False

        position = self.odom.pose.pose.position
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.odom.header.frame_id or "odom"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        # The subway track axis is +X. Keep the live Y position so a previous
        # heading error cannot send the goal diagonally into the tunnel wall.
        goal.pose.pose.position.x = position.x + distance
        goal.pose.pose.position.y = position.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            "Sending %.2f m relative goal: (%.3f, %.3f) -> (%.3f, %.3f)"
            % (
                distance,
                position.x,
                position.y,
                goal.pose.pose.position.x,
                goal.pose.pose.position.y,
            )
        )

        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("Timed out sending navigation goal")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            self.get_logger().error("Navigation timed out; requesting cancellation")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return False

        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Navigation finished with action status {wrapped_result.status}"
            )
            return False

        self.get_logger().info("Navigation succeeded")
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a relative Nav2 forward goal")
    parser.add_argument("distance", nargs="?", type=float, default=4.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = RelativeNavGoalClient()
    try:
        if not node.wait_for_odom(timeout_sec=15.0):
            node.get_logger().error("Timed out waiting for /odom")
            return 1
        if not node.wait_for_nav2(timeout_sec=60.0):
            node.get_logger().error("Timed out waiting for Nav2 to become active")
            return 1
        return 0 if node.navigate_forward(args.distance, args.timeout) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

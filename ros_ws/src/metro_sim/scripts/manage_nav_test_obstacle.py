#!/usr/bin/env python3

import argparse
import sys

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


MODEL_NAME = "nav_test_box"
BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="nav_test_box">
    <static>true</static>
    <link name="box_link">
      <collision name="collision">
        <geometry><box><size>0.4 0.4 2.0</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.4 0.4 2.0</size></box></geometry>
        <material>
          <ambient>0.8 0.15 0.05 1</ambient>
          <diffuse>0.9 0.2 0.05 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def call_service(node: Node, client, request, timeout_sec: float = 10.0):
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error(f"Service {client.srv_name} is unavailable")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        node.get_logger().error(f"Service {client.srv_name} timed out")
        return None
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn or delete the Nav2 test obstacle")
    parser.add_argument("command", choices=("spawn", "delete"))
    parser.add_argument("--x", type=float, default=-17.0)
    parser.add_argument("--y", type=float, default=0.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = Node("nav_test_obstacle_manager")
    try:
        if args.command == "spawn":
            client = node.create_client(SpawnEntity, "/spawn_entity")
            request = SpawnEntity.Request()
            request.name = MODEL_NAME
            request.xml = BOX_SDF
            request.robot_namespace = ""
            request.reference_frame = "world"
            request.initial_pose.position.x = args.x
            request.initial_pose.position.y = args.y
            request.initial_pose.position.z = 1.0
            request.initial_pose.orientation.w = 1.0
        else:
            client = node.create_client(DeleteEntity, "/delete_entity")
            request = DeleteEntity.Request()
            request.name = MODEL_NAME

        response = call_service(node, client, request)
        if response is None or not response.success:
            message = "no response" if response is None else response.status_message
            node.get_logger().error(f"{args.command} failed: {message}")
            return 1

        node.get_logger().info(f"{args.command} succeeded: {response.status_message}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

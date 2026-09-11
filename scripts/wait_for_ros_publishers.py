#!/usr/bin/env python3
"""Wait until every requested ROS 2 topic has at least one publisher."""

import argparse
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wait for ROS 2 topic publishers without subscribing."
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=90.0,
        help="Maximum wall-clock wait in seconds (default: 90).",
    )
    parser.add_argument("topics", nargs="+", help="Absolute ROS 2 topic names.")
    parser.add_argument(
        "--images",
        action="store_true",
        help="Require an actual nonempty Image on each topic.",
    )
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if any(not topic.startswith("/") for topic in args.topics):
        parser.error("every topic must be an absolute name beginning with '/'")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("wait_for_yolo_image_publishers")
    pending = set(args.topics)
    deadline = time.monotonic() + args.timeout
    subscriptions = []
    if args.images:

        def receive(topic, message):
            if (
                topic in pending
                and message.width > 0
                and message.height > 0
                and len(message.data)
            ):
                pending.remove(topic)
                print(f"[ready] image {topic}", flush=True)

        for topic in args.topics:
            subscriptions.append(
                node.create_subscription(
                    Image,
                    topic,
                    lambda message, topic=topic: receive(topic, message),
                    qos_profile_sensor_data,
                )
            )

    try:
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            ready = (
                set()
                if args.images
                else {topic for topic in pending if node.count_publishers(topic) > 0}
            )
            for topic in sorted(ready):
                print(f"[ready] publisher {topic}", flush=True)
            pending.difference_update(ready)
            if pending:
                time.sleep(0.3)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if pending:
        missing = ", ".join(sorted(pending))
        resource = "image frames" if args.images else "publishers"
        print(
            f"Timed out after {args.timeout:g}s waiting for {resource}: {missing}",
            flush=True,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

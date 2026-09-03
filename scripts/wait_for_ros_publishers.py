#!/usr/bin/env python3
"""Wait until every requested ROS 2 topic has at least one publisher."""

import argparse
import time

import rclpy


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

    try:
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            ready = {
                topic for topic in pending if node.count_publishers(topic) > 0
            }
            for topic in sorted(ready):
                print(f"[ready] publisher {topic}", flush=True)
            pending.difference_update(ready)
            if pending:
                time.sleep(0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if pending:
        missing = ", ".join(sorted(pending))
        print(
            f"Timed out after {args.timeout:g}s waiting for publishers: {missing}",
            flush=True,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

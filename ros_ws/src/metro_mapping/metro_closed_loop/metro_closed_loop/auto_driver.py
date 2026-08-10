import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class AutoDriver(Node):
    """Simple damage-aware driver.

    Baseline behaviour: publish a small forward /cmd_vel.
    Added behaviour: once /damage_point_global is available, keep approaching
    until the damage point is only stop_distance_m in front of base_footprint,
    then continuously publish zero velocity so the car stays stopped.
    """

    def __init__(self):
        super().__init__("auto_driver")
        self.declare_parameter("linear_x", 0.08)
        self.declare_parameter("angular_z", 0.0)
        self.declare_parameter("period", 0.1)
        self.declare_parameter("damage_point_topic", "/damage_point_global")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("stop_distance_m", 0.80)
        self.declare_parameter("slow_distance_m", 1.60)
        self.declare_parameter("min_linear_x", 0.025)
        self.declare_parameter("stop_lateral_tolerance_m", 5.0)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.damage_point = None
        self.stopped = False
        self.last_no_tf_log_ns = 0
        self.start_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.damage_sub = self.create_subscription(
            PointStamped,
            self.get_parameter("damage_point_topic").value,
            self.on_damage_point,
            10,
        )
        self.timer = self.create_timer(
            float(self.get_parameter("period").value), self.on_timer
        )
        self.get_logger().info(
            "Damage-aware auto drive started: speed="
            f"{float(self.get_parameter('linear_x').value):.3f} m/s, "
            "stop when damage is within "
            f"{float(self.get_parameter('stop_distance_m').value):.2f} m ahead."
        )

    def on_damage_point(self, msg: PointStamped):
        # Keep the latest localized damage position. In the current pipeline this
        # point is already in odom, but the frame_id is still checked in on_timer.
        self.damage_point = msg

    @staticmethod
    def yaw_from_quaternion(q):
        # Standard yaw extraction from geometry_msgs/Quaternion.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def zero_stop(self):
        self.pub.publish(Twist())

    def get_forward_lateral_distance(self):
        if self.damage_point is None:
            return None

        global_frame = str(self.get_parameter("global_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)

        if self.damage_point.header.frame_id and self.damage_point.header.frame_id != global_frame:
            self.get_logger().warn(
                "Damage point frame is "
                f"{self.damage_point.header.frame_id}, expected {global_frame}; "
                "current auto_driver only handles the global frame directly."
            )
            return None

        try:
            tf = self.tf_buffer.lookup_transform(global_frame, base_frame, Time())
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_no_tf_log_ns > 2_000_000_000:
                self.get_logger().warn(f"Waiting for TF {global_frame}->{base_frame}: {exc}")
                self.last_no_tf_log_ns = now_ns
            return None

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        yaw = self.yaw_from_quaternion(tf.transform.rotation)

        dx = self.damage_point.point.x - robot_x
        dy = self.damage_point.point.y - robot_y
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return forward, lateral

    def on_timer(self):
        if self.stopped:
            self.zero_stop()
            return

        distance = self.get_forward_lateral_distance()
        if distance is not None:
            forward_m, lateral_m = distance
            stop_distance_m = float(self.get_parameter("stop_distance_m").value)
            lateral_tol_m = float(self.get_parameter("stop_lateral_tolerance_m").value)

            if forward_m <= stop_distance_m and abs(lateral_m) <= lateral_tol_m:
                self.stopped = True
                self.zero_stop()
                self.get_logger().info(
                    "Damage reached stop position. Car stopped: "
                    f"forward={forward_m:.2f} m, lateral={lateral_m:.2f} m."
                )
                return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        target_speed = float(self.get_parameter("linear_x").value)
        if distance is not None:
            forward_m, _ = distance
            stop_distance_m = float(self.get_parameter("stop_distance_m").value)
            slow_distance_m = max(
                stop_distance_m, float(self.get_parameter("slow_distance_m").value)
            )
            if stop_distance_m < forward_m < slow_distance_m:
                ratio = (forward_m - stop_distance_m) / (slow_distance_m - stop_distance_m)
                min_speed = float(self.get_parameter("min_linear_x").value)
                target_speed = max(min_speed, target_speed * max(0.0, min(1.0, ratio)))

        msg = Twist()
        msg.linear.x = target_speed
        msg.angular.z = float(self.get_parameter("angular_z").value) * math.sin(
            elapsed * 0.35
        )
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AutoDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

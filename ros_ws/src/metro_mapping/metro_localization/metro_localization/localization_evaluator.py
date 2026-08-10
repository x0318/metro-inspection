import math
import statistics

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker


class LocalizationEvaluator(Node):
    """Compare the estimated global point with the known Gazebo wall position."""

    def __init__(self):
        super().__init__("localization_evaluator")
        self.declare_parameter("estimated_topic", "/damage_point_global")
        self.declare_parameter("truth_point_topic", "/damage_ground_truth")
        self.declare_parameter("error_topic", "/localization/error_m")
        self.declare_parameter("mae_topic", "/localization/mae_m")
        self.declare_parameter("rmse_topic", "/localization/rmse_m")
        self.declare_parameter("median_error_topic", "/localization/median_error_m")
        self.declare_parameter("truth_marker_topic", "/localization/truth_marker")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("truth_xyz", [2.96, -0.18, 0.65])
        self.declare_parameter("log_every_n", 10)

        self.truth_xyz = [
            float(value) for value in self.get_parameter("truth_xyz").value
        ]
        if len(self.truth_xyz) != 3:
            raise ValueError("truth_xyz must contain exactly three values")

        self.error_pub = self.create_publisher(
            Float64, self.get_parameter("error_topic").value, 10
        )
        self.mae_pub = self.create_publisher(
            Float64, self.get_parameter("mae_topic").value, 10
        )
        self.rmse_pub = self.create_publisher(
            Float64, self.get_parameter("rmse_topic").value, 10
        )
        self.median_error_pub = self.create_publisher(
            Float64, self.get_parameter("median_error_topic").value, 10
        )
        self.truth_point_pub = self.create_publisher(
            PointStamped, self.get_parameter("truth_point_topic").value, 10
        )
        self.truth_marker_pub = self.create_publisher(
            Marker, self.get_parameter("truth_marker_topic").value, 10
        )
        self.estimated_sub = self.create_subscription(
            PointStamped,
            self.get_parameter("estimated_topic").value,
            self.on_estimated_point,
            10,
        )
        self.marker_timer = self.create_timer(0.5, self.publish_truth)
        self.sample_count = 0
        self.error_sum = 0.0
        self.squared_error_sum = 0.0
        self.errors = []
        self.get_logger().info(
            "Localization evaluator truth: "
            f"({self.truth_xyz[0]:.3f}, {self.truth_xyz[1]:.3f}, {self.truth_xyz[2]:.3f}) m"
        )

    def make_truth_point(self):
        point = PointStamped()
        point.header.frame_id = str(self.get_parameter("global_frame").value)
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = self.truth_xyz[0]
        point.point.y = self.truth_xyz[1]
        point.point.z = self.truth_xyz[2]
        return point

    def publish_truth(self):
        point = self.make_truth_point()
        self.truth_point_pub.publish(point)

        marker = Marker()
        marker.header = point.header
        marker.ns = "damage_ground_truth"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r = 0.05
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.truth_marker_pub.publish(marker)

    def on_estimated_point(self, point):
        expected_frame = str(self.get_parameter("global_frame").value)
        if point.header.frame_id != expected_frame:
            self.get_logger().warn(
                f"Ignoring estimate in '{point.header.frame_id}', expected '{expected_frame}'."
            )
            return

        dx = float(point.point.x) - self.truth_xyz[0]
        dy = float(point.point.y) - self.truth_xyz[1]
        dz = float(point.point.z) - self.truth_xyz[2]
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.error_pub.publish(Float64(data=error))

        self.sample_count += 1
        self.error_sum += error
        self.squared_error_sum += error * error
        self.errors.append(error)

        mae = self.error_sum / self.sample_count
        rmse = math.sqrt(self.squared_error_sum / self.sample_count)
        median_error = statistics.median(self.errors)

        self.mae_pub.publish(Float64(data=mae))
        self.rmse_pub.publish(Float64(data=rmse))
        self.median_error_pub.publish(Float64(data=median_error))

        log_every_n = max(1, int(self.get_parameter("log_every_n").value))
        if self.sample_count == 1 or self.sample_count % log_every_n == 0:
            self.get_logger().info(
                f"3D error={error:.4f} m, MAE={mae:.4f} m, "
                f"RMSE={rmse:.4f} m, median={median_error:.4f} m "
                f"({self.sample_count} samples)"
            )


def main():
    rclpy.init()
    node = LocalizationEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
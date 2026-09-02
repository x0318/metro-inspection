// Copyright 2026 jo0625
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "metro_pointcloud_mapping/planar_motion_gate.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace metro_pointcloud_mapping
{

class MotionCloudGateNode : public rclcpp::Node
{
public:
  MotionCloudGateNode()
  : Node("motion_cloud_gate")
  {
    const double translation_threshold_m = declare_positive_parameter(
      "translation_threshold_m", 0.35, 10.0,
      "Planar displacement required before forwarding another SLAM cloud.");
    const double rotation_threshold_rad = declare_positive_parameter(
      "rotation_threshold_rad", 0.10, 3.14159265358979323846,
      "Yaw displacement required before forwarding another SLAM cloud.");
    maximum_odom_age_s_ = declare_positive_parameter(
      "maximum_odom_age_s", 0.25, 10.0,
      "Maximum absolute timestamp difference between a cloud and latest odometry.");
    motion_gate_ = std::make_unique<PlanarMotionGate>(
      translation_threshold_m, rotation_threshold_rad);

    auto cloud_qos = rclcpp::SensorDataQoS().keep_last(5);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_out", rclcpp::SensorDataQoS().keep_last(1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", cloud_qos,
      std::bind(
        &MotionCloudGateNode::cloud_callback, this,
        std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(
        &MotionCloudGateNode::odom_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Motion cloud gate thresholds: translation=%.3f m rotation=%.3f rad "
      "maximum_odom_age=%.3f s",
      translation_threshold_m, rotation_threshold_rad, maximum_odom_age_s_);
  }

private:
  struct OdomSample
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    std::int64_t stamp_ns{0};
  };

  double declare_positive_parameter(
    const std::string & name, double default_value, double maximum_value,
    const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.000001;
    range.to_value = maximum_value;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);
    return declare_parameter<double>(name, default_value, descriptor);
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    const double norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(norm_squared) ||
      norm_squared <= std::numeric_limits<double>::epsilon())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid filtered odometry pose");
      return;
    }

    const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y) /
      norm_squared;
    const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z) /
      norm_squared;
    const double yaw = std::atan2(sin_yaw, cos_yaw);
    const std::int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (!std::isfinite(yaw) || stamp_ns <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring filtered odometry with invalid yaw or timestamp");
      return;
    }

    latest_odom_ = OdomSample{position.x, position.y, yaw, stamp_ns};
    have_odom_ = true;
  }

  void reject_cloud(const char * reason)
  {
    ++rejected_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected SLAM cloud (%s); forwarded=%zu suppressed=%zu rejected=%zu",
      reason, forwarded_count_, suppressed_count_, rejected_count_);
  }

  void cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
  {
    ++received_count_;
    const std::int64_t cloud_stamp_ns =
      rclcpp::Time(message->header.stamp).nanoseconds();
    if (!have_odom_) {
      reject_cloud("filtered odometry unavailable");
      return;
    }
    if (cloud_stamp_ns <= 0) {
      reject_cloud("invalid cloud timestamp");
      return;
    }

    const double odom_age_s = std::abs(
      static_cast<double>(cloud_stamp_ns - latest_odom_.stamp_ns)) / 1.0e9;
    if (odom_age_s > maximum_odom_age_s_) {
      reject_cloud("filtered odometry timestamp is stale");
      return;
    }

    const MotionGateDecision decision = motion_gate_->evaluate(
      latest_odom_.x, latest_odom_.y, latest_odom_.yaw);
    if (decision == MotionGateDecision::kForward) {
      publisher_->publish(std::move(message));
      ++forwarded_count_;
    } else if (decision == MotionGateDecision::kSuppress) {
      ++suppressed_count_;
    } else {
      reject_cloud("non-finite planar pose");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Motion cloud gate received %zu, forwarded %zu, suppressed %zu and "
      "rejected %zu clouds",
      received_count_, forwarded_count_, suppressed_count_, rejected_count_);
  }

  double maximum_odom_age_s_{0.25};
  bool have_odom_{false};
  OdomSample latest_odom_;
  std::unique_ptr<PlanarMotionGate> motion_gate_;
  std::size_t received_count_{0U};
  std::size_t forwarded_count_{0U};
  std::size_t suppressed_count_{0U};
  std::size_t rejected_count_{0U};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

}  // namespace metro_pointcloud_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<metro_pointcloud_mapping::MotionCloudGateNode>());
  rclcpp::shutdown();
  return 0;
}

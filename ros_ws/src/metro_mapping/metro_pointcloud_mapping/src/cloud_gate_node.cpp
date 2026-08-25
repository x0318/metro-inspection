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

#include <cinttypes>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace metro_pointcloud_mapping
{

class CloudGateNode : public rclcpp::Node
{
public:
  CloudGateNode()
  : Node("cloud_gate")
  {
    auto minimum_descriptor = descriptor(
      "Minimum declared point count required for scan matching.");
    rcl_interfaces::msg::IntegerRange minimum_range;
    minimum_range.from_value = 1;
    minimum_range.to_value = 100000000;
    minimum_range.step = 1;
    minimum_descriptor.integer_range.push_back(minimum_range);
    minimum_points_ = static_cast<std::uint64_t>(
      declare_parameter<std::int64_t>(
        "minimum_points", 5000, minimum_descriptor));
    reject_out_of_order_ = declare_parameter<bool>(
      "reject_out_of_order", true,
      descriptor("Reject clouds whose header stamp is not newer than the last output."));

    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_valid", qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_raw", qos,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr message) {
        cloud_callback(std::move(message));
      });

    RCLCPP_INFO(
      get_logger(), "Cloud gate requires at least %" PRIu64 " points per scan",
      minimum_points_);
  }

private:
  static rcl_interfaces::msg::ParameterDescriptor descriptor(
    const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor result;
    result.description = description;
    result.read_only = true;
    return result;
  }

  void reject(const char * reason, std::uint64_t point_count)
  {
    ++rejected_clouds_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected point cloud (%s, points=%" PRIu64 "); accepted=%" PRIu64
      " rejected=%" PRIu64,
      reason, point_count, accepted_clouds_, rejected_clouds_);
  }

  void cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
  {
    const auto point_count =
      static_cast<std::uint64_t>(message->width) * message->height;
    if (point_count < minimum_points_) {
      reject("too few points", point_count);
      return;
    }

    const auto expected_bytes =
      static_cast<std::uint64_t>(message->row_step) * message->height;
    if (message->point_step == 0U || message->row_step == 0U ||
      expected_bytes > message->data.size())
    {
      reject("inconsistent PointCloud2 layout", point_count);
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      reject("missing timestamp", point_count);
      return;
    }
    if (reject_out_of_order_ && last_stamp_nanoseconds_ != kNoStamp &&
      stamp.nanoseconds() <= last_stamp_nanoseconds_)
    {
      reject("out-of-order timestamp", point_count);
      return;
    }

    last_stamp_nanoseconds_ = stamp.nanoseconds();
    ++accepted_clouds_;
    publisher_->publish(std::move(message));
  }

  static constexpr std::int64_t kNoStamp =
    std::numeric_limits<std::int64_t>::min();

  std::uint64_t minimum_points_{5000U};
  bool reject_out_of_order_{true};
  std::int64_t last_stamp_nanoseconds_{kNoStamp};
  std::uint64_t accepted_clouds_{0U};
  std::uint64_t rejected_clouds_{0U};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace metro_pointcloud_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<metro_pointcloud_mapping::CloudGateNode>());
  rclcpp::shutdown();
  return 0;
}

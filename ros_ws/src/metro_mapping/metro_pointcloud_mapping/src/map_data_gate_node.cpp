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

#include <functional>
#include <memory>
#include <string>

#include "metro_pointcloud_mapping/map_graph_change_detector.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/map_data.hpp"

namespace metro_pointcloud_mapping
{

class MapDataGateNode : public rclcpp::Node
{
public:
  MapDataGateNode()
  : Node("map_data_gate")
  {
    const double translation_threshold_m = declare_positive_parameter(
      "translation_threshold_m", 0.01,
      "Cumulative graph translation required before forwarding map data.");
    const double rotation_threshold_rad = declare_positive_parameter(
      "rotation_threshold_rad", 0.005,
      "Cumulative graph rotation required before forwarding map data.");
    detector_ = std::make_unique<MapGraphChangeDetector>(
      translation_threshold_m, rotation_threshold_rad);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    publisher_ = create_publisher<rtabmap_msgs::msg::MapData>("map_data_out", qos);
    subscription_ = create_subscription<rtabmap_msgs::msg::MapData>(
      "map_data_in", qos,
      std::bind(&MapDataGateNode::map_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Map data gate thresholds: translation=%.3f m rotation=%.3f rad",
      translation_threshold_m, rotation_threshold_rad);
  }

private:
  double declare_positive_parameter(
    const std::string & name, double default_value,
    const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.000001;
    range.to_value = 1.0;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);
    return declare_parameter<double>(name, default_value, descriptor);
  }

  void map_data_callback(
    const rtabmap_msgs::msg::MapData::ConstSharedPtr message)
  {
    try {
      if (detector_->accept(message->graph)) {
        publisher_->publish(*message);
        ++forwarded_count_;
      } else {
        ++suppressed_count_;
      }
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping invalid RTAB-Map graph: %s", error.what());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Map data gate forwarded %zu and suppressed %zu messages",
      forwarded_count_, suppressed_count_);
  }

  std::unique_ptr<MapGraphChangeDetector> detector_;
  std::size_t forwarded_count_{0U};
  std::size_t suppressed_count_{0U};
  rclcpp::Publisher<rtabmap_msgs::msg::MapData>::SharedPtr publisher_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr subscription_;
};

}  // namespace metro_pointcloud_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<metro_pointcloud_mapping::MapDataGateNode>());
  rclcpp::shutdown();
  return 0;
}

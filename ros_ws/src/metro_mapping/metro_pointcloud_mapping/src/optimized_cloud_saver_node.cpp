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

#include <pcl/common/point_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "metro_pointcloud_mapping/pcd_writer.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace metro_pointcloud_mapping
{

class OptimizedCloudSaverNode : public rclcpp::Node
{
public:
  OptimizedCloudSaverNode()
  : Node("optimized_cloud_saver")
  {
    pcd_path_ = declare_parameter<std::string>(
      "pcd_path", "/tmp/metro_optimized_cloud.pcd",
      descriptor("Destination for the optimized PCD map."));
    save_on_shutdown_ = declare_parameter<bool>(
      "save_on_shutdown", true,
      descriptor("Save the latest optimized cloud while the node shuts down."));

    auto minimum_descriptor = descriptor(
      "Minimum finite point count required before a map can be saved.");
    rcl_interfaces::msg::IntegerRange minimum_range;
    minimum_range.from_value = 1;
    minimum_range.to_value = 100000000;
    minimum_range.step = 1;
    minimum_descriptor.integer_range.push_back(minimum_range);
    minimum_points_ = static_cast<std::size_t>(
      declare_parameter<std::int64_t>(
        "minimum_points", 100, minimum_descriptor));

    if (pcd_path_.empty()) {
      throw std::invalid_argument("pcd_path must not be empty");
    }

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(1));
    cloud_qos.reliable().transient_local();
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_map", cloud_qos,
      std::bind(
        &OptimizedCloudSaverNode::cloud_callback, this,
        std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "save_map", std::bind(
        &OptimizedCloudSaverNode::save_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Waiting for optimized cloud_map; PCD destination: %s",
      pcd_path_.c_str());
  }

  ~OptimizedCloudSaverNode() override
  {
    if (save_on_shutdown_ && has_cloud()) {
      std::string message;
      if (save_snapshot(message)) {
        RCLCPP_INFO(get_logger(), "Shutdown save complete: %s", message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Shutdown save failed: %s", message.c_str());
      }
    }
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

  bool has_cloud() const
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return !latest_points_.empty();
  }

  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    try {
      pcl::fromROSMsg(*message, cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot decode optimized cloud_map: %s", error.what());
      return;
    }

    std::vector<Point3d> points;
    points.reserve(cloud.points.size());
    for (const auto & point : cloud.points) {
      if (pcl::isFinite(point)) {
        points.push_back(Point3d{point.x, point.y, point.z});
      }
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_points_ = std::move(points);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cached optimized cloud with %zu finite points", latest_points_size());
  }

  std::size_t latest_points_size() const
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return latest_points_.size();
  }

  bool save_snapshot(std::string & message) const
  {
    std::vector<Point3d> points;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      points = latest_points_;
    }
    if (points.size() < minimum_points_) {
      message = "optimized cloud has " + std::to_string(points.size()) +
        " points; at least " + std::to_string(minimum_points_) + " are required";
      return false;
    }

    const std::filesystem::path output_path =
      std::filesystem::absolute(pcd_path_).lexically_normal();
    try {
      if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
      }
    } catch (const std::exception & error) {
      message = std::string("cannot create output directory: ") + error.what();
      return false;
    }

    std::string error;
    if (!write_binary_pcd(output_path.string(), points, error)) {
      message = "cannot write " + output_path.string() + ": " + error;
      return false;
    }
    message = "saved " + std::to_string(points.size()) + " points to " +
      output_path.string();
    return true;
  }

  void save_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->success = save_snapshot(response->message);
    if (response->success) {
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  }

  std::string pcd_path_;
  bool save_on_shutdown_{true};
  std::size_t minimum_points_{100U};

  mutable std::mutex cloud_mutex_;
  std::vector<Point3d> latest_points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

}  // namespace metro_pointcloud_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<metro_pointcloud_mapping::OptimizedCloudSaverNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}

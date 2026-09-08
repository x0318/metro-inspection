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
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cinttypes>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "metro_pointcloud_mapping/pcd_writer.hpp"
#include "metro_pointcloud_mapping/voxel_map.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace metro_pointcloud_mapping
{

class CloudAccumulatorNode : public rclcpp::Node
{
public:
  CloudAccumulatorNode()
  : Node("cloud_accumulator")
  {
    target_frame_ = declare_string(
      "target_frame", "odom",
      "Frame used by the accumulated map.");
    base_frame_ = declare_string(
      "base_frame", "base_footprint",
      "Robot frame used by the self-filter box.");
    voxel_size_m_ =
      declare_double(
      "voxel_size_m", 0.05, 0.01, 1.0,
      "Edge length of each accumulated map voxel.");
    min_range_m_ =
      declare_double(
      "min_range_m", 0.2, 0.0, 100.0,
      "Minimum accepted range from the lidar origin.");
    max_range_m_ =
      declare_double(
      "max_range_m", 30.0, 0.01, 1000.0,
      "Maximum accepted range from the lidar origin.");
    min_z_m_ = declare_double(
      "min_z_m", -0.2, -1000.0, 1000.0,
      "Minimum accepted Z coordinate in target_frame.");
    max_z_m_ = declare_double(
      "max_z_m", 5.0, -1000.0, 1000.0,
      "Maximum accepted Z coordinate in target_frame.");
    transform_timeout_sec_ =
      declare_double(
      "transform_timeout_sec", 0.1, 0.0, 10.0,
      "Maximum wait for a timestamped TF lookup.");
    publish_period_sec_ =
      declare_double(
      "publish_period_sec", 1.0, 0.1, 60.0,
      "Wall-clock period between map publications.");
    max_voxels_ = static_cast<std::size_t>(
      declare_integer(
        "max_voxels", 2000000, 1, 20000000,
        "Maximum number of voxels retained in memory."));
    log_every_n_clouds_ =
      declare_integer(
      "log_every_n_clouds", 50, 1, 1000000,
      "Log map statistics every N input clouds.");
    pcd_path_ = declare_string(
      "pcd_path", "/tmp/metro_accumulated_cloud.pcd",
      "Destination used by save_map.");

    self_filter_enabled_ =
      declare_bool(
      "self_filter.enabled", true,
      "Remove returns inside the robot bounding box.");
    self_min_x_m_ = declare_double(
      "self_filter.min_x_m", -0.9, -10.0, 10.0,
      "Self-filter minimum X in base_frame.");
    self_max_x_m_ = declare_double(
      "self_filter.max_x_m", 0.9, -10.0, 10.0,
      "Self-filter maximum X in base_frame.");
    self_min_y_m_ = declare_double(
      "self_filter.min_y_m", -0.6, -10.0, 10.0,
      "Self-filter minimum Y in base_frame.");
    self_max_y_m_ = declare_double(
      "self_filter.max_y_m", 0.6, -10.0, 10.0,
      "Self-filter maximum Y in base_frame.");
    self_min_z_m_ = declare_double(
      "self_filter.min_z_m", -0.1, -10.0, 10.0,
      "Self-filter minimum Z in base_frame.");
    self_max_z_m_ = declare_double(
      "self_filter.max_z_m", 1.5, -10.0, 10.0,
      "Self-filter maximum Z in base_frame.");

    validate_parameters();
    voxel_map_ = std::make_unique<VoxelMap>(voxel_size_m_, max_voxels_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS(),
      std::bind(
        &CloudAccumulatorNode::cloud_callback, this,
        std::placeholders::_1));

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable().transient_local();
    map_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", map_qos);

    save_service_ = create_service<std_srvs::srv::Trigger>(
      "save_map", std::bind(
        &CloudAccumulatorNode::save_map_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset_map", std::bind(
        &CloudAccumulatorNode::reset_map_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto publish_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(publish_period_sec_));
    publish_timer_ =
      create_wall_timer(publish_period, [this]() {publish_map(false);});
    save_worker_ = std::thread(&CloudAccumulatorNode::save_worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Accumulating cloud_in into %s at %.3f m resolution "
      "(capacity=%zu voxels)",
      target_frame_.c_str(), voxel_size_m_, max_voxels_);
  }

  ~CloudAccumulatorNode() override
  {
    {
      std::lock_guard<std::mutex> lock(save_mutex_);
      stop_save_worker_ = true;
    }
    save_condition_.notify_one();
    if (save_worker_.joinable()) {
      save_worker_.join();
    }
  }

private:
  struct SaveRequest
  {
    std::vector<Point3d> points;
    std::string path;
  };

  static rcl_interfaces::msg::ParameterDescriptor
  descriptor(const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor result;
    result.description = description;
    result.read_only = true;
    return result;
  }

  double declare_double(
    const std::string & name, double default_value,
    double minimum, double maximum,
    const std::string & description)
  {
    auto parameter_descriptor = descriptor(description);
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    range.step = 0.0;
    parameter_descriptor.floating_point_range.push_back(range);
    return declare_parameter<double>(name, default_value, parameter_descriptor);
  }

  std::int64_t declare_integer(
    const std::string & name,
    std::int64_t default_value, std::int64_t minimum,
    std::int64_t maximum,
    const std::string & description)
  {
    auto parameter_descriptor = descriptor(description);
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    range.step = 1;
    parameter_descriptor.integer_range.push_back(range);
    return declare_parameter<std::int64_t>(
      name, default_value,
      parameter_descriptor);
  }

  std::string declare_string(
    const std::string & name,
    const std::string & default_value,
    const std::string & description)
  {
    return declare_parameter<std::string>(
      name, default_value,
      descriptor(description));
  }

  bool declare_bool(
    const std::string & name, bool default_value,
    const std::string & description)
  {
    return declare_parameter<bool>(
      name, default_value,
      descriptor(description));
  }

  void validate_parameters() const
  {
    if (target_frame_.empty() || base_frame_.empty()) {
      throw std::invalid_argument(
              "target_frame and base_frame must not be empty");
    }
    if (max_range_m_ <= min_range_m_) {
      throw std::invalid_argument(
              "max_range_m must be greater than min_range_m");
    }
    if (max_z_m_ <= min_z_m_) {
      throw std::invalid_argument("max_z_m must be greater than min_z_m");
    }
    if (self_max_x_m_ <= self_min_x_m_ || self_max_y_m_ <= self_min_y_m_ ||
      self_max_z_m_ <= self_min_z_m_)
    {
      throw std::invalid_argument(
              "self-filter maximum bounds must exceed minimum bounds");
    }
    if (pcd_path_.empty()) {
      throw std::invalid_argument("pcd_path must not be empty");
    }
  }

  geometry_msgs::msg::TransformStamped
  lookup_transform(
    const std::string & target_frame,
    const sensor_msgs::msg::PointCloud2 & message) const
  {
    return tf_buffer_->lookupTransform(
      target_frame, message.header.frame_id,
      rclcpp::Time(message.header.stamp),
      rclcpp::Duration::from_seconds(transform_timeout_sec_));
  }

  bool is_inside_self_filter(const pcl::PointXYZ & point) const
  {
    return point.x >= self_min_x_m_ && point.x <= self_max_x_m_ &&
           point.y >= self_min_y_m_ && point.y <= self_max_y_m_ &&
           point.z >= self_min_z_m_ && point.z <= self_max_z_m_;
  }

  void
  cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring point cloud with an empty frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped target_transform;
    geometry_msgs::msg::TransformStamped base_transform;
    try {
      target_transform = lookup_transform(target_frame_, *message);
      if (self_filter_enabled_) {
        base_transform = target_frame_ == base_frame_ ?
          target_transform :
          lookup_transform(base_frame_, *message);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping cloud because TF is unavailable: %s",
        error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> source_cloud;
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    pcl::PointCloud<pcl::PointXYZ> base_cloud;
    try {
      pcl::fromROSMsg(*message, source_cloud);
      const Eigen::Matrix4f target_matrix =
        tf2::transformToEigen(target_transform).matrix().cast<float>();
      pcl::transformPointCloud(source_cloud, target_cloud, target_matrix);
      if (self_filter_enabled_) {
        const Eigen::Matrix4f base_matrix =
          tf2::transformToEigen(base_transform).matrix().cast<float>();
        pcl::transformPointCloud(source_cloud, base_cloud, base_matrix);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to convert point cloud: %s", error.what());
      return;
    }

    if (source_cloud.points.size() != target_cloud.points.size() ||
      (self_filter_enabled_ &&
      source_cloud.points.size() != base_cloud.points.size()))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Point cloud transform changed the number of points");
      return;
    }

    const double min_range_squared = min_range_m_ * min_range_m_;
    const double max_range_squared = max_range_m_ * max_range_m_;
    std::size_t accepted_points = 0U;
    std::size_t capacity_drops = 0U;

    for (std::size_t index = 0U; index < source_cloud.points.size(); ++index) {
      const auto & source_point = source_cloud.points[index];
      const auto & target_point = target_cloud.points[index];
      if (!pcl::isFinite(source_point) || !pcl::isFinite(target_point) ||
        (self_filter_enabled_ && !pcl::isFinite(base_cloud.points[index])))
      {
        continue;
      }

      const double range_squared =
        static_cast<double>(source_point.x) * source_point.x +
        static_cast<double>(source_point.y) * source_point.y +
        static_cast<double>(source_point.z) * source_point.z;
      if (range_squared < min_range_squared ||
        range_squared > max_range_squared)
      {
        continue;
      }
      if (self_filter_enabled_ &&
        is_inside_self_filter(base_cloud.points[index]))
      {
        continue;
      }
      if (target_point.z < min_z_m_ || target_point.z > max_z_m_) {
        continue;
      }

      const InsertResult result =
        voxel_map_->add_point(
        Point3d{static_cast<double>(target_point.x),
          static_cast<double>(target_point.y),
          static_cast<double>(target_point.z)});
      if (result == InsertResult::kCapacityReached) {
        ++capacity_drops;
      } else if (result != InsertResult::kRejectedNonFinite) {
        ++accepted_points;
      }
    }

    ++cloud_count_;
    input_point_count_ += source_cloud.points.size();
    accepted_point_count_ += accepted_points;
    capacity_drop_count_ += capacity_drops;
    has_received_cloud_ = true;

    if (capacity_drops > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Voxel capacity reached; dropped %zu new voxels in this cloud",
        capacity_drops);
    }
    if (cloud_count_ % static_cast<std::uint64_t>(log_every_n_clouds_) == 0U) {
      RCLCPP_INFO(
        get_logger(),
        "clouds=%" PRIu64 " input_points=%" PRIu64
        " accepted_points=%" PRIu64 " voxels=%zu drops=%" PRIu64,
        cloud_count_, input_point_count_, accepted_point_count_,
        voxel_map_->size(), capacity_drop_count_);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot_map() const
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const auto map_points = voxel_map_->points();
    cloud->points.reserve(map_points.size());
    for (const auto & point : map_points) {
      cloud->points.emplace_back(
        static_cast<float>(point.x),
        static_cast<float>(point.y),
        static_cast<float>(point.z));
    }
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
  }

  void publish_map(bool publish_empty)
  {
    if (!has_received_cloud_ && !publish_empty) {
      return;
    }
    const auto cloud = snapshot_map();
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.frame_id = target_frame_;
    message.header.stamp = now();
    map_publisher_->publish(message);
  }

  void save_map_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    {
      std::lock_guard<std::mutex> lock(save_mutex_);
      if (pending_save_ || save_active_) {
        response->success = false;
        response->message = "A map save is already pending or active";
        return;
      }
    }
    if (voxel_map_->size() == 0U) {
      response->success = false;
      response->message = "The accumulated map is empty";
      return;
    }

    auto request = std::make_unique<SaveRequest>();
    request->points = voxel_map_->points();
    request->path =
      std::filesystem::absolute(pcd_path_).lexically_normal().string();
    const std::size_t point_count = request->points.size();
    const std::string output_path = request->path;
    {
      std::lock_guard<std::mutex> lock(save_mutex_);
      pending_save_ = std::move(request);
    }
    save_condition_.notify_one();

    response->success = true;
    response->message =
      "Queued " + std::to_string(point_count) + " points for " + output_path;
  }

  void
  reset_map_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const std::size_t previous_size = voxel_map_->size();
    voxel_map_->clear();
    cloud_count_ = 0U;
    input_point_count_ = 0U;
    accepted_point_count_ = 0U;
    capacity_drop_count_ = 0U;
    has_received_cloud_ = false;
    publish_map(true);

    response->success = true;
    response->message = "Cleared " + std::to_string(previous_size) + " voxels";
  }

  void save_worker_loop()
  {
    while (true) {
      std::unique_ptr<SaveRequest> request;
      {
        std::unique_lock<std::mutex> lock(save_mutex_);
        save_condition_.wait(
          lock, [this]() {return stop_save_worker_ || pending_save_;});
        if (stop_save_worker_ && !pending_save_) {
          return;
        }
        request = std::move(pending_save_);
        save_active_ = true;
      }

      bool success = false;
      std::string error_message;
      try {
        const std::filesystem::path output_path(request->path);
        if (output_path.has_parent_path()) {
          std::filesystem::create_directories(output_path.parent_path());
        }
        success = write_binary_pcd(request->path, request->points, error_message);
      } catch (const std::exception & error) {
        error_message = error.what();
      }

      if (success) {
        RCLCPP_INFO(
          get_logger(), "Saved %zu map points to %s",
          request->points.size(), request->path.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(), "Failed to save map to %s: %s",
          request->path.c_str(), error_message.c_str());
      }
      {
        std::lock_guard<std::mutex> lock(save_mutex_);
        save_active_ = false;
      }
    }
  }

  std::string target_frame_;
  std::string base_frame_;
  std::string pcd_path_;
  double voxel_size_m_{0.05};
  double min_range_m_{0.2};
  double max_range_m_{30.0};
  double min_z_m_{-0.2};
  double max_z_m_{5.0};
  double transform_timeout_sec_{0.1};
  double publish_period_sec_{1.0};
  std::size_t max_voxels_{2000000U};
  std::int64_t log_every_n_clouds_{50};

  bool self_filter_enabled_{true};
  double self_min_x_m_{-0.9};
  double self_max_x_m_{0.9};
  double self_min_y_m_{-0.6};
  double self_max_y_m_{0.6};
  double self_min_z_m_{-0.1};
  double self_max_z_m_{1.5};

  std::unique_ptr<VoxelMap> voxel_map_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::uint64_t cloud_count_{0U};
  std::uint64_t input_point_count_{0U};
  std::uint64_t accepted_point_count_{0U};
  std::uint64_t capacity_drop_count_{0U};
  bool has_received_cloud_{false};

  std::mutex save_mutex_;
  std::condition_variable save_condition_;
  std::unique_ptr<SaveRequest> pending_save_;
  bool save_active_{false};
  bool stop_save_worker_{false};
  std::thread save_worker_;
};

}  // namespace metro_pointcloud_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<metro_pointcloud_mapping::CloudAccumulatorNode>());
  rclcpp::shutdown();
  return 0;
}

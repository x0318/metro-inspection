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

#include "metro_pointcloud_mapping/map_graph_change_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace metro_pointcloud_mapping
{

MapGraphChangeDetector::MapGraphChangeDetector(
  double translation_threshold_m, double rotation_threshold_rad)
: translation_threshold_m_(translation_threshold_m),
  rotation_threshold_rad_(rotation_threshold_rad)
{
  if (translation_threshold_m_ <= 0.0 || rotation_threshold_rad_ <= 0.0) {
    throw std::invalid_argument("map graph thresholds must be positive");
  }
}

bool MapGraphChangeDetector::accept(const rtabmap_msgs::msg::MapGraph & graph)
{
  if (graph.poses_id.size() != graph.poses.size()) {
    throw std::invalid_argument("map graph pose IDs and poses have different sizes");
  }

  bool changed = !initialized_ ||
    accepted_pose_ids_.size() != graph.poses_id.size();

  if (!changed && !graph.poses.empty()) {
    const std::size_t stable_pose_count = graph.poses.size() - 1U;
    for (std::size_t index = 0; index < stable_pose_count; ++index) {
      if (accepted_pose_ids_[index] != graph.poses_id[index] ||
        pose_changed(accepted_poses_[index], graph.poses[index]))
      {
        changed = true;
        break;
      }
    }

    // RTAB-Map may replace the current intermediate node ID every cycle even
    // while stationary. Its pose, rather than that volatile ID, decides whether
    // the assembled map has materially changed.
    if (!changed && pose_changed(accepted_poses_.back(), graph.poses.back())) {
      changed = true;
    }
  }

  if (changed) {
    accepted_pose_ids_ = graph.poses_id;
    accepted_poses_ = graph.poses;
    initialized_ = true;
  }
  return changed;
}

double MapGraphChangeDetector::translation_distance(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right)
{
  const double dx = left.position.x - right.position.x;
  const double dy = left.position.y - right.position.y;
  const double dz = left.position.z - right.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double MapGraphChangeDetector::rotation_distance(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right)
{
  const auto & left_q = left.orientation;
  const auto & right_q = right.orientation;
  const double left_norm = std::sqrt(
    left_q.x * left_q.x + left_q.y * left_q.y +
    left_q.z * left_q.z + left_q.w * left_q.w);
  const double right_norm = std::sqrt(
    right_q.x * right_q.x + right_q.y * right_q.y +
    right_q.z * right_q.z + right_q.w * right_q.w);
  if (left_norm == 0.0 || right_norm == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double dot = std::abs(
    left_q.x * right_q.x + left_q.y * right_q.y +
    left_q.z * right_q.z + left_q.w * right_q.w) /
    (left_norm * right_norm);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

bool MapGraphChangeDetector::pose_changed(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right) const
{
  return translation_distance(left, right) >= translation_threshold_m_ ||
         rotation_distance(left, right) >= rotation_threshold_rad_;
}

}  // namespace metro_pointcloud_mapping

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

#ifndef METRO_POINTCLOUD_MAPPING__MAP_GRAPH_CHANGE_DETECTOR_HPP_
#define METRO_POINTCLOUD_MAPPING__MAP_GRAPH_CHANGE_DETECTOR_HPP_

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rtabmap_msgs/msg/map_graph.hpp"

namespace metro_pointcloud_mapping
{

class MapGraphChangeDetector
{
public:
  MapGraphChangeDetector(
    double translation_threshold_m, double rotation_threshold_rad);

  bool accept(const rtabmap_msgs::msg::MapGraph & graph);

private:
  static double translation_distance(
    const geometry_msgs::msg::Pose & left,
    const geometry_msgs::msg::Pose & right);
  static double rotation_distance(
    const geometry_msgs::msg::Pose & left,
    const geometry_msgs::msg::Pose & right);
  bool pose_changed(
    const geometry_msgs::msg::Pose & left,
    const geometry_msgs::msg::Pose & right) const;

  double translation_threshold_m_;
  double rotation_threshold_rad_;
  bool initialized_{false};
  std::vector<std::int32_t> accepted_pose_ids_;
  std::vector<geometry_msgs::msg::Pose> accepted_poses_;
};

}  // namespace metro_pointcloud_mapping

#endif  // METRO_POINTCLOUD_MAPPING__MAP_GRAPH_CHANGE_DETECTOR_HPP_

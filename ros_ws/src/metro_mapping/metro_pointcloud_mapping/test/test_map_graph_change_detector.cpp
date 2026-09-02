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

#include <gtest/gtest.h>

#include <initializer_list>
#include <stdexcept>

#include "metro_pointcloud_mapping/map_graph_change_detector.hpp"

namespace
{

rtabmap_msgs::msg::MapGraph make_graph(
  std::initializer_list<std::int32_t> ids,
  std::initializer_list<double> positions)
{
  rtabmap_msgs::msg::MapGraph graph;
  graph.poses_id.assign(ids);
  for (const double x : positions) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    graph.poses.push_back(pose);
  }
  return graph;
}

TEST(MapGraphChangeDetectorTest, AcceptsFirstGraph)
{
  metro_pointcloud_mapping::MapGraphChangeDetector detector(0.01, 0.005);

  EXPECT_TRUE(detector.accept(make_graph({1, 10}, {0.0, 1.0})));
}

TEST(MapGraphChangeDetectorTest, SuppressesStationaryVolatileTailReplacement)
{
  metro_pointcloud_mapping::MapGraphChangeDetector detector(0.01, 0.005);

  ASSERT_TRUE(detector.accept(make_graph({1, 10}, {0.0, 1.0})));
  EXPECT_FALSE(detector.accept(make_graph({1, 11}, {0.0, 1.005})));
  EXPECT_TRUE(detector.accept(make_graph({1, 12}, {0.0, 1.011})));
}

TEST(MapGraphChangeDetectorTest, AcceptsStableKeyframeReplacement)
{
  metro_pointcloud_mapping::MapGraphChangeDetector detector(0.01, 0.005);

  ASSERT_TRUE(detector.accept(make_graph({1, 10}, {0.0, 1.0})));
  EXPECT_TRUE(detector.accept(make_graph({2, 11}, {0.0, 1.0})));
}

TEST(MapGraphChangeDetectorTest, AcceptsOptimizedStablePose)
{
  metro_pointcloud_mapping::MapGraphChangeDetector detector(0.01, 0.005);

  ASSERT_TRUE(detector.accept(make_graph({1, 10}, {0.0, 1.0})));
  EXPECT_TRUE(detector.accept(make_graph({1, 11}, {0.02, 1.0})));
}

TEST(MapGraphChangeDetectorTest, RejectsMalformedGraph)
{
  metro_pointcloud_mapping::MapGraphChangeDetector detector(0.01, 0.005);
  auto graph = make_graph({1, 10}, {0.0});

  EXPECT_THROW(detector.accept(graph), std::invalid_argument);
}

}  // namespace

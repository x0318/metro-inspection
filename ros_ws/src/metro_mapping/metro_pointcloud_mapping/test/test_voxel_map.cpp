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

#include <cmath>
#include <limits>
#include <stdexcept>

#include "metro_pointcloud_mapping/voxel_map.hpp"

namespace metro_pointcloud_mapping
{

TEST(VoxelMapTest, AveragesPointsInsideOneVoxel) {
  VoxelMap map(0.1, 10U);

  EXPECT_EQ(map.add_point(Point3d{0.01, 0.02, 0.03}), InsertResult::kInserted);
  EXPECT_EQ(map.add_point(Point3d{0.09, 0.08, 0.07}), InsertResult::kUpdated);

  ASSERT_EQ(map.size(), 1U);
  const auto points = map.points();
  ASSERT_EQ(points.size(), 1U);
  EXPECT_DOUBLE_EQ(points.front().x, 0.05);
  EXPECT_DOUBLE_EQ(points.front().y, 0.05);
  EXPECT_DOUBLE_EQ(points.front().z, 0.05);
}

TEST(VoxelMapTest, UsesFloorForNegativeCoordinates) {
  VoxelMap map(0.1, 10U);

  EXPECT_EQ(map.add_point(Point3d{-0.01, 0.0, 0.0}), InsertResult::kInserted);
  EXPECT_EQ(map.add_point(Point3d{0.01, 0.0, 0.0}), InsertResult::kInserted);

  EXPECT_EQ(map.size(), 2U);
}

TEST(VoxelMapTest, EnforcesCapacityButStillUpdatesExistingVoxels) {
  VoxelMap map(1.0, 1U);

  EXPECT_EQ(map.add_point(Point3d{0.0, 0.0, 0.0}), InsertResult::kInserted);
  EXPECT_EQ(
    map.add_point(Point3d{2.0, 0.0, 0.0}),
    InsertResult::kCapacityReached);
  EXPECT_EQ(map.add_point(Point3d{0.5, 0.0, 0.0}), InsertResult::kUpdated);
  EXPECT_EQ(map.size(), 1U);
}

TEST(VoxelMapTest, RejectsNonFinitePoints) {
  VoxelMap map(0.1, 10U);

  EXPECT_EQ(
    map.add_point(
      Point3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}),
    InsertResult::kRejectedNonFinite);
  EXPECT_TRUE(map.points().empty());
}

TEST(VoxelMapTest, ClearsAllVoxels) {
  VoxelMap map(0.1, 10U);
  map.add_point(Point3d{0.0, 0.0, 0.0});

  map.clear();

  EXPECT_EQ(map.size(), 0U);
}

TEST(VoxelMapTest, RejectsInvalidConstructionParameters) {
  EXPECT_THROW(VoxelMap(0.0, 1U), std::invalid_argument);
  EXPECT_THROW(VoxelMap(0.1, 0U), std::invalid_argument);
}

}  // namespace metro_pointcloud_mapping

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

#ifndef METRO_POINTCLOUD_MAPPING__VOXEL_MAP_HPP_
#define METRO_POINTCLOUD_MAPPING__VOXEL_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace metro_pointcloud_mapping
{

struct Point3d
{
  double x;
  double y;
  double z;
};

enum class InsertResult
{
  kInserted,
  kUpdated,
  kCapacityReached,
  kRejectedNonFinite,
};

class VoxelMap
{
public:
  VoxelMap(double voxel_size_m, std::size_t max_voxels);

  InsertResult add_point(const Point3d & point);
  std::vector<Point3d> points() const;
  void clear();

  std::size_t size() const;
  double voxel_size_m() const;
  std::size_t max_voxels() const;

private:
  struct VoxelKey
  {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey & other) const;
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const;
  };

  struct VoxelAccumulator
  {
    Point3d centroid;
    std::uint64_t observation_count;
  };

  VoxelKey key_for(const Point3d & point) const;

  double voxel_size_m_;
  std::size_t max_voxels_;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
};

}  // namespace metro_pointcloud_mapping

#endif  // METRO_POINTCLOUD_MAPPING__VOXEL_MAP_HPP_

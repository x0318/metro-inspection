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

#include "metro_pointcloud_mapping/voxel_map.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace metro_pointcloud_mapping
{

namespace
{

void hash_combine(std::size_t & seed, std::size_t value)
{
  seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

}  // namespace

VoxelMap::VoxelMap(double voxel_size_m, std::size_t max_voxels)
: voxel_size_m_(voxel_size_m), max_voxels_(max_voxels)
{
  if (!std::isfinite(voxel_size_m_) || voxel_size_m_ <= 0.0) {
    throw std::invalid_argument(
            "voxel_size_m must be finite and greater than zero");
  }
  if (max_voxels_ == 0U) {
    throw std::invalid_argument("max_voxels must be greater than zero");
  }
  voxels_.reserve(max_voxels_ < 100000U ? max_voxels_ : 100000U);
}

InsertResult VoxelMap::add_point(const Point3d & point)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
    !std::isfinite(point.z))
  {
    return InsertResult::kRejectedNonFinite;
  }

  const VoxelKey key = key_for(point);
  const auto existing = voxels_.find(key);
  if (existing == voxels_.end()) {
    if (voxels_.size() >= max_voxels_) {
      return InsertResult::kCapacityReached;
    }
    voxels_.emplace(key, VoxelAccumulator{point, 1U});
    return InsertResult::kInserted;
  }

  auto & accumulator = existing->second;
  ++accumulator.observation_count;
  const double weight =
    1.0 / static_cast<double>(accumulator.observation_count);
  accumulator.centroid.x += (point.x - accumulator.centroid.x) * weight;
  accumulator.centroid.y += (point.y - accumulator.centroid.y) * weight;
  accumulator.centroid.z += (point.z - accumulator.centroid.z) * weight;
  return InsertResult::kUpdated;
}

std::vector<Point3d> VoxelMap::points() const
{
  std::vector<Point3d> result;
  result.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    result.push_back(entry.second.centroid);
  }
  return result;
}

void VoxelMap::clear() {voxels_.clear();}

std::size_t VoxelMap::size() const {return voxels_.size();}

double VoxelMap::voxel_size_m() const {return voxel_size_m_;}

std::size_t VoxelMap::max_voxels() const {return max_voxels_;}

bool VoxelMap::VoxelKey::operator==(const VoxelKey & other) const
{
  return x == other.x && y == other.y && z == other.z;
}

std::size_t VoxelMap::VoxelKeyHash::operator()(const VoxelKey & key) const
{
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  hash_combine(seed, std::hash<std::int64_t>{}(key.y));
  hash_combine(seed, std::hash<std::int64_t>{}(key.z));
  return seed;
}

VoxelMap::VoxelKey VoxelMap::key_for(const Point3d & point) const
{
  return VoxelKey{
    static_cast<std::int64_t>(std::floor(point.x / voxel_size_m_)),
    static_cast<std::int64_t>(std::floor(point.y / voxel_size_m_)),
    static_cast<std::int64_t>(std::floor(point.z / voxel_size_m_)),
  };
}

}  // namespace metro_pointcloud_mapping

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

#ifndef METRO_POINTCLOUD_MAPPING__PCD_WRITER_HPP_
#define METRO_POINTCLOUD_MAPPING__PCD_WRITER_HPP_

#include <string>
#include <vector>

#include "metro_pointcloud_mapping/voxel_map.hpp"

namespace metro_pointcloud_mapping
{

bool write_binary_pcd(
  const std::string & path, const std::vector<Point3d> & points,
  std::string & error_message);

}  // namespace metro_pointcloud_mapping

#endif  // METRO_POINTCLOUD_MAPPING__PCD_WRITER_HPP_

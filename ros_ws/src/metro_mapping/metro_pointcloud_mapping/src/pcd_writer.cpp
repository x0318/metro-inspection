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

#include "metro_pointcloud_mapping/pcd_writer.hpp"

#include <array>
#include <cmath>
#include <fstream>

namespace metro_pointcloud_mapping
{

bool write_binary_pcd(
  const std::string & path, const std::vector<Point3d> & points,
  std::string & error_message)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    error_message = "unable to open output file";
    return false;
  }

  output << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z\n"
         << "SIZE 4 4 4\n"
         << "TYPE F F F\n"
         << "COUNT 1 1 1\n"
         << "WIDTH " << points.size() << '\n'
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points.size() << '\n'
         << "DATA binary\n";

  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      error_message = "map contains a non-finite point";
      return false;
    }
    const std::array<float, 3> xyz{
      static_cast<float>(point.x),
      static_cast<float>(point.y),
      static_cast<float>(point.z),
    };
    output.write(
      reinterpret_cast<const char *>(xyz.data()),
      static_cast<std::streamsize>(xyz.size() * sizeof(float)));
  }

  output.flush();
  if (!output.good()) {
    error_message = "write failed before the complete map reached disk";
    return false;
  }
  error_message.clear();
  return true;
}

}  // namespace metro_pointcloud_mapping

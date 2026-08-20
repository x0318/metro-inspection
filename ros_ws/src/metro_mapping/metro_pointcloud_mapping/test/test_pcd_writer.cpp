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

#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "metro_pointcloud_mapping/pcd_writer.hpp"

namespace metro_pointcloud_mapping
{

class PcdWriterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    output_path_ = std::filesystem::temp_directory_path() /
      "metro_pointcloud_mapping_test.pcd";
    std::filesystem::remove(output_path_);
  }

  void TearDown() override
  {
    std::filesystem::remove(output_path_);
  }

  std::filesystem::path output_path_;
};

TEST_F(PcdWriterTest, WritesPcdHeaderAndPackedXyzPayload)
{
  const std::vector<Point3d> points{{1.0, 2.0, 3.0}, {-1.0, -2.0, -3.0}};
  std::string error;

  ASSERT_TRUE(write_binary_pcd(output_path_.string(), points, error)) << error;

  std::ifstream input(output_path_, std::ios::binary);
  const std::vector<char> bytes{
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  const std::string file(bytes.begin(), bytes.end());
  const std::string marker = "DATA binary\n";
  const std::size_t marker_position = file.find(marker);

  ASSERT_NE(file.find("FIELDS x y z\n"), std::string::npos);
  ASSERT_NE(file.find("WIDTH 2\n"), std::string::npos);
  ASSERT_NE(file.find("POINTS 2\n"), std::string::npos);
  ASSERT_NE(marker_position, std::string::npos);
  const std::size_t payload_offset = marker_position + marker.size();
  EXPECT_EQ(bytes.size() - payload_offset, 2U * 3U * sizeof(float));
}

TEST_F(PcdWriterTest, RejectsNonFinitePoint)
{
  const std::vector<Point3d> points{
    {std::numeric_limits<double>::infinity(), 0.0, 0.0}};
  std::string error;

  EXPECT_FALSE(write_binary_pcd(output_path_.string(), points, error));
  EXPECT_NE(error.find("non-finite"), std::string::npos);
}

}  // namespace metro_pointcloud_mapping

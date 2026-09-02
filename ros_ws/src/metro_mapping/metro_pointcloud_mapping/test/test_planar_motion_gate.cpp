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

#include <limits>
#include <stdexcept>

#include "metro_pointcloud_mapping/planar_motion_gate.hpp"

namespace
{

using metro_pointcloud_mapping::MotionGateDecision;
using metro_pointcloud_mapping::PlanarMotionGate;

TEST(PlanarMotionGateTest, ForwardsFirstValidPose)
{
  PlanarMotionGate gate(0.35, 0.10);

  EXPECT_EQ(gate.evaluate(1.0, 2.0, 0.0), MotionGateDecision::kForward);
}

TEST(PlanarMotionGateTest, UsesDisplacementFromLastForwardedPose)
{
  PlanarMotionGate gate(0.35, 0.10);

  ASSERT_EQ(gate.evaluate(0.0, 0.0, 0.0), MotionGateDecision::kForward);
  EXPECT_EQ(gate.evaluate(0.20, 0.0, 0.0), MotionGateDecision::kSuppress);
  EXPECT_EQ(gate.evaluate(0.34, 0.0, 0.0), MotionGateDecision::kSuppress);
  EXPECT_EQ(gate.evaluate(0.36, 0.0, 0.0), MotionGateDecision::kForward);
  EXPECT_EQ(gate.evaluate(0.50, 0.0, 0.0), MotionGateDecision::kSuppress);
}

TEST(PlanarMotionGateTest, HandlesYawWraparound)
{
  PlanarMotionGate gate(1.0, 0.10);

  ASSERT_EQ(gate.evaluate(0.0, 0.0, 3.12), MotionGateDecision::kForward);
  EXPECT_EQ(gate.evaluate(0.0, 0.0, -3.12), MotionGateDecision::kSuppress);
  EXPECT_EQ(gate.evaluate(0.0, 0.0, -2.95), MotionGateDecision::kForward);
}

TEST(PlanarMotionGateTest, RejectsNonFinitePoseWithoutChangingReference)
{
  PlanarMotionGate gate(0.35, 0.10);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(gate.evaluate(nan, 0.0, 0.0), MotionGateDecision::kReject);
  EXPECT_EQ(gate.evaluate(0.0, 0.0, 0.0), MotionGateDecision::kForward);
}

TEST(PlanarMotionGateTest, RejectsInvalidThresholds)
{
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_THROW(PlanarMotionGate(0.0, 0.10), std::invalid_argument);
  EXPECT_THROW(PlanarMotionGate(0.35, infinity), std::invalid_argument);
}

}  // namespace

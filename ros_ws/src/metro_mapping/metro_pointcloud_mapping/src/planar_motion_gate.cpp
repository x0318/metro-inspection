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

#include "metro_pointcloud_mapping/planar_motion_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace metro_pointcloud_mapping
{

PlanarMotionGate::PlanarMotionGate(
  double translation_threshold_m, double rotation_threshold_rad)
: translation_threshold_m_(translation_threshold_m),
  rotation_threshold_rad_(rotation_threshold_rad)
{
  if (!std::isfinite(translation_threshold_m_) ||
    !std::isfinite(rotation_threshold_rad_) ||
    translation_threshold_m_ <= 0.0 || rotation_threshold_rad_ <= 0.0)
  {
    throw std::invalid_argument("motion gate thresholds must be finite and positive");
  }
}

MotionGateDecision PlanarMotionGate::evaluate(
  double x, double y, double yaw)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    return MotionGateDecision::kReject;
  }

  if (!initialized_) {
    accepted_x_ = x;
    accepted_y_ = y;
    accepted_yaw_ = yaw;
    initialized_ = true;
    return MotionGateDecision::kForward;
  }

  const double dx = x - accepted_x_;
  const double dy = y - accepted_y_;
  const double translation = std::hypot(dx, dy);
  const double rotation = angular_distance(yaw, accepted_yaw_);
  if (translation < translation_threshold_m_ &&
    rotation < rotation_threshold_rad_)
  {
    return MotionGateDecision::kSuppress;
  }

  accepted_x_ = x;
  accepted_y_ = y;
  accepted_yaw_ = yaw;
  return MotionGateDecision::kForward;
}

double PlanarMotionGate::angular_distance(double left, double right)
{
  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
  return std::abs(std::remainder(left - right, kTwoPi));
}

}  // namespace metro_pointcloud_mapping

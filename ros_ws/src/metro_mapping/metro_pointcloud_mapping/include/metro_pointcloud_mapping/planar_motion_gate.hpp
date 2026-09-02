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

#ifndef METRO_POINTCLOUD_MAPPING__PLANAR_MOTION_GATE_HPP_
#define METRO_POINTCLOUD_MAPPING__PLANAR_MOTION_GATE_HPP_

namespace metro_pointcloud_mapping
{

enum class MotionGateDecision
{
  kForward,
  kSuppress,
  kReject,
};

class PlanarMotionGate
{
public:
  PlanarMotionGate(
    double translation_threshold_m, double rotation_threshold_rad);

  MotionGateDecision evaluate(double x, double y, double yaw);

private:
  static double angular_distance(double left, double right);

  double translation_threshold_m_;
  double rotation_threshold_rad_;
  bool initialized_{false};
  double accepted_x_{0.0};
  double accepted_y_{0.0};
  double accepted_yaw_{0.0};
};

}  // namespace metro_pointcloud_mapping

#endif  // METRO_POINTCLOUD_MAPPING__PLANAR_MOTION_GATE_HPP_

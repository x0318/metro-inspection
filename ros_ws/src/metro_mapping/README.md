# Metro mapping integration

This directory contains image-point-cloud damage localization code imported from
`origin/feature/mapping` at commit `ac976bf`.

It also contains `metro_pointcloud_mapping`, the first-stage odometry-based
accumulated cloud package. The simulation mapping profile starts it automatically:

```bash
cd /home/jo/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

Its `/mapping/cloud_map` output and PCD save service validate the accumulation
pipeline. The V2 entry scripts now start a local EKF that publishes
`/odometry/filtered` and owns `odom -> base_footprint`, using
`/wheel/odom_raw` plus `/odin1/imu`. This improves local motion stability but
does not perform scan registration or loop closure; a LiDAR/IMU SLAM source must
still provide global drift correction for the final map.

The current simulation remains owned by `metro_sim`. Do not use the teammate demo
`closed_loop.launch.py`: it starts a second Gazebo world, robot, camera, and lidar.
The imported `metro_closed_loop` package is intentionally reduced to its red-patch
placeholder detector and a launch adapter for the existing `subway_v2` sensors.

Build the two packages:

```bash
cd /home/jo/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-robot-localization
colcon build --symlink-install \
  --packages-select metro_localization metro_closed_loop
source install/setup.bash
```

Start the full sensor simulation first, then start fusion in a second terminal:

```bash
export ROS_DOMAIN_ID=70
ros2 launch metro_closed_loop subway_v2_fusion.launch.py \
  run_camera_info_calibrator:=false \
  run_placeholder_detector:=false
```

The full sensor startup script already publishes calibrated camera information.
Disabling the launch file's second calibrator keeps exactly one publisher on
`/odin1/rgb/camera_info`. The standalone fusion performance profile below still
uses the launch default (`true`) because it starts its own calibration relay.

For normal fusion work, prefer the single-command performance profile. It keeps
only the Odin1 RGB, lidar, and IMU sensors and removes the five unrelated cameras
from a temporary SDF:

```bash
cd /home/jo/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_fusion.sh
```

The adapter defaults to the only camera-lidar pair currently verified to have both
a calibrated transform and overlapping fields of view:

```text
image:       /odin1/rgb/image_raw
camera info: /odin1/rgb/camera_info
cloud:       /odin1/cloud_raw
global TF:   odom
```

The `xj1` through `xj4` cameras are not interchangeable defaults. Their current
views do not overlap the rear-facing Odin1 point cloud, even though their TF
frames exist. Select a different camera only after its field-of-view overlap and
lidar-to-camera extrinsic calibration have been verified.

The red-color detector is only a wiring test and can classify red tunnel hardware
as damage. It is disabled by default and remains disabled during normal fusion.
Enable it only for an explicit end-to-end wiring test:

```bash
ros2 launch metro_closed_loop subway_v2_fusion.launch.py \
  run_placeholder_detector:=true
```

A real detector must publish `vision_msgs/msg/Detection2DArray` on
`/damage_detections`. Keep the placeholder disabled while that detector is running.

`localization_evaluator` and `damage_semantic_mapper` are disabled by default.
Their ground-truth and tunnel-chainage constants belong to the teammate demo world
and must be configured for the current tunnel before use.

The fusion profile uses the supplied 1600x1296 Odin1 calibration (`fx=736.9688`,
`fy=737.0365`, `skew=0.2058`, `cx=766.6570`, `cy=642.9091`). Gazebo Classic's
camera plugin cannot publish distinct `fx`/`fy` or nonzero skew in `CameraInfo`,
so its approximate message is retained on `/odin1/rgb/camera_info_gazebo` and
`camera_info_calibrator` publishes the authoritative matrix on
`/odin1/rgb/camera_info`. No distortion coefficients were supplied; both the
rendered image and published calibration therefore use zero distortion.

The default timestamp tolerance is `0.08 s`. In this Gazebo profile the measured
nearest image/cloud timestamp differences are normally `0` or `0.1 s`. Increasing
the value may raise the fusion output rate, but it also permits more motion error;
do not tune it only to make `topic hz` look higher.

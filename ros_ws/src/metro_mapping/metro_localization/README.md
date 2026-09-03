# Metro localization

This package has two responsibilities: 2D-to-3D damage localization and local
robot odometry fusion.

## Local odometry fusion

The base configuration fuses wheel forward velocity, wheel yaw rate and IMU yaw
rate with `robot_localization`:

```text
/wheel/odom_raw + /odin1/imu
  -> /odometry/filtered + odom -> base_footprint
```

Absolute IMU orientation is excluded because the current simulated message has
zero orientation covariance. In graph-mapping mode, the launch also enables
relative-origin absolute x/y/yaw input from `/lidar/odom`:

```text
/wheel/odom_raw + /odin1/imu + /lidar/odom
  -> local EKF
```

This separation is intentional. The EKF supplies a continuous local estimate;
RTAB-Map owns only the global `map -> odom` correction. ICP does not publish TF,
and the Gazebo differential-drive plugin must keep `publish_odom_tf=false`, so
there is exactly one owner for each transform.

Install and build:

```bash
sudo apt install ros-humble-robot-localization
cd ~/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select metro_localization
source install/setup.bash
```

Run the base wheel/IMU fusion in simulation:

```bash
ros2 launch metro_localization odometry_fusion.launch.py use_sim_time:=true
```

For hardware, use wall time and start from these timing values:

```bash
ros2 launch metro_localization odometry_fusion.launch.py \
  use_sim_time:=false frequency:=50.0 transform_time_offset:=0.0
```

The graph-SLAM launch sets `fuse_lidar_odometry:=true` automatically. Do not
enable it unless `/lidar/odom` is being published. If scan matching is lost, the
EKF continues on wheel and IMU inputs instead of receiving a null ICP pose. The
lidar pose is relative to its first valid sample, so it anchors lateral drift
without importing the simulation world's absolute spawn position.

## 2D-to-3D damage localization

The localization node projects lidar points into a calibrated camera image,
selects points inside each 2D detection, and publishes a 3D damage location.
The validated simulation defaults are:

```text
image:       /odin1/rgb/image_raw
camera info: /odin1/rgb/camera_info
cloud:       /odin1/cloud_raw
detections:  /damage_detections
global TF:   odom
```

Important outputs are `/damage_point_camera`, `/damage_point_global`,
`/localization/debug_projection`, and `/localization/estimated_marker`. When
`publish_events:=true`, the node also preserves the YOLO class, confidence and
bbox in `metro_inspection_interfaces/msg/DefectEvent`, adds the current-cloud
3D point plus tunnel engineering semantics, and publishes
`/localized/defect_events` for the dashboard. Three unique frames are required
before a new spatial event is accepted, and later observations within 0.5 m
update the same event ID instead of filling the database once per video frame.

For the current `subway_v2` sensor simulation, use the adapter rather than the
teammate demo world:

```bash
ros2 launch metro_closed_loop subway_v2_fusion.launch.py \
  run_camera_info_calibrator:=false \
  run_placeholder_detector:=false
```

The placeholder red detector is only a wiring test. A real detector should
publish `vision_msgs/msg/Detection2DArray` on `/damage_detections`. Verify camera
and lidar field-of-view overlap, extrinsic calibration and timestamp alignment
before interpreting the output as a physical 3D location.

The default chainage and ring settings are simulation placeholders:

```text
chainage start: K12+000
ring start:     1000
ring length:    1.2 m
display name:   仿真环号
```

They make the coordinate-to-report path testable, but they are not surveyed
metro line data. Update `config/localization.yaml` when the real tunnel origin,
direction and ring spacing are available.

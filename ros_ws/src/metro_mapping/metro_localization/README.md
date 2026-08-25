# Metro localization

This ROS 2 package projects lidar points into a camera image, selects points inside
a 2D damage detection, and publishes the estimated 3D position in the camera and
`odom` frames.

It also owns the local odometry fusion configuration for the inspection robot:

```text
/wheel/odom_raw + /odin1/imu
        -> robot_localization EKF
        -> /odometry/filtered + odom -> base_footprint
```

The Gazebo wheel plugin must keep `publish_odom_tf=false`; the EKF is the only
publisher allowed to own `odom -> base_footprint`. The baseline intentionally
fuses wheel `vx`, wheel yaw rate, and IMU yaw rate. Absolute IMU orientation is
excluded because the current simulated message reports zero orientation
covariance.

Install the runtime dependency and build the package:

```bash
sudo apt install ros-humble-robot-localization
cd ~/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select metro_localization
source install/setup.bash
```

The V2 simulation entry scripts start this launch file automatically. For an
already running compatible robot or bag playback, start it directly with:

```bash
ros2 launch metro_localization odometry_fusion.launch.py use_sim_time:=true
```

The simulation launch defaults to `frequency:=10.0` because Gazebo publishes
`/clock` at 10 Hz. It also future-dates the local TF by `0.05 s` so timestamped
point clouds do not occasionally arrive just ahead of the latest EKF transform.
For hardware, use wall time, the 50 Hz configuration, and no time offset unless
measurements show that one is needed:

```bash
ros2 launch metro_localization odometry_fusion.launch.py \
  use_sim_time:=false frequency:=50.0 transform_time_offset:=0.0
```

For the current `subway_v2` simulation, start the adapter from
`metro_closed_loop` instead of launching the teammate demo world:

```bash
ros2 launch metro_closed_loop subway_v2_fusion.launch.py
```

Inputs used by the validated default configuration:

```text
/odin1/rgb/image_raw       sensor_msgs/msg/Image
/odin1/rgb/camera_info     sensor_msgs/msg/CameraInfo
/odin1/cloud_raw           sensor_msgs/msg/PointCloud2
/damage_detections         vision_msgs/msg/Detection2DArray
/damage_mask               sensor_msgs/msg/Image (optional)
/tf and /tf_static         lidar/camera/odom transforms
/wheel/odom_raw            unfiltered wheel odometry; never a TF source
/odin1/imu                 raw IMU measurement
/odometry/filtered         local EKF odometry
```

Important outputs:

```text
/damage_point_camera            geometry_msgs/msg/PointStamped
/damage_point_global            geometry_msgs/msg/PointStamped
/localization/debug_projection  sensor_msgs/msg/Image
/localization/estimated_marker  visualization_msgs/msg/Marker
```

`localization.launch.py` starts only the algorithm package. Its evaluator and
semantic mapper are off by default because their ground-truth and tunnel-chainage
constants must be configured for the current tunnel before use.

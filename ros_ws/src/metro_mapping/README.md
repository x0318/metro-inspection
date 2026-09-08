# Metro mapping integration

This directory contains two independent data paths:

- `metro_localization` projects a 2D damage detection into the lidar cloud and
  reports a 3D damage position.
- `metro_pointcloud_mapping` builds a loop-closing 3D lidar map.

The simulation mapping path is:

```text
/wheel/odom_raw + /odin1/imu + /lidar/odom
  -> robot_localization EKF
  -> /odometry/filtered + odom -> base_footprint

/odin1/cloud_raw -> cloud quality gate -> RTAB-Map ICP and pose graph
  -> map -> odom -> optimized /mapping/cloud_map
  -> PCD file + RTAB-Map database
```

TF ownership is deliberately unique: the EKF publishes
`odom -> base_footprint`, RTAB-Map publishes `map -> odom`, and ICP does not
publish TF. The Gazebo differential-drive plugin must keep
`publish_odom_tf=false`.

## Install and build

ROS 2 Humble runtime dependencies are not vendored in this repository:

```bash
sudo apt update
sudo apt install ros-humble-robot-localization ros-humble-rtabmap-ros

cd ~/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to metro_localization metro_pointcloud_mapping
source install/setup.bash
```

## Simulation workflow

Start the lidar/IMU-only simulation and mapping stack:

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

Open the mapping RViz in a second terminal:

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_v2_mapping_rviz.sh
```

Drive forward from a third terminal. Keep this publisher running while the
robot moves, then press `Ctrl+C` and send the zero command:

```bash
source /opt/ros/humble/setup.bash
source ~/my-project/metro-inspection/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=70

ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist '{}'
```

Save and validate the map:

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/check_subway_v2_slam.sh
```

The default outputs are:

```text
results/maps/subway_v2_optimized.pcd
results/maps/subway_v2_rtabmap.db
```

The launch script starts a new database by default. Resume a previous mapping
session without deleting its graph with:

```bash
SUBWAY_MAPPING_RESET_DATABASE=false \
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

A loop constraint can only be created after the robot physically revisits a
previously mapped area. A one-way recording proves the odometry, graph, map and
save chain, but it does not prove that a loop closure occurred.

## Hardware boundary

For a real robot, keep the same topic and TF contract but use wall time:

```bash
ros2 launch metro_pointcloud_mapping graph_slam.launch.py \
  use_sim_time:=false \
  ekf_frequency:=50.0 \
  ekf_transform_time_offset:=0.0 \
  reset_database:=true \
  database_path:=$PWD/results/maps/real_rtabmap.db \
  pcd_path:=$PWD/results/maps/real_optimized.pcd
```

Before trusting a real map, calibrate lidar/IMU extrinsics and time sync, verify
wheel and IMU covariances, tune the point-cloud gate and ICP against recorded
data, and drive repeated out-and-back loops to reject false closures.

The imported `metro_closed_loop` package remains only an adapter for 2D-to-3D
damage localization. Do not run its teammate demo world together with
`subway_v2`; that would create a second Gazebo world and duplicate sensors.

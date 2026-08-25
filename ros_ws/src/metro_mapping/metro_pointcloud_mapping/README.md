# Metro point-cloud mapping

This package supports both the original bounded voxel accumulator and the new
loop-closing RTAB-Map pipeline. Use `graph_slam.launch.py` for the final 3D map.

## Graph-SLAM interfaces

```text
input:   /odin1/cloud_raw       sensor_msgs/msg/PointCloud2
input:   /wheel/odom_raw        nav_msgs/msg/Odometry
input:   /odin1/imu             sensor_msgs/msg/Imu
output:  /mapping/cloud_valid   sensor_msgs/msg/PointCloud2
output:  /lidar/odom            nav_msgs/msg/Odometry
output:  /odometry/filtered     nav_msgs/msg/Odometry
output:  /mapping/cloud_map     sensor_msgs/msg/PointCloud2
service: /mapping/save_map      std_srvs/srv/Trigger
TF:      map -> odom -> base_footprint
```

`cloud_gate` removes empty, sparse, malformed and out-of-order lidar frames
before scan matching. Its simulation default is 5000 points because the
recorded baseline contained 617 empty frames out of 718, and only 64 frames had
more than 5000 points. This value is a dataset-specific guardrail. For hardware,
measure the normal valid point count first, then set `minimum_points` below the
lower tail of valid scans so real geometry is not discarded.

ICP adds short-range scan constraints and uses the EKF transform as its motion
guess. If ICP loses registration it publishes no invalid odometry; the EKF keeps
running from wheel odometry and IMU. RTAB-Map stores keyframes and sequential or
proximity constraints, optimizes their poses, and `map_assembler` rebuilds the
cloud from those optimized poses. This is what allows old points to move after
a loop closure; the legacy accumulator cannot correct points already inserted.

## Build and run

```bash
sudo apt install ros-humble-robot-localization ros-humble-rtabmap-ros
cd ~/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to metro_localization metro_pointcloud_mapping
source install/setup.bash

export ROS_DOMAIN_ID=70
ros2 launch metro_pointcloud_mapping graph_slam.launch.py \
  use_sim_time:=true \
  database_path:=$PWD/../results/maps/subway_v2_rtabmap.db \
  pcd_path:=$PWD/../results/maps/subway_v2_optimized.pcd
```

For a real sensor stack, set `use_sim_time:=false`, `ekf_frequency:=50.0`, and
`ekf_transform_time_offset:=0.0`. Topic arguments can remap each hardware source
without changing node code.

Save the latest optimized cloud:

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
```

The PCD contains the optimized point cloud for inspection and visualization.
The `.db` contains the keyframes, constraints and graph state needed for later
inspection or continuation. Set `reset_database:=false` only when intentionally
continuing that database.

## Loop-closure acceptance

Drive away from the starting area and return through the same geometry. In RViz,
watch `/mapping/cloud_map` and the `map -> odom` transform. After a valid loop is
accepted, the map may shift slightly while global optimization distributes the
accumulated error. Repeating tunnel structures can cause false matches, so a
closure must be checked against the real route and then repeated over multiple
runs before the map is accepted.

The legacy `accumulated_map.launch.py`, `/mapping/reset_map`, and bounded voxel
accumulator remain available for simple pipeline tests. They do not perform ICP,
pose-graph optimization, or loop closure.

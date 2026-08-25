# Metro point-cloud mapping

This package implements the first mapping stage: timestamped Odin1 clouds are
transformed into `odom`, filtered, and accumulated in a bounded voxel map. The
`odom -> base_footprint` transform comes from the wheel/IMU local EKF. It is a
locally fused odometry baseline, not loop-closing 3D SLAM.

## Interfaces

```text
input:   /odin1/cloud_raw       sensor_msgs/msg/PointCloud2
output:  /mapping/cloud_map     sensor_msgs/msg/PointCloud2
service: /mapping/save_map      std_srvs/srv/Trigger
service: /mapping/reset_map     std_srvs/srv/Trigger
```

The map publisher uses reliable, transient-local QoS so a late RViz subscriber
receives the latest map. Sensor input uses the ROS sensor-data QoS profile.

## Build and run

```bash
cd ~/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select metro_pointcloud_mapping
source install/setup.bash

export ROS_DOMAIN_ID=70
ros2 launch metro_pointcloud_mapping accumulated_map.launch.py
```

For the current simulation, the integrated one-command entry is preferred:

```bash
cd ~/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
```

Save or reset the current map:

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
ros2 service call /mapping/reset_map std_srvs/srv/Trigger '{}'
```

The default PCD path is `/tmp/metro_accumulated_cloud.pcd`. Override it with:

```bash
ros2 launch metro_pointcloud_mapping accumulated_map.launch.py \
  pcd_path:=$PWD/results/maps/subway_v2_accumulated.pcd
```

At 5 cm resolution, the default two-million-voxel limit bounds memory growth.
New voxels are rejected after the limit is reached, while observations in
existing voxels continue updating their centroids.

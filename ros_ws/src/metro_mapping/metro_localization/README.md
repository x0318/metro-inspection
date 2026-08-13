# Metro localization

This ROS 2 package projects lidar points into a camera image, selects points inside
a 2D damage detection, and publishes the estimated 3D position in the camera and
`odom` frames.

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

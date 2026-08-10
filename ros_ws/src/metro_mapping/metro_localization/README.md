# Metro image-point-cloud localization prototype

This workspace contains a minimal Gazebo closed loop for validating 3D damage
localization before the company dataset is available.

## What is real in this prototype

- Gazebo camera image: `/camera/image_raw`
- Gazebo lidar cloud: `/lidar/points` (`sensor_msgs/PointCloud2`)
- Timestamped detections: `/damage_detections` (`vision_msgs/Detection2DArray`)
- TF-based lidar-to-camera transform
- Camera projection using `/camera/camera_info`
- Robust median surface point from lidar points inside the detection ROI
- Camera-frame and global-frame outputs
- Gazebo ground-truth error measurement

The red-patch detector is only a replaceable placeholder. It performs color
segmentation and does **not** publish a synthetic depth value.

## Build

```bash
cd /home/yang/metro-inspection
source /opt/ros/humble/setup.bash
colcon build --packages-select metro_localization metro_closed_loop
source install/setup.bash
```

## Static validation (recommended first)

```bash
ros2 launch metro_closed_loop closed_loop.launch.py auto_drive:=false
```

Important outputs (run each command in a new terminal after sourcing the workspace):

```bash
source /opt/ros/humble/setup.bash
source /home/yang/metro-inspection/install/setup.bash
ros2 topic hz /lidar/points
ros2 topic echo /damage_point_camera
ros2 topic echo /damage_point_global
ros2 topic echo /localization/error_m
```

`ros2 topic echo` waits silently until the next message arrives. The launch
terminal now prints a `Pipeline status` line every two seconds; its
`published(camera/global)` counters are the quickest way to distinguish an
empty detector result from a projection, ROI, or TF problem.

In RViz, `Fusion Debug` uses:

- green: all lidar points projected into the image
- blue box: 2D damage detection
- yellow: lidar points selected inside the ROI
- red cross: estimated 3D point reprojected into the image

## Moving validation

After the static overlay and error are correct:

```bash
ros2 launch metro_closed_loop closed_loop.launch.py auto_drive:=true
```

The detector/localizer start at 4 s, while automatic driving starts at 9 s.
The default motion is a slow straight drive (`0.08 m/s`, `0 rad/s`) so the
reference damage remains inside the 60-degree camera field of view. The values
can be overridden when needed:

```bash
ros2 launch metro_closed_loop closed_loop.launch.py \
  auto_drive:=true auto_drive_delay:=9.0 \
  auto_linear_x:=0.08 auto_angular_z:=0.0
```

The synchronizer defaults to a queue of 20 and a maximum timestamp difference
of 0.08 s. These values can be tuned in
`metro_localization/config/localization.yaml`.

## Replacing the placeholder detector

A real YOLO or segmentation node only needs to publish
`vision_msgs/Detection2DArray` on `/damage_detections`. Each output header must
copy the source image header. The localization node must never receive a placeholder
or monocular guessed depth from the detector.
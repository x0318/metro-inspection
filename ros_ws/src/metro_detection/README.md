# metro_detection

ROS 2 YOLOv8 detector for the Metro simulation. The node subscribes to a raw
camera image and publishes standard `vision_msgs/msg/Detection2DArray`
messages, so it connects directly to `metro_localization`.

## Interfaces

Default input:

```text
/odin1/rgb/image_raw                sensor_msgs/msg/Image
```

Outputs:

```text
/damage_detections                 vision_msgs/msg/Detection2DArray
/damage_detection/annotated_image  sensor_msgs/msg/Image
```

The output header is copied from the source image. This is required by the
image, lidar, and detection time synchronizer in `damage_localizer`.

The current simulation model contains these eight classes:

```text
crack
water_leakage
segment_damage
foreign_object
fastener_missing
fastener_broken
fastener_loose
bracket_loose
```

## Install the inference environment

PyTorch and Ultralytics are installed in a project-local virtual environment,
not into the ROS system Python:

```bash
cd /home/jo/my-project/metro-inspection
./scripts/setup_yolo_environment.sh
```

The setup script uses a CUDA 12.8 PyTorch wheel when an NVIDIA GPU is visible.
It keeps NumPy below 2 and OpenCV below 4.12 for compatibility with the ROS 2
Humble `cv_bridge` binary.

If a reusable YOLO environment already contains NumPy 2.x, the setup script
creates a small `.yolo-ros-compat` NumPy 1.26 overlay. It does not downgrade or
otherwise modify that environment.

## Build and run

Start the full fusion simulation and both YOLO detectors together:

```bash
cd /home/jo/my-project/metro-inspection
./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_yolo.sh
```

The full entry point keeps two independent result streams:

```text
Odin1 forward image
  -> /damage_detections
  -> /damage_detection/annotated_image

Pitch ceiling image
  -> /damage_detections/pitch
  -> /damage_detection/pitch/annotated_image
```

The complete Pitch assembly is tilted 75 degrees above the robot forward axis.
Its approximately 34.5-degree vertical FOV therefore includes the tunnel crown.

Connect YOLO to an already running sensor or fusion simulation:

```bash
cd /home/jo/my-project/metro-inspection
./scripts/open_yolo_detector.sh
```

The default model is:

```text
/home/jo/incoming/yolov8n_sim_demo_best(1).pt
```

Override the model, topic, confidence, or device with environment variables:

```bash
METRO_YOLO_MODEL_PATH=/absolute/path/to/best.pt \
METRO_YOLO_IMAGE_TOPIC=/odin1/rgb/image_raw \
METRO_YOLO_DETECTIONS_TOPIC=/damage_detections \
METRO_YOLO_ANNOTATED_IMAGE_TOPIC=/damage_detection/annotated_image \
METRO_YOLO_NODE_NAME=yolo_detector_front \
METRO_YOLO_CONFIDENCE=0.45 \
METRO_YOLO_DEVICE=0 \
./scripts/open_yolo_detector.sh
```

`METRO_YOLO_DEVICE=auto` selects CUDA when available and otherwise uses CPU.
If CUDA inference fails because a wheel does not support the GPU architecture,
the node retries on CPU once.

Inspect the result:

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
export ROS_DOMAIN_ID=70

ros2 topic type /damage_detections
ros2 topic echo /damage_detections --once
ros2 topic hz /damage_detections
rqt_image_view /damage_detection/annotated_image
rqt_image_view /damage_detection/pitch/annotated_image
```

The Odin1 RGB camera is the default because it is calibrated against the
Odin1 point cloud. Pitch detections are valid 2D YOLO results, but they must not
be sent through the existing Odin1 2D-to-3D projection. A separate Pitch
projection path must use its own intrinsics, TF extrinsic, and overlapping
Odin1 point-cloud samples before it can publish trustworthy 3D coordinates.

## Important limitation

This model was trained from the `yolo_synthetic` simulation dataset recorded
in its checkpoint metadata. Successful inference proves that the ROS image and
detection pipeline works; it does not establish accuracy on real tunnel images.
Evaluate it on held-out simulation images and real labeled images separately.

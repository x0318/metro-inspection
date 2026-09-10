# metro_bringup

`metro_bringup` is the unified ROS 2 entry point for the Metro Inspection
sensor simulation, five-camera YOLO, the matching RViz view, and the defect
dashboard.
It orchestrates the existing `metro_sim` runtime assets and does not copy the
robot or tunnel meshes.

Build once after cloning or changing this package:

```bash
sudo apt install python3-pyqt5.qtwebengine

cd /home/jo/my-project/metro-inspection
./scripts/setup_yolo_environment.sh

cd /home/jo/my-project/metro-inspection/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to metro_bringup
source install/setup.bash
```

Start the complete platform:

```bash
ros2 launch metro_bringup inspection_platform.launch.py
```

The dashboard is served at `http://127.0.0.1:8088` and opens in an embedded Qt
window by default. The Qt window, browser mode, and dashboard backend all use
the same ROS bridge and SQLite database. The default launch starts the full
runtime stack but leaves the vehicle stopped until it receives a drive command.
Every normal drive request passes through the Odin1 point-cloud collision
monitor before reaching the existing command watchdog:

```text
Gazebo sensors and odometry
  -> five-camera YOLO monitoring and ten-site coverage
  -> Odin1 RGB YOLO + calibrated Odin1 point cloud
  -> 3D point + simulation chainage/ring semantics
  -> RViz + FastAPI/SQLite dashboard + Qt window

/cmd_vel_raw
  -> Odin1 front obstacle stop monitor
  -> /cmd_vel_safe
  -> command timeout / software e-stop watchdog
  -> /cmd_vel_drive
```

The front stop zone covers the vehicle's front half and full width, then extends
about 2 m beyond its front edge. More than five Odin1 points between 0.12 m and
1.80 m above
`base_footprint` stop the vehicle. This height filter excludes the simulated
drive surface and rail heads. The zone is published on
`/safety/front_stop_zone` for RViz inspection. If the collision monitor exits,
the complete platform shuts down; if its safe command stream disappears, the
final watchdog continuously commands zero velocity. The main launch also makes
fresh `/odin1/cloud_raw` data mandatory at the watchdog, so a missing or stale
point-cloud stream cannot silently permit motion.

The XJ1-XJ4 and Pitch streams remain the five visible monitor cameras. A sixth,
localization-only Odin1 RGB stream is processed because it is the camera with a
calibrated transform and overlapping field of view with `/odin1/cloud_raw`.
The dashboard displays class names as Chinese labels followed by the stable ROS
identifier, for example `异物入侵（foreign_object）`.

RViz shows one selected annotated stream by default while the dashboard retains
the five-camera overview. Change `Selected YOLO Camera -> Topic` in RViz to
switch between the XJ1-XJ4 and Pitch annotated topics.

Use the deterministic sensor-only profile when YOLO is not required:

```bash
ros2 launch metro_bringup inspection_platform.launch.py detection:=false
```

This selects `odin1_pointcloud.rviz`, which contains only topics always
published by the sensor simulation. The legacy HSV topics `/damage_mask` and
`/localization/debug_projection` are intentionally absent from both profiles.

To use the normal browser instead of Qt:

```bash
ros2 launch metro_bringup inspection_platform.launch.py \
  qt:=false open_browser:=true
```

Useful overrides:

```bash
# Headless runtime without Gazebo GUI or RViz.
ros2 launch metro_bringup inspection_platform.launch.py \
  gui:=false rviz:=false qt:=false open_browser:=false

# Run the complete ten-site test and start its bounded automatic drive.
ros2 launch metro_bringup inspection_platform.launch.py yolo_auto_drive:=true

# Bind the dashboard for an explicitly configured remote-access setup.
ros2 launch metro_bringup inspection_platform.launch.py \
  dashboard_bind_address:=0.0.0.0

# Start only the simulation and RViz.
ros2 launch metro_bringup inspection_platform.launch.py dashboard:=false
```

The engineering location currently uses simulation parameters from
`metro_localization/config/localization.yaml`: `K12+000`, starting ring 1000,
and ring length 1.2 m. The UI therefore labels it `仿真环号`. Replace these
three values with surveyed line data before treating the output as a real
route chainage or ring number.

Each launch creates a fresh timestamped inspection session, so records from a
previous run remain in SQLite without appearing as newly detected defects in
the current run. To reopen one named session intentionally, pass for example
`inspection_session_id:=simulation-test-01`.

Stopping the launch with `Ctrl+C`, or closing the Gazebo simulation, shuts
down RViz and the dashboard process as well. Gazebo Classic can take up to
about 15 seconds to finish its own shutdown sequence.

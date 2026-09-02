# metro_bringup

`metro_bringup` is the unified ROS 2 entry point for the Metro Inspection
sensor simulation, the prepared Odin1 RViz view, and the defect dashboard.
It orchestrates the existing `metro_sim` runtime assets and does not copy the
robot or tunnel meshes.

Build once after cloning or changing this package:

```bash
sudo apt install python3-pyqt5.qtwebengine

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
the same ROS bridge and SQLite database.

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

# Bind the dashboard for an explicitly configured remote-access setup.
ros2 launch metro_bringup inspection_platform.launch.py \
  dashboard_bind_address:=0.0.0.0

# Start only the simulation and RViz.
ros2 launch metro_bringup inspection_platform.launch.py dashboard:=false
```

Stopping the launch with `Ctrl+C`, or closing the Gazebo simulation, shuts
down RViz and the dashboard process as well. Gazebo Classic can take up to
about 15 seconds to finish its own shutdown sequence.

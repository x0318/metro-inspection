#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="${PROJECT_DIR}/ros_ws"
export PYTHONUNBUFFERED=1

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble is not installed: /opt/ros/humble/setup.bash"
  exit 2
fi
source /opt/ros/humble/setup.bash
cd "${ROS_WS_DIR}"

if [[ "${1:-}" == "--build" ]]; then
  if ! command -v colcon >/dev/null; then
    echo "[ERROR] Install python3-colcon-common-extensions and python3-rosdep."
    exit 2
  fi
  echo "[BUILD] Preparing ROS workspace"
  exec colcon build --symlink-install --packages-up-to metro_bringup \
    --event-handlers console_cohesion+
fi

if [[ ! -f install/setup.bash ]]; then
  echo "[ERROR] Workspace is not built. Use Prepare Workspace in the desktop application."
  exit 2
fi
source install/setup.bash

echo "[CHECK] Checking ROS packages and message support"
/usr/bin/python3 - <<'PY'
from ament_index_python.packages import get_package_prefix
from rosidl_generator_py import import_type_support
packages = (
    'metro_bringup', 'metro_dashboard_bridge', 'metro_inspection_interfaces',
    'metro_localization', 'metro_detection', 'metro_closed_loop',
    'gazebo_ros', 'gazebo_ros2_control', 'robot_state_publisher',
    'robot_localization', 'controller_manager', 'position_controllers',
)
for package in packages:
    print(f'{package}: {get_package_prefix(package)}', flush=True)
import_type_support('metro_inspection_interfaces')
import cv_bridge
import scipy
import fastapi
import uvicorn
print('[CHECK] ROS packages ready', flush=True)
PY

echo "[START] Starting inspection platform; automatic driving is disabled"
exec ros2 launch metro_bringup inspection_platform.launch.py "$@"

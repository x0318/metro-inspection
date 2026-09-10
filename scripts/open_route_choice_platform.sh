#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "${PROJECT_DIR}/ros_ws/install/metro_navigation_demo/share/metro_navigation_demo/package.xml" ]]; then
  (cd "${PROJECT_DIR}/ros_ws" && colcon build --symlink-install --packages-select metro_navigation_demo)
fi
source "${PROJECT_DIR}/ros_ws/install/setup.bash"
if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then export DISPLAY=:0; fi
echo "岔轨导航平台默认地址：http://127.0.0.1:8090"
echo "等待页面显示里程计在线后，再选择直行或分岔任务。Ctrl+C 停止本次运行。"
exec ros2 launch metro_navigation_demo route_choice_platform.launch.py "$@"

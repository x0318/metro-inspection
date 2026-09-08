#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${WORKSPACE_DIR}"
colcon build --symlink-install --packages-up-to metro_dashboard_bridge
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

export METRO_DASHBOARD_DIR="${PROJECT_DIR}/dashboard"
export METRO_DASHBOARD_BIND_ADDRESS="${METRO_DASHBOARD_BIND_ADDRESS:-127.0.0.1}"
export METRO_DASHBOARD_PORT="${METRO_DASHBOARD_PORT:-8088}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
DEFECT_TOPIC="${METRO_DASHBOARD_DEFECT_TOPIC:-/simulation/defect_events}"
DATABASE_PATH="${METRO_DASHBOARD_DATABASE_PATH:-${HOME}/.local/share/metro-inspection/defects.sqlite3}"
SESSION_ID="${METRO_INSPECTION_SESSION_ID:-simulation}"

echo "病害巡检平台：http://${METRO_DASHBOARD_BIND_ADDRESS}:${METRO_DASHBOARD_PORT}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "病害话题=${DEFECT_TOPIC}"
echo "病害数据库=${DATABASE_PATH}"
echo "巡检会话=${SESSION_ID}"
echo "按 Ctrl+C 停止服务"

exec ros2 run metro_dashboard_bridge defect_event_bridge \
  --ros-args \
  -p defect_topic:="${DEFECT_TOPIC}" \
  -p database_path:="${DATABASE_PATH}" \
  -p inspection_session_id:="${SESSION_ID}"

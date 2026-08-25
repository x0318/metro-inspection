#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
RVIZ_FILE="${METRO_SIM_DIR}/config/subway_v2_slam.rviz"

if [[ ! -f "${RVIZ_FILE}" ]]; then
  echo "RViz configuration not found: ${RVIZ_FILE}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROS_WS_DIR}/install/setup.bash" ]]; then
  source "${ROS_WS_DIR}/install/setup.bash"
else
  echo "ROS workspace is not built: ${ROS_WS_DIR}/install/setup.bash" >&2
  exit 1
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Opening loop-closing 3D mapping RViz"
exec rviz2 -d "${RVIZ_FILE}" --ros-args -p use_sim_time:=true

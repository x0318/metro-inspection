#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
REPOSITORY_DIR="$(cd "${ROS_WS_DIR}/.." && pwd)"
MAPPING_PCD_PATH="${SUBWAY_MAPPING_PCD_PATH:-${REPOSITORY_DIR}/results/maps/subway_v2_optimized.pcd}"
MAPPING_DB_PATH="${SUBWAY_MAPPING_DB_PATH:-${REPOSITORY_DIR}/results/maps/subway_v2_rtabmap.db}"

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

require_node() {
  local node_name="$1"
  if ! ros2 node list | grep -Fxq "${node_name}"; then
    echo "Missing node: ${node_name}" >&2
    exit 1
  fi
  echo "[OK] node ${node_name}"
}

require_topic_message() {
  local topic_name="$1"
  local timeout_seconds="${2:-30}"
  if ! timeout "${timeout_seconds}" ros2 topic echo "${topic_name}" --once \
      >/dev/null 2>&1; then
    echo "No message received from ${topic_name} within ${timeout_seconds}s" >&2
    exit 1
  fi
  echo "[OK] topic ${topic_name}"
}

require_transform() {
  local parent_frame="$1"
  local child_frame="$2"
  local output
  output="$(timeout 8 ros2 run tf2_ros tf2_echo \
    "${parent_frame}" "${child_frame}" 2>&1 || true)"
  if ! grep -q "Translation:" <<<"${output}"; then
    echo "Missing TF ${parent_frame} -> ${child_frame}" >&2
    exit 1
  fi
  echo "[OK] TF ${parent_frame} -> ${child_frame}"
}

for node_name in \
  /mapping/cloud_gate \
  /mapping/icp_odometry \
  /odometry_ekf \
  /mapping/rtabmap \
  /mapping/map_assembler \
  /mapping/optimized_cloud_saver; do
  require_node "${node_name}"
done

require_topic_message /mapping/cloud_valid 60
require_topic_message /lidar/odom
require_topic_message /odometry/filtered
require_topic_message /mapping/info
require_topic_message /mapping/cloud_map 60
require_transform odom base_footprint
require_transform map odom

if ! save_response="$(timeout 45 ros2 service call \
  /mapping/save_map std_srvs/srv/Trigger '{}' 2>&1)"; then
  echo "Optimized PCD save service did not complete within 45s:" >&2
  echo "${save_response}" >&2
  exit 1
fi
if ! grep -q "success=True" <<<"${save_response}"; then
  echo "Optimized PCD save service failed:" >&2
  echo "${save_response}" >&2
  exit 1
fi
echo "[OK] service /mapping/save_map"

if [[ ! -s "${MAPPING_PCD_PATH}" ]]; then
  echo "PCD file is missing or empty: ${MAPPING_PCD_PATH}" >&2
  exit 1
fi
if [[ ! -s "${MAPPING_DB_PATH}" ]]; then
  echo "RTAB-Map database is missing or empty: ${MAPPING_DB_PATH}" >&2
  exit 1
fi
echo "[OK] PCD ${MAPPING_PCD_PATH}"
echo "[OK] database ${MAPPING_DB_PATH}"
echo "SLAM_ACCEPTANCE=PASS"

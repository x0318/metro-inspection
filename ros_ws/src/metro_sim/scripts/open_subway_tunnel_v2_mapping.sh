#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
REPOSITORY_DIR="$(cd "${ROS_WS_DIR}/.." && pwd)"
DESCRIPTION_DIR="${ROS_WS_DIR}/src/metro_description"

WORLD_FILE="${METRO_SIM_DIR}/worlds/subway_tunnel_v2_mapping.world"
URDF_FILE="${DESCRIPTION_DIR}/urdf/subway_v2.urdf"
SCALE_FILE="${METRO_SIM_DIR}/config/subway_v2_model_scale.txt"
SCALE_SCRIPT="${SCRIPT_DIR}/scale_subway_v2_urdf.py"
MAPPING_MODEL_SCRIPT="${SCRIPT_DIR}/prepare_subway_v2_mapping_model.py"
SOURCE_MODEL_SDF="${METRO_SIM_DIR}/models/subway_v2/model.sdf"
MAPPING_PCD_PATH="${SUBWAY_MAPPING_PCD_PATH:-${REPOSITORY_DIR}/results/maps/subway_v2_accumulated.pcd}"
WATCHDOG_SCRIPT="${SCRIPT_DIR}/cmd_vel_watchdog.py"
WATCHDOG_PARAMS="${METRO_SIM_DIR}/config/tunnel_guard_production.yaml"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_subway_tunnel_v2_mapping.sh [gazebo_ros launch arguments]

Starts the lidar/IMU mapping profile and the odometry-based point-cloud
accumulator without the six RGB camera sensors. Gazebo GUI is disabled by
default. Pass gui:=true only when visual inspection is needed; it reduces the
high-density GPU lidar publication rate.

Environment overrides:
  ROS_DOMAIN_ID                         ROS 2 discovery domain (default: 70)
  SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI  Gazebo master URI
                                       (default: http://127.0.0.1:11372)
  SUBWAY_MAPPING_PCD_PATH              save_map output path
                                       (default: results/maps/subway_v2_accumulated.pcd)

Examples:
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_mapping.sh gui:=true
EOF
  exit 0
fi

for required_file in \
  "${WORLD_FILE}" \
  "${URDF_FILE}" \
  "${SCALE_FILE}" \
  "${SCALE_SCRIPT}" \
  "${MAPPING_MODEL_SCRIPT}" \
  "${SOURCE_MODEL_SDF}" \
  "${WATCHDOG_SCRIPT}" \
  "${WATCHDOG_PARAMS}" \
  "${METRO_SIM_DIR}/models/subway_tunnel_v2/model.sdf"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROS_WS_DIR}/install/setup.bash" ]]; then
  source "${ROS_WS_DIR}/install/setup.bash"
fi
set -u

for required_command in python3 check_urdf ros2 gz; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Required command not found: ${required_command}" >&2
    exit 1
  fi
done

for required_package in \
  metro_localization metro_pointcloud_mapping robot_localization; do
  if ! ros2 pkg prefix "${required_package}" >/dev/null 2>&1; then
    echo "Required ROS package not found: ${required_package}" >&2
    echo "Build the workspace and install ros-humble-robot-localization." >&2
    exit 1
  fi
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
export GAZEBO_MASTER_URI="${SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI:-http://127.0.0.1:11372}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export GAZEBO_MODEL_DATABASE_URI=""

if [[ "${GAZEBO_MASTER_URI}" =~ ^http://(127\.0\.0\.1|localhost):([0-9]+)$ ]]; then
  MASTER_PORT="${BASH_REMATCH[2]}"
  if [[ -n "$(ss -ltnH "sport = :${MASTER_PORT}")" ]]; then
    echo "Gazebo master ${GAZEBO_MASTER_URI} is already in use." >&2
    echo "Close that instance or choose another port." >&2
    exit 1
  fi
fi

RUN_DIR="$(mktemp -d)"
SCALED_URDF="${RUN_DIR}/subway_v2_scaled.urdf"
MAPPING_MODEL_DIR="${RUN_DIR}/subway_v2_mapping"
ROBOT_STATE_PUBLISHER_PID=""
POINTCLOUD_MAPPING_PID=""
CMD_VEL_WATCHDOG_PID=""
ODOMETRY_FUSION_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  for child_pid in \
    "${POINTCLOUD_MAPPING_PID}" \
    "${ODOMETRY_FUSION_PID}" \
    "${CMD_VEL_WATCHDOG_PID}" \
    "${ROBOT_STATE_PUBLISHER_PID}"; do
    if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
      kill "${child_pid}" 2>/dev/null || true
      wait "${child_pid}" 2>/dev/null || true
    fi
  done
  rm -rf -- "${RUN_DIR}"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

python3 "${MAPPING_MODEL_SCRIPT}" "${SOURCE_MODEL_SDF}" "${MAPPING_MODEL_DIR}"
gz sdf -k "${MAPPING_MODEL_DIR}/model.sdf"

ROBOT_SCALE="$(tr -d '[:space:]' < "${SCALE_FILE}")"
python3 "${SCALE_SCRIPT}" \
  "${URDF_FILE}" "${SCALED_URDF}" --scale "${ROBOT_SCALE}"
check_urdf "${SCALED_URDF}" >/dev/null

export GAZEBO_MODEL_PATH="${RUN_DIR}:${METRO_SIM_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "Mapping profile: Odin1 lidar + IMU, cameras disabled, gui=false"
echo "Accumulated map topic: /mapping/cloud_map"
echo "PCD save path: ${MAPPING_PCD_PATH}"
echo "Temporary model directory: ${MAPPING_MODEL_DIR}"
echo "Odometry: /wheel/odom_raw + /odin1/imu -> /odometry/filtered"
echo "Drive safety: /cmd_vel_safe -> watchdog -> /cmd_vel_drive"

ros2 run robot_state_publisher robot_state_publisher \
  "${SCALED_URDF}" \
  --ros-args -p use_sim_time:=true &
ROBOT_STATE_PUBLISHER_PID=$!

python3 "${WATCHDOG_SCRIPT}" \
  --ros-args --params-file "${WATCHDOG_PARAMS}" &
CMD_VEL_WATCHDOG_PID=$!

ros2 launch metro_localization odometry_fusion.launch.py \
  use_sim_time:=true &
ODOMETRY_FUSION_PID=$!

ros2 launch metro_pointcloud_mapping accumulated_map.launch.py \
  pcd_path:="${MAPPING_PCD_PATH}" &
POINTCLOUD_MAPPING_PID=$!

ros2 launch gazebo_ros gazebo.launch.py \
  world:="${WORLD_FILE}" \
  verbose:=true \
  gui:=false \
  "$@"

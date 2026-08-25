#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
DESCRIPTION_DIR="${ROS_WS_DIR}/src/metro_description"

WORLD_FILE="${METRO_SIM_DIR}/worlds/subway_tunnel_v2_fusion.world"
URDF_FILE="${DESCRIPTION_DIR}/urdf/subway_v2.urdf"
SCALE_FILE="${METRO_SIM_DIR}/config/subway_v2_model_scale.txt"
SCALE_SCRIPT="${SCRIPT_DIR}/scale_subway_v2_urdf.py"
FUSION_MODEL_SCRIPT="${SCRIPT_DIR}/prepare_subway_v2_fusion_model.py"
SOURCE_MODEL_SDF="${METRO_SIM_DIR}/models/subway_v2/model.sdf"
WATCHDOG_SCRIPT="${SCRIPT_DIR}/cmd_vel_watchdog.py"
WATCHDOG_PARAMS="${METRO_SIM_DIR}/config/tunnel_guard_production.yaml"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_subway_tunnel_v2_fusion.sh [gazebo_ros launch arguments]

Starts one subway_v2 robot with Odin1 RGB, lidar, and IMU. The five unrelated
RGB cameras are removed from a temporary runtime SDF. The script also starts the
damage-fusion adapter; no second Gazebo world or robot is created.

Environment overrides:
  ROS_DOMAIN_ID                         ROS 2 discovery domain (default: 70)
  SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI  Gazebo master URI
                                       (default: http://127.0.0.1:11374)

Examples:
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_fusion.sh
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_fusion.sh gui:=true
EOF
  exit 0
fi

for required_file in \
  "${WORLD_FILE}" \
  "${URDF_FILE}" \
  "${SCALE_FILE}" \
  "${SCALE_SCRIPT}" \
  "${FUSION_MODEL_SCRIPT}" \
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
for required_package in metro_closed_loop metro_localization robot_localization; do
  if ! ros2 pkg prefix "${required_package}" >/dev/null 2>&1; then
    echo "Required ROS package not found: ${required_package}" >&2
    echo "Build the workspace and install ros-humble-robot-localization." >&2
    exit 1
  fi
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
export GAZEBO_MASTER_URI="${SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI:-http://127.0.0.1:11374}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export GAZEBO_MODEL_DATABASE_URI=""
export ALSOFT_DRIVERS="${ALSOFT_DRIVERS:-null}"

# Two Gazebo instances in one ROS domain both publish /clock, TF, and the same
# sensor topics. Reject that state before creating any temporary runtime files.
for gzserver_pid in $(pgrep -x gzserver 2>/dev/null || true); do
  if [[ ! -r "/proc/${gzserver_pid}/environ" ]]; then
    continue
  fi
  existing_domain="$({ tr '\0' '\n' < "/proc/${gzserver_pid}/environ" || true; } \
    | sed -n 's/^ROS_DOMAIN_ID=//p' | head -n 1)"
  if [[ "${existing_domain:-0}" == "${ROS_DOMAIN_ID}" ]]; then
    existing_master="$({ tr '\0' '\n' < "/proc/${gzserver_pid}/environ" || true; } \
      | sed -n 's/^GAZEBO_MASTER_URI=//p' | head -n 1)"
    echo "A gzserver is already using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2
    echo "PID=${gzserver_pid} GAZEBO_MASTER_URI=${existing_master:-unknown}" >&2
    echo "Close it before starting the fusion profile to avoid duplicate topics." >&2
    exit 1
  fi
done

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
FUSION_MODEL_DIR="${RUN_DIR}/subway_v2_fusion"
ROBOT_STATE_PUBLISHER_PID=""
FUSION_LAUNCH_PID=""
CMD_VEL_WATCHDOG_PID=""
ODOMETRY_FUSION_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  for child_pid in \
    "${FUSION_LAUNCH_PID}" \
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

python3 "${FUSION_MODEL_SCRIPT}" "${SOURCE_MODEL_SDF}" "${FUSION_MODEL_DIR}"
gz sdf -k "${FUSION_MODEL_DIR}/model.sdf"

ROBOT_SCALE="$(tr -d '[:space:]' < "${SCALE_FILE}")"
python3 "${SCALE_SCRIPT}" \
  "${URDF_FILE}" "${SCALED_URDF}" --scale "${ROBOT_SCALE}"
check_urdf "${SCALED_URDF}" >/dev/null

export GAZEBO_MODEL_PATH="${RUN_DIR}:${METRO_SIM_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "Fusion profile: Odin1 RGB + lidar + IMU, other cameras disabled"
echo "Temporary model directory: ${FUSION_MODEL_DIR}"
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

ros2 launch metro_closed_loop subway_v2_fusion.launch.py \
  run_placeholder_detector:=false &
FUSION_LAUNCH_PID=$!

ros2 launch gazebo_ros gazebo.launch.py \
  world:="${WORLD_FILE}" \
  verbose:=true \
  gui:=false \
  "$@"

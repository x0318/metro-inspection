#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
REPOSITORY_DIR="$(cd "${ROS_WS_DIR}/.." && pwd)"
DESCRIPTION_DIR="${ROS_WS_DIR}/src/metro_description"

WORLD_FILE="${METRO_SIM_DIR}/worlds/subway_tunnel_v2_sensors.world"
URDF_FILE="${DESCRIPTION_DIR}/urdf/subway_v2.urdf"
SCALE_FILE="${METRO_SIM_DIR}/config/subway_v2_model_scale.txt"
SCALE_SCRIPT="${SCRIPT_DIR}/scale_subway_v2_urdf.py"
WATCHDOG_SCRIPT="${SCRIPT_DIR}/cmd_vel_watchdog.py"
WATCHDOG_PARAMS="${METRO_SIM_DIR}/config/tunnel_guard_production.yaml"
PITCH_COMMAND_SCRIPT="${SCRIPT_DIR}/set_subway_v2_pitch.sh"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_subway_tunnel_v2_sensors.sh [gazebo_ros launch arguments]

Environment overrides:
  ROS_DOMAIN_ID                         ROS 2 discovery domain (default: 70)
  SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI  Gazebo master URI
                                       (default: http://127.0.0.1:11370)
  SUBWAY_V2_INITIAL_PITCH_DEG          Pitch joint start command, -15 to +45
                                       (default: +45; upper-wall view: 30 deg)

Examples:
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh gui:=false
  SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI=http://127.0.0.1:11371 \
    ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh
EOF
  exit 0
fi

for required_file in \
  "${WORLD_FILE}" \
  "${URDF_FILE}" \
  "${SCALE_FILE}" \
  "${SCALE_SCRIPT}" \
  "${WATCHDOG_SCRIPT}" \
  "${WATCHDOG_PARAMS}" \
  "${PITCH_COMMAND_SCRIPT}" \
  "${METRO_SIM_DIR}/models/subway_tunnel_v2/model.sdf" \
  "${METRO_SIM_DIR}/models/subway_v2/model.sdf"; do
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

for required_package in \
  controller_manager \
  gazebo_ros2_control \
  metro_closed_loop \
  metro_localization \
  position_controllers \
  robot_localization; do
  if ! ros2 pkg prefix "${required_package}" >/dev/null 2>&1; then
    echo "Required ROS package not found: ${required_package}" >&2
    echo "Build the workspace and install ros-humble-robot-localization." >&2
    exit 1
  fi
done

for required_command in python3 check_urdf ros2; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Required command not found: ${required_command}" >&2
    exit 1
  fi
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
export GAZEBO_MASTER_URI="${SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI:-http://127.0.0.1:11370}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export GAZEBO_MODEL_PATH="${METRO_SIM_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
# All models are local; skip the legacy online model database lookup.
export GAZEBO_MODEL_DATABASE_URI=""

if [[ "${GAZEBO_MASTER_URI}" =~ ^http://(127\.0\.0\.1|localhost):([0-9]+)$ ]]; then
  MASTER_PORT="${BASH_REMATCH[2]}"
  if [[ -n "$(ss -ltnH "sport = :${MASTER_PORT}")" ]]; then
    echo "Gazebo master ${GAZEBO_MASTER_URI} is already in use." >&2
    echo "Close that Gazebo instance or select another port, for example:" >&2
    echo "  SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI=http://127.0.0.1:11371 $0" >&2
    exit 1
  fi
fi

RUN_DIR="$(mktemp -d)"
SCALED_URDF="${RUN_DIR}/subway_v2_scaled.urdf"
ROBOT_STATE_PUBLISHER_PID=""
CAMERA_INFO_CALIBRATOR_PID=""
CMD_VEL_WATCHDOG_PID=""
ODOMETRY_FUSION_PID=""
PITCH_INITIALIZER_PID=""
PITCH_CONTROLLER_SPAWNER_PID=""
GAZEBO_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  local child_pid
  local children=(
    "${GAZEBO_PID}" \
    "${PITCH_INITIALIZER_PID}" \
    "${PITCH_CONTROLLER_SPAWNER_PID}" \
    "${ODOMETRY_FUSION_PID}" \
    "${CAMERA_INFO_CALIBRATOR_PID}" \
    "${CMD_VEL_WATCHDOG_PID}" \
    "${ROBOT_STATE_PUBLISHER_PID}"
  )
  for child_pid in "${children[@]}"; do
    if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
      # ros2 run assumes its executable receives terminal signals as well.
      if [[ "${child_pid}" == "${ROBOT_STATE_PUBLISHER_PID}" ||
            "${child_pid}" == "${CAMERA_INFO_CALIBRATOR_PID}" ||
            "${child_pid}" == "${PITCH_CONTROLLER_SPAWNER_PID}" ]]; then
        pkill -INT -P "${child_pid}" 2>/dev/null || true
      fi
      kill -INT "${child_pid}" 2>/dev/null || true
    fi
  done
  for child_pid in "${children[@]}"; do
    if [[ -n "${child_pid}" ]]; then
      wait "${child_pid}" 2>/dev/null || true
    fi
  done
  rm -rf -- "${RUN_DIR}"
  exit "${exit_code}"
}
trap cleanup EXIT
trap 'exit 0' INT
trap 'exit 143' TERM

ROBOT_SCALE="$(tr -d '[:space:]' < "${SCALE_FILE}")"
python3 "${SCALE_SCRIPT}" \
  "${URDF_FILE}" "${SCALED_URDF}" --scale "${ROBOT_SCALE}"
check_urdf "${SCALED_URDF}" >/dev/null

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "Robot scale=${ROBOT_SCALE}"
echo "Starting robot_state_publisher with ${SCALED_URDF}"
echo "Odometry: /wheel/odom_raw + /odin1/imu -> /odometry/filtered"
echo "Drive safety: /cmd_vel_safe -> watchdog -> /cmd_vel_drive"

ros2 run robot_state_publisher robot_state_publisher \
  "${SCALED_URDF}" \
  --ros-args -p use_sim_time:=true &
ROBOT_STATE_PUBLISHER_PID=$!

ros2 run metro_closed_loop camera_info_calibrator \
  --ros-args -p use_sim_time:=true &
CAMERA_INFO_CALIBRATOR_PID=$!

python3 "${WATCHDOG_SCRIPT}" \
  --ros-args --params-file "${WATCHDOG_PARAMS}" &
CMD_VEL_WATCHDOG_PID=$!

env --default-signal=INT ros2 launch metro_localization odometry_fusion.launch.py \
  use_sim_time:=true &
ODOMETRY_FUSION_PID=$!

ros2 run controller_manager spawner pitch_position_controller \
  --controller-manager /subway_v2/controller_manager \
  --controller-manager-timeout 60 &
PITCH_CONTROLLER_SPAWNER_PID=$!

"${PITCH_COMMAND_SCRIPT}" "${SUBWAY_V2_INITIAL_PITCH_DEG:-45}" &
PITCH_INITIALIZER_PID=$!

echo "Opening ${WORLD_FILE}"
cd "${REPOSITORY_DIR}"
env --default-signal=INT ros2 launch gazebo_ros gazebo.launch.py \
  world:="${WORLD_FILE}" \
  server_required:=true \
  verbose:=true \
  "$@" &
GAZEBO_PID=$!

# Successful one-shot initialization is expected; persistent services must stay alive.
WATCHED_PIDS=(
  "${GAZEBO_PID}" "${ROBOT_STATE_PUBLISHER_PID}" "${CAMERA_INFO_CALIBRATOR_PID}"
  "${CMD_VEL_WATCHDOG_PID}" "${ODOMETRY_FUSION_PID}"
  "${PITCH_CONTROLLER_SPAWNER_PID}" "${PITCH_INITIALIZER_PID}"
)
while ((${#WATCHED_PIDS[@]})); do
  finished_pid=""
  exit_code=0
  wait -n -p finished_pid "${WATCHED_PIDS[@]}" || exit_code=$?
  if [[ "${exit_code}" == "0" && (
    "${finished_pid}" == "${PITCH_CONTROLLER_SPAWNER_PID}" ||
    "${finished_pid}" == "${PITCH_INITIALIZER_PID}"
  ) ]]; then
    remaining=()
    for child_pid in "${WATCHED_PIDS[@]}"; do
      [[ "${child_pid}" == "${finished_pid}" ]] || remaining+=("${child_pid}")
    done
    WATCHED_PIDS=("${remaining[@]}")
    continue
  fi
  echo "[ERROR] Simulation service PID ${finished_pid:-unknown} exited with code ${exit_code}." >&2
  exit 1
done

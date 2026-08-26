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
MAPPING_PCD_PATH="${SUBWAY_MAPPING_PCD_PATH:-${REPOSITORY_DIR}/results/maps/subway_v2_optimized.pcd}"
MAPPING_DB_PATH="${SUBWAY_MAPPING_DB_PATH:-${REPOSITORY_DIR}/results/maps/subway_v2_rtabmap.db}"
MAPPING_RESET_DATABASE="${SUBWAY_MAPPING_RESET_DATABASE:-true}"
WATCHDOG_SCRIPT="${SCRIPT_DIR}/cmd_vel_watchdog.py"
WATCHDOG_PARAMS="${METRO_SIM_DIR}/config/tunnel_guard_production.yaml"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_subway_tunnel_v2_mapping.sh [gazebo_ros launch arguments]

Starts the lidar/IMU mapping profile, ICP odometry, local EKF, RTAB-Map pose
graph, loop closure, optimized cloud assembler, and PCD saver. The six RGB
cameras are disabled. Gazebo GUI is disabled by default; pass gui:=true only
when visual inspection is needed.

Environment overrides:
  ROS_DOMAIN_ID                         ROS 2 discovery domain (default: 70)
  SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI  Gazebo master URI
                                       (default: http://127.0.0.1:11372)
  SUBWAY_MAPPING_PCD_PATH              save_map output path
                                       (default: results/maps/subway_v2_optimized.pcd)
  SUBWAY_MAPPING_DB_PATH               persistent RTAB-Map graph database
                                       (default: results/maps/subway_v2_rtabmap.db)
  SUBWAY_MAPPING_RESET_DATABASE        true starts a new graph; false resumes
                                       the database (default: true)

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

for required_command in python3 check_urdf ros2 gz ldd; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Required command not found: ${required_command}" >&2
    exit 1
  fi
done

for required_package in \
  metro_localization \
  metro_pointcloud_mapping \
  robot_localization \
  rtabmap_odom \
  rtabmap_slam \
  rtabmap_util; do
  if ! ros2 pkg prefix "${required_package}" >/dev/null 2>&1; then
    echo "Required ROS package not found: ${required_package}" >&2
    echo "Build the workspace and install runtime dependencies:" >&2
    echo "  sudo apt install ros-humble-robot-localization ros-humble-rtabmap-ros" >&2
    exit 1
  fi
done

# The Hikvision MVS SDK ships an older libusb in /opt/MVS. If MVS appears first
# in LD_LIBRARY_PATH, PCL fails at startup because libusb_set_option is missing.
# Mapping mode disables the cameras, so prefer Ubuntu system libraries only for
# the graph-SLAM process while leaving the caller's environment unchanged.
MAPPING_LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
ICP_ODOMETRY_BINARY="$(ros2 pkg prefix rtabmap_odom)/lib/rtabmap_odom/icp_odometry"
LINKAGE_ERRORS="$(
  env LD_LIBRARY_PATH="${MAPPING_LD_LIBRARY_PATH}" \
    ldd -r "${ICP_ODOMETRY_BINARY}" 2>&1 | \
    grep -E 'not found|undefined symbol' || true
)"
if [[ -n "${LINKAGE_ERRORS}" ]]; then
  echo "RTAB-Map runtime dependency check failed:" >&2
  echo "${LINKAGE_ERRORS}" >&2
  exit 1
fi

case "${MAPPING_RESET_DATABASE,,}" in
  true|false|1|0|yes|no|on|off) ;;
  *)
    echo "SUBWAY_MAPPING_RESET_DATABASE must be true or false." >&2
    exit 1
    ;;
esac

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
GRAPH_SLAM_PID=""
CMD_VEL_WATCHDOG_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  for child_pid in \
    "${GRAPH_SLAM_PID}" \
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
install -d -- "$(dirname -- "${MAPPING_PCD_PATH}")"
install -d -- "$(dirname -- "${MAPPING_DB_PATH}")"

export GAZEBO_MODEL_PATH="${RUN_DIR}:${METRO_SIM_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "Mapping profile: Odin1 lidar + IMU, cameras disabled, gui=false"
echo "SLAM: 3D ICP + robust pose graph + proximity loop closure"
echo "Optimized map topic: /mapping/cloud_map"
echo "Fused odometry: /wheel/odom_raw + /odin1/imu + /lidar/odom"
echo "TF owners: EKF odom -> base_footprint; RTAB-Map map -> odom"
echo "PCD save path: ${MAPPING_PCD_PATH}"
echo "RTAB-Map database: ${MAPPING_DB_PATH}"
echo "Reset database: ${MAPPING_RESET_DATABASE}"
echo "RTAB-Map runtime: Ubuntu system libraries take priority over MVS"
echo "Temporary model directory: ${MAPPING_MODEL_DIR}"
echo "Drive safety: /cmd_vel_safe -> watchdog -> /cmd_vel_drive"

ros2 run robot_state_publisher robot_state_publisher \
  "${SCALED_URDF}" \
  --ros-args -p use_sim_time:=true &
ROBOT_STATE_PUBLISHER_PID=$!

python3 "${WATCHDOG_SCRIPT}" \
  --ros-args --params-file "${WATCHDOG_PARAMS}" &
CMD_VEL_WATCHDOG_PID=$!

env LD_LIBRARY_PATH="${MAPPING_LD_LIBRARY_PATH}" \
  ros2 launch metro_pointcloud_mapping graph_slam.launch.py \
  use_sim_time:=true \
  database_path:="${MAPPING_DB_PATH}" \
  pcd_path:="${MAPPING_PCD_PATH}" \
  reset_database:="${MAPPING_RESET_DATABASE}" &
GRAPH_SLAM_PID=$!

ros2 launch gazebo_ros gazebo.launch.py \
  world:="${WORLD_FILE}" \
  verbose:=true \
  gui:=false \
  "$@"

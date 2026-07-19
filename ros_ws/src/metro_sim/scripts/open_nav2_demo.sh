#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_FILE="${PROJECT_DIR}/worlds/subway_track_tunnel.world"
URDF_FILE="${PROJECT_DIR}/urdf/gazebo_train_tf.urdf"
RVIZ_FILE="${PROJECT_DIR}/config/gazebo_lidar.rviz"
NAV2_PARAMS="${PROJECT_DIR}/config/nav2_odom_params.yaml"
TUNNEL_GUARD_PROFILE="${TUNNEL_GUARD_PROFILE:-demo}"
TUNNEL_GUARD_PARAMS="${PROJECT_DIR}/config/tunnel_guard_${TUNNEL_GUARD_PROFILE}.yaml"

if [[ ! -f "${TUNNEL_GUARD_PARAMS}" ]]; then
  echo "Unknown tunnel guard profile: ${TUNNEL_GUARD_PROFILE}" >&2
  echo "Expected parameter file: ${TUNNEL_GUARD_PARAMS}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

export GAZEBO_MODEL_PATH="${PROJECT_DIR}/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"

cleanup() {
  if [[ -n "${GAZEBO_CLIENT_PID:-}" ]]; then kill "${GAZEBO_CLIENT_PID}" 2>/dev/null || true; fi
  if [[ -n "${GAZEBO_SERVER_PID:-}" ]]; then kill "${GAZEBO_SERVER_PID}" 2>/dev/null || true; fi
  if [[ -n "${GUARD_PID:-}" ]]; then kill "${GUARD_PID}" 2>/dev/null || true; fi
  if [[ -n "${WATCHDOG_PID:-}" ]]; then kill "${WATCHDOG_PID}" 2>/dev/null || true; fi
  if [[ -n "${RSP_PID:-}" ]]; then kill "${RSP_PID}" 2>/dev/null || true; fi
  if [[ -n "${NAV2_PID:-}" ]]; then kill "${NAV2_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT

# Humble's top-level gazebo.launch.py does not forward world arguments, so launch
# the ROS-enabled server and GUI client explicitly.
ros2 launch gazebo_ros gzserver.launch.py \
  world:="${WORLD_FILE}" \
  verbose:=false &
GAZEBO_SERVER_PID=$!

ros2 launch gazebo_ros gzclient.launch.py &
GAZEBO_CLIENT_PID=$!

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p "robot_description:=$(tr '\n' ' ' < "${URDF_FILE}")" &
RSP_PID=$!

python3 "${PROJECT_DIR}/scripts/cmd_vel_watchdog.py" \
  --ros-args \
  --params-file "${TUNNEL_GUARD_PARAMS}" &
WATCHDOG_PID=$!

python3 "${PROJECT_DIR}/scripts/tunnel_obstacle_guard.py" \
  --ros-args \
  --params-file "${TUNNEL_GUARD_PARAMS}" &
GUARD_PID=$!

sleep 3

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  params_file:="${NAV2_PARAMS}" \
  > /tmp/subway_nav2.log 2>&1 &
NAV2_PID=$!

sleep 5

rviz2 -d "${RVIZ_FILE}" --ros-args -p use_sim_time:=true

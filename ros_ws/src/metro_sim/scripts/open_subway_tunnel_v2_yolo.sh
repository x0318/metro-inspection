#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
PROJECT_DIR="$(cd "${ROS_WS_DIR}/.." && pwd)"
YOLO_SCRIPT="${PROJECT_DIR}/scripts/open_yolo_detector.sh"
FUSION_SCRIPT="${SCRIPT_DIR}/open_subway_tunnel_v2_fusion.sh"
VENV_DIR="${METRO_YOLO_VENV:-${PROJECT_DIR}/.venv-yolo}"
MODEL_PATH="${METRO_YOLO_MODEL_PATH:-/home/jo/incoming/yolov8n_sim_demo_best(1).pt}"
FRONT_YOLO_PID=""
PITCH_YOLO_PID=""

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_subway_tunnel_v2_yolo.sh [gazebo_ros launch arguments]

Starts the fusion simulation, Odin1 image/lidar localization adapter, and two
YOLO detectors. Odin1 covers the forward view; Pitch covers the tunnel crown.
Configure YOLO with the METRO_YOLO_* environment variables documented by
scripts/open_yolo_detector.sh.

Examples:
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_yolo.sh
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_yolo.sh gui:=true
EOF
  exit 0
fi

for required_file in "${YOLO_SCRIPT}" "${FUSION_SCRIPT}" "${MODEL_PATH}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "YOLO environment not found: ${VENV_DIR}" >&2
  echo "Run ${PROJECT_DIR}/scripts/setup_yolo_environment.sh first." >&2
  exit 1
fi

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  for child_pid in "${FRONT_YOLO_PID}" "${PITCH_YOLO_PID}"; do
    if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
      kill "${child_pid}" 2>/dev/null || true
      wait "${child_pid}" 2>/dev/null || true
    fi
  done
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

export METRO_YOLO_MODEL_PATH="${MODEL_PATH}"
if [[ "${METRO_YOLO_SKIP_BUILD:-0}" != "1" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  (
    cd "${ROS_WS_DIR}"
    colcon build --symlink-install --packages-select metro_detection
  )
fi

METRO_YOLO_SKIP_BUILD=1 \
METRO_YOLO_NODE_NAME=yolo_detector_front \
METRO_YOLO_IMAGE_TOPIC=/odin1/rgb/image_raw \
METRO_YOLO_DETECTIONS_TOPIC=/damage_detections \
METRO_YOLO_ANNOTATED_IMAGE_TOPIC=/damage_detection/annotated_image \
  "${YOLO_SCRIPT}" &
FRONT_YOLO_PID=$!

METRO_YOLO_SKIP_BUILD=1 \
METRO_YOLO_NODE_NAME=yolo_detector_pitch \
METRO_YOLO_IMAGE_TOPIC=/subway_v2/pitch_camera/image_raw \
METRO_YOLO_DETECTIONS_TOPIC=/damage_detections/pitch \
METRO_YOLO_ANNOTATED_IMAGE_TOPIC=/damage_detection/pitch/annotated_image \
  "${YOLO_SCRIPT}" &
PITCH_YOLO_PID=$!

# The detectors may start before Gazebo; that is valid because they wait for
# their image topics. Catch immediate setup failures before starting Gazebo.
sleep 1
for child_pid in "${FRONT_YOLO_PID}" "${PITCH_YOLO_PID}"; do
  if ! kill -0 "${child_pid}" 2>/dev/null; then
    wait "${child_pid}"
  fi
done

"${FUSION_SCRIPT}" "$@"

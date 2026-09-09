#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="${PROJECT_DIR}/ros_ws"
VENV_DIR="${METRO_YOLO_VENV:-${PROJECT_DIR}/.venv-yolo}"
COMPAT_DIR="${METRO_YOLO_COMPAT_DIR:-${PROJECT_DIR}/.yolo-ros-compat}"
MODEL_PATH="${METRO_YOLO_MODEL_PATH:-/home/jo/incoming/yolov8n_sim_demo_best(1).pt}"
PUBLISHER_WAITER="${SCRIPT_DIR}/wait_for_ros_publishers.py"
IMAGE_TOPICS=(
  /subway_v2/xj1/image_raw
  /subway_v2/xj2/image_raw
  /subway_v2/xj3/image_raw
  /subway_v2/xj4/image_raw
  /subway_v2/pitch_camera/image_raw
  /odin1/rgb/image_raw
)

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_yolo_coverage.sh [additional ros2 launch arguments]

Runs one shared YOLO model against XJ1-XJ4, Pitch, and the localization-only
Odin1 RGB stream. The five monitor cameras evaluate ten-site coverage; Odin1
provides detections aligned with its lidar for 3D localization.

Environment overrides:
  METRO_YOLO_MODEL_PATH  YOLO checkpoint path
  METRO_YOLO_CONFIDENCE  Confidence threshold (default: 0.35)
  METRO_YOLO_DEVICE      auto, cpu, or CUDA index (default: auto)
  METRO_YOLO_MAX_RATE    Maximum rate per camera in Hz (default: 3.0)
  METRO_YOLO_TOPIC_WAIT_TIMEOUT
                         Seconds to wait for all image publishers (default: 90)
  METRO_COVERAGE_AUTO_DRIVE
                         Start the bounded automatic pass (default: 0)
  METRO_YOLO_SKIP_BUILD  Set to 1 to skip the package build
  ROS_DOMAIN_ID          ROS 2 discovery domain (default: 70)
EOF
  exit 0
fi

for required_file in \
  "${MODEL_PATH}" \
  "${VENV_DIR}/bin/python" \
  "${PUBLISHER_WAITER}"; do
  if [[ ! -e "${required_file}" ]]; then
    echo "Required YOLO file not found: ${required_file}" >&2
    echo "Run ${PROJECT_DIR}/scripts/setup_yolo_environment.sh if needed." >&2
    exit 1
  fi
done
if [[ ! -d "${COMPAT_DIR}/numpy" || ! -d "${COMPAT_DIR}/cv2" ]]; then
  echo "ROS NumPy compatibility overlay not found: ${COMPAT_DIR}" >&2
  echo "Run ${PROJECT_DIR}/scripts/setup_yolo_environment.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ "${METRO_YOLO_SKIP_BUILD:-0}" != "1" ]]; then
  (
    cd "${ROS_WS_DIR}"
    colcon build --symlink-install --packages-up-to metro_detection
  )
fi
set +u
source "${ROS_WS_DIR}/install/setup.bash"
set -u

VENV_SITE_PACKAGES="$("${VENV_DIR}/bin/python" -c \
  'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${COMPAT_DIR}:${VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "YOLO model=${MODEL_PATH}"
echo "Coverage: five cameras -> /simulation/defect_coverage"
echo "Simulation events: /simulation/defect_events"
echo "Localization detections: /damage_detections/odin1"
echo "Waiting for actual frames from six Gazebo cameras before starting YOLO"

python3 "${PUBLISHER_WAITER}" \
  --images \
  --timeout "${METRO_YOLO_TOPIC_WAIT_TIMEOUT:-90}" \
  "${IMAGE_TOPICS[@]}"

exec ros2 launch metro_detection yolov8_coverage.launch.py \
  model_path:="${MODEL_PATH}" \
  confidence_threshold:="${METRO_YOLO_CONFIDENCE:-0.35}" \
  device:="${METRO_YOLO_DEVICE:-auto}" \
  max_inference_rate_hz:="${METRO_YOLO_MAX_RATE:-3.0}" \
  auto_drive:="${METRO_COVERAGE_AUTO_DRIVE:-false}" \
  "$@"

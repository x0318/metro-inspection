#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="${PROJECT_DIR}/ros_ws"
VENV_DIR="${METRO_YOLO_VENV:-${PROJECT_DIR}/.venv-yolo}"
COMPAT_DIR="${METRO_YOLO_COMPAT_DIR:-${PROJECT_DIR}/.yolo-ros-compat}"
MODEL_PATH="${METRO_YOLO_MODEL_PATH:-/home/jo/incoming/best.pt}"
PUBLISHER_WAITER="${SCRIPT_DIR}/wait_for_ros_publishers.py"
IMAGE_TOPIC="${METRO_YOLO_IMAGE_TOPIC:-/odin1/rgb/image_raw}"
DETECTIONS_TOPIC="${METRO_YOLO_DETECTIONS_TOPIC:-/damage_detections}"
ANNOTATED_IMAGE_TOPIC="${METRO_YOLO_ANNOTATED_IMAGE_TOPIC:-/damage_detection/annotated_image}"
NODE_NAME="${METRO_YOLO_NODE_NAME:-yolo_detector}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_yolo_detector.sh [additional ros2 launch arguments]

Environment overrides:
  METRO_YOLO_MODEL_PATH    Checkpoint path
  METRO_YOLO_IMAGE_TOPIC   Raw ROS image topic (default: /odin1/rgb/image_raw)
  METRO_YOLO_DETECTIONS_TOPIC
                           Detection output topic (default: /damage_detections)
  METRO_YOLO_ANNOTATED_IMAGE_TOPIC
                           Annotated image output topic
  METRO_YOLO_NODE_NAME     ROS node name (default: yolo_detector)
  METRO_YOLO_CONFIDENCE    Confidence threshold (default: 0.35)
  METRO_YOLO_DEVICE        auto, cpu, or CUDA index such as 0 (default: auto)
  METRO_YOLO_MAX_RATE      Maximum inference rate in Hz (default: 10.0)
  METRO_YOLO_TOPIC_WAIT_TIMEOUT
                           Seconds to wait for the image publisher (default: 90)
  METRO_YOLO_VENV          Python environment created by setup script
  METRO_YOLO_COMPAT_DIR    ROS NumPy compatibility overlay
  METRO_YOLO_SKIP_BUILD    Set to 1 to skip the package build
  ROS_DOMAIN_ID            ROS 2 discovery domain (default: 70)
EOF
  exit 0
fi

if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "YOLO checkpoint not found: ${MODEL_PATH}" >&2
  exit 1
fi
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "YOLO environment not found: ${VENV_DIR}" >&2
  echo "Run ${PROJECT_DIR}/scripts/setup_yolo_environment.sh first." >&2
  exit 1
fi
if [[ ! -f "${PUBLISHER_WAITER}" ]]; then
  echo "ROS publisher waiter not found: ${PUBLISHER_WAITER}" >&2
  exit 1
fi
if [[ ! -d "${COMPAT_DIR}/numpy" || ! -d "${COMPAT_DIR}/cv2" ]]; then
  echo "ROS NumPy compatibility overlay not found: ${COMPAT_DIR}" >&2
  echo "Run ${PROJECT_DIR}/scripts/setup_yolo_environment.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "${METRO_YOLO_SKIP_BUILD:-0}" != "1" ]]; then
  cd "${ROS_WS_DIR}"
  colcon build --symlink-install --packages-select metro_detection
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
echo "YOLO node=${NODE_NAME}"
echo "YOLO image=${IMAGE_TOPIC}"
echo "YOLO detections=${DETECTIONS_TOPIC}"
echo "YOLO annotated image=${ANNOTATED_IMAGE_TOPIC}"
echo "Waiting for Gazebo camera publisher ${IMAGE_TOPIC} before starting YOLO"

python3 "${PUBLISHER_WAITER}" \
  --timeout "${METRO_YOLO_TOPIC_WAIT_TIMEOUT:-90}" \
  "${IMAGE_TOPIC}"

exec ros2 launch metro_detection yolov8_detector.launch.py \
  node_name:="${NODE_NAME}" \
  model_path:="${MODEL_PATH}" \
  image_topic:="${IMAGE_TOPIC}" \
  detections_topic:="${DETECTIONS_TOPIC}" \
  annotated_image_topic:="${ANNOTATED_IMAGE_TOPIC}" \
  confidence_threshold:="${METRO_YOLO_CONFIDENCE:-0.35}" \
  device:="${METRO_YOLO_DEVICE:-auto}" \
  max_inference_rate_hz:="${METRO_YOLO_MAX_RATE:-10.0}" \
  "$@"

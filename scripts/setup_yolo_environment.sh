#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${METRO_YOLO_VENV:-${PROJECT_DIR}/.venv-yolo}"
COMPAT_DIR="${METRO_YOLO_COMPAT_DIR:-${PROJECT_DIR}/.yolo-ros-compat}"
REQUIREMENTS_FILE="${PROJECT_DIR}/ros_ws/src/metro_detection/requirements-yolo.txt"

if [[ ! -f "${REQUIREMENTS_FILE}" ]]; then
  echo "YOLO requirements file not found: ${REQUIREMENTS_FILE}" >&2
  exit 1
fi

if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "python3-venv is required. Install it with:" >&2
  echo "  sudo apt install python3-venv" >&2
  exit 1
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Creating YOLO environment: ${VENV_DIR}"
  python3 -m venv --system-site-packages "${VENV_DIR}"
fi

if ! "${VENV_DIR}/bin/python" -c 'import torch, ultralytics' \
  >/dev/null 2>&1; then
  "${VENV_DIR}/bin/python" -m pip install --upgrade \
    'pip<26' 'setuptools<80' wheel

  if command -v nvidia-smi >/dev/null 2>&1 \
    && nvidia-smi >/dev/null 2>&1; then
    echo "NVIDIA GPU detected; installing CUDA 12.8 PyTorch."
    "${VENV_DIR}/bin/python" -m pip install \
      --index-url https://download.pytorch.org/whl/cu128 \
      torch torchvision
  fi

  "${VENV_DIR}/bin/python" -m pip install -r "${REQUIREMENTS_FILE}"
else
  echo "Reusing the existing PyTorch/Ultralytics environment: ${VENV_DIR}"
fi

# ROS 2 Humble's cv_bridge binary uses the NumPy 1.x C API. Keep this small
# overlay ahead of a reusable YOLO environment that may legitimately use
# NumPy 2.x for unrelated work.
mkdir -p "${COMPAT_DIR}"
"${VENV_DIR}/bin/python" -m pip install \
  --upgrade --target "${COMPAT_DIR}" --no-deps \
  'numpy==1.26.4' 'opencv-python==4.11.0.86'

VENV_SITE_PACKAGES="$("${VENV_DIR}/bin/python" -c \
  'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${COMPAT_DIR}:${VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

set +u
source /opt/ros/humble/setup.bash
set -u
"${VENV_DIR}/bin/python" - <<'PY'
import cv2
import cv_bridge
import numpy as np
import torch
import ultralytics
from cv_bridge import CvBridge

print(f"ultralytics={ultralytics.__version__}")
print(f"torch={torch.__version__}")
print(f"opencv={cv2.__version__}")
print(f"numpy={np.__version__}")
print(f"cuda_available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"cuda_device={torch.cuda.get_device_name(0)}")
print(f"cv_bridge={cv_bridge.__file__}")
bridge = CvBridge()
message = bridge.cv2_to_imgmsg(np.zeros((8, 8, 3), dtype=np.uint8), "bgr8")
assert bridge.imgmsg_to_cv2(message, "bgr8").shape == (8, 8, 3)
print("cv_bridge_image_round_trip=ok")
PY

echo "YOLO environment is ready: ${VENV_DIR}"
echo "ROS NumPy compatibility overlay: ${COMPAT_DIR}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DESCRIPTION_DIR="$(cd "${METRO_SIM_DIR}/../metro_description" && pwd)"
URDF_FILE="${DESCRIPTION_DIR}/urdf/subway_v2.urdf"
SOURCE_MESH_DIR="${DESCRIPTION_DIR}/meshes"
MODEL_DIR="${METRO_SIM_DIR}/models/subway_v2"
MODEL_MESH_DIR="${MODEL_DIR}/meshes"
SDF_FILE="${MODEL_DIR}/model.sdf"
SCALE_FILE="${METRO_SIM_DIR}/config/subway_v2_model_scale.txt"

GENERATED_FILE="$(mktemp)"
STAGED_FILE="$(mktemp)"
SCALED_URDF="$(mktemp)"
cleanup() {
  rm -f -- "${GENERATED_FILE}" "${STAGED_FILE}" "${SCALED_URDF}"
}
trap cleanup EXIT

if ! command -v check_urdf >/dev/null 2>&1 || ! command -v gz >/dev/null 2>&1; then
  echo "check_urdf and Gazebo's gz command are required." >&2
  exit 1
fi
if ! command -v rsync >/dev/null 2>&1; then
  echo "rsync is required to synchronize Gazebo meshes." >&2
  exit 1
fi

ROBOT_SCALE="$(tr -d '[:space:]' < "${SCALE_FILE}")"
python3 "${SCRIPT_DIR}/scale_subway_v2_urdf.py" \
  "${URDF_FILE}" "${SCALED_URDF}" --scale "${ROBOT_SCALE}"
check_urdf "${SCALED_URDF}" >/dev/null
gz sdf -p "${SCALED_URDF}" > "${GENERATED_FILE}"
python3 "${SCRIPT_DIR}/configure_subway_v2_sdf.py" \
  "${GENERATED_FILE}" "${STAGED_FILE}" --urdf "${SCALED_URDF}"
if grep -q 'metro_description/meshes' "${STAGED_FILE}"; then
  echo "Generated SDF still contains unresolved package mesh URIs." >&2
  exit 1
fi
gz sdf -k "${STAGED_FILE}"

install -d "${MODEL_MESH_DIR}"
rsync --archive --checksum --delete "${SOURCE_MESH_DIR}/" "${MODEL_MESH_DIR}/"
install -m 0644 "${STAGED_FILE}" "${SDF_FILE}"

echo "Generated ${SDF_FILE} at scale ${ROBOT_SCALE}"

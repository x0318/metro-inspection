#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORLD_FILE="${METRO_SIM_DIR}/worlds/subway_tunnel_v2_preview.world"
ROBOT_MODEL_DIR="${METRO_SIM_DIR}/models/subway_v2"
PREVIEW_MODEL_SCRIPT="${SCRIPT_DIR}/prepare_subway_v2_preview_model.py"

set +u
source /opt/ros/humble/setup.bash
set -u

export GAZEBO_MASTER_URI="${SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI:-http://127.0.0.1:11362}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
# This preview uses local models only; avoid waiting for the legacy online database.
export GAZEBO_MODEL_DATABASE_URI=""

if [[ ! -f "${METRO_SIM_DIR}/models/subway_tunnel_v2/model.sdf" ]]; then
  echo "subway_tunnel_v2/model.sdf was not found under ${METRO_SIM_DIR}/models." >&2
  exit 1
fi

if [[ ! -f "${ROBOT_MODEL_DIR}/model.sdf" ]]; then
  echo "subway_v2/model.sdf was not found under ${METRO_SIM_DIR}/models." >&2
  exit 1
fi

PREVIEW_ROOT="$(mktemp -d)"
cleanup() {
  rm -rf -- "${PREVIEW_ROOT}"
}
trap cleanup EXIT

python3 "${PREVIEW_MODEL_SCRIPT}" \
  "${ROBOT_MODEL_DIR}/model.sdf" \
  "${ROBOT_MODEL_DIR}/model.config" \
  "${PREVIEW_ROOT}/subway_v2_collision_preview"

export GAZEBO_MODEL_PATH="${PREVIEW_ROOT}:${METRO_SIM_DIR}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

if [[ "${GAZEBO_MASTER_URI}" =~ ^http://(127\.0\.0\.1|localhost):([0-9]+)$ ]]; then
  MASTER_PORT="${BASH_REMATCH[2]}"
  if [[ -n "$(ss -ltnH "sport = :${MASTER_PORT}")" ]]; then
    echo "Gazebo master ${GAZEBO_MASTER_URI} is already in use." >&2
    echo "Close that Gazebo process or set SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI to another port." >&2
    exit 1
  fi
fi

echo "Opening subway_tunnel_v2 with subway_v2 on ${GAZEBO_MASTER_URI}."
gazebo --verbose "${WORLD_FILE}" "$@"

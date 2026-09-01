#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METRO_SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DIR="$(cd "${METRO_SIM_DIR}/../.." && pwd)"
PROJECT_DIR="$(cd "${ROS_WS_DIR}/.." && pwd)"
YOLO_SCRIPT="${PROJECT_DIR}/scripts/open_yolo_coverage.sh"
SENSORS_SCRIPT="${SCRIPT_DIR}/open_subway_tunnel_v2_sensors.sh"
COVERAGE_PID=""

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: open_subway_tunnel_v2_yolo_coverage.sh [gazebo_ros launch arguments]

Starts the full sensor simulation, one shared five-camera YOLO detector, and the
truth-assisted ten-site coverage evaluator. The bounded automatic pass stops
at 10/10 or x=31 m. Set METRO_COVERAGE_AUTO_DRIVE=false for manual control.

Example:
  ./ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_yolo_coverage.sh gui:=true
EOF
  exit 0
fi

for required_file in "${YOLO_SCRIPT}" "${SENSORS_SCRIPT}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${COVERAGE_PID}" ]] && kill -0 "${COVERAGE_PID}" 2>/dev/null; then
    kill "${COVERAGE_PID}" 2>/dev/null || true
    wait "${COVERAGE_PID}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

METRO_COVERAGE_AUTO_DRIVE="${METRO_COVERAGE_AUTO_DRIVE:-true}" \
  "${YOLO_SCRIPT}" &
COVERAGE_PID=$!

sleep 1
if ! kill -0 "${COVERAGE_PID}" 2>/dev/null; then
  wait "${COVERAGE_PID}"
fi

"${SENSORS_SCRIPT}" "$@"

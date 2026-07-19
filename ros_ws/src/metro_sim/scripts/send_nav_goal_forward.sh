#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

DISTANCE="${1:-4.0}"
TIMEOUT="${2:-120.0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/send_nav_goal_forward.py" "${DISTANCE}" \
  --timeout "${TIMEOUT}" \
  --ros-args -p use_sim_time:=true

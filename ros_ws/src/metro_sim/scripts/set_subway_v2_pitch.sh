#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ANGLE_DEGREES="${1:-0}"
TOPIC="/subway_v2/pitch_position_controller/commands"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: set_subway_v2_pitch.sh [joint-angle-degrees]

Valid range: -15 to +30 degrees.
  -15 deg: camera optical axis points straight up at the tunnel crown
    0 deg: camera optical axis points 75 degrees upward
  +30 deg: camera optical axis points 45 degrees upward
EOF
  exit 0
fi

if [[ ! "${ANGLE_DEGREES}" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "Pitch angle must be a number in degrees: ${ANGLE_DEGREES}" >&2
  exit 2
fi
if ! awk -v angle="${ANGLE_DEGREES}" \
  'BEGIN { exit !(angle >= -15.0 && angle <= 30.0) }'; then
  echo "Pitch angle must be between -15 and +30 degrees." >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROS_WS_DIR}/install/setup.bash" ]]; then
  source "${ROS_WS_DIR}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
ANGLE_RADIANS="$(
  awk -v angle="${ANGLE_DEGREES}" \
    'BEGIN { printf "%.15g", angle * atan2(0, -1) / 180.0 }'
)"

for ((attempt = 1; attempt <= 360; attempt++)); do
  if ros2 topic info "${TOPIC}" >/dev/null 2>&1; then
    ros2 topic pub --times 5 --rate 10 \
      --qos-reliability best_effort \
      --qos-durability volatile \
      "${TOPIC}" \
      std_msgs/msg/Float64MultiArray \
      "{data: [${ANGLE_RADIANS}]}" \
      >/dev/null
    echo "Pitch command: ${ANGLE_DEGREES} deg (${ANGLE_RADIANS} rad joint position)"
    exit 0
  fi
  sleep 0.25
done

echo "Pitch controller topic did not appear within 90 seconds: ${TOPIC}" >&2
exit 1

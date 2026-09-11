#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ANGLE_DEGREES="${1:-45}"
TOPIC="/subway_v2/pitch_position_controller/commands"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: set_subway_v2_pitch.sh [joint-angle-degrees]

Valid range: -15 to +45 degrees.
  -15 deg: camera optical axis points straight up at the tunnel crown
    0 deg: camera optical axis points 75 degrees upward
  +15 deg: camera optical axis points 60 degrees upward
  +30 deg: camera optical axis points 45 degrees upward
  +45 deg: camera optical axis points 30 degrees upward (default)
EOF
  exit 0
fi

if [[ ! "${ANGLE_DEGREES}" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "Pitch angle must be a number in degrees: ${ANGLE_DEGREES}" >&2
  exit 2
fi
if ! awk -v angle="${ANGLE_DEGREES}" \
  'BEGIN { exit !(angle >= -15.0 && angle <= 45.0) }'; then
  echo "Pitch angle must be between -15 and +45 degrees." >&2
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

if timeout 90 ros2 topic pub --times 30 --rate 10 \
  --wait-matching-subscriptions 1 \
  --keep-alive 1.0 \
  --qos-reliability best_effort \
  --qos-durability volatile \
  "${TOPIC}" \
  std_msgs/msg/Float64MultiArray \
  "{data: [${ANGLE_RADIANS}]}" \
  >/dev/null; then
  echo "Pitch command: ${ANGLE_DEGREES} deg (${ANGLE_RADIANS} rad joint position)"
  exit 0
fi

echo "Pitch controller did not subscribe within 90 seconds: ${TOPIC}" >&2
exit 1

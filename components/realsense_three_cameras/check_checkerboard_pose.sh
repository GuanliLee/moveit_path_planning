#!/usr/bin/env bash
set -euo pipefail

# Check one checkerboard pose topic.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

CAMERA="${1:-cam_high}"
TOPIC="/checkerboard/${CAMERA}/pose"

echo "== ${TOPIC} =="
ros2 topic info "${TOPIC}" || true
echo
echo "== one pose sample, timeout 6s =="
timeout 6s ros2 topic echo --once "${TOPIC}" || {
  echo "No checkerboard pose received within 6 seconds." >&2
  exit 1
}

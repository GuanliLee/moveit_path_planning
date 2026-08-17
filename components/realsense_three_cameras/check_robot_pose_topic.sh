#!/usr/bin/env bash
set -euo pipefail

# Check whether the robot pose topic is publishing fresh Pose messages.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

TOPIC="${1:-/left/gripper_end_pos}"

echo "== ${TOPIC} =="
ros2 topic info "${TOPIC}" || true
echo
echo "== one robot pose sample, timeout 6s =="
timeout 6s ros2 topic echo --once "${TOPIC}" || {
  echo "No robot pose received within 6 seconds." >&2
  exit 1
}

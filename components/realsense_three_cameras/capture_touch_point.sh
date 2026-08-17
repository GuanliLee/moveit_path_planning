#!/usr/bin/env bash
set -euo pipefail

# Capture one touched checkerboard inner corner from a Pose topic.
# Usage:
#   ./capture_touch_point.sh P0 0 0 /left/gripper_end_pos
#   ./capture_touch_point.sh P1 7 0 /left/gripper_end_pos
#   ./capture_touch_point.sh P2 0 10 /left/gripper_end_pos

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${1:?point name required: P0, P1, or P2}"
COL="${2:?checkerboard column index required}"
ROW="${3:?checkerboard row index required}"
TOPIC="${4:-/left/gripper_end_pos}"

exec python3 "${SCRIPT_DIR}/capture_touch_point.py" \
  --name "${NAME}" \
  --col "${COL}" \
  --row "${ROW}" \
  --topic "${TOPIC}"

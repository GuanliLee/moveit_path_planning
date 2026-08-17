#!/usr/bin/env bash
set -euo pipefail

# Start one checkerboard pose publisher.
# Usage:
#   ./start_checkerboard_pose.sh cam_high
#   ./start_checkerboard_pose.sh cam_left
#   ./start_checkerboard_pose.sh cam_right

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA="${1:-cam_high}"
if [[ $# -gt 0 ]]; then
  shift
fi

exec python3 "${SCRIPT_DIR}/checkerboard_pose_node.py" --ros-args \
  -p camera_name:="${CAMERA}" \
  -p pattern_cols:=8 \
  -p pattern_rows:=11 \
  -p square_size_m:=0.02 \
  "$@"

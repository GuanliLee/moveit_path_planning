#!/usr/bin/env bash
set -euo pipefail

# Start checkerboard pose publishers for all three cameras.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pids=()

cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

for camera in cam_high cam_left cam_right; do
  python3 "${SCRIPT_DIR}/checkerboard_pose_node.py" --ros-args \
    -p camera_name:="${camera}" \
    -p pattern_cols:=8 \
    -p pattern_rows:=11 \
    -p square_size_m:=0.02 \
    "$@" &
  pids+=("$!")
done

wait

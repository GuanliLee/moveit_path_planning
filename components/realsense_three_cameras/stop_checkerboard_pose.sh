#!/usr/bin/env bash
set -euo pipefail

PATTERN="/home/ligl/realsense_three_cameras/checkerboard_pose_node.py"

if pgrep -f "${PATTERN}" >/dev/null; then
  echo "[stop_checkerboard_pose] stopping checkerboard pose nodes"
  pkill -TERM -f "${PATTERN}" || true
  sleep 1
fi

if pgrep -f "${PATTERN}" >/dev/null; then
  echo "[stop_checkerboard_pose] force stopping checkerboard pose nodes"
  pkill -KILL -f "${PATTERN}" || true
fi

echo "[stop_checkerboard_pose] done"

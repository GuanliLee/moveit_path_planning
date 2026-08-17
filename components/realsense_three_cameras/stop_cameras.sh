#!/usr/bin/env bash
set -euo pipefail

# Stop both the multi-camera launcher and older per-camera manual launchers.

patterns=(
  "ros2 launch realsense_multi_camera_config three_d435i.launch.py"
  "ros2 launch realsense2_camera rs_launch.py"
  "/opt/ros/jazzy/lib/realsense2_camera/realsense2_camera_node"
)

for pattern in "${patterns[@]}"; do
  if pgrep -f "${pattern}" >/dev/null; then
    echo "[stop_cameras] stopping: ${pattern}"
    pkill -TERM -f "${pattern}" || true
  fi
done

sleep 2

for pattern in "${patterns[@]}"; do
  if pgrep -f "${pattern}" >/dev/null; then
    echo "[stop_cameras] force stopping: ${pattern}"
    pkill -KILL -f "${pattern}" || true
  fi
done

echo "[stop_cameras] done"

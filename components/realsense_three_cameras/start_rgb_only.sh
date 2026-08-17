#!/usr/bin/env bash
set -euo pipefail

# Three D435i cameras, RGB only, 640x480@30fps.
# Use this for stable chessboard observation and calibration image capture.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/shared/xiangpc/realsense_ros2_ws/install/setup.bash
set -u

CONFIG_FILE="/opt/shared/xiangpc/realsense_ros2_ws/install/realsense_multi_camera_config/share/realsense_multi_camera_config/config/rgb_only_640x480_30.yaml"

exec ros2 launch realsense_multi_camera_config three_d435i.launch.py \
  config_file:="${CONFIG_FILE}" \
  "$@"

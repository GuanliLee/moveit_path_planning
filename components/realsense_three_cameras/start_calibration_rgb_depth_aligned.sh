#!/usr/bin/env bash
set -euo pipefail

# Three D435i cameras for calibration:
#   RGB: 640x480@30fps
#   Depth: 640x480@30fps
#   aligned_depth_to_color enabled

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/shared/xiangpc/realsense_ros2_ws/install/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/calibration_rgb_depth_aligned_640x480_30.yaml"

exec ros2 launch realsense_multi_camera_config three_d435i.launch.py \
  config_file:="${CONFIG_FILE}" \
  "$@"

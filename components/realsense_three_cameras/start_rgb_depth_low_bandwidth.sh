#!/usr/bin/env bash
set -euo pipefail

# Three D435i cameras, RGB + depth, conservative low-bandwidth profile.
# Default profile:
#   RGB:   424x240@6fps
#   Depth: 480x270@6fps

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/shared/xiangpc/realsense_ros2_ws/install/setup.bash
set -u

CONFIG_FILE="/opt/shared/xiangpc/realsense_ros2_ws/install/realsense_multi_camera_config/share/realsense_multi_camera_config/config/d435i_low_bandwidth.yaml"

exec ros2 launch realsense_multi_camera_config three_d435i.launch.py \
  config_file:="${CONFIG_FILE}" \
  "$@"

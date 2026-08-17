#!/bin/bash
PIPER_ROS_DIR="${PIPER_ROS_DIR:-/home/agilex/piper_ros}"

conda deactivate || true
cd "${PIPER_ROS_DIR}"
bash can_config.sh
ros2 launch piper start_two_piper.launch.py

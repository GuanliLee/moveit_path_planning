#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA="${1:-cam_left}"
ROBOT_POSE_TOPIC="${2:-/left/gripper_end_pos}"
RESULT_JSON="${3:-/home/ligl/agilex_xpc/calibration/results/${CAMERA}_eye_in_hand_checkerboard/latest_checkerboard_handeye.json}"

exec python3 "${SCRIPT_DIR}/visualize_handeye_application.py" \
  --camera "${CAMERA}" \
  --robot-pose-topic "${ROBOT_POSE_TOPIC}" \
  --result-json "${RESULT_JSON}"

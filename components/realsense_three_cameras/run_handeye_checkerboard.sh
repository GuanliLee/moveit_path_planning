#!/usr/bin/env bash
set -euo pipefail

# Run the checkerboard-based hand-eye collector.
#
# Usage:
#   ./run_handeye_checkerboard.sh eye_to_hand cam_high /left/gripper_end_pos
#   ./run_handeye_checkerboard.sh eye_in_hand cam_left /left/gripper_end_pos

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-eye_to_hand}"
CAMERA="${2:-cam_high}"
ROBOT_POSE_TOPIC="${3:-/left/gripper_end_pos}"
TARGET_POSE_TOPIC="/checkerboard/${CAMERA}/pose"
RESULT_DIR="/home/ligl/agilex_xpc/calibration/results/${CAMERA}_${MODE}_checkerboard"

exec python3 "${SCRIPT_DIR}/checkerboard_handeye_calibration.py" \
  --mode "${MODE}" \
  --robot-pose-topic "${ROBOT_POSE_TOPIC}" \
  --target-pose-topic "${TARGET_POSE_TOPIC}" \
  --min-samples "${MIN_SAMPLES:-15}" \
  --max-age-sec "${MAX_AGE_SEC:-2.0}" \
  --result-dir "${RESULT_DIR}"

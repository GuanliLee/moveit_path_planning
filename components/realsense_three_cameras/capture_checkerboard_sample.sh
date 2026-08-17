#!/usr/bin/env bash
set -euo pipefail

# Capture one calibration snapshot from all three cameras.
# Assumes start_calibration_rgb_depth_aligned.sh is running.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

NAME="${1:-}"

args=(
  --pattern-cols 8
  --pattern-rows 11
  --timeout-sec 10
)

if [[ -n "${NAME}" ]]; then
  args+=(--name "${NAME}")
fi

exec python3 /home/ligl/agilex_xpc/bridge/scripts/capture_calibration_sample.py "${args[@]}"

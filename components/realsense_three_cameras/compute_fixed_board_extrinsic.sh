#!/usr/bin/env bash
set -euo pipefail

# Compute external camera extrinsic from one checkerboard snapshot plus P0/P1/P2 touch points.
# Usage:
#   ./compute_fixed_board_extrinsic.sh board_check_001 cam_high

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAMPLE="${1:?snapshot sample name required}"
CAMERA="${2:-cam_high}"
TOUCH_FILE="${3:-/home/ligl/agilex_xpc/calibration/touch_points/checkerboard_touch_points.json}"

exec python3 "${SCRIPT_DIR}/compute_fixed_board_extrinsic.py" \
  --sample "${SAMPLE}" \
  --camera "${CAMERA}" \
  --touch-file "${TOUCH_FILE}"

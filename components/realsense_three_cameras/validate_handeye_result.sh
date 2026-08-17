#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_JSON="${1:-/home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard/latest_checkerboard_handeye.json}"

exec python3 "${SCRIPT_DIR}/validate_handeye_result.py" "${RESULT_JSON}"

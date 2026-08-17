#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_JSON="${1:-/home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard/latest_checkerboard_handeye.json}"
EXCLUDE="${2:-}"

args=("${RESULT_JSON}")
if [[ -n "${EXCLUDE}" ]]; then
  args+=(--exclude "${EXCLUDE}")
fi

exec python3 "${SCRIPT_DIR}/recompute_handeye_result.py" "${args[@]}"

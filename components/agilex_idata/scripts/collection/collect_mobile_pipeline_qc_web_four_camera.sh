#!/usr/bin/env bash
# Isolated selectable three/four-camera QC console. Core behavior is delegated
# to the existing three-camera-compatible launcher with explicit resources.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PIPELINE_ROOT="${PIPELINE_ROOT:-${REPO_ROOT}/scripts/embodied_data_pipeline-main}"
if [ -n "${PIPELINE_DATA_ROOT:-}" ]; then
  FOUR_CAMERA_LOG_ROOT="${PIPELINE_DATA_ROOT%/}/logs/pipeline_qc_web_four_camera"
else
  FOUR_CAMERA_LOG_ROOT="${PIPELINE_ROOT}/logs/pipeline_qc_web_four_camera"
fi

export WEB_PORT="${WEB_PORT:-8001}"
export WEB_LOG_ROOT="${WEB_LOG_ROOT:-${FOUR_CAMERA_LOG_ROOT}}"
export PIPELINE_DEFAULT_PROFILE="${PIPELINE_DEFAULT_PROFILE:-${PIPELINE_ROOT}/robot_profiles/aloha_four_camera.yaml}"
export PIPELINE_ALOHA_YAML="${PIPELINE_ALOHA_YAML:-${PIPELINE_ROOT}/mcap_conversion/topic_configs/aloha_four_camera_data_params.yaml}"
export PIPELINE_CAMERA_LAYOUT="${PIPELINE_CAMERA_LAYOUT:-three_camera_global}"
export PIPELINE_CAMERA_VARIANT_SELECTABLE="${PIPELINE_CAMERA_VARIANT_SELECTABLE:-1}"
export PIPELINE_CAMERA_COUNT="${PIPELINE_CAMERA_COUNT:-3}"
export PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-global}"
export PIPELINE_DEFAULT_CONVERT_JOBS="${PIPELINE_DEFAULT_CONVERT_JOBS:-6}"
export PIPELINE_DEFAULT_GPU_DEVICE="${PIPELINE_DEFAULT_GPU_DEVICE:-0}"
export PIPELINE_DEFAULT_ALOHA_INCLUDE_BASE_ACTION="${PIPELINE_DEFAULT_ALOHA_INCLUDE_BASE_ACTION:-0}"

exec bash "${SCRIPT_DIR}/collect_mobile_pipeline_qc_web.sh" "$@"

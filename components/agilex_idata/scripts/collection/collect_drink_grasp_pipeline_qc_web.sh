#!/usr/bin/env bash
# Local QC/conversion console for MCAP datasets recorded by drink_grasp_web.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PIPELINE_ROOT="${PIPELINE_ROOT:-${REPO_ROOT}/scripts/embodied_data_pipeline-main}"

export PIPELINE_ALOHA_YAML="${PIPELINE_ALOHA_YAML:-${PIPELINE_ROOT}/mcap_conversion/topic_configs/drink_grasp_three_camera_data_params.yaml}"
export PIPELINE_CAMERA_LAYOUT="${PIPELINE_CAMERA_LAYOUT:-three_camera_global}"
export PIPELINE_CAMERA_COUNT="${PIPELINE_CAMERA_COUNT:-3}"
export PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-global}"

exec bash "${SCRIPT_DIR}/collect_mobile_pipeline_qc_web_four_camera.sh" "$@"

#!/usr/bin/env bash
# Unified web console wrapper for fixed-stage mobile ALOHA data collection.
#
# Default fixed stage:
#   stage 2 only. The state machine should run stage 2 between:
#     /state_machine/start=true -> data collection starts
#     /state_machine/end=true   -> data collection saves this episode
#
# Usage:
#   bash scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh [dataset_dir] [start_episode_index]
#   bash scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh --fixed-stage 2 [staged-script-args...]
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/collection_ros_env.sh"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
STAGED_SCRIPT="${COLLECT_MOBILE_PIPELINE_WEB_STAGED_SCRIPT:-${SCRIPT_DIR}/collect_mobile_pipeline_web_staged.sh}"
MOBILE_YAML="${MOBILE_YAML:-${REPO_ROOT}/ros2_ws/src/data_tools/config/aloha_mobile_data_params.yaml}"
FIXED_COLLECTION_STAGE="${FIXED_COLLECTION_STAGE:-2}"
FORWARD_ARGS=()

usage() {
    cat <<EOF
Usage:
  $0 [--fixed-stage 2] [dataset_dir] [start_episode_index] [--grade A|B|F]
  $0 [--fixed-stage 2] [start_episode_index]

This wrapper runs fixed-stage collection by reusing collect_mobile_pipeline_web_staged.sh.
It starts one episode when ${STATE_MACHINE_START_TOPIC:-/state_machine/start}=true,
publishes ${DATA_COLLECTION_START_TOPIC:-/data_collection/start}=true after recording starts,
and saves the episode when ${STATE_MACHINE_END_TOPIC:-/state_machine/end}=true.
After saving, the collection page asks the operator to choose A/B/F quality
or abandon this episode. Per-episode quality and scene inventory are written
into episodeN_0_info.json.
Choose left- and right-hand target items in the collection web page.

Options:
  --fixed-stage 2       Fixed phase to collect. Currently only stage 2 is supported.
  -h, --help            Show this help.

Four RGB cameras: left, front, right, head
  head image topic: /camera_h/color/image_raw
  head calibration: unavailable; /camera_h/color/camera_info is not recorded

Forwarded arguments are the same as collect_mobile_pipeline_web_staged.sh.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --fixed-stage)
            [ "$#" -ge 2 ] || {
                echo "[错误] --fixed-stage 需要阶段编号" >&2
                exit 1
            }
            FIXED_COLLECTION_STAGE="$2"
            shift 2
            ;;
        --fixed-stage=*)
            FIXED_COLLECTION_STAGE="${1#*=}"
            shift
            ;;
        *)
            FORWARD_ARGS+=("$1")
            shift
            ;;
    esac
done

case "${FIXED_COLLECTION_STAGE}" in
    2)
        ;;
    *)
        echo "[错误] 固定阶段采集脚本目前只支持第二阶段: --fixed-stage 2" >&2
        exit 1
        ;;
esac

[ -f "${STAGED_SCRIPT}" ] || {
    echo "[错误] 找不到全阶段采集脚本: ${STAGED_SCRIPT}" >&2
    exit 1
}

export FIXED_COLLECTION_STAGE
export COLLECTION_STAGED_CAPTURE_DEFAULT=0
export STATE_MACHINE_STAGED_DEFAULT=0
export STATE_MACHINE_BRIDGE_ENABLE=1
export STATE_MACHINE_AUTO_ACTIVE_DEFAULT=1
export STATE_MACHINE_START_TOPIC=/state_machine/start
export STATE_MACHINE_END_TOPIC=/state_machine/end
export DATA_COLLECTION_START_TOPIC=/data_collection/start
export MOBILE_YAML
export COLLECTION_SCENE_INVENTORY_JSON="${COLLECTION_SCENE_INVENTORY_JSON:-${SCRIPT_DIR}/market_scene_inventory.json}"

exec bash "${STAGED_SCRIPT}" "${FORWARD_ARGS[@]}"

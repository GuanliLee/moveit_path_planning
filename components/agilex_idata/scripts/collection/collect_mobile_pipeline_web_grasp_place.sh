#!/usr/bin/env bash
# Web collection launcher for paired grasp and place recordings.
#
# Usage:
#   bash scripts/collection/collect_mobile_pipeline_web_grasp_place.sh \
#     /home/agilex/data/stage2_twohand/<configuration> [start_episode_index] [--grade A|B|F]
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/collection_ros_env.sh"
STAGED_SCRIPT="${COLLECT_MOBILE_PIPELINE_WEB_STAGED_SCRIPT:-${SCRIPT_DIR}/collect_mobile_pipeline_web_staged.sh}"
FORWARD_ARGS=()

usage() {
    cat <<EOF
Usage:
  $0 /home/agilex/data/stage2_twohand/<configuration> [start_episode_index] [--grade A|B|F]

Records two independent captures for each logical episode:
  1. ${STATE_MACHINE_START_TOPIC:-/state_machine/start}=true starts grasp recording.
  2. ${GRASP_PLACE_GRASP_END_TOPIC:-/state_machine/grasping/end}=true saves grasp data, then asks for A/B/F.
  3. ${DATA_COLLECTION_SAVE_SUCCESS_TOPIC:-/state_collect/end}=true is published after the review finishes.
  4. ${GRASP_PLACE_START_TOPIC:-/state_place/start}=true starts place recording after grasp review.
  5. ${STATE_MACHINE_END_TOPIC:-/state_machine/end}=true saves place data, then asks for A/B/F.

The grasp data stays in the supplied configuration directory. Place data is
stored in ${GRASP_PLACE_DATA_ROOT:-/home/agilex/data/place}/<configuration> with
the same episode number. The next episode starts after both reviews finish.

Options:
  --grade A|B|F   Automatically apply this grade to both captures.
  -h, --help      Show this help.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        *)
            FORWARD_ARGS+=("$1")
            shift
            ;;
    esac
done

[ -f "${STAGED_SCRIPT}" ] || {
    echo "[错误] 找不到全阶段采集脚本: ${STAGED_SCRIPT}" >&2
    exit 1
}

export GRASP_PLACE_COLLECTION_ENABLE=1
export GRASP_PLACE_DATA_ROOT="${GRASP_PLACE_DATA_ROOT:-/home/agilex/data/place}"
export GRASP_PLACE_START_TOPIC="${GRASP_PLACE_START_TOPIC:-/state_place/start}"
export GRASP_PLACE_GRASP_END_TOPIC="${GRASP_PLACE_GRASP_END_TOPIC:-/state_machine/grasping/end}"
export COLLECTION_STAGED_CAPTURE_DEFAULT=0
export STATE_MACHINE_STAGED_DEFAULT=0
export STATE_MACHINE_BRIDGE_ENABLE=1
export STATE_MACHINE_AUTO_ACTIVE_DEFAULT=1
export STATE_MACHINE_START_TOPIC="${STATE_MACHINE_START_TOPIC:-/state_machine/start}"
export STATE_MACHINE_END_TOPIC="${STATE_MACHINE_END_TOPIC:-/state_machine/end}"
export DATA_COLLECTION_START_TOPIC="${DATA_COLLECTION_START_TOPIC:-/data_collection/start}"
export DATA_COLLECTION_SAVE_SUCCESS_TOPIC="${DATA_COLLECTION_SAVE_SUCCESS_TOPIC:-/state_collect/end}"

exec bash "${STAGED_SCRIPT}" "${FORWARD_ARGS[@]}"

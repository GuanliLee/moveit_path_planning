#!/usr/bin/env bash
# Web controller for KAI0/ALOHA episode collection.
#
# It keeps the deployed ROS capture path and episode metadata/review behavior.
# New collection sessions stop at raw MCAP; HDF5/QC remains a later QC concern.
#
# Usage:
#   bash scripts/collection/collect_mobile_episode_web.sh [start_episode_index]
#
# Open:
#   http://<robot-ip>:8000/
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/collection_ros_env.sh"
collection_acquire_session_lock
SCRIPT_PATH="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

usage() {
    cat <<EOF
Usage:
  $0 [start_episode_index]

Environment defaults:
  DATA_DIR=/home/agilex/data/market_2
  WITH_BASE=1                    1: aloha_mobile, 0: aloha
  STAGED_CAPTURE=1               write episodeN_0_info.json stage marks
  COLLECTION_SCENE_INVENTORY_JSON=market_scene_inventory.json
                                  add matched coarse shelf scene info to each episode info json
  RAW_MCAP_ONLY=1                fixed production policy: no collection-time HDF5/QC
  COLLECTION_AUTO_GRADE=<empty> automatic grade for every saved episode: A, B, or F
  REQUIRE_TOPIC_CHECK_ON_START=1 run required ROS topic checks before recording;
                                 failures warn but do not block collection
  ALLOW_MISSING_CAPTURE_HEALTH=0 reject new mobile episodes without recorder health evidence
  WEB_HOST=192.168.3.101
  WEB_PORT=8000

Web controls:
  Start, Stage, End and save, Abandon, Check topics, Exit
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi

if [ "${1:-}" = "--process-only" ]; then
    PROCESS_ONLY=1
    shift
fi

EPISODE_INDEX="${1:-${EPISODE_INDEX:-0}}"

DATA_ROS_WS="${DATA_ROS_WS:-${REPO_ROOT}/ros2_ws}"
DATA_DIR="${DATA_DIR:-/home/agilex/data/market_2}"
ALOHA_YAML_OVERRIDE="${ALOHA_YAML:-}"
MOBILE_YAML_OVERRIDE="${MOBILE_YAML:-}"
WITH_BASE="${WITH_BASE:-1}"
STAGED_CAPTURE="${STAGED_CAPTURE:-1}"
STAGE_AUTOSAVE_ON_PRESET="${STAGE_AUTOSAVE_ON_PRESET:-1}"
WRITE_COMPAT_INSTRUCTIONS="${WRITE_COMPAT_INSTRUCTIONS:-1}"
SEGMENT_TEXT_SEPARATOR="${SEGMENT_TEXT_SEPARATOR:-|}"
STAGE_PRESET_INFO_JSON="${STAGE_PRESET_INFO_JSON:-/home/agilex/data/task_presets/shelf_bottles_to_cart_info.json}"
COLLECTION_METADATA_SCRIPT="${COLLECTION_METADATA_SCRIPT:-${SCRIPT_DIR}/collection_episode_metadata.py}"
COLLECTION_SCENE_INVENTORY_JSON="${COLLECTION_SCENE_INVENTORY_JSON:-${SCRIPT_DIR}/market_scene_inventory.json}"
COLLECTION_TARGETS_MODULE="${COLLECTION_TARGETS_MODULE:-${SCRIPT_DIR}/collection_targets.py}"
TARGET_OPTIONS_JSON="$(python3 - "${SCRIPT_DIR}" <<'PY'
import json
import sys

sys.path.insert(0, sys.argv[1])
from collection_targets import TARGET_OPTIONS

print(json.dumps(TARGET_OPTIONS, ensure_ascii=False))
PY
)"
TARGET_BOTTLE_A=""
TARGET_BOTTLE_B=""
TARGET_BOTTLE_A_ZH=""
TARGET_BOTTLE_B_ZH=""
COLLECTION_AUTO_GRADE="${COLLECTION_AUTO_GRADE:-}"
COLLECTION_AUTO_GRADE="$(printf '%s' "${COLLECTION_AUTO_GRADE}" | tr '[:lower:]' '[:upper:]')"
case "${COLLECTION_AUTO_GRADE}" in
    ""|A|B|F) ;;
    *)
        echo "[错误] COLLECTION_AUTO_GRADE 只能是 A、B 或 F: ${COLLECTION_AUTO_GRADE}" >&2
        exit 1
        ;;
esac

RAW_MCAP_ONLY=1
RUN_CONVERT="${RUN_CONVERT:-0}"
RUN_QC="${RUN_QC:-0}"
RUN_LEROBOT="${RUN_LEROBOT:-0}"
BACKGROUND_PROCESSING="${BACKGROUND_PROCESSING:-0}"
MAX_BACKGROUND_JOBS="${MAX_BACKGROUND_JOBS:-2}"
REUSE_CAPTURE_SERVICE="${REUSE_CAPTURE_SERVICE:-1}"

case "${RAW_MCAP_ONLY}" in
    1|true|TRUE|yes|YES|on|ON)
        RUN_CONVERT=0
        RUN_QC=0
        RUN_LEROBOT=0
        BACKGROUND_PROCESSING=0
        ;;
esac

CAPTURE_SERVICE_TIMEOUT="${CAPTURE_SERVICE_TIMEOUT:-25}"
CAPTURE_HZ="${CAPTURE_HZ:-20}"
CAPTURE_TIMEOUT="${CAPTURE_TIMEOUT:-2}"
USE_TOPIC_STAMP="${USE_TOPIC_STAMP:-false}"
TOPIC_SAMPLE_TIMEOUT="${TOPIC_SAMPLE_TIMEOUT:-10}"
REQUIRE_TOPIC_CHECK_ON_START="${REQUIRE_TOPIC_CHECK_ON_START:-1}"
SELF_CHECK_CACHE_SECONDS="${SELF_CHECK_CACHE_SECONDS:-60}"
TIME_DIFF_LIMIT="${TIME_DIFF_LIMIT:-0.10}"
MOBILE_MIN_EFFECTIVE_FPS="${MOBILE_MIN_EFFECTIVE_FPS:-20}"
MIN_FRAMES="${MIN_FRAMES:-20}"
ALLOW_MISSING_CAPTURE_HEALTH="${ALLOW_MISSING_CAPTURE_HEALTH:-0}"
PIPER_RESET_SCRIPT="${PIPER_RESET_SCRIPT:-/home/caizj/piper_slow_return_zero.sh}"
TOPIC_CHECK_SCRIPT="${TOPIC_CHECK_SCRIPT:-${SCRIPT_DIR}/check_mobile_topics.sh}"

if [ -x /home/agilex/miniforge3/envs/lerobot/bin/python ] &&
    { [ -z "${LEROBOT_PYTHON:-}" ] || [ "${LEROBOT_PYTHON}" = "python3" ]; }; then
    LEROBOT_PYTHON=/home/agilex/miniforge3/envs/lerobot/bin/python
else
    LEROBOT_PYTHON="${LEROBOT_PYTHON:-python3}"
fi
LEROBOT_TARGET_DIR="${LEROBOT_TARGET_DIR:-${DATA_DIR}/lerobot}"
LEROBOT_DATASET_NAME="${LEROBOT_DATASET_NAME:-$(basename "${DATA_DIR}")_aloha_mobile}"
LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE:-cobot_magic}"
LEROBOT_FPS="${LEROBOT_FPS:-30}"
REPLAY_REFERENCE_ROOT_OVERRIDE="${REPLAY_REFERENCE_ROOT:-}"
REPLAY_HDF5_ROOT_OVERRIDE="${REPLAY_HDF5_ROOT:-}"
REPLAY_LEROBOT_ROOT_OVERRIDE="${REPLAY_LEROBOT_ROOT:-}"
REPLAY_REFERENCE_ROOT=""
REPLAY_HDF5_ROOT=""
REPLAY_LEROBOT_ROOT=""

WEB_HOST="${WEB_HOST:-192.168.3.101}"
WEB_PORT="${WEB_PORT:-8000}"
WEB_RUNTIME_DIR="${WEB_RUNTIME_DIR:-${DATA_DIR}/logs/collect_mobile_episode_web/runtime}"
CONTROL_FIFO="${WEB_RUNTIME_DIR}/control.fifo"
STATUS_FILE="${WEB_RUNTIME_DIR}/status.json"

CAPTURE_LAUNCH_PID=""
CAPTURE_SERVICE_LOG=""
CAPTURE_SERVICE_READY=0
CAPTURE_RUNNING=0
CURRENT_EPISODE="${EPISODE_INDEX}"
CURRENT_STAGE_START=""
BACKGROUND_PIDS=()
BACKGROUND_LOGS=()
REPLAY_PID=""
REPLAY_STATUS="idle"
REPLAY_LOG=""
REPLAY_OUTPUT=""
REPLAY_COMMAND=""
REPLAY_STARTED_AT=""
REPLAY_FINISHED_AT=""
REPLAY_EXIT_CODE=""
WEB_SERVER_PID=""
CONTROL_FD=""
ROS_ENV_SOURCED=""

STAGE_START_TIMES=()
STAGE_END_TIMES=()
PRESET_SEGMENT_COUNT=0
PRESET_STAGE_LABELS=""
PRESET_FULL_INSTRUCTIONS=""
LAST_CHECK_OK=0
LAST_CHECK_SUMMARY="尚未自检"
LAST_CHECK_DETAILS=""
LAST_CHECK_SCOPE=""
LAST_CHECK_AT_EPOCH=0
QUALITY_REVIEW_PENDING=0
QUALITY_PENDING_EPISODE=""
LAST_SAVED_EPISODE=""
LAST_SAVED_DATA_DIR=""

log() {
    printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

die() {
    echo "[错误] $*" >&2
    exit 1
}

is_truthy() {
    case "${1:-}" in
        1|true|TRUE|yes|YES|on|ON) return 0 ;;
        *) return 1 ;;
    esac
}

json_bool() {
    if is_truthy "$1"; then
        printf '1'
    else
        printf '0'
    fi
}

check_scope_fingerprint() {
    configure_profile
    printf '%s|%s|%s\n' "${DATA_ROS_WS}" "${WITH_BASE}" "${CAPTURE_PROFILE}"
}

capture_service_scope_fingerprint() {
    configure_profile
    printf '%s|%s|%s|%s|%s|%s|%s\n' \
        "${DATA_DIR}" \
        "${CAPTURE_PROFILE}" \
        "${CAPTURE_LAUNCH_FILE}" \
        "${ALOHA_YAML}" \
        "${CAPTURE_HZ}" \
        "${CAPTURE_TIMEOUT}" \
        "${USE_TOPIC_STAMP}"
}

print_cmd() {
    printf '+'
    printf ' %q' "$@"
    printf '\n'
}

run_cmd() {
    print_cmd "$@"
    "$@"
}

source_if_exists() {
    local setup_file="$1"
    if [ -f "${setup_file}" ]; then
        # shellcheck disable=SC1090
        source "${setup_file}"
    fi
}

source_ros_env() {
    if [ -n "${ROS_ENV_SOURCED:-}" ]; then
        return 0
    fi
    [ -f /opt/ros/humble/setup.bash ] || die "找不到 /opt/ros/humble/setup.bash"
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    source_if_exists /home/agilex/agilex_ws/install/setup.bash
    source_if_exists /home/agilex/camera_ros/install/setup.bash
    source_if_exists /home/agilex/piper_ros/install/setup.bash
    [ -f "${DATA_ROS_WS}/install/setup.bash" ] || die "找不到 ${DATA_ROS_WS}/install/setup.bash"
    # shellcheck disable=SC1090
    source "${DATA_ROS_WS}/install/setup.bash"
    export FASTDDS_BUILTIN_TRANSPORTS
    ROS_ENV_SOURCED=1
}

refresh_replay_roots() {
    local raw_data_parent
    local derived_reference_root
    raw_data_parent="$(dirname -- "${DATA_DIR}")"
    derived_reference_root="${raw_data_parent}/three_camera_global"
    REPLAY_REFERENCE_ROOT="${REPLAY_REFERENCE_ROOT_OVERRIDE:-${derived_reference_root}}"
    REPLAY_HDF5_ROOT="${REPLAY_HDF5_ROOT_OVERRIDE:-${REPLAY_REFERENCE_ROOT}}"
    REPLAY_LEROBOT_ROOT="${REPLAY_LEROBOT_ROOT_OVERRIDE:-${REPLAY_REFERENCE_ROOT}}"
}

configure_profile() {
    DATA_DIR="${DATA_DIR%/}"
    refresh_replay_roots
    ALOHA_DIR="${DATA_DIR}/aloha"
    DATA_TOOLS_SCRIPTS="${DATA_TOOLS_SCRIPTS:-${DATA_ROS_WS}/src/data_tools/scripts}"
    LOG_DIR="${DATA_DIR}/logs/collect_mobile_episode_web"
    BACKGROUND_LOG_DIR="${LOG_DIR}/processing"
    QC_REPORT_ROOT="${ALOHA_DIR}/qc_reports"

    if is_truthy "${WITH_BASE}"; then
        CAPTURE_PROFILE="aloha_mobile"
        HDF5_TYPE="aloha_mobile"
        CAPTURE_LAUNCH_FILE="run_aloha_mobile_data_capture_to_mcap.launch.py"
        ALOHA_YAML="${MOBILE_YAML_OVERRIDE:-${DATA_ROS_WS}/src/data_tools/config/aloha_mobile_data_params.yaml}"
    else
        CAPTURE_PROFILE="aloha"
        HDF5_TYPE="aloha"
        CAPTURE_LAUNCH_FILE="run_aloha_data_capture_to_mcap.launch.py"
        ALOHA_YAML="${ALOHA_YAML_OVERRIDE:-${DATA_ROS_WS}/src/data_tools/config/aloha_data_params.yaml}"
    fi

    if [ "${LEROBOT_DATASET_NAME}" = "$(basename "${DATA_DIR}")_aloha_mobile" ] && ! is_truthy "${WITH_BASE}"; then
        LEROBOT_DATASET_NAME="$(basename "${DATA_DIR}")_aloha"
    fi
    LEROBOT_TARGET_DIR="${LEROBOT_TARGET_DIR:-${DATA_DIR}/lerobot}"
    WEB_RUNTIME_DIR="${WEB_RUNTIME_DIR:-${LOG_DIR}/runtime}"
    CONTROL_FIFO="${WEB_RUNTIME_DIR}/control.fifo"
    STATUS_FILE="${WEB_RUNTIME_DIR}/status.json"
}

require_files() {
    configure_profile
    [ -f "${DATA_ROS_WS}/install/setup.bash" ] || die "找不到 ${DATA_ROS_WS}/install/setup.bash，请先构建 data_tools 工作空间"
    [ -f "${ALOHA_YAML}" ] || die "找不到采集配置: ${ALOHA_YAML}"
    [ -f "${COLLECTION_METADATA_SCRIPT}" ] || die "缺少 collection metadata helper: ${COLLECTION_METADATA_SCRIPT}"
    if ! is_truthy "${RAW_MCAP_ONLY}"; then
        [ -f "${DATA_TOOLS_SCRIPTS}/mcap_to_aloha_data.py" ] || die "缺少 mcap_to_aloha_data.py: ${DATA_TOOLS_SCRIPTS}"
        [ -f "${DATA_TOOLS_SCRIPTS}/data_to_hdf5.py" ] || die "缺少 data_to_hdf5.py: ${DATA_TOOLS_SCRIPTS}"
        [ -f "${DATA_TOOLS_SCRIPTS}/hdf5_to_lerobot.py" ] || die "缺少 hdf5_to_lerobot.py: ${DATA_TOOLS_SCRIPTS}"
        if is_truthy "${RUN_QC}" && is_truthy "${WITH_BASE}"; then
            [ -f "${SCRIPT_DIR}/verify_mobile_hdf5.py" ] || die "缺少 verify_mobile_hdf5.py: ${SCRIPT_DIR}"
        fi
    fi
}

timestamp_now() {
    python3 - <<'PY'
import time
ns = time.time_ns()
print(f"{ns // 1_000_000_000}.{ns % 1_000_000_000:09d}")
PY
}

reset_stage_state() {
    STAGE_START_TIMES=()
    STAGE_END_TIMES=()
    CURRENT_STAGE_START=""
}

join_unit_sep() {
    local first=1
    local item
    for item in "$@"; do
        if [ "${first}" -eq 0 ]; then
            printf '\037'
        fi
        printf '%s' "${item}"
        first=0
    done
}

refresh_stage_preset() {
    local exports
    export TARGET_BOTTLE_A TARGET_BOTTLE_B TARGET_BOTTLE_A_ZH TARGET_BOTTLE_B_ZH
    exports="$(python3 - "${STAGE_PRESET_INFO_JSON}" "${SEGMENT_TEXT_SEPARATOR}" "${SCRIPT_DIR}" <<'PY'
import json
import os
import shlex
import sys

path = sys.argv[1]
separator = sys.argv[2]
sys.path.insert(0, sys.argv[3])

from collection_targets import render_target_prompts

left_target = os.environ.get("TARGET_BOTTLE_A", "")
right_target = os.environ.get("TARGET_BOTTLE_B", "")

def first_text(value):
    if isinstance(value, list) and value:
        return str(value[0])
    if isinstance(value, str):
        return value
    return ""

def apply_placeholders(text):
    replacements = {
        "{bottle_a}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{{bottle_a}}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{BOTTLE_A}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{{BOTTLE_A}}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{bottle_b}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{{bottle_b}}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{BOTTLE_B}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{{BOTTLE_B}}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{bottle_a_zh}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{{bottle_a_zh}}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{BOTTLE_A_ZH}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{{BOTTLE_A_ZH}}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{bottle_b_zh}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
        "{{bottle_b_zh}}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
        "{BOTTLE_B_ZH}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
        "{{BOTTLE_B_ZH}}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    text = render_target_prompts(text, left_target, right_target)
    return text

full = ""
labels = []
warning = ""
if path and os.path.exists(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    mark = data.get("mark", data)
    full = apply_placeholders(first_text(mark.get("full-instructions-en")))
    if not full:
        full = apply_placeholders(first_text(mark.get("full-instructions-zh")))
    if not full:
        full = apply_placeholders(first_text(mark.get("full-instructions")))
    for segment in mark.get("segment-instructions") or []:
        text = apply_placeholders(first_text(segment.get("description_en")))
        if not text:
            text = apply_placeholders(first_text(segment.get("description_zh")))
        if not text:
            text = apply_placeholders(first_text(segment.get("instructions")))
        labels.append(text)
elif path:
    warning = f"Preset not found: {path}"

values = {
    "PRESET_SEGMENT_COUNT": str(len(labels)),
    "PRESET_STAGE_LABELS": "\x1f".join(labels),
    "PRESET_FULL_INSTRUCTIONS": full,
    "PRESET_WARNING": warning,
}
for key, value in values.items():
    print(f"{key}={shlex.quote(value)}")
PY
)"
    eval "${exports}"
}

stage_label_at() {
    local idx="$1"
    local labels=()
    local old_ifs="${IFS}"
    IFS=$'\037' read -r -a labels <<< "${PRESET_STAGE_LABELS:-}"
    IFS="${old_ifs}"
    if [ "${idx}" -ge 1 ] && [ "${idx}" -le "${#labels[@]}" ] && [ -n "${labels[$((idx - 1))]}" ]; then
        printf '%s' "${labels[$((idx - 1))]}"
    else
        printf 'Stage %s' "${idx}"
    fi
}

write_status() {
    local state="$1"
    local episode="$2"
    local message="$3"
    local completed="${#STAGE_END_TIMES[@]}"
    local current_stage=0
    local current_stage_text=""
    local tmp="${STATUS_FILE}.tmp"

    configure_profile
    mkdir -p "${WEB_RUNTIME_DIR}"
    if [ "${CAPTURE_RUNNING}" = "1" ] && is_truthy "${STAGED_CAPTURE}"; then
        current_stage=$((completed + 1))
        current_stage_text="$(stage_label_at "${current_stage}")"
    fi

    export STATUS_STATE="${state}"
    export STATUS_EPISODE="${episode}"
    export STATUS_MESSAGE="${message}"
    export STATUS_UPDATED_AT="$(date '+%F %T')"
    export STATUS_CAPTURE_RUNNING="${CAPTURE_RUNNING}"
    export STATUS_SERVICE_READY="${CAPTURE_SERVICE_READY}"
    export STATUS_DATA_DIR="${DATA_DIR}"
    export STATUS_ALOHA_DIR="${ALOHA_DIR}"
    export STATUS_LOG_DIR="${LOG_DIR}"
    export STATUS_PROFILE="${CAPTURE_PROFILE}"
    export STATUS_WITH_BASE="$(json_bool "${WITH_BASE}")"
    export STATUS_STAGED_CAPTURE="$(json_bool "${STAGED_CAPTURE}")"
    export STATUS_RAW_MCAP_ONLY="$(json_bool "${RAW_MCAP_ONLY}")"
    export STATUS_RUN_CONVERT="$(json_bool "${RUN_CONVERT}")"
    export STATUS_RUN_QC="$(json_bool "${RUN_QC}")"
    export STATUS_RUN_LEROBOT="$(json_bool "${RUN_LEROBOT}")"
    export STATUS_BACKGROUND_PROCESSING="$(json_bool "${BACKGROUND_PROCESSING}")"
    export STATUS_AUTOSAVE="$(json_bool "${STAGE_AUTOSAVE_ON_PRESET}")"
    export STATUS_CURRENT_STAGE="${current_stage}"
    export STATUS_COMPLETED_STAGES="${completed}"
    export STATUS_PRESET_STAGE_COUNT="${PRESET_SEGMENT_COUNT:-0}"
    export STATUS_CURRENT_STAGE_TEXT="${current_stage_text}"
    export STATUS_STAGE_LABELS="${PRESET_STAGE_LABELS:-}"
    export STATUS_PRESET_INFO_JSON="${STAGE_PRESET_INFO_JSON}"
    export STATUS_TARGET_BOTTLE_A="${TARGET_BOTTLE_A}"
    export STATUS_TARGET_BOTTLE_B="${TARGET_BOTTLE_B}"
    export STATUS_TARGET_OPTIONS_JSON="${TARGET_OPTIONS_JSON}"
    export STATUS_LEROBOT_TARGET_DIR="${LEROBOT_TARGET_DIR}"
    export STATUS_LEROBOT_DATASET_NAME="${LEROBOT_DATASET_NAME}"
    export STATUS_REPLAY_REFERENCE_ROOT="${REPLAY_REFERENCE_ROOT}"
    export STATUS_REPLAY_HDF5_ROOT="${REPLAY_HDF5_ROOT}"
    export STATUS_REPLAY_LEROBOT_ROOT="${REPLAY_LEROBOT_ROOT}"
    export STATUS_BACKGROUND_PIDS="${BACKGROUND_PIDS[*]:-}"
    export STATUS_BACKGROUND_LOGS="$(join_unit_sep "${BACKGROUND_LOGS[@]:-}")"
    export STATUS_REPLAY_RUNNING="0"
    if [ -n "${REPLAY_PID:-}" ]; then
        STATUS_REPLAY_RUNNING="1"
    fi
    export STATUS_REPLAY_RUNNING
    export STATUS_REPLAY_PID="${REPLAY_PID:-}"
    export STATUS_REPLAY_STATUS="${REPLAY_STATUS:-idle}"
    export STATUS_REPLAY_LOG="${REPLAY_LOG:-}"
    export STATUS_REPLAY_OUTPUT="${REPLAY_OUTPUT:-}"
    export STATUS_REPLAY_COMMAND="${REPLAY_COMMAND:-}"
    export STATUS_REPLAY_STARTED_AT="${REPLAY_STARTED_AT:-}"
    export STATUS_REPLAY_FINISHED_AT="${REPLAY_FINISHED_AT:-}"
    export STATUS_REPLAY_EXIT_CODE="${REPLAY_EXIT_CODE:-}"
    export STATUS_PRESET_WARNING="${PRESET_WARNING:-}"
    export STATUS_LAST_CHECK_OK="${LAST_CHECK_OK:-0}"
    export STATUS_LAST_CHECK_SUMMARY="${LAST_CHECK_SUMMARY:-尚未自检}"
    export STATUS_LAST_CHECK_DETAILS="${LAST_CHECK_DETAILS:-}"
    export STATUS_SCENE_INVENTORY_JSON="${COLLECTION_SCENE_INVENTORY_JSON}"
    export STATUS_QUALITY_REVIEW_PENDING="${QUALITY_REVIEW_PENDING}"
    export STATUS_QUALITY_AWAITING_EPISODE="${QUALITY_PENDING_EPISODE}"
    export STATUS_LAST_SAVED_EPISODE="${LAST_SAVED_EPISODE}"
    export STATUS_LAST_SAVED_DATA_DIR="${LAST_SAVED_DATA_DIR}"

    python3 - "${tmp}" <<'PY'
import json
import os
import sys

def env(name, default=""):
    return os.environ.get(name, default)

def env_int(name, default=0):
    try:
        return int(env(name, str(default)))
    except ValueError:
        return default

def env_bool(name):
    return env(name) in {"1", "true", "TRUE", "yes", "YES", "on", "ON"}

stage_labels = [item for item in env("STATUS_STAGE_LABELS").split("\x1f") if item]
background_logs = [item for item in env("STATUS_BACKGROUND_LOGS").split("\x1f") if item]
payload = {
    "state": env("STATUS_STATE"),
    "episode": env_int("STATUS_EPISODE"),
    "message": env("STATUS_MESSAGE"),
    "updated_at": env("STATUS_UPDATED_AT"),
    "capture_running": env_bool("STATUS_CAPTURE_RUNNING"),
    "service_ready": env_bool("STATUS_SERVICE_READY"),
    "data_dir": env("STATUS_DATA_DIR"),
    "aloha_dir": env("STATUS_ALOHA_DIR"),
    "log_dir": env("STATUS_LOG_DIR"),
    "profile": env("STATUS_PROFILE"),
    "with_base": env_bool("STATUS_WITH_BASE"),
    "staged_capture": env_bool("STATUS_STAGED_CAPTURE"),
    "raw_mcap_only": env_bool("STATUS_RAW_MCAP_ONLY"),
    "run_convert": env_bool("STATUS_RUN_CONVERT"),
    "run_qc": env_bool("STATUS_RUN_QC"),
    "run_lerobot": env_bool("STATUS_RUN_LEROBOT"),
    "background_processing": env_bool("STATUS_BACKGROUND_PROCESSING"),
    "auto_save_on_preset": env_bool("STATUS_AUTOSAVE"),
    "current_stage": env_int("STATUS_CURRENT_STAGE"),
    "completed_stages": env_int("STATUS_COMPLETED_STAGES"),
    "preset_stage_count": env_int("STATUS_PRESET_STAGE_COUNT"),
    "current_stage_text": env("STATUS_CURRENT_STAGE_TEXT"),
    "stage_labels": stage_labels,
    "preset_info_json": env("STATUS_PRESET_INFO_JSON"),
    "target_bottle_a": env("STATUS_TARGET_BOTTLE_A"),
    "target_bottle_b": env("STATUS_TARGET_BOTTLE_B"),
    "target_options": json.loads(env("STATUS_TARGET_OPTIONS_JSON", "[]")),
    "lerobot_target_dir": env("STATUS_LEROBOT_TARGET_DIR"),
    "lerobot_dataset_name": env("STATUS_LEROBOT_DATASET_NAME"),
    "replay_reference_root": env("STATUS_REPLAY_REFERENCE_ROOT"),
    "replay_hdf5_root": env("STATUS_REPLAY_HDF5_ROOT"),
    "replay_lerobot_root": env("STATUS_REPLAY_LEROBOT_ROOT"),
    "background_pids": [p for p in env("STATUS_BACKGROUND_PIDS").split() if p],
    "background_logs": background_logs,
    "replay_running": env_bool("STATUS_REPLAY_RUNNING"),
    "replay_pid": env("STATUS_REPLAY_PID"),
    "replay_status": env("STATUS_REPLAY_STATUS"),
    "replay_log": env("STATUS_REPLAY_LOG"),
    "replay_output": env("STATUS_REPLAY_OUTPUT"),
    "replay_command": env("STATUS_REPLAY_COMMAND"),
    "replay_started_at": env("STATUS_REPLAY_STARTED_AT"),
    "replay_finished_at": env("STATUS_REPLAY_FINISHED_AT"),
    "replay_exit_code": env("STATUS_REPLAY_EXIT_CODE"),
    "preset_warning": env("STATUS_PRESET_WARNING"),
    "last_check_ok": env_bool("STATUS_LAST_CHECK_OK"),
    "last_check_summary": env("STATUS_LAST_CHECK_SUMMARY"),
    "last_check_details": [item for item in env("STATUS_LAST_CHECK_DETAILS").split("\x1f") if item],
    "scene_inventory_json": env("STATUS_SCENE_INVENTORY_JSON"),
    "quality_review_pending": env_bool("STATUS_QUALITY_REVIEW_PENDING"),
    "quality_awaiting_episode": env("STATUS_QUALITY_AWAITING_EPISODE"),
    "last_saved_episode": env("STATUS_LAST_SAVED_EPISODE"),
    "last_saved_data_dir": env("STATUS_LAST_SAVED_DATA_DIR"),
}
with open(sys.argv[1], "w", encoding="utf-8") as f:
    json.dump(payload, f, ensure_ascii=False, indent=2)
    f.write("\n")
PY
    mv "${tmp}" "${STATUS_FILE}"
}

capture_service_request() {
    local start="$1"
    local end="$2"
    local idx="$3"
    local request
    request="{start: ${start}, end: ${end}, episode_index: ${idx}, dataset_dir: '${DATA_DIR}', instructions: '', task_name: '', task_descriptions: '', task_id: ''}"
    run_cmd ros2 service call /data_tools_dataCapture/capture_service data_msgs/srv/CaptureService "${request}"
}

capture_service_available() {
    ros2 service type /data_tools_dataCapture/capture_service >/dev/null 2>&1 && return 0
    python3 - <<'PY' >/dev/null 2>&1
import sys

try:
    import rclpy
    from data_msgs.srv import CaptureService
except Exception:
    raise SystemExit(1)

node = None
ok = False
try:
    rclpy.init(args=None)
    node = rclpy.create_node("capture_service_probe")
    client = node.create_client(CaptureService, "/data_tools_dataCapture/capture_service")
    ok = client.wait_for_service(timeout_sec=1.0)
finally:
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

raise SystemExit(0 if ok else 1)
PY
}

wait_for_capture_service() {
    local waited=0
    while [ "${waited}" -lt "${CAPTURE_SERVICE_TIMEOUT}" ]; do
        if [ -n "${CAPTURE_LAUNCH_PID}" ] && ! kill -0 "${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1; then
            echo "[错误] 采集服务进程已退出。日志: ${CAPTURE_SERVICE_LOG}" >&2
            [ -f "${CAPTURE_SERVICE_LOG}" ] && tail -100 "${CAPTURE_SERVICE_LOG}" >&2 || true
            return 1
        fi
        if capture_service_available; then
            CAPTURE_SERVICE_READY=1
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "[错误] 等待 /data_tools_dataCapture/capture_service 超时，请检查 ${CAPTURE_SERVICE_LOG}" >&2
    [ -f "${CAPTURE_SERVICE_LOG}" ] && tail -80 "${CAPTURE_SERVICE_LOG}" >&2 || true
    return 1
}

start_capture_service_launch() {
    configure_profile
    require_files
    source_ros_env
    mkdir -p "${DATA_DIR}" "${LOG_DIR}" "${BACKGROUND_LOG_DIR}" "${QC_REPORT_ROOT}"

    if [ "${CAPTURE_SERVICE_READY}" = "1" ]; then
        return 0
    fi

    if is_truthy "${REUSE_CAPTURE_SERVICE}" &&
        ros2 service type /data_tools_dataCapture/capture_service >/dev/null 2>&1; then
        CAPTURE_LAUNCH_PID=""
        CAPTURE_SERVICE_LOG="existing ROS service"
        CAPTURE_SERVICE_READY=1
        log "复用已存在的 /data_tools_dataCapture/capture_service"
        return 0
    fi

    local cmd=(
        ros2 launch data_tools "${CAPTURE_LAUNCH_FILE}"
        useService:=true
        datasetDir:="${DATA_DIR}"
        episodeIndex:="${CURRENT_EPISODE}"
        paramsFile:="${ALOHA_YAML}"
        hz:="${CAPTURE_HZ}"
        timeout:="${CAPTURE_TIMEOUT}"
    )
    if [ "${CAPTURE_PROFILE}" = "aloha_mobile" ]; then
        cmd+=(useTopicStamp:="${USE_TOPIC_STAMP}")
    fi

    CAPTURE_SERVICE_LOG="${LOG_DIR}/capture_service_$(date +%Y%m%d_%H%M%S).log"
    log "启动采集服务: ${CAPTURE_LAUNCH_FILE}"
    if command -v setsid >/dev/null 2>&1; then
        setsid "${cmd[@]}" >"${CAPTURE_SERVICE_LOG}" 2>&1 < /dev/null &
    else
        "${cmd[@]}" >"${CAPTURE_SERVICE_LOG}" 2>&1 < /dev/null &
    fi
    CAPTURE_LAUNCH_PID="$!"
    wait_for_capture_service
}

stop_capture_service_launch() {
    CAPTURE_SERVICE_READY=0
    if [ -z "${CAPTURE_LAUNCH_PID}" ]; then
        return 0
    fi
    if kill -0 "${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1; then
        log "停止采集服务"
        kill -INT "-${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1 || kill -INT "${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1 || true
        local waited=0
        while kill -0 "${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1 && [ "${waited}" -lt 30 ]; do
            sleep 0.5
            waited=$((waited + 1))
        done
        if kill -0 "${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1; then
            kill -TERM "-${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1 || kill -TERM "${CAPTURE_LAUNCH_PID}" >/dev/null 2>&1 || true
        fi
        wait "${CAPTURE_LAUNCH_PID}" 2>/dev/null || true
    fi
    CAPTURE_LAUNCH_PID=""
}

copy_instruction_sidecars() {
    local idx="$1"
    local raw_episode_dir="${DATA_DIR}/episode${idx}"
    local aloha_episode_dir="${ALOHA_DIR}/episode${idx}"
    mkdir -p "${aloha_episode_dir}"
    if [ -f "${raw_episode_dir}/instructions.json" ]; then
        cp "${raw_episode_dir}/instructions.json" "${aloha_episode_dir}/instructions.json"
    fi
    if [ -f "${raw_episode_dir}/episode${idx}_0_info.json" ]; then
        cp "${raw_episode_dir}/episode${idx}_0_info.json" "${aloha_episode_dir}/episode${idx}_0_info.json"
    fi
    return 0
}

clear_aloha_episode_output() {
    local idx="$1"
    local aloha_episode_dir="${ALOHA_DIR}/episode${idx}"
    local aloha_root aloha_path
    aloha_root="$(realpath -m "${ALOHA_DIR}")"
    aloha_path="$(realpath -m "${aloha_episode_dir}")"
    case "${aloha_path}" in
        "${aloha_root}"/episode*)
            if [ -d "${aloha_episode_dir}" ]; then
                log "清理旧的 ALOHA/HDF5 输出: ${aloha_episode_dir}"
                rm -rf "${aloha_episode_dir}"
            fi
            ;;
        *) die "拒绝删除异常路径: ${aloha_path}" ;;
    esac
}

write_staged_info_files() {
    local idx="$1"
    local episode_dir="${DATA_DIR}/episode${idx}"
    local starts ends

    if [ "${#STAGE_START_TIMES[@]}" -eq 0 ]; then
        die "没有记录到阶段时间，拒绝写空的阶段标注"
    fi
    mkdir -p "${episode_dir}"
    starts="$(join_unit_sep "${STAGE_START_TIMES[@]}")"
    ends="$(join_unit_sep "${STAGE_END_TIMES[@]}")"

    export STAGED_STARTS="${starts}"
    export STAGED_ENDS="${ends}"
    export STAGE_PRESET_INFO_JSON
    export TARGET_BOTTLE_A TARGET_BOTTLE_B TARGET_BOTTLE_A_ZH TARGET_BOTTLE_B_ZH

    python3 - "${episode_dir}" "${idx}" "${WRITE_COMPAT_INSTRUCTIONS}" "${SCRIPT_DIR}" <<'PY'
import json
import os
import sys
import uuid

episode_dir = sys.argv[1]
episode_index = sys.argv[2]
write_compat = sys.argv[3].lower() in {"1", "true", "yes", "on"}
sys.path.insert(0, sys.argv[4])

from collection_targets import render_target_prompts

left_target = os.environ.get("TARGET_BOTTLE_A", "")
right_target = os.environ.get("TARGET_BOTTLE_B", "")
sep = "\x1f"

def split_env(name):
    value = os.environ.get(name, "")
    return [] if value == "" else value.split(sep)

def first_text(value):
    if isinstance(value, list) and value:
        return str(value[0])
    if isinstance(value, str):
        return value
    return ""

def apply_placeholders(text):
    replacements = {
        "{bottle_a}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{{bottle_a}}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{BOTTLE_A}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{{BOTTLE_A}}": os.environ.get("TARGET_BOTTLE_A", "bottle A"),
        "{bottle_b}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{{bottle_b}}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{BOTTLE_B}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{{BOTTLE_B}}": os.environ.get("TARGET_BOTTLE_B", "bottle B"),
        "{bottle_a_zh}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{{bottle_a_zh}}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{BOTTLE_A_ZH}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{{BOTTLE_A_ZH}}": os.environ.get("TARGET_BOTTLE_A_ZH", os.environ.get("TARGET_BOTTLE_A", "bottle A")),
        "{bottle_b_zh}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
        "{{bottle_b_zh}}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
        "{BOTTLE_B_ZH}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
        "{{BOTTLE_B_ZH}}": os.environ.get("TARGET_BOTTLE_B_ZH", os.environ.get("TARGET_BOTTLE_B", "bottle B")),
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    text = render_target_prompts(text, left_target, right_target)
    return text

starts = split_env("STAGED_STARTS")
ends = split_env("STAGED_ENDS")
if not starts or len(starts) != len(ends):
    raise SystemExit("invalid staged timestamps")

full_zh = "null"
full_en = "null"
segment_zh = []
segment_en = []
preset_path = os.environ.get("STAGE_PRESET_INFO_JSON", "")
if preset_path and os.path.exists(preset_path):
    with open(preset_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    mark = data.get("mark", data)
    full_zh = apply_placeholders(first_text(mark.get("full-instructions-zh"))) or "null"
    full_en = apply_placeholders(first_text(mark.get("full-instructions-en"))) or "null"
    if full_zh == "null":
        full_zh = apply_placeholders(first_text(mark.get("full-instructions"))) or "null"
    if full_en == "null":
        full_en = apply_placeholders(first_text(mark.get("instructions"))) or "null"
    for segment in mark.get("segment-instructions") or []:
        zh = apply_placeholders(first_text(segment.get("description_zh")))
        en = apply_placeholders(first_text(segment.get("description_en")))
        compat = apply_placeholders(first_text(segment.get("instructions")))
        segment_zh.append(zh or compat)
        segment_en.append(en or compat)

segments = []
compat_segments = []
for i, (start_time, end_time) in enumerate(zip(starts, ends)):
    zh = segment_zh[i] if i < len(segment_zh) else f"Stage {i + 1}"
    en = segment_en[i] if i < len(segment_en) else f"Stage {i + 1}"
    segment_id = str(uuid.uuid4())
    segments.append({
        "id": segment_id,
        "start_time": start_time,
        "end_time": end_time,
        "description_zh": [zh],
        "description_en": [en],
    })
    compat_segments.append({
        "start_time": start_time,
        "end_time": end_time,
        "instructions": [en or zh],
    })

mark_payload = {
    "mark": {
        "full-instructions-zh": [full_zh],
        "full-instructions-en": [full_en],
        "segment-instructions": segments,
    }
}
info_path = os.path.join(episode_dir, f"episode{episode_index}_0_info.json")
with open(info_path, "w", encoding="utf-8") as f:
    json.dump(mark_payload, f, ensure_ascii=False, indent=2)
    f.write("\n")
print(f"wrote {info_path}")

if write_compat:
    compat_payload = {
        "full-instructions": [full_en if full_en != "null" else full_zh],
        "segment-instructions": compat_segments,
    }
    compat_path = os.path.join(episode_dir, "instructions.json")
    with open(compat_path, "w", encoding="utf-8") as f:
        json.dump(compat_payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(f"wrote {compat_path}")
PY
}

abandon_episode() {
    local idx="$1"
    local raw_episode_dir="${DATA_DIR}/episode${idx}"
    local aloha_episode_dir="${ALOHA_DIR}/episode${idx}"
    local data_root raw_path aloha_root aloha_path
    data_root="$(realpath -m "${DATA_DIR}")"
    raw_path="$(realpath -m "${raw_episode_dir}")"
    aloha_root="$(realpath -m "${ALOHA_DIR}")"
    aloha_path="$(realpath -m "${aloha_episode_dir}")"
    case "${raw_path}" in
        "${data_root}"/episode*)
            if [ -d "${raw_episode_dir}" ]; then
                rm -rf "${raw_episode_dir}"
            fi
            ;;
        *) die "拒绝删除异常路径: ${raw_path}" ;;
    esac
    case "${aloha_path}" in
        "${aloha_root}"/episode*)
            if [ -d "${aloha_episode_dir}" ]; then
                rm -rf "${aloha_episode_dir}"
            fi
            ;;
        *) die "拒绝删除异常路径: ${aloha_path}" ;;
    esac
    return 0
}

metadata_target_args() {
    METADATA_TARGET_ARGS=()
    METADATA_HAND_TARGET_ARGS=(--left-target "${TARGET_BOTTLE_A}" --right-target "${TARGET_BOTTLE_B}")
    if [ -n "${TARGET_BOTTLE_A:-}" ]; then
        METADATA_TARGET_ARGS+=(--target "${TARGET_BOTTLE_A}")
    fi
    if [ -n "${TARGET_BOTTLE_A_ZH:-}" ] && [ "${TARGET_BOTTLE_A_ZH}" != "${TARGET_BOTTLE_A:-}" ]; then
        METADATA_TARGET_ARGS+=(--target "${TARGET_BOTTLE_A_ZH}")
    fi
    if [ -n "${TARGET_BOTTLE_B:-}" ]; then
        METADATA_TARGET_ARGS+=(--target "${TARGET_BOTTLE_B}")
    fi
    if [ -n "${TARGET_BOTTLE_B_ZH:-}" ] && [ "${TARGET_BOTTLE_B_ZH}" != "${TARGET_BOTTLE_B:-}" ]; then
        METADATA_TARGET_ARGS+=(--target "${TARGET_BOTTLE_B_ZH}")
    fi
}

prepare_episode_metadata() {
    local idx="$1"
    configure_profile
    metadata_target_args
    python3 "${COLLECTION_METADATA_SCRIPT}" prepare \
        --data-dir "${DATA_DIR}" \
        --episode "${idx}" \
        "${METADATA_HAND_TARGET_ARGS[@]}" \
        "${METADATA_TARGET_ARGS[@]}"
}

finalize_quality_review_from_json() {
    local payload="$1"
    local exports
    local review_episode
    local review_action
    local review_grade
    local review_reason_note
    local review_reason_codes
    local saved_episode
    configure_profile
    if [ "${QUALITY_REVIEW_PENDING}" != "1" ] || [ -z "${QUALITY_PENDING_EPISODE}" ]; then
        write_status "idle" "${CURRENT_EPISODE}" "当前没有等待审核的 episode。"
        return 0
    fi
    if ! exports="$(python3 - "${payload}" <<'PY'
import json
import shlex
import sys

data = json.loads(sys.argv[1] or "{}")

def text(name, default=""):
    value = data.get(name, default)
    if value is None:
        return default
    return str(value).strip()

episode = text("episode_id") or text("episode")
if not episode.isdigit():
    raise SystemExit("episode_id must be a non-negative integer")

action = text("action", "review").lower()
grade = text("grade", "").upper()
if action not in {"review", "discard"}:
    raise SystemExit("action must be review or discard")
if action == "review" and grade not in {"A", "B", "F"}:
    raise SystemExit("grade must be A, B, or F")

reason_codes = data.get("reason_codes", [])
if isinstance(reason_codes, str):
    reason_codes = [item for item in reason_codes.replace(",", " ").split() if item]
elif isinstance(reason_codes, list):
    reason_codes = [str(item).strip() for item in reason_codes if str(item).strip()]
else:
    raise SystemExit("reason_codes must be an array")

values = {
    "REVIEW_EPISODE": episode,
    "REVIEW_ACTION": action,
    "REVIEW_GRADE": grade,
    "REVIEW_REASON_CODES": "\x1f".join(reason_codes),
    "REVIEW_REASON_NOTE": text("reason_note"),
}
for key, value in values.items():
    print(f"{key}={shlex.quote(value)}")
PY
    )"; then
        write_status "warning" "${CURRENT_EPISODE}" "质量审核参数无效，请重试。"
        return 0
    fi
    eval "${exports}"
    review_episode="${REVIEW_EPISODE}"
    review_action="${REVIEW_ACTION}"
    review_grade="${REVIEW_GRADE}"
    review_reason_codes="${REVIEW_REASON_CODES}"
    review_reason_note="${REVIEW_REASON_NOTE}"
    saved_episode="${QUALITY_PENDING_EPISODE}"

    if [ "${review_episode}" != "${saved_episode}" ]; then
        write_status "review" "${CURRENT_EPISODE}" "审核 episode 不匹配：当前等待 episode${saved_episode}。"
        return 0
    fi

    if [ "${review_action}" = "discard" ]; then
        abandon_episode "${saved_episode}"
        QUALITY_REVIEW_PENDING=0
        QUALITY_PENDING_EPISODE=""
        LAST_SAVED_EPISODE=""
        LAST_SAVED_DATA_DIR=""
        reset_stage_state
        write_status "idle" "${CURRENT_EPISODE}" "episode${saved_episode} 已放弃，编号不增加。"
        return 0
    fi

    metadata_target_args
    local cmd=(
        python3 "${COLLECTION_METADATA_SCRIPT}" review
        --data-dir "${DATA_DIR}"
        --aloha-dir "${ALOHA_DIR}"
        --episode "${saved_episode}"
        --grade "${review_grade}"
        --reason-note "${review_reason_note}"
        --source collection_web
        "${METADATA_HAND_TARGET_ARGS[@]}"
        "${METADATA_TARGET_ARGS[@]}"
    )
    if [ -n "${COLLECTION_SCENE_INVENTORY_JSON:-}" ]; then
        cmd+=(--inventory-json "${COLLECTION_SCENE_INVENTORY_JSON}")
    fi
    local old_ifs="${IFS}"
    IFS=$'\037' read -r -a REVIEW_REASON_CODE_ITEMS <<< "${review_reason_codes}"
    IFS="${old_ifs}"
    local code
    for code in "${REVIEW_REASON_CODE_ITEMS[@]:-}"; do
        [ -n "${code}" ] && cmd+=(--reason-code "${code}")
    done
    if ! "${cmd[@]}"; then
        write_status "review" "${CURRENT_EPISODE}" "episode${saved_episode} 质量信息写入失败，请重试。"
        return 0
    fi
    CURRENT_EPISODE=$((CURRENT_EPISODE + 1))
    QUALITY_REVIEW_PENDING=0
    QUALITY_PENDING_EPISODE=""
    LAST_SAVED_EPISODE="${saved_episode}"
    LAST_SAVED_DATA_DIR="${DATA_DIR}"
    reset_stage_state
    write_status "idle" "${CURRENT_EPISODE}" "episode${saved_episode} 已记录质量等级 ${review_grade}，等待开始 episode${CURRENT_EPISODE}。"
    start_saved_episode_processing "${saved_episode}"
}

qc_aloha_hdf5_episode() {
    local idx="$1"
    local hdf5_path="$2"
    local qc_report_dir="${QC_REPORT_ROOT}/episode${idx}_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "${qc_report_dir}"
    python3 - "${hdf5_path}" "${MIN_FRAMES}" >"${qc_report_dir}/summary.txt" <<'PY'
import sys
from pathlib import Path

import h5py

hdf5_path = Path(sys.argv[1])
min_frames = int(sys.argv[2])
required = [
    "camera/color/left",
    "camera/color/front",
    "camera/color/right",
    "arm/jointStatePosition/masterLeft",
    "arm/jointStatePosition/masterRight",
    "arm/jointStatePosition/puppetLeft",
    "arm/jointStatePosition/puppetRight",
]

print(f"hdf5: {hdf5_path}")
if not hdf5_path.exists():
    raise SystemExit(f"missing hdf5: {hdf5_path}")
print(f"size_mb: {hdf5_path.stat().st_size / 1024 / 1024:.2f}")
with h5py.File(hdf5_path, "r") as root:
    lengths = {}
    for key in required:
        if key not in root:
            raise SystemExit(f"missing dataset: {key}")
        shape = root[key].shape
        lengths[key] = int(shape[0]) if shape else 0
        print(f"{key}: shape={shape}")
    min_len = min(lengths.values())
    if min_len < min_frames:
        raise SystemExit(f"too few frames: {min_len} < {min_frames}")
    print(f"min_frames: {min_len}")
print("qc: PASS")
PY
    echo "QC report: ${qc_report_dir}"
}

run_lerobot_conversion() {
    is_truthy "${RUN_LEROBOT}" || return 0
    mkdir -p "${LEROBOT_TARGET_DIR}" "${LOG_DIR}"
    local lock_dir="${LOG_DIR}/lerobot.lock"
    while ! mkdir "${lock_dir}" 2>/dev/null; do
        log "等待 LeRobot 转换锁: ${lock_dir}"
        sleep 5
    done
    (
        cd "${DATA_TOOLS_SCRIPTS}"
        run_cmd env \
            OPENCV_NUM_THREADS="${OPENCV_NUM_THREADS:-1}" \
            OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}" \
            OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}" \
            MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}" \
            "${LEROBOT_PYTHON}" hdf5_to_lerobot.py \
            --datasetDir "${ALOHA_DIR}" \
            --datasetName "${LEROBOT_DATASET_NAME}" \
            --type "${HDF5_TYPE}" \
            --targetDir "${LEROBOT_TARGET_DIR}" \
            --robotType "${LEROBOT_ROBOT_TYPE}" \
            --fps "${LEROBOT_FPS}"
    )
    local rc=$?
    rm -rf "${lock_dir}"
    return "${rc}"
}

process_episode() {
    local idx="$1"
    local out="${ALOHA_DIR}/episode${idx}/episode${idx}.hdf5"
    configure_profile
    require_files
    source_ros_env
    mkdir -p "${DATA_DIR}" "${ALOHA_DIR}" "${QC_REPORT_ROOT}"

    log "开始处理 episode${idx}"
    if is_truthy "${RUN_CONVERT}"; then
        clear_aloha_episode_output "${idx}"
        (
            cd "${DATA_TOOLS_SCRIPTS}"
            run_cmd python3 mcap_to_aloha_data.py \
                --datasetDir "${DATA_DIR}" \
                --episodeIndex "${idx}" \
                --alohaYaml "${ALOHA_YAML}"
        )
        copy_instruction_sidecars "${idx}"

        run_cmd ros2 launch data_tools run_data_sync.launch.py \
            datasetDir:="${ALOHA_DIR}" \
            episodeIndex:="${idx}" \
            type:="${HDF5_TYPE}" \
            paramsFile:="${ALOHA_YAML}" \
            timeDiffLimit:="${TIME_DIFF_LIMIT}"

        (
            cd "${DATA_TOOLS_SCRIPTS}"
            run_cmd python3 data_to_hdf5.py \
                --datasetDir "${ALOHA_DIR}" \
                --episodeName "episode${idx}" \
                --targetDir "${ALOHA_DIR}" \
                --type "${HDF5_TYPE}" \
                --paramsFile "${ALOHA_YAML}"
        )
    fi

    if is_truthy "${RUN_QC}"; then
        if is_truthy "${WITH_BASE}"; then
            local qc_report_dir="${QC_REPORT_ROOT}/episode${idx}_$(date +%Y%m%d_%H%M%S)"
            run_cmd mkdir -p "${qc_report_dir}"
            local qc_cmd=(
                python3 "${SCRIPT_DIR}/verify_mobile_hdf5.py"
                --input "${out}"
                --output "${qc_report_dir}"
                --min-frames "${MIN_FRAMES}"
                --min-effective-fps "${MOBILE_MIN_EFFECTIVE_FPS}"
            )
            if [ -f "${DATA_DIR}/episode${idx}/capture_health.json" ]; then
                qc_cmd+=(--capture-health "${DATA_DIR}/episode${idx}/capture_health.json")
            elif is_truthy "${ALLOW_MISSING_CAPTURE_HEALTH}"; then
                log "警告: episode${idx} 缺少 capture_health.json，仅执行同步后 HDF5 检查。"
            else
                die "episode${idx} 缺少 capture_health.json，无法验证全程相机覆盖率；如需处理旧数据，请显式设置 ALLOW_MISSING_CAPTURE_HEALTH=1。"
            fi
            run_cmd "${qc_cmd[@]}"
            echo "QC report: ${qc_report_dir}"
        else
            qc_aloha_hdf5_episode "${idx}" "${out}"
        fi
    fi

    run_lerobot_conversion
    log "episode${idx} 处理完成"
}

prune_background_jobs() {
    local active=()
    local active_logs=()
    local i pid
    for i in "${!BACKGROUND_PIDS[@]}"; do
        pid="${BACKGROUND_PIDS[$i]}"
        if kill -0 "${pid}" >/dev/null 2>&1; then
            active+=("${pid}")
            active_logs+=("${BACKGROUND_LOGS[$i]:-}")
        else
            wait "${pid}" 2>/dev/null || true
        fi
    done
    BACKGROUND_PIDS=("${active[@]}")
    BACKGROUND_LOGS=("${active_logs[@]}")
}

wait_for_background_slot() {
    prune_background_jobs
    while [ "${#BACKGROUND_PIDS[@]}" -ge "${MAX_BACKGROUND_JOBS}" ]; do
        log "后台处理任务已达上限(${MAX_BACKGROUND_JOBS})，等待空位..."
        sleep 2
        prune_background_jobs
    done
}

start_saved_episode_processing() {
    local idx="$1"
    if ! is_truthy "${RUN_CONVERT}" && ! is_truthy "${RUN_QC}" && ! is_truthy "${RUN_LEROBOT}"; then
        return 0
    fi
    if is_truthy "${BACKGROUND_PROCESSING}"; then
        wait_for_background_slot
        mkdir -p "${BACKGROUND_LOG_DIR}"
        local log_file="${BACKGROUND_LOG_DIR}/episode${idx}_$(date +%Y%m%d_%H%M%S).log"
        log "episode${idx} 后台处理启动: ${log_file}"
        if command -v setsid >/dev/null 2>&1; then
            setsid env \
                PROCESS_ONLY=1 \
                DATA_ROS_WS="${DATA_ROS_WS}" \
                DATA_DIR="${DATA_DIR}" \
                ALOHA_YAML="${ALOHA_YAML}" \
                MOBILE_YAML="${MOBILE_YAML}" \
                WITH_BASE="${WITH_BASE}" \
                STAGED_CAPTURE="${STAGED_CAPTURE}" \
                RUN_CONVERT="${RUN_CONVERT}" \
                RUN_QC="${RUN_QC}" \
                RUN_LEROBOT="${RUN_LEROBOT}" \
                LEROBOT_PYTHON="${LEROBOT_PYTHON}" \
                LEROBOT_TARGET_DIR="${LEROBOT_TARGET_DIR}" \
                LEROBOT_DATASET_NAME="${LEROBOT_DATASET_NAME}" \
                LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE}" \
                LEROBOT_FPS="${LEROBOT_FPS}" \
                TIME_DIFF_LIMIT="${TIME_DIFF_LIMIT}" \
                MOBILE_MIN_EFFECTIVE_FPS="${MOBILE_MIN_EFFECTIVE_FPS}" \
                TOPIC_CHECK_SCRIPT="${TOPIC_CHECK_SCRIPT}" \
                PIPER_RESET_SCRIPT="${PIPER_RESET_SCRIPT}" \
                FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
                bash "${SCRIPT_PATH}" "${idx}" >"${log_file}" 2>&1 < /dev/null &
        else
            env \
                PROCESS_ONLY=1 \
                DATA_ROS_WS="${DATA_ROS_WS}" \
                DATA_DIR="${DATA_DIR}" \
                ALOHA_YAML="${ALOHA_YAML}" \
                MOBILE_YAML="${MOBILE_YAML}" \
                WITH_BASE="${WITH_BASE}" \
                STAGED_CAPTURE="${STAGED_CAPTURE}" \
                RUN_CONVERT="${RUN_CONVERT}" \
                RUN_QC="${RUN_QC}" \
                RUN_LEROBOT="${RUN_LEROBOT}" \
                LEROBOT_PYTHON="${LEROBOT_PYTHON}" \
                LEROBOT_TARGET_DIR="${LEROBOT_TARGET_DIR}" \
                LEROBOT_DATASET_NAME="${LEROBOT_DATASET_NAME}" \
                LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE}" \
                LEROBOT_FPS="${LEROBOT_FPS}" \
                TIME_DIFF_LIMIT="${TIME_DIFF_LIMIT}" \
                MOBILE_MIN_EFFECTIVE_FPS="${MOBILE_MIN_EFFECTIVE_FPS}" \
                TOPIC_CHECK_SCRIPT="${TOPIC_CHECK_SCRIPT}" \
                PIPER_RESET_SCRIPT="${PIPER_RESET_SCRIPT}" \
                FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
                bash "${SCRIPT_PATH}" "${idx}" >"${log_file}" 2>&1 < /dev/null &
        fi
        BACKGROUND_PIDS+=("$!")
        BACKGROUND_LOGS+=("${log_file}")
    else
        process_episode "${idx}"
    fi
}

prune_replay_job() {
    if [ -z "${REPLAY_PID:-}" ]; then
        return 0
    fi
    if kill -0 "${REPLAY_PID}" >/dev/null 2>&1; then
        return 0
    fi
    local exit_code=0
    wait "${REPLAY_PID}" 2>/dev/null || exit_code=$?
    REPLAY_EXIT_CODE="${exit_code}"
    REPLAY_FINISHED_AT="$(date '+%F %T')"
    REPLAY_PID=""
    if [ "${exit_code}" -eq 0 ]; then
        REPLAY_STATUS="done"
        write_status "idle" "${CURRENT_EPISODE}" "回放任务完成。"
    else
        REPLAY_STATUS="failed"
        write_status "warning" "${CURRENT_EPISODE}" "回放任务失败，退出码 ${exit_code}，请查看日志。"
    fi
}

stop_replay_job() {
    if [ -z "${REPLAY_PID:-}" ]; then
        REPLAY_STATUS="idle"
        write_status "idle" "${CURRENT_EPISODE}" "当前没有运行中的回放任务。"
        return 0
    fi
    log "停止回放任务 pid=${REPLAY_PID}"
    kill -INT "-${REPLAY_PID}" >/dev/null 2>&1 || kill -INT "${REPLAY_PID}" >/dev/null 2>&1 || true
    local waited=0
    while kill -0 "${REPLAY_PID}" >/dev/null 2>&1 && [ "${waited}" -lt 30 ]; do
        sleep 0.1
        waited=$((waited + 1))
    done
    if kill -0 "${REPLAY_PID}" >/dev/null 2>&1; then
        kill -TERM "-${REPLAY_PID}" >/dev/null 2>&1 || kill -TERM "${REPLAY_PID}" >/dev/null 2>&1 || true
        waited=0
        while kill -0 "${REPLAY_PID}" >/dev/null 2>&1 && [ "${waited}" -lt 20 ]; do
            sleep 0.1
            waited=$((waited + 1))
        done
    fi
    if kill -0 "${REPLAY_PID}" >/dev/null 2>&1; then
        kill -KILL "-${REPLAY_PID}" >/dev/null 2>&1 || kill -KILL "${REPLAY_PID}" >/dev/null 2>&1 || true
    fi
    wait "${REPLAY_PID}" 2>/dev/null || true
    REPLAY_PID=""
    REPLAY_STATUS="stopped"
    REPLAY_FINISHED_AT="$(date '+%F %T')"
    REPLAY_EXIT_CODE="stopped"
    write_status "idle" "${CURRENT_EPISODE}" "回放任务已停止。"
}

parse_replay_payload() {
    local payload="$1"
    python3 - "${payload}" <<'PY'
import json
import shlex
import sys

data = json.loads(sys.argv[1] or "{}")

def text(name, default=""):
    value = data.get(name, default)
    if value is None:
        return default
    return str(value).strip()

def number(name, default):
    value = text(name, str(default))
    try:
        parsed = float(value)
    except ValueError:
        raise SystemExit(f"{name} must be numeric")
    return str(parsed)

def integer(name, default):
    value = text(name, str(default))
    try:
        parsed = int(float(value))
    except ValueError:
        raise SystemExit(f"{name} must be an integer")
    return str(max(parsed, 0))

values = {
    "REPLAY_FORMAT_IN": text("format", "hdf5").lower(),
    "REPLAY_MODE_IN": text("mode", "mobile").lower(),
    "REPLAY_PATH_IN": text("path", ""),
    "REPLAY_EPISODE_IN": integer("episode", 0),
    "REPLAY_RATE_IN": number("rate", 1.0),
    "REPLAY_FPS_IN": number("fps", 30.0),
    "REPLAY_SOURCE_IN": text("source", ""),
    "REPLAY_BASE_SOURCE_IN": text("base_source", "auto").lower(),
    "REPLAY_MAX_LINEAR_IN": number("max_linear", 0.3),
    "REPLAY_MAX_ANGULAR_IN": number("max_angular", 0.8),
    "REPLAY_MAX_FRAMES_IN": integer("max_frames", 0),
}
for key, value in values.items():
    print(f"{key}={shlex.quote(value)}")
PY
}

run_replay_background() {
    local kind="$1"
    shift
    local cmd=("$@")
    mkdir -p "${LOG_DIR}/replay"
    REPLAY_LOG="${LOG_DIR}/replay/${kind}_$(date +%Y%m%d_%H%M%S).log"
    REPLAY_COMMAND="$(printf '%q ' "${cmd[@]}")"
    REPLAY_STATUS="running"
    REPLAY_STARTED_AT="$(date '+%F %T')"
    REPLAY_FINISHED_AT=""
    REPLAY_EXIT_CODE=""
    log "启动回放任务: ${REPLAY_LOG}"
    print_cmd "${cmd[@]}" >"${REPLAY_LOG}"
    if command -v setsid >/dev/null 2>&1; then
        setsid "${cmd[@]}" >>"${REPLAY_LOG}" 2>&1 < /dev/null &
    else
        "${cmd[@]}" >>"${REPLAY_LOG}" 2>&1 < /dev/null &
    fi
    REPLAY_PID="$!"
}

resolve_hdf5_replay_path() {
    local replay_path="$1"
    local replay_episode="$2"
    python3 - "${replay_path}" "${replay_episode}" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1]).expanduser().resolve()
episode = str(sys.argv[2] or "0")
episode_name = episode if episode.startswith("episode") else f"episode{int(episode)}"

if path.is_file():
    print(path)
    raise SystemExit(0)
if not path.is_dir():
    raise SystemExit(f"HDF5 path does not exist: {path}")

def aligned_files(episode_dir):
    return (
        episode_dir / "states" / "aligned_joints.h5",
        episode_dir / "states" / "aligned_joints.hdf5",
        episode_dir / "aligned_joints.h5",
        episode_dir / "aligned_joints.hdf5",
    )

for candidate in aligned_files(path):
    if candidate.is_file():
        print(candidate)
        raise SystemExit(0)

containers = [path]
hdf5_episodes = path / "hdf5_episodes"
if hdf5_episodes.is_dir():
    containers.append(hdf5_episodes)

candidates = []
for container in containers:
    candidates.extend(aligned_files(container / episode_name))
    try:
        dataset_dirs = [item for item in container.iterdir() if item.is_dir()]
    except OSError:
        dataset_dirs = []
    for dataset_dir in dataset_dirs:
        candidates.extend(aligned_files(dataset_dir / episode_name))

matches = sorted({candidate for candidate in candidates if candidate.is_file()})
if len(matches) == 1:
    print(matches[0])
    raise SystemExit(0)
if not matches:
    raise SystemExit(f"{episode_name}/states/aligned_joints.h5 not found under {path}")
raise SystemExit("Multiple HDF5 matches found:\n" + "\n".join(str(item) for item in matches))
PY
}

resolve_lerobot_replay_target() {
    local replay_path="$1"
    local replay_episode="$2"
    python3 - "${replay_path}" "${replay_episode}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1]).expanduser().resolve()
episode_text = str(sys.argv[2] or "0")
episode_number = int(episode_text.removeprefix("episode"))
source_name = f"episode{episode_number}"

if path.is_file():
    print(f"{path}\x1f{episode_number}")
    raise SystemExit(0)
if not path.is_dir():
    raise SystemExit(f"LeRobot path does not exist: {path}")

direct_mapping = path / "meta" / "episode_name_mapping.json"
if direct_mapping.is_file():
    mapping_files = [direct_mapping]
else:
    mapping_files = sorted(path.glob("**/meta/episode_name_mapping.json"))

matches = []
for mapping_file in mapping_files:
    try:
        payload = json.loads(mapping_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        continue
    dataset_root = mapping_file.parent.parent
    for row in payload.get("episodes", []):
        if str(row.get("source_episode_name", "")) != source_name:
            continue
        rel = str(row.get("lerobot_data_file", "")).strip()
        index = int(row.get("lerobot_episode_index", row.get("grade_episode_index", 0)))
        parquet = dataset_root / rel if rel else dataset_root / f"data/chunk-{index // 1000:03d}/episode_{index:06d}.parquet"
        if parquet.is_file():
            matches.append((parquet.resolve(), index))

unique = sorted(set(matches), key=lambda item: str(item[0]))
if len(unique) == 1:
    print(f"{unique[0][0]}\x1f{unique[0][1]}")
    raise SystemExit(0)
if len(unique) > 1:
    raise SystemExit("Multiple LeRobot mappings found for " + source_name)
if mapping_files:
    raise SystemExit(f"No LeRobot mapping for {source_name} under {path}")

# Compatibility for a generic, ungraded LeRobot dataset without a source-name
# mapping: in that case the UI episode is already the dataset episode_index.
dataset_roots = []
if (path / "meta" / "info.json").is_file():
    dataset_roots = [path]
else:
    dataset_roots = sorted(info.parent.parent for info in path.glob("**/meta/info.json"))
if len(dataset_roots) == 1:
    dataset_root = dataset_roots[0]
    info = json.loads((dataset_root / "meta" / "info.json").read_text(encoding="utf-8"))
    chunk_size = int(info.get("chunks_size", 1000))
    template = info.get("data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet")
    rel = template.format(episode_chunk=episode_number // chunk_size, episode_index=episode_number)
    parquet = dataset_root / rel
    if not parquet.is_file():
        found = sorted(dataset_root.glob(f"data/**/episode_{episode_number:06d}.parquet"))
        parquet = found[0] if found else parquet
    if parquet.is_file():
        print(f"{parquet.resolve()}\x1f{episode_number}")
        raise SystemExit(0)

raise SystemExit(f"No LeRobot mapping for {source_name} under {path}")
PY
}

validate_replay_target() {
    local replay_format="$1"
    local replay_path="$2"
    local replay_episode="$3"

    if [ "${replay_format}" = "mcap" ]; then
        if python3 - "${replay_path}" "${replay_episode}" <<'PY' >/dev/null 2>&1
import sys
from pathlib import Path

path = Path(sys.argv[1]).expanduser().resolve()
episode = str(sys.argv[2])
if path.is_file() and path.suffix == ".mcap":
    raise SystemExit(0)
if path.is_dir() and list(path.glob("*.mcap")) and (path / "metadata.yaml").is_file():
    raise SystemExit(0)
name = episode if episode.startswith("episode") else f"episode{episode}"
candidate = path / name
raise SystemExit(0 if candidate.is_dir() and list(candidate.glob("*.mcap")) else 1)
PY
        then
            return 0
        fi
        write_status "warning" "${CURRENT_EPISODE}" "MCAP episode 不存在: path=${replay_path}, episode=${replay_episode}。"
        return 1
    fi

    if [ "${replay_format}" = "hdf5" ]; then
        if [ -f "${replay_path}" ] && python3 - "${replay_path}" <<'PY' >/dev/null 2>&1
import sys
import h5py

path = sys.argv[1]
with h5py.File(path, "r") as f:
    for key in ("size", "timestamp", "arm/jointStatePosition/masterLeft", "arm/jointStatePosition/masterRight"):
        if key not in f:
            raise SystemExit(f"missing {key}")
PY
        then
            return 0
        fi
        if is_agibot_hdf5_target "${replay_path}" "${replay_episode}"; then
            return 0
        fi
        write_status "warning" "${CURRENT_EPISODE}" "HDF5 路径还不能读取或未生成完整: ${replay_path}；支持 episode.hdf5、hdf5_episodes 根目录、episode 目录或 states/aligned_joints.h5。"
        return 1
    fi

    if [ ! -e "${replay_path}" ]; then
        write_status "warning" "${CURRENT_EPISODE}" "LeRobot 路径不存在: ${replay_path}；如果刚保存，请等 LeRobot 转换完成后再回放。"
        return 1
    fi
    if [ -d "${replay_path}" ]; then
        local episode_file
        episode_file="$(printf 'episode_%06d.parquet' "${replay_episode}")"
        if ! find "${replay_path}/data" -type f -name "${episode_file}" -print -quit 2>/dev/null | grep -q .; then
            write_status "warning" "${CURRENT_EPISODE}" "LeRobot episode 不存在: ${episode_file} in ${replay_path}；请先完成 LeRobot 转换。"
            return 1
        fi
    fi
    return 0
}

is_agibot_hdf5_target() {
    local replay_path="$1"
    local replay_episode="$2"
    python3 - "${replay_path}" "${replay_episode}" <<'PY' >/dev/null 2>&1
import sys
from pathlib import Path
import h5py

path = Path(sys.argv[1]).expanduser().resolve()
episode = str(sys.argv[2] or "0")

def natural_key(item):
    import re
    out = []
    for part in re.split(r"(\d+)", item.name):
        if part:
            out.append(int(part) if part.isdigit() else part.lower())
    return out

def resolve(path, episode):
    if path.is_file():
        return path
    for candidate in (
        path / "states" / "aligned_joints.h5",
        path / "states" / "aligned_joints.hdf5",
        path / "aligned_joints.h5",
        path / "aligned_joints.hdf5",
    ):
        if candidate.is_file():
            return candidate
    episode_dirs = sorted(
        [item for item in path.iterdir() if item.is_dir() and (item / "states" / "aligned_joints.h5").is_file()],
        key=natural_key,
    )
    if not episode_dirs:
        raise FileNotFoundError(path)
    if episode.isdigit():
        return episode_dirs[int(episode)] / "states" / "aligned_joints.h5"
    by_name = {item.name: item for item in episode_dirs}
    return by_name[episode] / "states" / "aligned_joints.h5"

h5_path = resolve(path, episode)
with h5py.File(h5_path, "r") as f:
    frame_keys = sorted((key for key in f.keys() if key.isdigit()), key=lambda key: int(key))
    if not frame_keys:
        raise SystemExit("missing frame groups")
    first = f[frame_keys[0]]
    for key in ("main_timestamp", "action/joint/position"):
        if key not in first:
            raise SystemExit(f"missing {key}")
PY
}

start_replay_from_json() {
    local kind="$1"
    local payload="$2"
    configure_profile
    prune_replay_job
    if [ -n "${REPLAY_PID:-}" ]; then
        write_status "warning" "${CURRENT_EPISODE}" "已有回放任务在运行，请先停止。"
        return 0
    fi
    if [ "${CAPTURE_RUNNING}" = "1" ] && [ "${kind}" = "robot" ]; then
        write_status "warning" "${CURRENT_EPISODE}" "正在采集时不启动真机回放。"
        return 0
    fi

    local exports
    exports="$(parse_replay_payload "${payload}")"
    eval "${exports}"

    local replay_format="${REPLAY_FORMAT_IN}"
    local replay_mode="${REPLAY_MODE_IN}"
    local replay_path="${REPLAY_PATH_IN}"
    local replay_episode="${REPLAY_EPISODE_IN}"
    local replay_rate="${REPLAY_RATE_IN}"
    local replay_fps="${REPLAY_FPS_IN}"
    local replay_source="${REPLAY_SOURCE_IN}"
    local replay_base_source="${REPLAY_BASE_SOURCE_IN}"
    local replay_max_linear="${REPLAY_MAX_LINEAR_IN}"
    local replay_max_angular="${REPLAY_MAX_ANGULAR_IN}"
    local replay_max_frames="${REPLAY_MAX_FRAMES_IN}"
    local requested_episode="${REPLAY_EPISODE_IN}"

    case "${replay_format}" in
        mcap|hdf5|lerobot) ;;
        *) write_status "warning" "${CURRENT_EPISODE}" "未知回放格式: ${replay_format}"; return 0 ;;
    esac
    case "${replay_mode}" in
        mobile|arms) ;;
        *) write_status "warning" "${CURRENT_EPISODE}" "未知回放模式: ${replay_mode}"; return 0 ;;
    esac
    case "${replay_base_source}" in
        auto|raw|hdf5|odom) ;;
        *) replay_base_source="auto" ;;
    esac

    if [ -z "${replay_path}" ]; then
        if [ "${replay_format}" = "mcap" ]; then
            replay_path="${DATA_DIR}"
        elif [ "${replay_format}" = "hdf5" ]; then
            replay_path="${REPLAY_HDF5_ROOT}"
        else
            replay_path="${REPLAY_LEROBOT_ROOT}"
        fi
    fi
    if [ "${replay_format}" = "hdf5" ]; then
        local resolved_hdf5
        if ! resolved_hdf5="$(resolve_hdf5_replay_path "${replay_path}" "${requested_episode}" 2>&1)"; then
            write_status "warning" "${CURRENT_EPISODE}" "HDF5 episode${requested_episode} 不可用: ${resolved_hdf5}"
            return 0
        fi
        replay_path="${resolved_hdf5}"
    elif [ "${replay_format}" = "lerobot" ]; then
        local resolved_lerobot
        if ! resolved_lerobot="$(resolve_lerobot_replay_target "${replay_path}" "${requested_episode}" 2>&1)"; then
            write_status "warning" "${CURRENT_EPISODE}" "LeRobot episode${requested_episode} 不可用: ${resolved_lerobot}"
            return 0
        fi
        IFS=$'\037' read -r replay_path replay_episode <<< "${resolved_lerobot}"
    fi
    if ! validate_replay_target "${replay_format}" "${replay_path}" "${replay_episode}"; then
        return 0
    fi

    local cmd=()
    if [ "${kind}" = "visualize" ]; then
        if [ "${replay_format}" = "mcap" ]; then
            write_status "warning" "${CURRENT_EPISODE}" "原始 MCAP 当前只提供真机回放；视频查看请使用质检模块。"
            return 0
        fi
        if [ "${replay_format}" = "hdf5" ] && is_agibot_hdf5_target "${replay_path}" "${replay_episode}"; then
            write_status "warning" "${CURRENT_EPISODE}" \
                "aligned_joints.h5 在采集页只用于真机回放；视频查看请使用后续质检模块。"
            return 0
        fi
        REPLAY_OUTPUT="${LOG_DIR}/replay/${replay_format}_episode${requested_episode}_$(date +%Y%m%d_%H%M%S).mp4"
        if [ "${replay_format}" = "hdf5" ]; then
            cmd=(python3 "${REPO_ROOT}/scripts/replay/replay_hdf5.py" "${replay_path}" --rate "${replay_fps}" --output "${REPLAY_OUTPUT}")
            if [ "${replay_max_frames}" -gt 0 ]; then
                cmd+=(--max-frames "${replay_max_frames}")
            fi
        else
            cmd=(python3 "${REPO_ROOT}/scripts/replay/visualize_lerobot.py" "${replay_path}" --episode "${replay_episode}" --fps "${replay_fps}" --output "${REPLAY_OUTPUT}")
            if [ "${replay_max_frames}" -gt 0 ]; then
                cmd+=(--max-frames "${replay_max_frames}")
            fi
        fi
        run_replay_background "visualize_${replay_format}" "${cmd[@]}"
        write_status "replay" "${CURRENT_EPISODE}" "已启动${replay_format}可视化，输出: ${REPLAY_OUTPUT}"
        return 0
    fi

    source_ros_env
    REPLAY_OUTPUT=""
    if [ "${replay_format}" = "mcap" ]; then
        case "${replay_source}" in
            puppet) ;;
            *) replay_source="master" ;;
        esac
        cmd=(
            python3 "${REPO_ROOT}/scripts/replay/replay_mcap_mobile_to_robot.py" "${replay_path}"
            --episode "${replay_episode}"
            --rate "${replay_rate}"
            --source "${replay_source}"
            --publish-hz 30
            --arm-interp linear
            --base-interp linear
            --max-linear "${replay_max_linear}"
            --max-angular "${replay_max_angular}"
            --no-confirm
        )
        if [ "${replay_mode}" = "arms" ]; then
            cmd+=(--disable-base --disable-lift)
        fi
    elif [ "${replay_format}" = "hdf5" ]; then
        if is_agibot_hdf5_target "${replay_path}" "${replay_episode}"; then
            local agibot_source="action"
            local agibot_base_source="action"
            case "${replay_source}" in
                state|puppet) agibot_source="state" ;;
                *) agibot_source="action" ;;
            esac
            case "${replay_base_source}" in
                odom) agibot_base_source="state" ;;
                *) agibot_base_source="action" ;;
            esac
            cmd=(
                python3 "${REPO_ROOT}/scripts/replay/replay_agibot_mobile_to_robot.py" "${replay_path}"
                --episode "${replay_episode}"
                --rate "${replay_rate}"
                --source "${agibot_source}"
                --base-source "${agibot_base_source}"
                --lift-source action
                --publish-hz 30
                --arm-interp linear
                --base-interp linear
                --max-linear "${replay_max_linear}"
                --max-angular "${replay_max_angular}"
                --no-confirm
            )
        else
            case "${replay_source}" in
                puppet) ;;
                *) replay_source="master" ;;
            esac
            cmd=(
                python3 "${REPO_ROOT}/scripts/replay/replay_mobile_to_robot.py" "${replay_path}"
                --rate "${replay_rate}"
                --source "${replay_source}"
                --arm-publish-hz 50
                --arm-interp linear
                --base-source "${replay_base_source}"
                --base-publish-hz 30
                --base-interp linear
                --max-linear "${replay_max_linear}"
                --max-angular "${replay_max_angular}"
                --no-confirm
            )
        fi
        if [ "${replay_mode}" = "arms" ]; then
            cmd+=(--disable-base --disable-lift)
        fi
    else
        case "${replay_source}" in
            state) ;;
            *) replay_source="action" ;;
        esac
        cmd=(
            python3 "${REPO_ROOT}/scripts/replay/replay_lerobot_mobile_to_robot.py" "${replay_path}"
            --episode "${replay_episode}"
            --rate "${replay_rate}"
            --source "${replay_source}"
            --publish-hz 30
            --max-linear "${replay_max_linear}"
            --max-angular "${replay_max_angular}"
            --no-confirm
        )
        if [ "${replay_mode}" = "arms" ]; then
            cmd+=(--disable-base --disable-lift)
        fi
    fi
    run_replay_background "robot_${replay_format}_${replay_mode}" "${cmd[@]}"
    write_status "replay" "${CURRENT_EPISODE}" "已启动真机回放: ${replay_format}/${replay_mode}, episode${requested_episode}。"
}

apply_web_config_json() {
    local payload="$1"
    local exports
    local old_check_scope
    local new_check_scope
    local old_service_scope
    local new_service_scope
    if [ "${CAPTURE_RUNNING}" = "1" ]; then
        write_status "recording" "${CURRENT_EPISODE}" "正在采集中，不能修改配置。"
        return 0
    fi
    if [ "${QUALITY_REVIEW_PENDING}" = "1" ]; then
        write_status "review" "${CURRENT_EPISODE}" "episode${QUALITY_PENDING_EPISODE} 正在等待质量审核，不能修改配置。"
        return 0
    fi
    old_check_scope="$(check_scope_fingerprint)"
    old_service_scope="$(capture_service_scope_fingerprint)"

    if ! exports="$(python3 - "${payload}" "${TARGET_BOTTLE_A}" "${TARGET_BOTTLE_B}" "${SCRIPT_DIR}" <<'PY'
import json
import shlex
import sys

data = json.loads(sys.argv[1] or "{}")
current_target_a = sys.argv[2]
current_target_b = sys.argv[3]
sys.path.insert(0, sys.argv[4])

from collection_targets import validate_target

def text(name, default=""):
    value = data.get(name, default)
    if value is None:
        return default
    return str(value).strip()

def flag(name, default=False):
    value = data.get(name, default)
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (int, float)):
        return "1" if value else "0"
    return "1" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "0"

episode = text("episode_index", "")
if episode:
    if not episode.isdigit():
        raise SystemExit("episode_index must be a non-negative integer")

values = {
    "DATA_DIR": text("data_dir", "/home/agilex/data/market_2"),
    "CURRENT_EPISODE": episode or "0",
    "WITH_BASE": flag("with_base", True),
    "STAGED_CAPTURE": flag("staged_capture", True),
    "STAGE_AUTOSAVE_ON_PRESET": flag("auto_save_on_preset", True),
    "RUN_CONVERT": "0",
    "RUN_QC": "0",
    "RUN_LEROBOT": "0",
    "BACKGROUND_PROCESSING": "0",
    "STAGE_PRESET_INFO_JSON": text("preset_info_json", ""),
    "TARGET_BOTTLE_A": validate_target(text("target_bottle_a", current_target_a)),
    "TARGET_BOTTLE_B": validate_target(text("target_bottle_b", current_target_b)),
    "LEROBOT_TARGET_DIR": text("lerobot_target_dir", ""),
    "LEROBOT_DATASET_NAME": text("lerobot_dataset_name", ""),
}
for key, value in values.items():
    print(f"{key}={shlex.quote(value)}")
PY
)"; then
        write_status "warning" "${CURRENT_EPISODE}" "配置无效：目标物必须从下拉菜单中选择。"
        return 0
    fi
    eval "${exports}"
    if is_truthy "${RAW_MCAP_ONLY}"; then
        RUN_CONVERT=0
        RUN_QC=0
        RUN_LEROBOT=0
        BACKGROUND_PROCESSING=0
    fi
    [ -z "${LEROBOT_TARGET_DIR}" ] && LEROBOT_TARGET_DIR="${DATA_DIR}/lerobot"
    [ -z "${LEROBOT_DATASET_NAME}" ] && LEROBOT_DATASET_NAME="$(basename "${DATA_DIR}")_aloha_mobile"
    TARGET_BOTTLE_A_ZH="${TARGET_BOTTLE_A}"
    TARGET_BOTTLE_B_ZH="${TARGET_BOTTLE_B}"
    configure_profile
    new_check_scope="$(check_scope_fingerprint)"
    new_service_scope="$(capture_service_scope_fingerprint)"
    if [ "${old_check_scope}" != "${new_check_scope}" ]; then
        LAST_CHECK_OK=0
        LAST_CHECK_SUMMARY="采集模式已变更，请重新自检"
        LAST_CHECK_DETAILS=""
        LAST_CHECK_SCOPE=""
    fi
    refresh_stage_preset
    if [ "${CAPTURE_SERVICE_READY}" = "1" ] &&
        [ -n "${CAPTURE_LAUNCH_PID}" ] &&
        [ "${old_service_scope}" != "${new_service_scope}" ]; then
        stop_capture_service_launch
    fi
    write_status "idle" "${CURRENT_EPISODE}" "配置已应用。"
}

finish_current_stage() {
    local stage_end
    local stage_start
    if [ -z "${CURRENT_STAGE_START}" ]; then
        CURRENT_STAGE_START="$(timestamp_now)"
    fi
    stage_start="${CURRENT_STAGE_START}"
    stage_end="$(timestamp_now)"
    STAGE_START_TIMES+=("${stage_start}")
    STAGE_END_TIMES+=("${stage_end}")
    CURRENT_STAGE_START="${stage_end}"
    log "阶段 ${#STAGE_END_TIMES[@]} 已标记: ${stage_start} -> ${stage_end}"
}

save_current_episode() {
    local finalize_current_stage="${1:-1}"
    local saved_episode="${CURRENT_EPISODE}"
    if [ "${CAPTURE_RUNNING}" != "1" ]; then
        write_status "idle" "${CURRENT_EPISODE}" "当前没有正在采集的 episode。"
        return 0
    fi
    if [ "${QUALITY_REVIEW_PENDING}" = "1" ]; then
        write_status "review" "${CURRENT_EPISODE}" "episode${QUALITY_PENDING_EPISODE} 正在等待质量审核。"
        return 0
    fi
    if is_truthy "${STAGED_CAPTURE}" && [ "${finalize_current_stage}" = "1" ]; then
        finish_current_stage
    fi
    write_status "stopping" "${saved_episode}" "正在停止并保存 episode${saved_episode}..."
    log "结束并保存 MCAP: episode${saved_episode}"
    capture_service_request false true "${saved_episode}"
    CAPTURE_RUNNING=0
    if is_truthy "${STAGED_CAPTURE}"; then
        write_staged_info_files "${saved_episode}"
    else
        prepare_episode_metadata "${saved_episode}"
    fi
    LAST_SAVED_EPISODE="${saved_episode}"
    LAST_SAVED_DATA_DIR="${DATA_DIR}"
    QUALITY_REVIEW_PENDING=1
    QUALITY_PENDING_EPISODE="${saved_episode}"
    if [ -n "${COLLECTION_AUTO_GRADE}" ]; then
        log "episode${saved_episode} 已保存，正在自动记录质量等级 ${COLLECTION_AUTO_GRADE}。"
        finalize_quality_review_from_json \
            "{\"episode_id\":\"${saved_episode}\",\"action\":\"review\",\"grade\":\"${COLLECTION_AUTO_GRADE}\",\"reason_codes\":[],\"reason_note\":\"\"}"
    else
        write_status "review" "${CURRENT_EPISODE}" \
            "episode${saved_episode} 已保存，请选择质量等级或放弃。"
    fi
}

check_required_topics() {
    configure_profile
    mkdir -p "${LOG_DIR}"
    local check_log="${LOG_DIR}/topic_check_$(date +%Y%m%d_%H%M%S).log"
    local check_status
    set +e
    WITH_BASE="${WITH_BASE}" DATA_ROS_WS="${DATA_ROS_WS}" TOPIC_SAMPLE_TIMEOUT="${TOPIC_SAMPLE_TIMEOUT}" \
        bash "${TOPIC_CHECK_SCRIPT}" >"${check_log}" 2>&1
    check_status=$?
    set -e

    local missing_lines
    local camera_failure_lines
    local failure_lines
    local details
    local missing_count
    missing_lines="$(grep '^MISSING:' "${check_log}" || true)"
    camera_failure_lines="$(grep '^相机时间戳自检失败：' "${check_log}" | sort -u || true)"
    failure_lines="$(printf '%s\n%s\n' "${missing_lines}" "${camera_failure_lines}" | sed '/^[[:space:]]*$/d')"
    details="$(printf '%s\n' "${failure_lines}" | paste -sd $'\037' -)"
    missing_count="$(printf '%s\n' "${missing_lines}" | sed '/^[[:space:]]*$/d' | wc -l | tr -d ' ')"

    if [ "${check_status}" -eq 0 ]; then
        LAST_CHECK_OK=1
        LAST_CHECK_SCOPE="$(check_scope_fingerprint)"
        LAST_CHECK_AT_EPOCH="$(date +%s)"
        LAST_CHECK_SUMMARY="自检通过：所有必需项都读到数值。日志: ${check_log}"
        LAST_CHECK_DETAILS=""
        write_status "idle" "${CURRENT_EPISODE}" "${LAST_CHECK_SUMMARY}"
        return 0
    fi

    LAST_CHECK_OK=0
    LAST_CHECK_SCOPE="$(check_scope_fingerprint)"
    LAST_CHECK_AT_EPOCH="$(date +%s)"
    if [ "${missing_count}" -gt 0 ]; then
        LAST_CHECK_SUMMARY="自检失败：${missing_count} 个位置没有读到数值。日志: ${check_log}"
    elif [ -n "${camera_failure_lines}" ]; then
        LAST_CHECK_SUMMARY="自检失败：相机时间戳检查未通过。日志: ${check_log}"
    else
        LAST_CHECK_SUMMARY="自检失败：检查脚本异常退出。日志: ${check_log}"
    fi
    LAST_CHECK_DETAILS="${details}"
    log "${LAST_CHECK_SUMMARY}"
    if [ -n "${failure_lines}" ]; then
        while IFS= read -r line; do
            [ -n "${line}" ] && log "${line}"
        done <<< "${failure_lines}"
    fi
    write_status "warning" "${CURRENT_EPISODE}" "${LAST_CHECK_SUMMARY}"
    return 1
}

soft_stop_hold_robot() {
    local final_message="${1:-软急停已发送：底盘零速度，双臂和升降柱保持当前位置。}"
    configure_profile
    source_ros_env
    log "执行软急停：发布底盘零速度，保持当前双臂/升降柱位置"
    if python3 - <<'PY'
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from lifting_msg_pkg.msg import LiftMotorMsg
    from lifting_msg_pkg.srv import LiftMotorSrv
except ImportError:
    try:
        from bt_task_msgs.msg import LiftMotorMsg
        from bt_task_msgs.srv import LiftMotorSrv
    except ImportError:
        LiftMotorMsg = None
        LiftMotorSrv = None

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


class SoftStopNode(Node):
    def __init__(self):
        super().__init__("aloha_web_soft_stop")
        self.left = None
        self.right = None
        self.lift_height = None
        self.left_pub = self.create_publisher(JointState, "/joint_left_states", 10)
        self.right_pub = self.create_publisher(JointState, "/joint_right_states", 10)
        self.base_pubs = [
            self.create_publisher(Twist, "/cmd_vel", 10),
            self.create_publisher(Twist, "/cmd_vel_smoothed", 10),
        ]
        self.create_subscription(JointState, "/puppet/joint_left", self._left_cb, 10)
        self.create_subscription(JointState, "/puppet/joint_right", self._right_cb, 10)
        self.lift_client = None
        if LiftMotorMsg is not None and LiftMotorSrv is not None:
            self.lift_client = self.create_client(LiftMotorSrv, "/LiftingMotorService")
            self.create_subscription(LiftMotorMsg, "/LiftMotorStatePub", self._lift_cb, 10)

    def _left_cb(self, msg):
        if len(msg.position) >= 7:
            self.left = list(msg.position[:7])

    def _right_cb(self, msg):
        if len(msg.position) >= 7:
            self.right = list(msg.position[:7])

    def _lift_cb(self, msg):
        value = getattr(msg, "back_height", None)
        if value is None:
            value = getattr(msg, "backHeight", None)
        if value is not None:
            self.lift_height = int(value)

    def _joint_msg(self, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in position]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        return msg

    def hold_lift(self):
        if self.lift_client is None or self.lift_height is None:
            return
        if not self.lift_client.service_is_ready() and not self.lift_client.wait_for_service(timeout_sec=0.1):
            return
        req = LiftMotorSrv.Request()
        req.val = int(self.lift_height)
        req.mode = 0
        self.lift_client.call_async(req)

    def pulse(self):
        deadline = time.time() + 0.8
        while time.time() < deadline and rclpy.ok():
            if self.left is not None and self.right is not None and (self.lift_client is None or self.lift_height is not None):
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        self.hold_lift()
        period = 1.0 / 50.0
        for step in range(150):
            zero = Twist()
            for pub in self.base_pubs:
                pub.publish(zero)
            if self.left is not None:
                self.left_pub.publish(self._joint_msg(self.left))
            if self.right is not None:
                self.right_pub.publish(self._joint_msg(self.right))
            if step % 25 == 0:
                self.hold_lift()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


rclpy.init()
node = SoftStopNode()
try:
    node.pulse()
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
    then
        write_status "stopped" "${CURRENT_EPISODE}" "${final_message}"
        return 0
    fi
    write_status "warning" "${CURRENT_EPISODE}" "软急停执行失败，请立即使用实体急停。"
    return 1
}

emergency_stop() {
    configure_profile
    log "执行急停：中断回放/采集并保持机器人"
    write_status "stopping" "${CURRENT_EPISODE}" "急停中：正在中断回放/采集并保持机器人..."

    local stopped_replay=0
    local stopped_capture=0
    if [ -n "${REPLAY_PID:-}" ]; then
        stop_replay_job || true
        stopped_replay=1
    fi

    if [ "${CAPTURE_RUNNING}" = "1" ]; then
        local stopped_episode="${CURRENT_EPISODE}"
        log "急停中断采集 episode${stopped_episode}"
        if capture_service_request false true "${stopped_episode}"; then
            CAPTURE_RUNNING=0
            abandon_episode "${stopped_episode}" || true
            reset_stage_state
            stopped_capture=1
        else
            write_status "warning" "${CURRENT_EPISODE}" "急停请求已发送，但停止采集服务失败，请检查实体急停和采集服务。"
        fi
    fi

    local message="急停已执行：机器人保持命令已发送。"
    if [ "${stopped_replay}" = "1" ] && [ "${stopped_capture}" = "1" ]; then
        message="急停已执行：已中断回放和当前采集，当前 episode 已丢弃，机器人保持命令已发送。"
    elif [ "${stopped_replay}" = "1" ]; then
        message="急停已执行：已中断回放，机器人保持命令已发送。"
    elif [ "${stopped_capture}" = "1" ]; then
        message="急停已执行：已中断当前采集并丢弃当前 episode，机器人保持命令已发送。"
    fi
    soft_stop_hold_robot "${message}" || true
}

start_arm_reset() {
    configure_profile
    if [ ! -x "${PIPER_RESET_SCRIPT}" ]; then
        write_status "warning" "${CURRENT_EPISODE}" "机械臂复位脚本不可执行: ${PIPER_RESET_SCRIPT}"
        return 1
    fi
    mkdir -p "${LOG_DIR}"
    local log_file="${LOG_DIR}/piper_return_zero_$(date +%Y%m%d_%H%M%S).log"
    log "启动机械臂复位: ${log_file}"
    if command -v setsid >/dev/null 2>&1; then
        setsid "${PIPER_RESET_SCRIPT}" >"${log_file}" 2>&1 < /dev/null &
    else
        "${PIPER_RESET_SCRIPT}" >"${log_file}" 2>&1 < /dev/null &
    fi
    BACKGROUND_PIDS+=("$!")
    BACKGROUND_LOGS+=("${log_file}")
    write_status "resetting" "${CURRENT_EPISODE}" "机械臂复位已启动，日志: ${log_file}"
}

handle_web_command() {
    local command="$1"
    case "${command}" in
        config$'\t'*)
            apply_web_config_json "${command#*$'\t'}"
            ;;
        replay_visualize$'\t'*)
            start_replay_from_json "visualize" "${command#*$'\t'}"
            ;;
        replay_robot$'\t'*)
            start_replay_from_json "robot" "${command#*$'\t'}"
            ;;
        replay_stop)
            stop_replay_job
            ;;
        quality_review$'\t'*)
            finalize_quality_review_from_json "${command#*$'\t'}"
            ;;
        start)
            if [ "${CAPTURE_RUNNING}" = "1" ]; then
                write_status "recording" "${CURRENT_EPISODE}" "episode${CURRENT_EPISODE} 已在采集中。"
                return 0
            fi
            if [ "${QUALITY_REVIEW_PENDING}" = "1" ]; then
                write_status "review" "${CURRENT_EPISODE}" "请先完成 episode${QUALITY_PENDING_EPISODE} 的质量审核。"
                return 0
            fi
            prune_replay_job
            if [ -n "${REPLAY_PID:-}" ]; then
                write_status "warning" "${CURRENT_EPISODE}" "真机/数据回放正在运行，请先停止回放再开始采集。"
                return 0
            fi
            configure_profile
            refresh_stage_preset
            local episode_target="${DATA_DIR}/episode${CURRENT_EPISODE}"
            if [ -e "${episode_target}" ] || [ -L "${episode_target}" ]; then
                write_status "warning" "${CURRENT_EPISODE}" \
                    "不能开始：${episode_target} 已存在。为防止原 recorder 删除同名数据，请更换 episode 编号或先明确处理该目录。"
                return 0
            fi
            local check_scope
            local check_age
            local check_now
            local self_check_warning=""
            check_scope="$(check_scope_fingerprint)"
            if is_truthy "${REQUIRE_TOPIC_CHECK_ON_START}" || is_truthy "${WITH_BASE}"; then
                check_now="$(date +%s)"
                check_age=$((check_now - LAST_CHECK_AT_EPOCH))
                if [ "${LAST_CHECK_OK:-0}" = "1" ] &&
                    [ "${LAST_CHECK_SCOPE:-}" = "${check_scope}" ] &&
                    [ "${check_age}" -ge 0 ] &&
                    [ "${check_age}" -le "${SELF_CHECK_CACHE_SECONDS}" ]; then
                    write_status "starting" "${CURRENT_EPISODE}" "复用 ${check_age}s 前的启动自检，正在启动 episode${CURRENT_EPISODE} 采集..."
                else
                    write_status "checking" "${CURRENT_EPISODE}" "开始采集前正在等待必需 ROS topics（最多 ${TOPIC_SAMPLE_TIMEOUT}s）..."
                    if ! check_required_topics; then
                        self_check_warning="${LAST_CHECK_SUMMARY}"
                        write_status "warning" "${CURRENT_EPISODE}" \
                            "红色预警：必需 ROS topics 自检未通过；仍继续启动 episode${CURRENT_EPISODE} 采集。"
                    else
                        write_status "starting" "${CURRENT_EPISODE}" "自检通过，正在启动 episode${CURRENT_EPISODE} 采集..."
                    fi
                fi
            elif [ "${LAST_CHECK_OK:-0}" = "1" ] && [ "${LAST_CHECK_SCOPE:-}" = "${check_scope}" ]; then
                write_status "starting" "${CURRENT_EPISODE}" "自检已通过，正在启动 episode${CURRENT_EPISODE} 采集..."
            else
                self_check_warning="${LAST_CHECK_SUMMARY:-自检未通过或已失效}"
                write_status "starting" "${CURRENT_EPISODE}" "自检未通过或已失效，仍继续启动 episode${CURRENT_EPISODE} 采集..."
            fi
            start_capture_service_launch
            log "开始采集 MCAP: episode${CURRENT_EPISODE}"
            capture_service_request true false "${CURRENT_EPISODE}"
            CAPTURE_RUNNING=1
            reset_stage_state
            if is_truthy "${STAGED_CAPTURE}"; then
                CURRENT_STAGE_START="$(timestamp_now)"
                if [ -n "${self_check_warning}" ]; then
                    write_status "recording" "${CURRENT_EPISODE}" \
                        "红色预警：启动前自检未通过，但采集已继续。正在采集 episode${CURRENT_EPISODE} / 阶段 1。"
                else
                    write_status "recording" "${CURRENT_EPISODE}" "正在采集 episode${CURRENT_EPISODE} / 阶段 1。"
                fi
            else
                if [ -n "${self_check_warning}" ]; then
                    write_status "recording" "${CURRENT_EPISODE}" \
                        "红色预警：启动前自检未通过，但采集已继续。正在采集 episode${CURRENT_EPISODE}。"
                else
                    write_status "recording" "${CURRENT_EPISODE}" "正在采集 episode${CURRENT_EPISODE}。"
                fi
            fi
            ;;
        stage)
            if [ "${CAPTURE_RUNNING}" != "1" ]; then
                write_status "idle" "${CURRENT_EPISODE}" "当前没有正在采集的 episode。"
                return 0
            fi
            if ! is_truthy "${STAGED_CAPTURE}"; then
                write_status "recording" "${CURRENT_EPISODE}" "当前未启用分阶段采集。"
                return 0
            fi
            finish_current_stage
            if [ "${PRESET_SEGMENT_COUNT:-0}" -gt 0 ] &&
                is_truthy "${STAGE_AUTOSAVE_ON_PRESET}" &&
                [ "${#STAGE_END_TIMES[@]}" -ge "${PRESET_SEGMENT_COUNT}" ]; then
                save_current_episode 0
            else
                write_status "recording" "${CURRENT_EPISODE}" "已进入阶段 $((${#STAGE_END_TIMES[@]} + 1))。"
            fi
            ;;
        save|stop)
            save_current_episode 1
            ;;
        abandon|discard)
            if [ "${QUALITY_REVIEW_PENDING}" = "1" ] && [ -n "${QUALITY_PENDING_EPISODE}" ]; then
                abandon_episode "${QUALITY_PENDING_EPISODE}"
                QUALITY_REVIEW_PENDING=0
                QUALITY_PENDING_EPISODE=""
                LAST_SAVED_EPISODE=""
                LAST_SAVED_DATA_DIR=""
                reset_stage_state
                write_status "idle" "${CURRENT_EPISODE}" "episode${CURRENT_EPISODE} 已放弃，编号不增加。"
                return 0
            fi
            if [ "${CAPTURE_RUNNING}" != "1" ]; then
                write_status "idle" "${CURRENT_EPISODE}" "当前没有正在采集的 episode，无法放弃。"
                return 0
            fi
            write_status "stopping" "${CURRENT_EPISODE}" "正在停止并放弃 episode${CURRENT_EPISODE}..."
            capture_service_request false true "${CURRENT_EPISODE}" || true
            CAPTURE_RUNNING=0
            abandon_episode "${CURRENT_EPISODE}"
            reset_stage_state
            write_status "idle" "${CURRENT_EPISODE}" "episode${CURRENT_EPISODE} 已放弃，编号不增加。"
            ;;
        check)
            write_status "checking" "${CURRENT_EPISODE}" "正在检查必需 ROS topics..."
            check_required_topics || write_status "warning" "${CURRENT_EPISODE}" "Topic 检查失败，请看终端日志。"
            ;;
        estop|soft_stop)
            emergency_stop || true
            ;;
        reset_arm|return_zero)
            start_arm_reset || true
            ;;
        exit|quit)
            shutdown_web_capture 0
            ;;
        "")
            ;;
        *)
            write_status "idle" "${CURRENT_EPISODE}" "忽略未知命令: ${command}"
            ;;
    esac
}

print_web_urls() {
    local display_host="${WEB_PUBLIC_HOST:-${WEB_HOST}}"
    local local_host="${WEB_HOST}"
    local route_ip=""
    local candidate_ips=""
    if [ "${WEB_HOST}" = "0.0.0.0" ] || [ "${WEB_HOST}" = "::" ]; then
        local_host="127.0.0.1"
        if [ -z "${WEB_PUBLIC_HOST:-}" ]; then
            route_ip="$(ip route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
            display_host="${route_ip:-127.0.0.1}"
        fi
        candidate_ips="$(hostname -I 2>/dev/null | tr ' ' '\n' | sed '/^$/d' | paste -sd ' ' -)"
    fi
    echo ""
    echo "===== 网页采集控制已启动 ====="
    echo "本机访问: http://${local_host}:${WEB_PORT}/"
    echo "外部访问: http://${display_host}:${WEB_PORT}/"
    if [ -n "${candidate_ips}" ]; then
        echo "本机 IP 候选: ${candidate_ips}"
    fi
}

start_web_control_server() {
    command -v python3 >/dev/null 2>&1 || die "找不到 python3，无法启动网页"
    configure_profile
    mkdir -p "${WEB_RUNTIME_DIR}"
    rm -f "${CONTROL_FIFO}"
    mkfifo "${CONTROL_FIFO}"
    exec {CONTROL_FD}<>"${CONTROL_FIFO}"
    refresh_stage_preset
    write_status "idle" "${CURRENT_EPISODE}" "等待配置或开始采集。"

    python3 -u - "${CONTROL_FIFO}" "${STATUS_FILE}" "${WEB_HOST}" "${WEB_PORT}" <<'PY' &
import errno
import json
import os
import sys
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

fifo_path = sys.argv[1]
status_path = sys.argv[2]
host = sys.argv[3]
port = int(sys.argv[4])

COMMANDS = {
    "/start": "start",
    "/stage": "stage",
    "/stop": "save",
    "/save": "save",
    "/abandon": "abandon",
    "/discard": "abandon",
    "/check": "check",
    "/estop": "estop",
    "/soft_stop": "estop",
    "/reset_arm": "reset_arm",
    "/return_zero": "reset_arm",
    "/exit": "exit",
    "/quit": "exit",
}

def read_status():
    try:
        with open(status_path, "r", encoding="utf-8") as handle:
            return handle.read()
    except FileNotFoundError:
        return json.dumps({"state": "starting", "message": "status is not ready"}, ensure_ascii=False)

def write_command(command):
    payload = (command.replace("\n", " ") + "\n").encode("utf-8")
    fd = os.open(fifo_path, os.O_WRONLY | os.O_NONBLOCK)
    try:
        pipe_buf = os.fpathconf(fd, "PC_PIPE_BUF")
        if len(payload) > pipe_buf:
            raise OSError(errno.EMSGSIZE, f"control command exceeds PIPE_BUF ({pipe_buf} bytes)")
        written = os.write(fd, payload)
        if written != len(payload):
            raise OSError(errno.EIO, f"short FIFO write: {written}/{len(payload)} bytes")
    finally:
        os.close(fd)

class Handler(BaseHTTPRequestHandler):
    server_version = "MobileCollectWeb/1.0"

    def send_bytes(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, status, payload):
        self.send_bytes(status, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        self.handle_request()

    def do_POST(self):
        self.handle_request()

    def handle_request(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        if path == "/":
            self.send_bytes(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        if path == "/status":
            self.send_bytes(200, read_status().encode("utf-8"), "application/json; charset=utf-8")
            return
        if path == "/artifact":
            artifact_path = urllib.parse.parse_qs(parsed.query).get("path", [""])[0]
            artifact_path = os.path.abspath(os.path.expanduser(artifact_path))
            try:
                status = json.loads(read_status() or "{}")
            except json.JSONDecodeError:
                status = {}
            allowed_path = os.path.abspath(os.path.expanduser(status.get("replay_output") or ""))
            if artifact_path != allowed_path or not artifact_path.endswith(".mp4"):
                self.send_json(403, {"ok": False, "error": "artifact is not allowed"})
                return
            if not os.path.isfile(artifact_path):
                self.send_json(404, {"ok": False, "error": "file not found"})
                return
            with open(artifact_path, "rb") as handle:
                self.send_bytes(200, handle.read(), "video/mp4")
            return
        if path == "/config":
            if self.command == "GET":
                self.send_bytes(200, read_status().encode("utf-8"), "application/json; charset=utf-8")
                return
            length = int(self.headers.get("Content-Length", "0") or "0")
            body = self.rfile.read(min(length, 65536)).decode("utf-8")
            try:
                json.loads(body or "{}")
            except json.JSONDecodeError as exc:
                self.send_json(400, {"ok": False, "error": str(exc)})
                return
            try:
                write_command("config\t" + body)
            except OSError as exc:
                self.send_json(503 if exc.errno == errno.ENXIO else 500, {"ok": False, "error": str(exc)})
                return
            self.send_json(202, {"ok": True, "command": "config"})
            return
        if path in {"/replay/visualize", "/replay/robot"}:
            length = int(self.headers.get("Content-Length", "0") or "0")
            body = self.rfile.read(min(length, 65536)).decode("utf-8")
            try:
                json.loads(body or "{}")
            except json.JSONDecodeError as exc:
                self.send_json(400, {"ok": False, "error": str(exc)})
                return
            command = "replay_visualize" if path.endswith("visualize") else "replay_robot"
            try:
                write_command(command + "\t" + body)
            except OSError as exc:
                self.send_json(503 if exc.errno == errno.ENXIO else 500, {"ok": False, "error": str(exc)})
                return
            self.send_json(202, {"ok": True, "command": command})
            return
        if path == "/replay/stop":
            try:
                write_command("replay_stop")
            except OSError as exc:
                self.send_json(503 if exc.errno == errno.ENXIO else 500, {"ok": False, "error": str(exc)})
                return
            self.send_json(202, {"ok": True, "command": "replay_stop"})
            return
        if path == "/quality/review":
            length = int(self.headers.get("Content-Length", "0") or "0")
            body = self.rfile.read(min(length, 65536)).decode("utf-8")
            try:
                payload = json.loads(body or "{}")
            except json.JSONDecodeError as exc:
                self.send_json(400, {"ok": False, "error": str(exc)})
                return
            episode = str(payload.get("episode_id", payload.get("episode", ""))).strip()
            action = str(payload.get("action", "review")).strip().lower()
            grade = str(payload.get("grade", "")).strip().upper()
            reason_codes = payload.get("reason_codes", [])
            if not episode.isdigit():
                self.send_json(400, {"ok": False, "error": "episode_id must be a non-negative integer"})
                return
            if action not in {"review", "discard"}:
                self.send_json(400, {"ok": False, "error": "action must be review or discard"})
                return
            if action == "review" and grade not in {"A", "B", "F"}:
                self.send_json(400, {"ok": False, "error": "grade must be A, B, or F"})
                return
            if isinstance(reason_codes, str):
                payload["reason_codes"] = [item for item in reason_codes.replace(",", " ").split() if item]
            elif not isinstance(reason_codes, list):
                self.send_json(400, {"ok": False, "error": "reason_codes must be an array"})
                return
            try:
                write_command("quality_review\t" + json.dumps(payload, ensure_ascii=False))
            except OSError as exc:
                self.send_json(503 if exc.errno == errno.ENXIO else 500, {"ok": False, "error": str(exc)})
                return
            self.send_json(202, {"ok": True, "command": "quality_review"})
            return

        command = COMMANDS.get(path)
        if command is None:
            self.send_json(404, {"ok": False, "error": "unknown endpoint"})
            return
        try:
            write_command(command)
        except OSError as exc:
            self.send_json(503 if exc.errno == errno.ENXIO else 500, {"ok": False, "error": str(exc)})
            return
        self.send_json(202, {"ok": True, "command": command})

    def log_message(self, fmt, *args):
        sys.stderr.write("[%s] %s\n" % (time.strftime("%F %T"), fmt % args))

HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>松灵 ALOHA 数据采集</title>
  <style>
    :root {
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --page: #f6f1ff;
      --panel: rgba(255, 255, 255, 0.9);
      --panel-solid: #ffffff;
      --panel-soft: #f5fafc;
      --line: #c7d6e2;
      --line-strong: #8ea4b7;
      --ink: #17202a;
      --muted: #52606d;
      --green: #137c4b;
      --blue: #1f6feb;
      --red: #b42318;
      --amber: #b7791f;
      --slate: #334155;
      --teal: #0f766e;
      --rose: #9f1239;
      --shadow: 0 18px 44px rgba(76, 65, 118, 0.13);
      background: #f6f1ff;
      color: var(--ink);
    }
    body {
      margin: 0;
      min-height: 100vh;
      padding: 20px;
      box-sizing: border-box;
      background:
        radial-gradient(circle at 9% 14%, rgba(255, 255, 255, 0.98) 0 2px, transparent 3px),
        radial-gradient(circle at 22% 26%, rgba(255, 255, 255, 0.86) 0 1px, transparent 2px),
        radial-gradient(circle at 74% 18%, rgba(255, 255, 255, 0.92) 0 2px, transparent 3px),
        radial-gradient(circle at 88% 34%, rgba(255, 255, 255, 0.84) 0 1px, transparent 2px),
        radial-gradient(ellipse at 16% 78%, rgba(255, 255, 255, 0.76) 0 10%, transparent 11%),
        radial-gradient(ellipse at 72% 82%, rgba(255, 255, 255, 0.68) 0 12%, transparent 13%),
        linear-gradient(180deg, rgba(196, 181, 253, 0.42), rgba(224, 242, 254, 0.46) 48%, rgba(255, 247, 214, 0.58)),
        linear-gradient(92deg, transparent 0 18%, rgba(255, 255, 255, 0.74) 18% 19%, transparent 19% 58%, rgba(250, 204, 21, 0.14) 58% 60%, transparent 60%),
        linear-gradient(0deg, rgba(147, 112, 219, 0.18) 0 8%, transparent 8%),
        repeating-linear-gradient(90deg, transparent 0 82px, rgba(255, 255, 255, 0.30) 82px 84px, transparent 84px 168px),
        var(--page);
    }
    body.light-tech-shell {
      position: relative;
      overflow-x: hidden;
    }
    .operator-shell {
      width: min(980px, 100%);
      margin: 0 auto;
      display: grid;
      gap: 14px;
      position: relative;
      z-index: 1;
    }
    section {
      position: relative;
      overflow: hidden;
      background: linear-gradient(180deg, rgba(255, 255, 255, 0.92), rgba(248, 252, 253, 0.88));
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
      box-shadow: var(--shadow);
    }
    section::before {
      content: "";
      position: absolute;
      inset: 0 0 auto 0;
      height: 3px;
      background: linear-gradient(90deg, var(--green), var(--blue), var(--amber));
      opacity: 0.74;
    }
    .hero-panel {
      position: relative;
      overflow: hidden;
      border-left: 5px solid var(--green);
      background:
        radial-gradient(circle at 90% 18%, rgba(250, 204, 21, 0.20) 0 10%, transparent 11%),
        linear-gradient(120deg, rgba(255, 255, 255, 0.96), rgba(244, 239, 255, 0.92)),
        linear-gradient(90deg, rgba(196, 181, 253, 0.20), rgba(186, 230, 253, 0.18));
    }
    .storybook-sheen {
      position: absolute;
      inset: 0;
      pointer-events: none;
      background:
        radial-gradient(circle at 16% 24%, rgba(250, 204, 21, 0.28) 0 2px, transparent 3px),
        radial-gradient(circle at 34% 20%, rgba(255, 255, 255, 0.78) 0 2px, transparent 3px),
        linear-gradient(105deg, transparent 0 54%, rgba(255, 255, 255, 0.62) 54% 56%, transparent 56%);
      opacity: 0.78;
    }
    h1 {
      margin: 0;
      font-size: 24px;
      line-height: 1.25;
      letter-spacing: 0;
      color: #102033;
    }
    h2 {
      margin: 0 0 12px;
      font-size: 16px;
      letter-spacing: 0;
      color: #1f3449;
    }
    .top {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 12px;
      align-items: center;
    }
    .episode {
      font-size: 36px;
      font-weight: 750;
      line-height: 1;
      margin-top: 8px;
      color: #0f172a;
    }
    .state {
      display: inline-flex;
      align-items: center;
      min-height: 28px;
      padding: 0 10px;
      border-radius: 8px;
      background: linear-gradient(180deg, #edf4f7, #dfeaf0);
      border: 1px solid #bfd0dd;
      font-size: 14px;
      font-weight: 650;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.7);
    }
    .recording {
      background: linear-gradient(180deg, #ffe9e6, #ffd8d4);
      color: #a31912;
      border-color: #f6b0aa;
    }
    .preflight-warning {
      display: none;
      margin-top: 12px;
      padding: 10px 12px;
      border: 2px solid #dc2626;
      border-radius: 6px;
      color: #991b1b;
      background: #fee2e2;
      font-size: 14px;
      font-weight: 750;
      line-height: 1.45;
    }
    .preflight-warning.visible { display: block; }
    .selfcheck-error { color: #b42318; font-weight: 750; }
    .status-rail {
      display: grid;
      grid-template-columns: 1.2fr 1fr 0.8fr;
      gap: 7px;
      width: min(360px, 100%);
      margin-top: 14px;
    }
    .status-rail span {
      display: block;
      height: 6px;
      border-radius: 999px;
      background: #cbd5e1;
      box-shadow: 0 0 0 1px rgba(255, 255, 255, 0.62), 0 6px 12px rgba(32, 58, 84, 0.1);
    }
    .status-rail span:nth-child(1) { background: var(--green); }
    .status-rail span:nth-child(2) { background: var(--blue); }
    .status-rail span:nth-child(3) { background: var(--amber); }
    .topActions {
      display: grid;
      gap: 8px;
      justify-items: end;
      align-self: start;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px 12px;
    }
    label {
      display: grid;
      gap: 5px;
      font-size: 13px;
      color: #52606d;
      font-weight: 650;
    }
    input[type="text"], input[type="number"], select {
      width: 100%;
      min-height: 38px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 0 10px;
      box-sizing: border-box;
      font-size: 14px;
      color: #17202a;
      background: linear-gradient(180deg, #ffffff, #f9fcfd);
      box-shadow: inset 0 1px 2px rgba(32, 58, 84, 0.05);
    }
    input[type="text"]:focus, input[type="number"]:focus, select:focus {
      outline: 2px solid rgba(31, 111, 235, 0.18);
      border-color: var(--blue);
      box-shadow: 0 0 0 3px rgba(31, 111, 235, 0.08), inset 0 1px 2px rgba(32, 58, 84, 0.05);
    }
    .checks {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px 12px;
      margin-top: 12px;
    }
    .checks label {
      display: flex;
      align-items: center;
      gap: 8px;
      min-height: 32px;
      color: #17202a;
      padding: 0 8px;
      border: 1px solid rgba(148, 163, 184, 0.28);
      border-radius: 6px;
      background: rgba(255, 255, 255, 0.62);
    }
    .actions {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
    }
    button {
      min-height: 46px;
      border: 1px solid rgba(15, 23, 42, 0.12);
      border-radius: 6px;
      padding: 0 12px;
      font-size: 15px;
      font-weight: 700;
      color: #ffffff;
      background: linear-gradient(135deg, var(--slate), #475569);
      box-shadow: inset 0 -2px 0 rgba(15, 23, 42, 0.18);
      cursor: pointer;
      touch-action: manipulation;
      transition: transform 140ms ease, box-shadow 140ms ease, filter 140ms ease;
    }
    button:hover:not(:disabled) {
      transform: translateY(-1px);
      box-shadow: 0 10px 20px rgba(32, 58, 84, 0.16), inset 0 -2px 0 rgba(15, 23, 42, 0.18);
    }
    button:disabled {
      opacity: 0.45;
      cursor: not-allowed;
    }
    .start { background: linear-gradient(135deg, var(--green), #0f8a57); }
    .stage { background: linear-gradient(135deg, var(--blue), #2c7be8); }
    .save { background: linear-gradient(135deg, var(--red), #dc3f32); }
    .fail { background: linear-gradient(135deg, #92400e, #b45309); }
    .estop { background: linear-gradient(135deg, #7f1d1d, #991b1b); }
    .secondary { background: linear-gradient(135deg, #5f6b7a, #728196); }
    .exit { background: linear-gradient(135deg, #1f2933, #374151); }
    .replay { background: linear-gradient(135deg, var(--teal), #12847d); }
    .robot { background: linear-gradient(135deg, var(--rose), #be185d); }
    .quality-dialog {
      position: fixed;
      inset: 0;
      display: none;
      align-items: center;
      justify-content: center;
      padding: 18px;
      background:
        repeating-linear-gradient(0deg, rgba(255, 255, 255, 0.07) 0 1px, transparent 1px 28px),
        repeating-linear-gradient(90deg, rgba(255, 255, 255, 0.07) 0 1px, transparent 1px 28px),
        rgba(15, 23, 42, 0.52);
      z-index: 20;
    }
    .quality-dialog.open { display: flex; }
    .quality-modal {
      width: min(760px, 100%);
      position: relative;
      overflow: hidden;
      background: linear-gradient(180deg, #ffffff, #f7fbfc);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 18px;
      box-shadow: 0 28px 70px rgba(15, 23, 42, 0.32);
      display: grid;
      gap: 12px;
    }
    .quality-dialog-sheen {
      position: absolute;
      inset: 0 0 auto 0;
      height: 5px;
      background: linear-gradient(90deg, var(--green), var(--blue), var(--amber), var(--rose));
      pointer-events: none;
    }
    .quality-wish-panel {
      position: absolute;
      inset: 0;
      pointer-events: none;
      background:
        radial-gradient(circle at 12% 22%, rgba(250, 204, 21, 0.20) 0 2px, transparent 3px),
        radial-gradient(circle at 84% 18%, rgba(196, 181, 253, 0.24) 0 3px, transparent 4px),
        linear-gradient(110deg, transparent 0 62%, rgba(186, 230, 253, 0.20) 62% 66%, transparent 66%);
      opacity: 0.86;
    }
    .quality-encouragement {
      position: relative;
      min-height: 44px;
      display: flex;
      align-items: center;
      padding: 0 14px;
      border: 1px solid #bbf7d0;
      border-left: 5px solid var(--green);
      border-radius: 8px;
      background: linear-gradient(90deg, #effdf5, #f7fbff);
      color: #14532d;
      font-size: 18px;
      font-weight: 850;
      line-height: 1.35;
      box-shadow: 0 10px 24px rgba(19, 124, 75, 0.1);
    }
    .quality-modal h2 { margin: 0; font-size: 22px; }
    .quality-grade-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
    }
    .quality-grade {
      min-height: 94px;
      font-size: 38px;
      line-height: 1;
    }
    .quality-grade span {
      display: block;
      margin-top: 8px;
      font-size: 16px;
      font-weight: 800;
    }
    .quality-a { background: linear-gradient(135deg, #137c4b, #0f8a57); }
    .quality-b { background: linear-gradient(135deg, #b7791f, #d69e2e); }
    .quality-f { background: linear-gradient(135deg, #b42318, #dc3f32); }
    .quality-discard { background: linear-gradient(135deg, #4b5563, #64748b); }
    .quality-reason-panel {
      display: none;
      gap: 10px;
      padding-top: 8px;
      border-top: 1px solid #d8dee8;
    }
    .quality-reason-panel.open { display: grid; }
    .reason-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .reason-chip {
      min-height: 56px;
      background: linear-gradient(180deg, #f8fafc, #e9eef4);
      color: #17202a;
      border: 2px solid #cbd5e1;
      font-size: 18px;
    }
    .reason-chip.selected {
      background: linear-gradient(135deg, #14532d, #0f766e);
      border-color: #14532d;
      color: #ffffff;
    }
    .quality-summary {
      min-height: 34px;
      font-size: 20px;
      font-weight: 800;
      color: #17202a;
    }
    body.robot-replay-view .collection-config,
    body.robot-replay-view .collection-actions,
    body.robot-replay-view .replay-visual-only,
    body.robot-replay-view #replayArtifact {
      display: none;
    }
    body.robot-replay-view main {
      width: min(820px, 100%);
    }
    body.robot-replay-view h1::after {
      content: " / 真机回放";
      color: #9f1239;
    }
    .topStop {
      align-items: center;
      min-width: 168px;
      justify-content: center;
      display: none;
    }
    .topFail {
      align-items: center;
      min-width: 168px;
      justify-content: center;
      display: none;
      box-shadow: 0 0 0 3px rgba(146, 64, 14, 0.18);
    }
    .meta {
      display: grid;
      gap: 6px;
      font-size: 13px;
      color: #52606d;
      overflow-wrap: anywhere;
      background: linear-gradient(180deg, rgba(255, 255, 255, 0.82), rgba(245, 250, 252, 0.82));
    }
    .meta a {
      color: #1f6feb;
      font-weight: 650;
    }
    .stageText {
      margin-top: 10px;
      font-size: 15px;
      line-height: 1.45;
      color: #243b53;
    }
    @media (max-width: 760px) {
      body { padding: 12px; }
      .top, .grid, .checks, .actions, .quality-grade-grid, .reason-grid {
        grid-template-columns: 1fr;
      }
      .topActions { justify-items: stretch; }
      .episode { font-size: 30px; }
      button { min-height: 52px; }
    }
  </style>
</head>
<body class="light-tech-shell fairytale-canvas castle-silhouette starlight-field cloudscape">
  <main class="operator-shell">
    <section class="top hero-panel">
      <div class="storybook-sheen" aria-hidden="true"></div>
      <div>
        <h1>松灵 ALOHA 数据采集</h1>
        <div class="episode" id="episode">episode-</div>
        <div class="stageText" id="message">连接中...</div>
        <div class="stageText" id="stageText"></div>
        <div class="preflight-warning" id="preflightWarning" role="alert"></div>
        <div class="status-rail signal-strip" aria-hidden="true"><span></span><span></span><span></span></div>
      </div>
      <div class="topActions">
        <span class="state" id="state">starting</span>
        <button class="save topStop" id="stopTop" onclick="sendCommand('/save')">结束采集并保存</button>
      </div>
    </section>

    <section class="collection-config">
      <h2>配置</h2>
      <div class="grid">
        <label>数据目录
          <input id="data_dir" type="text">
        </label>
        <label>起始/当前 episode
          <input id="episode_index" type="number" min="0">
        </label>
        <label>阶段 preset
          <input id="preset_info_json" type="text">
        </label>
        <label>LeRobot 数据集名
          <input id="lerobot_dataset_name" type="text">
        </label>
        <label>左手目标物品
          <select id="target_bottle_a">
            <option value="">空</option>
          </select>
        </label>
        <label>右手目标物品
          <select id="target_bottle_b">
            <option value="">空</option>
          </select>
        </label>
      </div>
      <div class="checks">
        <label><input id="with_base" type="checkbox"> 带底盘</label>
        <label><input id="staged_capture" type="checkbox"> 分阶段</label>
        <label><input id="auto_save_on_preset" type="checkbox"> 阶段完成自动保存</label>
        <label id="rawOnlyNotice"> 采集阶段仅保存原格式 MCAP</label>
      </div>
    </section>

    <section class="actions collection-actions">
      <button class="start" id="start" onclick="startCapture()">开始采集</button>
      <button class="stage" id="stage" onclick="sendCommand('/stage')">阶段切换</button>
      <button class="save" id="save" onclick="sendCommand('/save')">结束采集并保存</button>
      <button class="estop" id="estop" onclick="sendCommand('/estop')">急停中断</button>
      <button class="secondary" id="check" onclick="checkTopics()">启动自检</button>
      <button class="secondary" id="resetArm" onclick="sendCommand('/reset_arm')">机械臂复位</button>
      <button class="exit" id="exit" onclick="sendCommand('/exit')">退出</button>
    </section>

    <section class="replay-section">
      <h2>回放</h2>
      <div class="grid">
        <label>格式
          <select id="replay_format">
            <option value="mcap">原始 MCAP</option>
            <option value="hdf5">HDF5</option>
            <option value="lerobot">LeRobot</option>
          </select>
        </label>
        <label>模式
          <select id="replay_mode">
            <option value="mobile">底盘 + 升降柱 + 双臂</option>
            <option value="arms">仅双臂</option>
          </select>
        </label>
        <label>数据路径
          <input id="replay_path" type="text">
        </label>
        <label>回放 episode（原始编号）
          <input id="replay_episode" type="number" min="0" value="0">
        </label>
        <label>真机倍速
          <input id="replay_rate" type="number" min="0.1" step="0.1" value="0.3">
        </label>
        <label class="replay-visual-only">可视化 FPS
          <input id="replay_fps" type="number" min="1" step="1" value="30">
        </label>
        <label>轨迹源
          <select id="replay_source">
            <option value="master">HDF5 master / LeRobot action</option>
            <option value="puppet">HDF5 puppet</option>
            <option value="state">LeRobot state</option>
          </select>
        </label>
        <label>底盘源
          <select id="replay_base_source">
            <option value="auto">auto</option>
            <option value="odom">odom</option>
            <option value="raw">raw action</option>
            <option value="hdf5">hdf5 action</option>
          </select>
        </label>
        <label>最大线速度
          <input id="replay_max_linear" type="number" min="0" step="0.05" value="0.3">
        </label>
        <label>最大角速度
          <input id="replay_max_angular" type="number" min="0" step="0.05" value="0.8">
        </label>
        <label class="replay-visual-only">可视化最大帧数
          <input id="replay_max_frames" type="number" min="0" step="1" value="0">
        </label>
      </div>
      <div class="actions" style="margin-top: 12px;">
        <button class="replay replay-visual-only" id="replayVisualize" onclick="startReplay('visualize')">可视化回放</button>
        <button class="robot" id="replayRobot" onclick="startReplay('robot')">真机回放</button>
        <button class="secondary" id="replayStop" onclick="stopReplay()">停止回放</button>
      </div>
    </section>

    <section class="meta">
      <div id="updated">更新时间: -</div>
      <div id="selfcheck">自检: -</div>
      <div id="profile">模式: -</div>
      <div id="paths">路径: -</div>
      <div id="processing">后台: -</div>
      <div id="replayStatus">回放: -</div>
      <div id="replayArtifact">可视化: -</div>
    </section>
  </main>
  <div class="quality-dialog" id="qualityDialog" role="dialog" aria-modal="true">
    <div class="quality-modal">
      <div class="quality-dialog-sheen" aria-hidden="true"></div>
      <div class="quality-wish-panel" aria-hidden="true"></div>
      <div class="quality-encouragement" id="qualityEncouragement">这条数据收得很稳，继续保持。</div>
      <h2 id="qualityDialogTitle">质量确认</h2>
      <div class="quality-grade-grid">
        <button class="quality-grade quality-a" type="button" onclick="submitQualityReview('A')">A<span>合格</span></button>
        <button class="quality-grade quality-b" type="button" onclick="selectQualityGrade('B')">B<span>小问题</span></button>
        <button class="quality-grade quality-f" type="button" onclick="selectQualityGrade('F')">F<span>失败</span></button>
        <button class="quality-grade quality-discard" type="button" onclick="submitQualityReview('discard')">放弃<span>重采本编号</span></button>
      </div>
      <div class="quality-reason-panel" id="qualityReasonPanel">
        <div class="quality-summary" id="qualityReasonTitle">选择原因（可不选）</div>
        <div class="reason-grid" id="qualityReasons"></div>
        <div class="quality-summary" id="qualityReasonSummary">未选择原因</div>
        <label>备注
          <input id="qualityNote" type="text" placeholder="可选">
        </label>
        <div class="actions">
          <button class="save" id="qualitySubmit" type="button" onclick="submitSelectedQualityReview()">保存等级</button>
        </div>
      </div>
    </div>
  </div>
  <script>
    const pageParams = new URLSearchParams(window.location.search);
    if (pageParams.get('view') === 'robot_replay') {
      document.body.classList.add('robot-replay-view');
    }
    const ids = [
      'episode', 'message', 'stageText', 'preflightWarning', 'state', 'updated', 'selfcheck', 'profile', 'paths', 'processing',
      'replayStatus', 'replayArtifact',
      'data_dir', 'episode_index', 'preset_info_json', 'target_bottle_a', 'target_bottle_b',
      'lerobot_dataset_name', 'with_base', 'staged_capture', 'auto_save_on_preset',
      'replay_format', 'replay_mode', 'replay_path', 'replay_episode', 'replay_rate', 'replay_fps',
      'replay_source', 'replay_base_source', 'replay_max_linear', 'replay_max_angular', 'replay_max_frames',
      'start', 'stage', 'save', 'stopTop', 'check', 'estop', 'resetArm',
      'qualityDialog', 'qualityDialogTitle', 'qualityReasonPanel', 'qualityReasons', 'qualityReasonTitle',
      'qualityReasonSummary', 'qualityNote', 'qualitySubmit', 'qualityEncouragement',
      'replayVisualize', 'replayRobot', 'replayStop'
    ];
    const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));
    let loadedOnce = false;
    let lastDefaultReplayPath = '';
    let replayPathDirty = false;
    let lastAutoReplayEpisode = '';
    let lastStatus = null;
    let qualityDialogEpisode = '';
    let promptedQualityEpisode = '';
    let selectedQualityGrade = '';
    let lastEncouragementMessage = '';
    const ENCOURAGEMENT_MESSAGES = [
      '这条数据收得很稳，继续保持。',
      '动作链路完整，下一条可以更丝滑。',
      '采集节奏在线，质量正在累积。',
      '这一条很有训练价值。',
      '现场状态不错，继续稳住。',
      '数据资产又多了一条可靠样本。',
      '这一轮配合很顺，保持这个手感。',
      '保存完成，离更强的模型又近一步。',
      '这一条记录清晰，值得留下。',
      '采集闭环完成，下一条继续推进。',
      '动作和时序都在轨道上。',
      '这条样本很扎实，继续扩大优势。',
      '节奏很好，数据质量会说话。',
      '操作稳定，训练集正在变厚。',
      '这条轨迹干净利落，模型会喜欢这种稳定。',
      '下一条继续保持节奏，数据会越来越有分量。',
      '一次稳定采集，就是一次训练集升级。',
      '动作边界清楚，后处理会少操很多心。',
      '这条记录很完整，可以安心进入质检。',
      '采集手感在线，今天的数据很有盼头。',
      '状态机配合顺畅，现场节奏很稳。',
      '关键帧很清楚，训练信号更扎实。',
      '这条样本像是认真对齐过的答案。',
      '保存成功，训练集又多了一块可靠积木。',
      '动作没有拖泥带水，下一条继续稳。',
      '现场状态稳定，数据资产正在增长。',
      '这一条的节奏不错，值得给自己点个头。',
      '轨迹清晰，质量确认会更轻松。',
      '数据闭环完成，继续把样本墙垒高。',
      '采得越稳，模型越少猜。',
      '这一条很有参考价值，继续扩充覆盖面。',
      '操作链路顺滑，下一条可以继续同款节奏。',
      '记录完整，训练时会少一点迷茫。',
      '这条样本可靠，离更好的策略又近一步。',
      '现场配合到位，数据质量正在发光。',
      '动作节拍很准，采集节奏保持住。',
      '每一条稳样本，都是模型的底气。',
      '这轮表现扎实，下一条继续拉满专注。',
      '时序清楚，质量标签也更有底气。',
      '采集完成，今天的训练素材又厚了一点。',
      '这条数据有章法，后面会感谢现在的认真。',
      '动作路径清爽，质检压力少一截。',
      '这条收得漂亮，下一条继续稳中求准。',
      '样本质量在线，继续让数据说话。',
      '录制闭环顺利，现场节奏值得复用。',
      '这一条很稳，模型会记住这份认真。',
      '数据质量上来了，训练效果才有底气。',
      '动作目标明确，样本价值更清楚。',
      '保存到位，下一条继续把误差压低。',
      '采集手感不错，保持这个控制节奏。',
      '这条轨迹很顺，像给模型递了一份清晰教材。',
      '现场判断准确，样本标签也更可靠。',
      '完成一条稳样本，训练集就更像样一点。',
      '节奏、路径、状态都在线，继续推进。',
      '这条记录很规整，后续分析会省时间。',
      '采集质量稳定，下一条继续少一点抖动。',
      '保存完成，数据管线又收到一条好材料。',
      '操作稳定就是生产力，继续保持。',
      '这条样本够清楚，值得进入下一轮检查。',
      '机器人动作配合顺，数据也跟着顺。',
      '这条很稳，下一条继续把细节打磨好。',
      '采集进度向前一格，模型能力也向前一格。',
      '动作完成度不错，质量池又添一员。',
      '这一条的节奏值得复刻。',
      '保持这样的采集密度，数据集会很有说服力。',
    ];
    const QUALITY_REASONS = {
      B: [
        {code: 'minor_collision', label: '轻微碰撞'},
        {code: 'unsmooth_motion', label: '轨迹不平滑'},
        {code: 'retry_success', label: '重试后成功'},
        {code: 'minor_visual_issue', label: '轻微视觉异常'},
        {code: 'others', label: '其他'},
      ],
      F: [
        {code: 'grasp_failure', label: '抓取失败'},
        {code: 'object_dropped', label: '物体掉落'},
        {code: 'wrong_placement', label: '放置错误'},
        {code: 'wrong_target', label: '目标错误'},
        {code: 'object_knocked_over', label: '物体碰倒'},
        {code: 'task_abandoned', label: '任务中止'},
        {code: 'others', label: '其他'},
      ],
    };

    function collectConfig() {
      return {
        data_dir: el.data_dir.value.trim(),
        episode_index: String(el.episode_index.value || '0'),
        preset_info_json: el.preset_info_json.value.trim(),
        target_bottle_a: el.target_bottle_a.value.trim(),
        target_bottle_b: el.target_bottle_b.value.trim(),
        lerobot_dataset_name: el.lerobot_dataset_name.value.trim(),
        with_base: el.with_base.checked,
        staged_capture: el.staged_capture.checked,
        auto_save_on_preset: el.auto_save_on_preset.checked,
      };
    }

    function fillTargetSelect(select, options, currentValue) {
      select.replaceChildren();
      const emptyOption = document.createElement('option');
      emptyOption.value = '';
      emptyOption.textContent = '空';
      select.appendChild(emptyOption);
      for (const target of options || []) {
        const option = document.createElement('option');
        option.value = String(target);
        option.textContent = String(target);
        select.appendChild(option);
      }
      select.value = currentValue || '';
    }

    async function waitForTargetSelection(leftTarget, rightTarget, timeoutMs = 5000) {
      const deadline = Date.now() + timeoutMs;
      while (Date.now() < deadline) {
        const res = await fetch('/status', {cache: 'no-store'});
        if (!res.ok) throw new Error(await res.text());
        const data = await res.json();
        if ((data.target_bottle_a || '') === leftTarget && (data.target_bottle_b || '') === rightTarget) {
          return data;
        }
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      throw new Error('等待目标物配置生效超时');
    }

    async function persistTargetSelection() {
      const leftTarget = el.target_bottle_a.value;
      const rightTarget = el.target_bottle_b.value;
      try {
        await applyConfig();
        await waitForTargetSelection(leftTarget, rightTarget);
        el.message.textContent = '左右手目标物配置已应用。';
        setTimeout(refresh, 100);
      } catch (err) {
        el.message.textContent = '目标物配置失败: ' + err;
      }
    }

    function defaultReplayPath(data) {
      if (el.replay_format.value === 'mcap') {
        if (data?.last_saved_episode && data?.last_saved_data_dir) return data.last_saved_data_dir;
        return data?.data_dir || el.data_dir.value || '/home/agilex/data/market_2';
      }
      if (el.replay_format.value === 'lerobot') {
        return data?.replay_lerobot_root || derivedReplayRoot(data);
      }
      return data?.replay_hdf5_root || derivedReplayRoot(data);
    }

    function derivedReplayRoot(data) {
      const raw = String(data?.data_dir || el.data_dir.value || '').replace(/\/+$/, '');
      if (!raw) return '';
      if (raw.endsWith('/three_camera_global')) return raw;
      const separator = raw.lastIndexOf('/');
      return (separator > 0 ? raw.slice(0, separator) : raw) + '/three_camera_global';
    }

    function setReplayPathToDefault(data) {
      lastDefaultReplayPath = defaultReplayPath(data);
      el.replay_path.value = lastDefaultReplayPath;
      replayPathDirty = false;
    }

    function refreshReplayPathDefault(data) {
      const nextDefault = defaultReplayPath(data);
      if (!replayPathDirty || !el.replay_path.value.trim() || el.replay_path.value.trim() === lastDefaultReplayPath) {
        el.replay_path.value = nextDefault;
        replayPathDirty = false;
      }
      lastDefaultReplayPath = nextDefault;
    }

    function collectReplayConfig() {
      return {
        format: el.replay_format.value,
        mode: el.replay_mode.value,
        path: el.replay_path.value.trim(),
        episode: String(el.replay_episode.value || '0'),
        rate: String(el.replay_rate.value || '0.3'),
        fps: String(el.replay_fps.value || '30'),
        source: el.replay_source.value,
        base_source: el.replay_base_source.value,
        max_linear: String(el.replay_max_linear.value || '0.3'),
        max_angular: String(el.replay_max_angular.value || '0.8'),
        max_frames: String(el.replay_max_frames.value || '0'),
      };
    }

    async function startReplay(kind) {
      try {
        if (!el.replay_path.value.trim()) {
          el.replay_path.value = defaultReplayPath();
        }
        if (kind === 'robot') {
          const ok = window.confirm('确认开始真机回放？机器人会按数据运动。');
          if (!ok) return;
        }
        const res = await fetch('/replay/' + kind, {
          method: 'POST',
          cache: 'no-store',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(collectReplayConfig()),
        });
        if (!res.ok) throw new Error(await res.text());
        setTimeout(refresh, 300);
      } catch (err) {
        el.message.textContent = '回放启动失败: ' + err;
      }
    }

    async function stopReplay() {
      await sendCommand('/replay/stop');
    }

    async function applyConfig() {
      const res = await fetch('/config', {
        method: 'POST',
        cache: 'no-store',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(collectConfig()),
      });
      if (!res.ok) throw new Error(await res.text());
    }

    async function startCapture() {
      try {
        await applyConfig();
        await sendCommand('/start');
      } catch (err) {
        el.message.textContent = '开始失败: ' + err;
      }
    }

    async function checkTopics() {
      try {
        await applyConfig();
        await sendCommand('/check');
      } catch (err) {
        el.message.textContent = '检查失败: ' + err;
      }
    }

    async function sendCommand(endpoint) {
      try {
        const res = await fetch(endpoint, { method: 'POST', cache: 'no-store' });
        if (!res.ok) throw new Error(await res.text());
        setTimeout(refresh, 200);
        return true;
      } catch (err) {
        el.message.textContent = '命令失败: ' + err;
        return false;
      }
    }

    function nextEncouragementMessage() {
      if (ENCOURAGEMENT_MESSAGES.length <= 1) {
        lastEncouragementMessage = ENCOURAGEMENT_MESSAGES[0] || '';
        return lastEncouragementMessage;
      }
      const candidates = ENCOURAGEMENT_MESSAGES.filter(item => item !== lastEncouragementMessage);
      const pool = candidates.length ? candidates : ENCOURAGEMENT_MESSAGES;
      const next = pool[Math.floor(Math.random() * pool.length)];
      lastEncouragementMessage = next;
      return next;
    }

    function openQualityDialog(episode) {
      qualityDialogEpisode = String(episode);
      promptedQualityEpisode = qualityDialogEpisode;
      selectedQualityGrade = '';
      el.qualityEncouragement.textContent = nextEncouragementMessage();
      el.qualityDialogTitle.textContent = 'episode' + qualityDialogEpisode + ' 质量确认';
      el.qualityNote.value = '';
      el.qualityReasonPanel.classList.remove('open');
      el.qualityReasons.innerHTML = '';
      el.qualityReasonSummary.textContent = '未选择原因';
      el.qualityDialog.classList.add('open');
    }

    function selectQualityGrade(grade) {
      selectedQualityGrade = grade;
      const reasons = QUALITY_REASONS[grade] || [];
      el.qualityReasonTitle.textContent = (grade === 'B' ? 'B 等级原因（可多选，也可不选）' : 'F 等级原因（可多选，也可不选）');
      el.qualityReasons.innerHTML = reasons.map(item => (
        '<button class="reason-chip" type="button" data-code="' + item.code + '">' + escapeHtml(item.label) + '</button>'
      )).join('');
      for (const button of Array.from(el.qualityReasons.querySelectorAll('.reason-chip'))) {
        button.addEventListener('click', () => {
          button.classList.toggle('selected');
          updateQualityReasonSummary();
        });
      }
      updateQualityReasonSummary();
      el.qualityReasonPanel.classList.add('open');
    }

    function selectedReasonCodes() {
      return Array.from(el.qualityReasons.querySelectorAll('.reason-chip.selected'))
        .map(button => button.dataset.code)
        .filter(Boolean);
    }

    function updateQualityReasonSummary() {
      const selected = Array.from(el.qualityReasons.querySelectorAll('.reason-chip.selected'))
        .map(button => button.textContent.trim())
        .filter(Boolean);
      el.qualityReasonSummary.textContent = selected.length ? ('已选择：' + selected.join('，')) : '未选择原因';
    }

    function submitSelectedQualityReview() {
      submitQualityReview(selectedQualityGrade);
    }

    async function submitQualityReview(gradeOrAction) {
      try {
        if (!qualityDialogEpisode) return;
        const action = gradeOrAction === 'discard' ? 'discard' : 'review';
        const grade = action === 'discard' ? '' : String(gradeOrAction || selectedQualityGrade || '').toUpperCase();
        if (action === 'review' && !['A', 'B', 'F'].includes(grade)) return;
        el.qualitySubmit.disabled = true;
        const body = {
          episode_id: qualityDialogEpisode,
          action,
          grade,
          reason_codes: ['B', 'F'].includes(grade) ? selectedReasonCodes() : [],
          reason_note: ['B', 'F'].includes(grade) ? el.qualityNote.value.trim() : '',
        };
        const res = await fetch('/quality/review', {
          method: 'POST',
          cache: 'no-store',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(body),
        });
        if (!res.ok) throw new Error(await res.text());
        el.qualityDialog.classList.remove('open');
        qualityDialogEpisode = '';
        selectedQualityGrade = '';
        setTimeout(refresh, 300);
      } catch (err) {
        el.message.textContent = '质量审核保存失败: ' + err;
      } finally {
        el.qualitySubmit.disabled = false;
      }
    }

    function escapeHtml(value) {
      return String(value || '').replace(/[&<>"']/g, ch => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
      })[ch]);
    }

    function fillConfig(data) {
      if (loadedOnce) return;
      el.data_dir.value = data.data_dir || '';
      el.episode_index.value = data.episode ?? 0;
      el.preset_info_json.value = data.preset_info_json || '';
      fillTargetSelect(el.target_bottle_a, data.target_options, data.target_bottle_a);
      fillTargetSelect(el.target_bottle_b, data.target_options, data.target_bottle_b);
      el.lerobot_dataset_name.value = data.lerobot_dataset_name || '';
      el.with_base.checked = !!data.with_base;
      el.staged_capture.checked = !!data.staged_capture;
      el.auto_save_on_preset.checked = !!data.auto_save_on_preset;
      setReplayPathToDefault(data);
      const savedEpisode = String(data.last_saved_episode || '');
      el.replay_episode.value = savedEpisode || 0;
      lastAutoReplayEpisode = savedEpisode;
      loadedOnce = true;
    }

    async function refresh() {
      try {
        const res = await fetch('/status', { cache: 'no-store' });
        const data = await res.json();
        lastStatus = data;
        fillConfig(data);
        const running = !!data.capture_running;
        const pendingReview = !!data.quality_review_pending;
        const replayRunning = !!data.replay_running;
        if (!running && document.activeElement !== el.episode_index) {
          el.episode_index.value = data.episode ?? 0;
        }
        const savedEpisode = String(data.last_saved_episode || '');
        if (!savedEpisode) lastAutoReplayEpisode = '';
        if (el.replay_format.value === 'mcap' && savedEpisode && savedEpisode !== lastAutoReplayEpisode
            && document.activeElement !== el.replay_episode) {
          el.replay_episode.value = savedEpisode;
          lastAutoReplayEpisode = savedEpisode;
        }
        refreshReplayPathDefault(data);
        el.episode.textContent = 'episode' + (data.episode ?? '-');
        el.message.textContent = data.message || '';
        el.state.textContent = data.state || '-';
        el.state.classList.toggle('recording', running);
        if (running && data.staged_capture) {
          el.stageText.textContent = '阶段 ' + data.current_stage + ': ' + (data.current_stage_text || '');
        } else {
          el.stageText.textContent = data.preset_warning || '';
        }
        el.updated.textContent = '更新时间: ' + (data.updated_at || '-');
        const details = (data.last_check_details || []).map(line => String(line)).join(' | ');
        const checkSummary = String(data.last_check_summary || '尚未自检');
        const selfCheckFailed = !data.last_check_ok && checkSummary.startsWith('自检失败');
        el.selfcheck.textContent = '自检: ' + (data.last_check_ok ? '通过' : checkSummary) + (details ? ' | ' + details : '');
        el.selfcheck.classList.toggle('selfcheck-error', selfCheckFailed);
        el.preflightWarning.classList.toggle('visible', selfCheckFailed);
        el.preflightWarning.textContent = selfCheckFailed
          ? '⚠ 启动前自检未通过，但不会阻止采集。' + checkSummary + (details ? ' | ' + details : '')
          : '';
        el.profile.textContent = '模式: ' + (data.profile || '-') + ' | 阶段: ' + (data.completed_stages || 0) + '/' + (data.preset_stage_count || 0);
        el.paths.textContent = '原始 MCAP: ' + (data.data_dir || '-') + (data.raw_mcap_only ? ' | 采集阶段不生成 HDF5/QC' : '');
        el.processing.textContent = '日志: ' + (data.log_dir || '-')
          + ' | 场景清单: ' + (data.scene_inventory_json || '-');
        el.replayStatus.textContent = '回放: ' + (data.replay_status || 'idle')
          + (data.replay_pid ? ' | pid=' + data.replay_pid : '')
          + (data.replay_log ? ' | 日志: ' + data.replay_log : '')
          + (data.replay_exit_code ? ' | exit=' + data.replay_exit_code : '');
        if (data.replay_output) {
          const href = '/artifact?path=' + encodeURIComponent(data.replay_output);
          el.replayArtifact.innerHTML = '可视化: <a target="_blank" href="' + href + '">' + escapeHtml(data.replay_output) + '</a>';
        } else {
          el.replayArtifact.textContent = '可视化: -';
        }

        el.start.disabled = running || pendingReview || replayRunning;
        el.stage.disabled = !running || !data.staged_capture;
        el.save.disabled = !running;
        el.stopTop.disabled = !running;
        el.stopTop.style.display = running ? 'inline-flex' : 'none';
        el.estop.disabled = false;
        el.resetArm.disabled = false;
        el.replayVisualize.disabled = replayRunning || el.replay_format.value === 'mcap';
        el.replayRobot.disabled = replayRunning || running;
        el.replayStop.disabled = !replayRunning;
        for (const id of ['data_dir', 'episode_index', 'preset_info_json', 'target_bottle_a', 'target_bottle_b',
                          'lerobot_dataset_name', 'with_base', 'staged_capture', 'auto_save_on_preset']) {
          el[id].disabled = running || pendingReview;
        }
        const awaitingEpisode = String(data.quality_awaiting_episode || '');
        if (awaitingEpisode && awaitingEpisode !== promptedQualityEpisode) {
          openQualityDialog(awaitingEpisode);
        }
        if (!awaitingEpisode) {
          promptedQualityEpisode = '';
          if (qualityDialogEpisode) {
            el.qualityDialog.classList.remove('open');
            qualityDialogEpisode = '';
          }
        }
      } catch (err) {
        el.message.textContent = '状态刷新失败: ' + err;
      }
    }

    el.replay_format.addEventListener('change', () => {
      setReplayPathToDefault(lastStatus);
      if (el.replay_format.value === 'mcap' && lastStatus?.last_saved_episode) {
        el.replay_episode.value = String(lastStatus.last_saved_episode);
        lastAutoReplayEpisode = String(lastStatus.last_saved_episode);
      }
      if (el.replay_format.value === 'lerobot' && el.replay_source.value === 'master') {
        el.replay_source.value = 'master';
      }
    });
    el.replay_path.addEventListener('input', () => {
      replayPathDirty = el.replay_path.value.trim() !== lastDefaultReplayPath;
    });
    el.target_bottle_a.addEventListener('change', persistTargetSelection);
    el.target_bottle_b.addEventListener('change', persistTargetSelection);

    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""

class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True

server = ReusableThreadingHTTPServer((host, port), Handler)
print(f"web control listening on http://{host}:{port}/", flush=True)
server.serve_forever()
PY
    WEB_SERVER_PID="$!"
    sleep 0.5
    if ! kill -0 "${WEB_SERVER_PID}" >/dev/null 2>&1; then
        wait "${WEB_SERVER_PID}" 2>/dev/null || true
        die "网页启动失败，请检查 WEB_PORT=${WEB_PORT} 是否被占用"
    fi
    print_web_urls
}

stop_web_control_server() {
    if [ -n "${WEB_SERVER_PID}" ] && kill -0 "${WEB_SERVER_PID}" >/dev/null 2>&1; then
        kill "${WEB_SERVER_PID}" >/dev/null 2>&1 || true
        wait "${WEB_SERVER_PID}" 2>/dev/null || true
    fi
    if [ -n "${CONTROL_FD:-}" ]; then
        exec {CONTROL_FD}>&- || true
        CONTROL_FD=""
    fi
    rm -f "${CONTROL_FIFO}"
}

run_startup_preflight_check() {
    local startup_check_ok=0
    write_status "checking" "${CURRENT_EPISODE}" "脚本启动自检中..."
    if check_required_topics; then
        startup_check_ok=1
        write_status "starting" "${CURRENT_EPISODE}" "启动自检通过，正在预热原 MCAP 采集服务..."
    else
        write_status "warning" "${CURRENT_EPISODE}" \
            "红色预警：启动自检失败；仍会预热采集服务，并允许继续采集。详情见自检日志。"
    fi
    if start_capture_service_launch; then
        if [ "${startup_check_ok}" = "1" ]; then
            write_status "idle" "${CURRENT_EPISODE}" "启动自检通过，MCAP 采集服务已就绪，等待开始 episode${CURRENT_EPISODE}。"
        else
            write_status "warning" "${CURRENT_EPISODE}" \
                "红色预警：启动自检未通过，但 MCAP 采集服务已就绪；仍可开始 episode${CURRENT_EPISODE}。"
        fi
    else
        write_status "warning" "${CURRENT_EPISODE}" "MCAP 采集服务预热失败；启动时会再次尝试，请查看采集日志。"
    fi
    return 0
}

cleanup_runtime() {
    set +e
    if [ -n "${REPLAY_PID:-}" ]; then
        stop_replay_job || true
    fi
    stop_web_control_server
    if [ "${CAPTURE_RUNNING}" = "1" ]; then
        capture_service_request false true "${CURRENT_EPISODE}" || true
        CAPTURE_RUNNING=0
    fi
    stop_capture_service_launch
}

shutdown_web_capture() {
    local code="${1:-0}"
    set +e
    if [ "${CAPTURE_RUNNING}" = "1" ]; then
        write_status "stopping" "${CURRENT_EPISODE}" "退出前停止并保存 episode${CURRENT_EPISODE}。"
        save_current_episode 1 || true
    fi
    write_status "exited" "${CURRENT_EPISODE}" "网页采集控制已退出。"
    stop_web_control_server
    stop_capture_service_launch
    exit "${code}"
}

handle_signal() {
    echo ""
    log "收到退出信号，准备退出。"
    shutdown_web_capture 130
}

run_web_loop() {
    local command
    configure_profile
    refresh_stage_preset
    mkdir -p "${WEB_RUNTIME_DIR}" "${LOG_DIR}"
    start_web_control_server
    trap cleanup_runtime EXIT
    trap handle_signal INT TERM
    run_startup_preflight_check

    while true; do
        prune_background_jobs
        prune_replay_job
        if ! kill -0 "${WEB_SERVER_PID}" >/dev/null 2>&1; then
            die "网页服务已退出"
        fi
        if IFS= read -r -t 1 -u "${CONTROL_FD}" command; then
            handle_web_command "${command}"
        fi
    done
}

configure_profile

if is_truthy "${PROCESS_ONLY:-0}"; then
    process_episode "${EPISODE_INDEX}"
    exit 0
fi

log "网页采集配置"
echo "  DATA_ROS_WS=${DATA_ROS_WS}"
echo "  DATA_DIR=${DATA_DIR}"
echo "  WITH_BASE=${WITH_BASE}, STAGED_CAPTURE=${STAGED_CAPTURE}"
echo "  ALOHA_YAML=${ALOHA_YAML}"
echo "  COLLECTION_SCENE_INVENTORY_JSON=${COLLECTION_SCENE_INVENTORY_JSON}"
echo "  TOPIC_CHECK_SCRIPT=${TOPIC_CHECK_SCRIPT}"
echo "  RUN_CONVERT=${RUN_CONVERT}, RUN_QC=${RUN_QC}, RUN_LEROBOT=${RUN_LEROBOT}"
echo "  WEB_HOST=${WEB_HOST}, WEB_PORT=${WEB_PORT}"

run_web_loop

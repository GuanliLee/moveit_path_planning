#!/usr/bin/env bash
# Standalone web console for MCAP conversion, HDF5 QC, repair, replay,
# and LeRobot export. This intentionally does not start collection,
# staged automation, or the unified gateway.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PIPELINE_ROOT="${PIPELINE_ROOT:-${REPO_ROOT}/scripts/embodied_data_pipeline-main}"
PIPELINE_APP="${PIPELINE_APP:-${PIPELINE_ROOT}/scripts/pipeline_web_app.py}"

WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-8012}"
INSTALL_DEPS=0
CHECK_DEPS=1
DATA_PATH_ARG=""
PIPELINE_PYTHON_ARG=""

PIPELINE_DATA_ROOT_VALUE="${PIPELINE_DATA_ROOT:-}"
PIPELINE_DEFAULT_MCAP_PATH_VALUE="${PIPELINE_DEFAULT_MCAP_PATH:-}"
PIPELINE_DEFAULT_DATASET_NAME_VALUE="${PIPELINE_DEFAULT_DATASET_NAME:-}"
PIPELINE_DEFAULT_HDF5_ROOT_VALUE="${PIPELINE_DEFAULT_HDF5_ROOT:-}"
PIPELINE_DEFAULT_QC_ROOT_VALUE="${PIPELINE_DEFAULT_QC_ROOT:-}"
PIPELINE_DEFAULT_LEROBOT_ROOT_VALUE="${PIPELINE_DEFAULT_LEROBOT_ROOT:-}"
PIPELINE_DEFAULT_ROBOT_TYPE_VALUE="${PIPELINE_DEFAULT_ROBOT_TYPE:-aloha}"
PIPELINE_DEFAULT_TASK_TEXT_VALUE="${PIPELINE_DEFAULT_TASK_TEXT:-}"
PIPELINE_DEFAULT_REPO_ID_VALUE="${PIPELINE_DEFAULT_REPO_ID:-}"
PIPELINE_DEFAULT_GPU_DEVICE_VALUE="${PIPELINE_DEFAULT_GPU_DEVICE:-0}"
PIPELINE_DEFAULT_CONVERT_JOBS_VALUE="${PIPELINE_DEFAULT_CONVERT_JOBS:-6}"
PIPELINE_DEFAULT_USE_DOCKER_VALUE="${PIPELINE_DEFAULT_USE_DOCKER:-0}"
PIPELINE_FAILURE_ANNOTATIONS_JSON_VALUE="${PIPELINE_FAILURE_ANNOTATIONS_JSON:-}"
PIPELINE_DATA_SCAN_ROOT_VALUE="${PIPELINE_DATA_SCAN_ROOT:-/home/ligl/piper_data}"
PIPELINE_DEFAULT_PROFILE_VALUE="${PIPELINE_DEFAULT_PROFILE:-}"
PIPELINE_ALOHA_YAML_VALUE="${PIPELINE_ALOHA_YAML:-}"
PIPELINE_CAMERA_LAYOUT_VALUE="${PIPELINE_CAMERA_LAYOUT:-}"
PIPELINE_OUTPUT_NAMESPACE_VALUE="${PIPELINE_OUTPUT_NAMESPACE:-}"
PIPELINE_CAMERA_VARIANT_SELECTABLE_VALUE="${PIPELINE_CAMERA_VARIANT_SELECTABLE:-0}"
PIPELINE_CAMERA_COUNT_VALUE="${PIPELINE_CAMERA_COUNT:-}"
PIPELINE_HEAD_CAMERA_SOURCE_VALUE="${PIPELINE_HEAD_CAMERA_SOURCE:-}"

usage() {
    cat <<EOF
Usage:
  $0
  $0 --install-deps

Examples:
  $0
  $0 --mcap-path /home/agilex/data/market_6 --dataset-name market_6
  $0 --hdf5-root /home/agilex/data/stage2/hdf5_episodes/market_6 --dataset-name market_6

Options:
  --host HOST                 Default: ${WEB_HOST}
  --port PORT                 Default: ${WEB_PORT}
  --python PYTHON             Python used to run the pipeline web app.
  --install-deps              Install quality_pipeline/requirements.txt into the selected Python.
  --no-deps-check             Skip Python dependency check.
  --data-root DIR             Optional page preset.
  --mcap-path PATH            Optional page preset.
  --dataset-name NAME         Optional page preset.
  --hdf5-root DIR             Optional page preset.
  --qc-root DIR               Optional page preset.
  --lerobot-root DIR          Optional page preset.
  --robot-type aloha|g2       Optional page preset.
  --task-text TEXT            Fill only missing HDF5 tasks; never override existing tasks.
  --repo-id ID                Optional page preset.
  --gpu-device ID             Optional page preset.
  --jobs N                    Optional page preset.
  --camera-count 3|4          Initial saved camera count for selectable camera pages.
  --head-camera front|global  Initial head source for three-camera mode.
  PIPELINE_FAILURE_ANNOTATIONS_JSON Optional manual failure sidecar path.
  -h, --help                  Show this help.

Environment:
  PIPELINE_ROOT               Default: ${PIPELINE_ROOT}
  PIPELINE_DATA_SCAN_ROOT     Default: ${PIPELINE_DATA_SCAN_ROOT_VALUE}
  PIPELINE_MCAP_PYTHON        Default: /usr/bin/python3 when it can import rosbag2_py.
  LEROBOT_PYTHON              Default: first Python with the current LeRobot dataset API.

Open:
  http://127.0.0.1:${WEB_PORT}/
EOF
}

log() {
    printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

die() {
    echo "[error] $*" >&2
    exit 1
}

source_if_exists() {
    local setup_file="$1"
    if [ -f "${setup_file}" ]; then
        # shellcheck disable=SC1090
        source "${setup_file}"
    fi
}

source_ros_env() {
    source_if_exists /opt/ros/humble/setup.bash
    source_if_exists /opt/ros/jazzy/setup.bash
    source_if_exists /home/agilex/agilex_ws/install/setup.bash
    source_if_exists /home/agilex/camera_ros/install/setup.bash
    source_if_exists /home/agilex/piper_ros/install/setup.bash
    source_if_exists /home/caizj/agilex_idata/ros2_ws/install/setup.bash
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
}

resolve_python() {
    local candidate="$1"
    if [ -z "${candidate}" ]; then
        return 1
    fi
    if [[ "${candidate}" == */* ]]; then
        [ -x "${candidate}" ] || return 1
        printf '%s\n' "${candidate}"
        return 0
    fi
    command -v "${candidate}" 2>/dev/null || return 1
}

python_has_modules() {
    local py="$1"
    shift
    "${py}" - "$@" <<'PY' >/dev/null 2>&1
import importlib.util
import sys

missing = [name for name in sys.argv[1:] if importlib.util.find_spec(name) is None]
raise SystemExit(1 if missing else 0)
PY
}

python_has_rosbag2() {
    local py="$1"
    "${py}" - <<'PY' >/dev/null 2>&1
try:
    import rosbag2_py  # noqa: F401
except Exception:
    raise SystemExit(1)
raise SystemExit(0)
PY
}

python_has_lerobot_dataset_api() {
    local py="$1"
    "${py}" - <<'PY' >/dev/null 2>&1
try:
    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
    except ImportError:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
except Exception:
    raise SystemExit(1)
raise SystemExit(0)
PY
}

choose_pipeline_python() {
    local explicit="${PIPELINE_PYTHON_ARG:-${PIPELINE_PYTHON:-}}"
    local py
    if py="$(resolve_python "${explicit}")"; then
        printf '%s\n' "${py}"
        return 0
    fi

    local candidates=(
        "/home/agilex/miniforge3/envs/lerobot/bin/python"
        "/home/caizj/miniforge3/envs/lerobot/bin/python"
        "/home/agilex/openpi/.venv/bin/python"
        "python3"
    )

    local fallback=""
    local dep_ok=""
    for candidate in "${candidates[@]}"; do
        py="$(resolve_python "${candidate}")" || continue
        [ -n "${fallback}" ] || fallback="${py}"
        if python_has_modules "${py}" h5py lerobot numpy cv2 pyarrow yaml scipy tqdm &&
            python_has_lerobot_dataset_api "${py}"; then
            [ -n "${dep_ok}" ] || dep_ok="${py}"
            if python_has_rosbag2 "${py}"; then
                printf '%s\n' "${py}"
                return 0
            fi
        fi
    done

    if [ -n "${dep_ok}" ]; then
        printf '%s\n' "${dep_ok}"
        return 0
    fi
    if [ -n "${fallback}" ]; then
        printf '%s\n' "${fallback}"
        return 0
    fi
    return 1
}

choose_lerobot_python() {
    local py
    if py="$(resolve_python "${LEROBOT_PYTHON:-}")" && python_has_lerobot_dataset_api "${py}"; then
        printf '%s\n' "${py}"
        return 0
    fi
    for candidate in \
        "/home/agilex/openpi/.venv/bin/python" \
        "${PIPELINE_PYTHON}" \
        "/home/agilex/miniforge3/envs/lerobot/bin/python" \
        "/home/caizj/miniforge3/envs/lerobot/bin/python" \
        "python3"
    do
        py="$(resolve_python "${candidate}")" || continue
        if python_has_lerobot_dataset_api "${py}"; then
            printf '%s\n' "${py}"
            return 0
        fi
    done
    printf '%s\n' "${PIPELINE_PYTHON}"
}

choose_mcap_python() {
    local py
    if py="$(resolve_python "${PIPELINE_MCAP_PYTHON:-}")" && python_has_rosbag2 "${py}"; then
        printf '%s\n' "${py}"
        return 0
    fi
    if py="$(resolve_python "/usr/bin/python3")" && python_has_rosbag2 "${py}"; then
        printf '%s\n' "${py}"
        return 0
    fi
    if python_has_rosbag2 "${PIPELINE_PYTHON}"; then
        printf '%s\n' "${PIPELINE_PYTHON}"
        return 0
    fi
    if py="$(resolve_python "${PIPELINE_MCAP_PYTHON:-}")"; then
        printf '%s\n' "${py}"
        return 0
    fi
    if py="$(resolve_python "/usr/bin/python3")"; then
        printf '%s\n' "${py}"
        return 0
    fi
    printf '%s\n' "${PIPELINE_PYTHON}"
}

install_deps() {
    [ -f "${PIPELINE_ROOT}/quality_pipeline/requirements.txt" ] ||
        die "Missing requirements: ${PIPELINE_ROOT}/quality_pipeline/requirements.txt"
    log "Installing pipeline Python dependencies with ${PIPELINE_PYTHON}"
    "${PIPELINE_PYTHON}" -m pip install --upgrade pip setuptools wheel
    "${PIPELINE_PYTHON}" -m pip install -r "${PIPELINE_ROOT}/quality_pipeline/requirements.txt"
}

check_deps() {
    "${PIPELINE_PYTHON}" - "$PIPELINE_PYTHON" <<'PY'
import importlib.util
import sys

required = {
    "h5py": "h5py",
    "lerobot": "lerobot",
    "numpy": "numpy",
    "opencv-python": "cv2",
    "pandas": "pandas",
    "pyarrow": "pyarrow",
    "PyYAML": "yaml",
    "scipy": "scipy",
    "tf-transformations": "tf_transformations",
    "tqdm": "tqdm",
}

def has_module(module):
    try:
        return importlib.util.find_spec(module) is not None
    except Exception:
        return False

missing = [pkg for pkg, module in required.items() if not has_module(module)]
if missing:
    print("Selected Python:", sys.argv[1], file=sys.stderr)
    print("Missing Python packages: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
PY
}

check_port_free() {
    python3 - "$WEB_HOST" "$WEB_PORT" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
bind_host = "0.0.0.0" if host in {"", "::"} else host
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((bind_host, port))
    except OSError as exc:
        raise SystemExit(f"port {port} is not available on {host}: {exc}")
PY
}

print_urls() {
    local display_host="${WEB_PUBLIC_HOST:-${WEB_HOST}}"
    local route_ip=""
    local candidate_ips=""
    if [ "${WEB_HOST}" = "0.0.0.0" ] || [ "${WEB_HOST}" = "::" ]; then
        if [ -z "${WEB_PUBLIC_HOST:-}" ]; then
            route_ip="$(ip route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
            display_host="${route_ip:-127.0.0.1}"
        fi
        candidate_ips="$(hostname -I 2>/dev/null | tr ' ' '\n' | sed '/^$/d' | paste -sd ' ' -)"
    fi

    echo ""
    echo "===== Pipeline QC web is running ====="
    echo "Local:   http://127.0.0.1:${WEB_PORT}/"
    echo "Network: http://${display_host}:${WEB_PORT}/"
    if [ -n "${candidate_ips}" ]; then
        echo "Host IP candidates: ${candidate_ips}"
    fi
    echo "Log:     ${PIPELINE_LOG}"
    echo ""
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --install-deps)
            INSTALL_DEPS=1
            shift
            ;;
        --no-deps-check)
            CHECK_DEPS=0
            shift
            ;;
        --host)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            WEB_HOST="$2"
            shift 2
            ;;
        --host=*)
            WEB_HOST="${1#*=}"
            shift
            ;;
        --port)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            WEB_PORT="$2"
            shift 2
            ;;
        --port=*)
            WEB_PORT="${1#*=}"
            shift
            ;;
        --python)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_PYTHON_ARG="$2"
            shift 2
            ;;
        --python=*)
            PIPELINE_PYTHON_ARG="${1#*=}"
            shift
            ;;
        --data-root)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DATA_ROOT_VALUE="$2"
            shift 2
            ;;
        --data-root=*)
            PIPELINE_DATA_ROOT_VALUE="${1#*=}"
            shift
            ;;
        --mcap-path)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_MCAP_PATH_VALUE="$2"
            shift 2
            ;;
        --mcap-path=*)
            PIPELINE_DEFAULT_MCAP_PATH_VALUE="${1#*=}"
            shift
            ;;
        --dataset-name)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_DATASET_NAME_VALUE="$2"
            shift 2
            ;;
        --dataset-name=*)
            PIPELINE_DEFAULT_DATASET_NAME_VALUE="${1#*=}"
            shift
            ;;
        --hdf5-root)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_HDF5_ROOT_VALUE="$2"
            shift 2
            ;;
        --hdf5-root=*)
            PIPELINE_DEFAULT_HDF5_ROOT_VALUE="${1#*=}"
            shift
            ;;
        --qc-root)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_QC_ROOT_VALUE="$2"
            shift 2
            ;;
        --qc-root=*)
            PIPELINE_DEFAULT_QC_ROOT_VALUE="${1#*=}"
            shift
            ;;
        --lerobot-root)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_LEROBOT_ROOT_VALUE="$2"
            shift 2
            ;;
        --lerobot-root=*)
            PIPELINE_DEFAULT_LEROBOT_ROOT_VALUE="${1#*=}"
            shift
            ;;
        --robot-type)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_ROBOT_TYPE_VALUE="$2"
            shift 2
            ;;
        --robot-type=*)
            PIPELINE_DEFAULT_ROBOT_TYPE_VALUE="${1#*=}"
            shift
            ;;
        --task-text)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_TASK_TEXT_VALUE="$2"
            shift 2
            ;;
        --task-text=*)
            PIPELINE_DEFAULT_TASK_TEXT_VALUE="${1#*=}"
            shift
            ;;
        --repo-id)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_REPO_ID_VALUE="$2"
            shift 2
            ;;
        --repo-id=*)
            PIPELINE_DEFAULT_REPO_ID_VALUE="${1#*=}"
            shift
            ;;
        --gpu-device)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_GPU_DEVICE_VALUE="$2"
            shift 2
            ;;
        --gpu-device=*)
            PIPELINE_DEFAULT_GPU_DEVICE_VALUE="${1#*=}"
            shift
            ;;
        --jobs)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_DEFAULT_CONVERT_JOBS_VALUE="$2"
            shift 2
            ;;
        --jobs=*)
            PIPELINE_DEFAULT_CONVERT_JOBS_VALUE="${1#*=}"
            shift
            ;;
        --camera-count)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_CAMERA_COUNT_VALUE="$2"
            shift 2
            ;;
        --camera-count=*)
            PIPELINE_CAMERA_COUNT_VALUE="${1#*=}"
            shift
            ;;
        --head-camera)
            [ "$#" -ge 2 ] || die "$1 requires a value"
            PIPELINE_HEAD_CAMERA_SOURCE_VALUE="$2"
            shift 2
            ;;
        --head-camera=*)
            PIPELINE_HEAD_CAMERA_SOURCE_VALUE="${1#*=}"
            shift
            ;;
        *)
            die "Unknown argument: $1"
            ;;
    esac
done

[[ "${WEB_PORT}" =~ ^[0-9]+$ ]] || die "WEB_PORT must be a number: ${WEB_PORT}"
if [ -n "${PIPELINE_DEFAULT_CONVERT_JOBS_VALUE}" ]; then
    [[ "${PIPELINE_DEFAULT_CONVERT_JOBS_VALUE}" =~ ^[0-9]+$ ]] ||
        die "--jobs must be a positive integer: ${PIPELINE_DEFAULT_CONVERT_JOBS_VALUE}"
    [ "${PIPELINE_DEFAULT_CONVERT_JOBS_VALUE}" -ge 1 ] ||
        die "--jobs must be >= 1"
fi
if [ -n "${PIPELINE_CAMERA_COUNT_VALUE}" ] &&
    [ "${PIPELINE_CAMERA_COUNT_VALUE}" != "3" ] &&
    [ "${PIPELINE_CAMERA_COUNT_VALUE}" != "4" ]; then
    die "--camera-count must be 3 or 4: ${PIPELINE_CAMERA_COUNT_VALUE}"
fi
if [ -n "${PIPELINE_HEAD_CAMERA_SOURCE_VALUE}" ] &&
    [ "${PIPELINE_HEAD_CAMERA_SOURCE_VALUE}" != "front" ] &&
    [ "${PIPELINE_HEAD_CAMERA_SOURCE_VALUE}" != "global" ]; then
    die "--head-camera must be front or global: ${PIPELINE_HEAD_CAMERA_SOURCE_VALUE}"
fi

[ -d "${PIPELINE_ROOT}" ] || die "Missing pipeline root: ${PIPELINE_ROOT}"
[ -f "${PIPELINE_APP}" ] || die "Missing pipeline app: ${PIPELINE_APP}"

LOG_ROOT="${WEB_LOG_ROOT:-}"
if [ -z "${LOG_ROOT}" ]; then
    if [ -n "${PIPELINE_DATA_ROOT_VALUE}" ]; then
        LOG_ROOT="${PIPELINE_DATA_ROOT_VALUE%/}/logs/pipeline_qc_web"
    else
        LOG_ROOT="${PIPELINE_ROOT}/logs/pipeline_qc_web"
    fi
fi
mkdir -p "${LOG_ROOT}"
PIPELINE_LOG="${PIPELINE_LOG:-${LOG_ROOT}/pipeline_qc_web_$(date +%Y%m%d_%H%M%S).log}"

source_ros_env

PIPELINE_PYTHON="$(choose_pipeline_python)" || die "Could not find a usable Python"
LEROBOT_PYTHON="$(choose_lerobot_python)"
PIPELINE_MCAP_PYTHON="$(choose_mcap_python)"
export PIPELINE_PYTHON LEROBOT_PYTHON PIPELINE_MCAP_PYTHON

if [ "${INSTALL_DEPS}" -eq 1 ]; then
    install_deps
fi
if [ "${CHECK_DEPS}" -eq 1 ]; then
    if ! check_deps; then
        echo "" >&2
        echo "Run this once to install missing packages into the selected Python:" >&2
        echo "  $0 --install-deps" >&2
        exit 1
    fi
fi
if ! python_has_rosbag2 "${PIPELINE_MCAP_PYTHON}"; then
    echo "[warning] ${PIPELINE_MCAP_PYTHON} cannot import rosbag2_py." >&2
    echo "[warning] Existing HDF5 QC and LeRobot export can still work, but host MCAP conversion may fail." >&2
    echo "[warning] For host MCAP conversion, use a Python that can import rosbag2_py." >&2
fi

check_port_free

log "Pipeline root: ${PIPELINE_ROOT}"
log "Pipeline Python: ${PIPELINE_PYTHON}"
log "MCAP conversion Python: ${PIPELINE_MCAP_PYTHON}"
log "LeRobot Python: ${LEROBOT_PYTHON}"
log "Dataset scan root: ${PIPELINE_DATA_SCAN_ROOT_VALUE}"
log "Local dataset scan root: ${PIPELINE_DATA_SCAN_ROOT_VALUE}"
log "Starting standalone QC/conversion web on ${WEB_HOST}:${WEB_PORT}"

print_urls

env \
    PIPELINE_DATA_ROOT="${PIPELINE_DATA_ROOT_VALUE}" \
    PIPELINE_DEFAULT_MCAP_PATH="${PIPELINE_DEFAULT_MCAP_PATH_VALUE}" \
    PIPELINE_DEFAULT_DATASET_NAME="${PIPELINE_DEFAULT_DATASET_NAME_VALUE}" \
    PIPELINE_DEFAULT_HDF5_ROOT="${PIPELINE_DEFAULT_HDF5_ROOT_VALUE}" \
    PIPELINE_DEFAULT_QC_ROOT="${PIPELINE_DEFAULT_QC_ROOT_VALUE}" \
    PIPELINE_DEFAULT_LEROBOT_ROOT="${PIPELINE_DEFAULT_LEROBOT_ROOT_VALUE}" \
    PIPELINE_DEFAULT_ROBOT_TYPE="${PIPELINE_DEFAULT_ROBOT_TYPE_VALUE}" \
    PIPELINE_DEFAULT_TASK_TEXT="${PIPELINE_DEFAULT_TASK_TEXT_VALUE}" \
    PIPELINE_DEFAULT_REPO_ID="${PIPELINE_DEFAULT_REPO_ID_VALUE}" \
    PIPELINE_DEFAULT_GPU_DEVICE="${PIPELINE_DEFAULT_GPU_DEVICE_VALUE}" \
    PIPELINE_DEFAULT_CONVERT_JOBS="${PIPELINE_DEFAULT_CONVERT_JOBS_VALUE}" \
    PIPELINE_DEFAULT_USE_DOCKER="${PIPELINE_DEFAULT_USE_DOCKER_VALUE}" \
    PIPELINE_FAILURE_ANNOTATIONS_JSON="${PIPELINE_FAILURE_ANNOTATIONS_JSON_VALUE}" \
    PIPELINE_DATA_SCAN_ROOT="${PIPELINE_DATA_SCAN_ROOT_VALUE}" \
    PIPELINE_MCAP_PYTHON="${PIPELINE_MCAP_PYTHON}" \
    LEROBOT_PYTHON="${LEROBOT_PYTHON}" \
    PIPELINE_DEFAULT_PROFILE="${PIPELINE_DEFAULT_PROFILE_VALUE}" \
    PIPELINE_ALOHA_YAML="${PIPELINE_ALOHA_YAML_VALUE}" \
    PIPELINE_CAMERA_LAYOUT="${PIPELINE_CAMERA_LAYOUT_VALUE}" \
    PIPELINE_OUTPUT_NAMESPACE="${PIPELINE_OUTPUT_NAMESPACE_VALUE}" \
    PIPELINE_CAMERA_VARIANT_SELECTABLE="${PIPELINE_CAMERA_VARIANT_SELECTABLE_VALUE}" \
    PIPELINE_CAMERA_COUNT="${PIPELINE_CAMERA_COUNT_VALUE}" \
    PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE_VALUE}" \
    "${PIPELINE_PYTHON}" -u "${PIPELINE_APP}" --host "${WEB_HOST}" --port "${WEB_PORT}" \
    2>&1 | tee -a "${PIPELINE_LOG}"

#!/usr/bin/env bash
# One operator-facing Web entry point for all supported collection modes.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/collection_ros_env.sh"

WEB_HOST="${WEB_HOST:-192.168.3.101}"
WEB_PORT="${WEB_PORT:-8000}"
COLLECTION_LAUNCHER_RUNTIME_ROOT="${COLLECTION_LAUNCHER_RUNTIME_ROOT:-/home/agilex/data/logs/unified_collection_web}"

exec python3 -u "${SCRIPT_DIR}/collection_web_launcher.py" \
    --host "${WEB_HOST}" \
    --port "${WEB_PORT}" \
    --runtime-root "${COLLECTION_LAUNCHER_RUNTIME_ROOT}"

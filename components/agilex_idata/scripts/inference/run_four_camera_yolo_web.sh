#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-7788}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
YOLO_URL="${YOLO_URL:-http://192.168.4.121:7881}"
DEFAULT_INTERVAL_SEC="${DEFAULT_INTERVAL_SEC:-0.2}"
REQUEST_TIMEOUT="${REQUEST_TIMEOUT:-10.0}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if [[ ! "${WEB_PORT}" =~ ^[0-9]+$ ]] || (( WEB_PORT < 1 || WEB_PORT > 65535 )); then
  echo "[错误] WEB_PORT 必须是 1–65535 的整数：${WEB_PORT}" >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[错误] 找不到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/agilex/camera_ros/install/setup.bash ]]; then
  source /home/agilex/camera_ros/install/setup.bash
fi
set -u

if command -v ss >/dev/null 2>&1 && ss -ltnH "sport = :${WEB_PORT}" | grep -q .; then
  echo "[错误] 端口 ${WEB_PORT} 已被占用；现有监听进程保持不变。" >&2
  exit 1
fi

export ROS_DOMAIN_ID
exec "${PYTHON_BIN}" -u "${SCRIPT_DIR}/four_camera_yolo_web.py" \
  --host "${WEB_HOST}" \
  --port "${WEB_PORT}" \
  --ros-domain-id "${ROS_DOMAIN_ID}" \
  --yolo-url "${YOLO_URL}" \
  --default-interval-sec "${DEFAULT_INTERVAL_SEC}" \
  --request-timeout "${REQUEST_TIMEOUT}" \
  "$@"

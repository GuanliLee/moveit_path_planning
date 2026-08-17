#!/usr/bin/env bash
# Shared startup environment for every collection UI entry point.
# Keep these values aligned with the robot's deployed ROS graph.

if ! command -v proxy_off >/dev/null 2>&1; then
    proxy_off() {
        unset http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY
    }
fi
proxy_off >/dev/null 2>&1 || true
unset http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY all_proxy ALL_PROXY

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[错误] 找不到 /opt/ros/humble/setup.bash" >&2
    return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ ! -f /home/agilex/camera_ros/install/setup.bash ]; then
    echo "[错误] 找不到 /home/agilex/camera_ros/install/setup.bash" >&2
    return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source /home/agilex/camera_ros/install/setup.bash

export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=99

collection_acquire_session_lock() {
    if [ "${COLLECTION_SESSION_LOCK_HELD:-0}" = "1" ]; then
        return 0
    fi
    local lock_file="${COLLECTION_SESSION_LOCK_FILE:-/tmp/agilex_idata_collection_session.lock}"
    command -v flock >/dev/null 2>&1 || {
        echo "[错误] 缺少 flock，无法保证人工采集和推理录制互斥" >&2
        return 1
    }
    mkdir -p "$(dirname "${lock_file}")"
    exec {COLLECTION_SESSION_LOCK_FD}>"${lock_file}"
    if ! flock -n "${COLLECTION_SESSION_LOCK_FD}"; then
        echo "[错误] 已有人工采集或推理录制会话运行中；请先从原页面退出" >&2
        return 1
    fi
    export COLLECTION_SESSION_LOCK_HELD=1
}

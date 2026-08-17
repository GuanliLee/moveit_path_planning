#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
SERVER_BIN="${WORKSPACE}/install/path_planning_server/lib/path_planning_server/path_planning_server_node"
SESSION_NAME="${PATH_PLANNING_SESSION_NAME:-path_planning_server0807}"
LEGACY_SESSION_NAME="path_planning_node_cancel"
LOG_FILE="${PATH_PLANNING_LOG_FILE:-/tmp/path_planning_server0807.log}"
LOCK_FILE="/tmp/path_planning_server0807_restart.lock"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
SCENE_ID="${PATH_PLANNING_SCENE_ID:-1}"
EXECUTE_TRAJECTORY="${PATH_PLANNING_EXECUTE_TRAJECTORY:-true}"

usage()
{
  cat <<'EOF'
用法:
  ./restart_path_planning_server.sh [restart|start|stop|status|logs] [--build]

动作:
  restart   重启规划服务（默认）
  start     启动规划服务
  stop      停止规划服务
  status    查看进程、screen 会话和 ROS 服务状态
  logs      持续查看规划服务日志
  --build   操作前先编译 path_planning_server

环境变量:
  ROS_DOMAIN_ID=66
  PATH_PLANNING_SCENE_ID=1
  PATH_PLANNING_EXECUTE_TRAJECTORY=true
  PATH_PLANNING_LOG_FILE=/tmp/path_planning_server0807.log
EOF
}

ACTION="restart"
BUILD_FIRST=false
ACTION_SET=false
for argument in "$@"; do
  case "${argument}" in
    start|stop|restart|status|logs)
      if ${ACTION_SET}; then
        echo "只能指定一个动作" >&2
        usage
        exit 2
      fi
      ACTION="${argument}"
      ACTION_SET=true
      ;;
    --build)
      BUILD_FIRST=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: ${argument}" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ ! "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
  echo "ROS_DOMAIN_ID 必须是非负整数" >&2
  exit 2
fi
if [[ ! "${SCENE_ID}" =~ ^[0-9]+$ ]]; then
  echo "PATH_PLANNING_SCENE_ID 必须是非负整数" >&2
  exit 2
fi
if [[ "${EXECUTE_TRAJECTORY}" != "true" && "${EXECUTE_TRAJECTORY}" != "false" ]]; then
  echo "PATH_PLANNING_EXECUTE_TRAJECTORY 必须为 true 或 false" >&2
  exit 2
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "找不到 ROS 环境: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
if [[ -f "${WORKSPACE_SETUP}" ]]; then
  source "${WORKSPACE_SETUP}"
fi
set -u

export ROS_DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE
export RMW_IMPLEMENTATION

server_pids()
{
  pgrep -u "$(id -u)" -f "^${SERVER_BIN}([[:space:]]|$)" || true
}

screen_session_exists()
{
  local session_name="$1"
  screen -ls 2>/dev/null | grep -Eq "[0-9]+[.]${session_name}[[:space:]]"
}

ros_service_exists()
{
  local service_name="$1"
  timeout 6 ros2 service list --no-daemon --spin-time 3 2>/dev/null |
    grep -Fxq "${service_name}"
}

build_server()
{
  echo "[1/3] 编译 path_planning_server..."
  (
    cd "${WORKSPACE}"
    colcon build --packages-select \
      path_planning_interfaces path_planning_server --symlink-install
  )
  set +u
  source "${WORKSPACE_SETUP}"
  set -u
  echo "编译完成"
}

cancel_active_request()
{
  if ros_service_exists "/cancel_plan_execution"; then
    echo "请求取消可能正在执行的规划或轨迹..."
    timeout 4 ros2 service call \
      /cancel_plan_execution std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  fi
}

stop_server()
{
  local pids
  pids="$(server_pids)"
  if [[ -z "${pids}" ]]; then
    echo "规划服务当前没有运行"
  else
    cancel_active_request
    echo "停止规划服务 PID: ${pids//$'\n'/ }"
    kill -INT ${pids}

    for _ in {1..50}; do
      [[ -z "$(server_pids)" ]] && break
      sleep 0.1
    done

    pids="$(server_pids)"
    if [[ -n "${pids}" ]]; then
      echo "进程未在 5 秒内退出，发送 SIGTERM: ${pids//$'\n'/ }"
      kill -TERM ${pids}
      for _ in {1..20}; do
        [[ -z "$(server_pids)" ]] && break
        sleep 0.1
      done
    fi

    if [[ -n "$(server_pids)" ]]; then
      echo "规划服务未能停止，请人工检查；未发送 SIGKILL" >&2
      return 1
    fi
  fi

  for session_name in "${SESSION_NAME}" "${LEGACY_SESSION_NAME}"; do
    if screen_session_exists "${session_name}"; then
      screen -S "${session_name}" -X quit || true
    fi
  done
}

start_server()
{
  if [[ -n "$(server_pids)" ]]; then
    echo "规划服务已经运行，PID: $(server_pids | tr '\n' ' ')" >&2
    return 1
  fi
  if [[ ! -x "${SERVER_BIN}" ]]; then
    echo "找不到规划服务程序，请先使用 --build: ${SERVER_BIN}" >&2
    return 1
  fi
  if ! ros_service_exists "/get_planning_scene"; then
    echo "警告: 当前未发现 /get_planning_scene；请确认 move_group 正在运行" >&2
  fi

  if screen_session_exists "${SESSION_NAME}"; then
    screen -S "${SESSION_NAME}" -X quit || true
  fi

  echo "[2/3] 启动规划服务，日志: ${LOG_FILE}"
  screen -L -Logfile "${LOG_FILE}" -dmS "${SESSION_NAME}" \
    ros2 launch path_planning_server planning_server_only.launch.py \
    "scene_id:=${SCENE_ID}" \
    "execute_trajectory:=${EXECUTE_TRAJECTORY}"

  for _ in {1..60}; do
    if [[ -n "$(server_pids)" ]] &&
      screen_session_exists "${SESSION_NAME}" &&
      ros_service_exists "/plan_to_pose" &&
      ros_service_exists "/plan_to_joints" &&
      ros_service_exists "/cancel_plan_execution"
    then
      echo "[3/3] 规划服务已就绪"
      return 0
    fi

    if [[ -z "$(server_pids)" ]] && ! screen_session_exists "${SESSION_NAME}"; then
      echo "规划服务启动失败，最近日志如下:" >&2
      tail -n 80 "${LOG_FILE}" >&2 || true
      return 1
    fi
    sleep 0.5
  done

  echo "规划进程已启动，但 30 秒内未发现 ROS 服务，请检查日志:" >&2
  tail -n 80 "${LOG_FILE}" >&2 || true
  return 1
}

show_status()
{
  local pids
  pids="$(server_pids)"
  if [[ -n "${pids}" ]]; then
    echo "进程: 运行中，PID ${pids//$'\n'/ }"
  else
    echo "进程: 未运行"
  fi

  if screen_session_exists "${SESSION_NAME}"; then
    echo "screen: ${SESSION_NAME} 运行中"
  else
    echo "screen: ${SESSION_NAME} 不存在"
  fi

  if [[ -n "${pids}" ]] && ros_service_exists "/plan_to_pose"; then
    echo "ROS 服务: /plan_to_pose 已就绪"
  elif ros_service_exists "/plan_to_pose"; then
    echo "ROS 服务: 图中仍有 /plan_to_pose，但本机进程不存在（可能是残留发现信息）"
  else
    echo "ROS 服务: /plan_to_pose 未发现"
  fi

  if ros_service_exists "/get_planning_scene"; then
    echo "MoveIt: /get_planning_scene 已就绪"
  else
    echo "MoveIt: /get_planning_scene 未发现"
  fi

  echo "日志: ${LOG_FILE}"
}

if [[ "${ACTION}" != "status" && "${ACTION}" != "logs" ]]; then
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    echo "另一个规划服务启停操作正在进行" >&2
    exit 1
  fi
fi

if ${BUILD_FIRST}; then
  build_server
fi

case "${ACTION}" in
  start)
    start_server
    ;;
  stop)
    stop_server
    ;;
  restart)
    stop_server
    start_server
    ;;
  status)
    show_status
    ;;
  logs)
    touch "${LOG_FILE}"
    tail -n 100 -F "${LOG_FILE}"
    ;;
esac

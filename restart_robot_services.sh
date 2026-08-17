#!/usr/bin/env bash
set -euo pipefail

PLANNING_WS="/home/ligl/path_planning_service0807"
GRASPNET_WS="/home/ligl/graspnet_path_planning_bridge0804"
CAMERA_DIR="/home/ligl/realsense_three_cameras"
IMAGE_BRIDGE_DIR="/home/ligl/agilex_xpc/bridge"
WEB_DIR="/home/ligl/drink_grasp_web"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
LOCK_FILE="/tmp/restart_robot_services.lock"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
DISPLAY="${DISPLAY:-:1}"
XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
PIPER_AUTO_ENABLE="${PIPER_AUTO_ENABLE:-true}"

STATE_SESSION="grasp_state_machine0807"
STATE_LOG="/tmp/grasp_bridge_state_machine0807.log"
STATE_PATTERN="${PLANNING_WS}/install/path_planning_server/lib/path_planning_server/grasp_bridge_state_machine([[:space:]]|$)"

BRIDGE_SESSION="grasp_bridge0807"
BRIDGE_LOG="/tmp/graspnet_path_planning_bridge0807.log"
BRIDGE_PATTERN="${GRASPNET_WS}/install/graspnet_path_planning_bridge/lib/graspnet_path_planning_bridge/bridge_node([[:space:]]|$)"

CAMERA_SESSION="realsense_rgbd_aligned"
CAMERA_LOG="/tmp/realsense_rgbd_aligned_20260812.log"
CAMERA_PATTERN="ros2 launch realsense_multi_camera_config three_d435i[.]launch[.]py"

IMAGE_SESSION="agilex_image_bridge0807"
IMAGE_LOG="/tmp/agilex_image_bridge0807.log"
IMAGE_PATTERN="^/usr/bin/python3 -m image_bridge[.]main([[:space:]]|$)"

RVIZ_SESSION="path_planning_tcp_rviz"
RVIZ_LOG="/tmp/path_planning_tcp_rviz.log"
RVIZ_PATTERN="/opt/ros/jazzy/lib/rviz2/rviz2 .*__node:=path_planning_rviz"

HANDEYE_SESSION="handeye_camera_tf0807"
HANDEYE_LOG="/tmp/handeye_camera_tf0807.log"
HANDEYE_PATTERN="static_transform_publisher .*__node:=(left|right)_link6_to_cam_(left|right)_link"

HARDWARE_SESSION="path_planning0807_rviz"
HARDWARE_LOG="/tmp/path_planning_service0807_hardware_rviz.log"
HARDWARE_PATTERN="ros2 launch path_planning_server dual_piper_hardware[.]launch[.]py"

usage()
{
  cat <<'EOF'
用法:
  ./restart_robot_services.sh <组件> [restart|start|stop|status|logs] [选项]

组件:
  planning         路径规划服务 /path_planning_server
  state-machine    抓取任务状态机
  graspnet-bridge  GraspNet 到规划服务的桥
  cameras          cam_high/cam_left/cam_right 三台 RealSense
  image-bridge     AgileX 图像/控制 WebSocket 桥
  web              饮料抓取 Web 服务（8090）
  rviz             RViz 界面
  handeye-tf       左右腕相机静态手眼 TF
  software         纯软件链路：planning/image-bridge/graspnet/state-machine/web
  hardware         双臂驱动、轨迹桥、joint state、MoveIt、规划服务、手眼 TF
  all              仅支持 status

动作:
  restart          重启（默认）
  start            启动
  stop             停止
  status           查看状态
  logs             查看实时日志

选项:
  --build          先编译该组件所属工作区
  --confirm-hardware
                   hardware 的 start/stop/restart 必须显式提供

例子:
  ./restart_robot_services.sh planning restart --build
  ./restart_robot_services.sh state-machine restart --build
  ./restart_robot_services.sh cameras restart
  ./restart_robot_services.sh all status
  ./restart_robot_services.sh hardware restart --build --confirm-hardware
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

COMPONENT="$1"
shift
ACTION="restart"
ACTION_SET=false
BUILD_FIRST=false
CONFIRM_HARDWARE=false

for argument in "$@"; do
  case "${argument}" in
    restart|start|stop|status|logs)
      if ${ACTION_SET}; then
        echo "只能指定一个动作" >&2
        exit 2
      fi
      ACTION="${argument}"
      ACTION_SET=true
      ;;
    --build)
      BUILD_FIRST=true
      ;;
    --confirm-hardware)
      CONFIRM_HARDWARE=true
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

case "${COMPONENT}" in
  planning|state-machine|graspnet-bridge|cameras|image-bridge|web|rviz|handeye-tf|software|hardware|all) ;;
  *)
    echo "未知组件: ${COMPONENT}" >&2
    usage
    exit 2
    ;;
esac

if [[ "${COMPONENT}" == "all" && "${ACTION}" != "status" ]]; then
  echo "all 只支持 status，避免误重启整套设备" >&2
  exit 2
fi
if [[ "${COMPONENT}" == "hardware" &&
  "${ACTION}" != "status" && "${ACTION}" != "logs" &&
  "${CONFIRM_HARDWARE}" != "true" ]]
then
  echo "hardware 会中断机械臂驱动并按 PIPER_AUTO_ENABLE=${PIPER_AUTO_ENABLE} 重新启动。" >&2
  echo "确认机械臂周围安全后，增加 --confirm-hardware。" >&2
  exit 2
fi

set +u
source "${ROS_SETUP}"
if [[ -f "${GRASPNET_WS}/install/setup.bash" ]]; then
  source "${GRASPNET_WS}/install/setup.bash"
fi
if [[ -f "${PLANNING_WS}/install/setup.bash" ]]; then
  source "${PLANNING_WS}/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE
export RMW_IMPLEMENTATION
export DISPLAY
export XAUTHORITY
export XDG_RUNTIME_DIR
export PATH_PLANNING_WS="${PLANNING_WS}"

screen_exists()
{
  local session="$1"
  screen -ls 2>/dev/null | grep -Eq "[0-9]+[.]${session}[[:space:]]"
}

matching_pids()
{
  local pattern="$1"
  pgrep -u "$(id -u)" -f "${pattern}" || true
}

ros_service_exists()
{
  local service="$1"
  timeout 6 ros2 service list --no-daemon --spin-time 3 2>/dev/null |
    grep -Fxq "${service}"
}

cancel_active_tasks()
{
  if ros_service_exists "/cancel_named_grasp_task"; then
    echo "请求取消当前抓取任务..."
    timeout 5 ros2 service call       /cancel_named_grasp_task std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  fi
  if ros_service_exists "/cancel_plan_execution"; then
    echo "请求取消当前规划或轨迹..."
    timeout 5 ros2 service call       /cancel_plan_execution std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  fi
}

stop_processes()
{
  local label="$1"
  local pattern="$2"
  local session="$3"
  local pids
  pids="$(matching_pids "${pattern}")"

  if [[ -n "${pids}" ]]; then
    echo "停止 ${label}，PID: ${pids//$'\n'/ }"
    kill -INT ${pids}
    for _ in {1..80}; do
      [[ -z "$(matching_pids "${pattern}")" ]] && break
      sleep 0.1
    done
  fi

  pids="$(matching_pids "${pattern}")"
  if [[ -n "${pids}" ]]; then
    echo "${label} 未正常退出，发送 SIGTERM: ${pids//$'\n'/ }"
    kill -TERM ${pids}
    for _ in {1..30}; do
      [[ -z "$(matching_pids "${pattern}")" ]] && break
      sleep 0.1
    done
  fi

  if [[ -n "$(matching_pids "${pattern}")" ]]; then
    echo "${label} 仍未退出；未发送 SIGKILL，请人工检查" >&2
    return 1
  fi

  if [[ -n "${session}" ]] && screen_exists "${session}"; then
    screen -S "${session}" -X quit || true
  fi
}

wait_for_process()
{
  local label="$1"
  local pattern="$2"
  for _ in {1..60}; do
    if [[ -n "$(matching_pids "${pattern}")" ]]; then
      echo "${label} 已启动"
      return 0
    fi
    sleep 0.5
  done
  echo "${label} 启动超时" >&2
  return 1
}

build_planning()
{
  (
    cd "${PLANNING_WS}"
    colcon build --packages-select \
      path_planning_interfaces path_planning_server --symlink-install
  )
  set +u
  source "${PLANNING_WS}/install/setup.bash"
  set -u
}

build_graspnet()
{
  (
    cd "${GRASPNET_WS}"
    colcon build --packages-select \
      graspnet_bridge_interfaces path_planning_interfaces \
      graspnet_path_planning_bridge --symlink-install
  )
  set +u
  source "${GRASPNET_WS}/install/setup.bash"
  source "${PLANNING_WS}/install/setup.bash"
  set -u
}

build_hardware_workspace()
{
  (
    cd "${PLANNING_WS}"
    colcon build --symlink-install
  )
  set +u
  source "${PLANNING_WS}/install/setup.bash"
  set -u
}

start_state_machine()
{
  if [[ -n "$(matching_pids "${STATE_PATTERN}")" ]]; then
    echo "state-machine 已运行"
    return 0
  fi
  screen -L -Logfile "${STATE_LOG}" -dmS "${STATE_SESSION}"     ros2 launch path_planning_server grasp_bridge_state_machine.launch.py
  wait_for_process "state-machine" "${STATE_PATTERN}"
  for _ in {1..15}; do
    ros_service_exists "/execute_named_grasp_task" && return 0
  done
  echo "state-machine 进程存在，但服务未发现" >&2
  return 1
}

stop_state_machine()
{
  cancel_active_tasks
  stop_processes "state-machine" "${STATE_PATTERN}" "${STATE_SESSION}"
}

start_graspnet_bridge()
{
  if [[ -n "$(matching_pids "${BRIDGE_PATTERN}")" ]]; then
    echo "graspnet-bridge 已运行"
    return 0
  fi
  screen -L -Logfile "${BRIDGE_LOG}" -dmS "${BRIDGE_SESSION}"     ros2 launch graspnet_path_planning_bridge bridge.launch.py
  wait_for_process "graspnet-bridge" "${BRIDGE_PATTERN}"
}

stop_graspnet_bridge()
{
  stop_processes "graspnet-bridge" "${BRIDGE_PATTERN}" "${BRIDGE_SESSION}"
}

start_cameras()
{
  if [[ -n "$(matching_pids "${CAMERA_PATTERN}")" ]]; then
    echo "cameras 已运行"
    return 0
  fi
  screen -L -Logfile "${CAMERA_LOG}" -dmS "${CAMERA_SESSION}"     "${CAMERA_DIR}/start_calibration_rgb_depth_aligned.sh"
  wait_for_process "cameras" "${CAMERA_PATTERN}"
}

stop_cameras()
{
  stop_processes "cameras" "${CAMERA_PATTERN}" "${CAMERA_SESSION}"
}

start_image_bridge()
{
  if [[ -n "$(matching_pids "${IMAGE_PATTERN}")" ]]; then
    echo "image-bridge 已运行"
    return 0
  fi
  screen -L -Logfile "${IMAGE_LOG}" -dmS "${IMAGE_SESSION}"     "${IMAGE_BRIDGE_DIR}/scripts/run_bridge.sh"
  wait_for_process "image-bridge" "${IMAGE_PATTERN}"
}

stop_image_bridge()
{
  stop_processes "image-bridge" "${IMAGE_PATTERN}" "${IMAGE_SESSION}"
}

start_web()
{
  "${WEB_DIR}/start.sh"
}

stop_web()
{
  "${WEB_DIR}/stop.sh"
}

start_rviz()
{
  if [[ -n "$(matching_pids "${RVIZ_PATTERN}")" ]]; then
    echo "rviz 已运行"
    return 0
  fi
  screen -L -Logfile "${RVIZ_LOG}" -dmS "${RVIZ_SESSION}"     ros2 launch path_planning_server rviz_only.launch.py
  wait_for_process "rviz" "${RVIZ_PATTERN}"
}

stop_rviz()
{
  stop_processes "rviz" "${RVIZ_PATTERN}" "${RVIZ_SESSION}"
}

start_handeye_tf()
{
  if [[ -n "$(matching_pids "${HANDEYE_PATTERN}")" ]]; then
    echo "handeye-tf 已运行"
    return 0
  fi
  screen -L -Logfile "${HANDEYE_LOG}" -dmS "${HANDEYE_SESSION}"     ros2 launch path_planning_server handeye_camera_tf.launch.py
  for _ in {1..40}; do
    if [[ "$(matching_pids "${HANDEYE_PATTERN}" | wc -l)" -ge 2 ]]; then
      echo "handeye-tf 已启动"
      return 0
    fi
    sleep 0.5
  done
  echo "handeye-tf 启动超时" >&2
  return 1
}

stop_handeye_tf()
{
  stop_processes "handeye-tf" "${HANDEYE_PATTERN}" "${HANDEYE_SESSION}"
}

start_hardware()
{
  if [[ -n "$(matching_pids "${HARDWARE_PATTERN}")" ]]; then
    echo "hardware 已运行"
    return 0
  fi
  stop_handeye_tf || true
  screen -L -Logfile "${HARDWARE_LOG}" -dmS "${HARDWARE_SESSION}"     ros2 launch path_planning_server dual_piper_hardware.launch.py     can_left_port:=can0     can_right_port:=can1     auto_enable:="${PIPER_AUTO_ENABLE}"     use_rviz:=false

  for _ in {1..120}; do
    if [[ -n "$(matching_pids "${HARDWARE_PATTERN}")" ]] &&
      pgrep -u "$(id -u)" -f "piper_left_ctrl_node" >/dev/null &&
      pgrep -u "$(id -u)" -f "piper_right_ctrl_node" >/dev/null &&
      pgrep -u "$(id -u)" -f "moveit_ros_move_group/move_group" >/dev/null &&
      ros_service_exists "/get_planning_scene" &&
      ros_service_exists "/plan_to_pose" &&
      ros_service_exists "/plan_to_joints"
    then
      echo "hardware/MoveIt/规划服务已启动"
      return 0
    fi
    sleep 0.5
  done
  echo "hardware 启动超时，请检查 ${HARDWARE_LOG}" >&2
  return 1
}

stop_hardware()
{
  cancel_active_tasks
  "${PLANNING_WS}/restart_path_planning_server.sh" stop || true
  stop_handeye_tf || true
  stop_processes "hardware" "${HARDWARE_PATTERN}" "${HARDWARE_SESSION}"
}

planning_action()
{
  local action="$1"
  local args=("${action}")
  if ${BUILD_FIRST} && [[ "${COMPONENT}" == "planning" ]]; then
    args+=("--build")
  fi
  "${PLANNING_WS}/restart_path_planning_server.sh" "${args[@]}"
}

simple_action()
{
  local component="$1"
  local action="$2"
  case "${component}:${action}" in
    state-machine:start) start_state_machine ;;
    state-machine:stop) stop_state_machine ;;
    state-machine:restart) stop_state_machine; start_state_machine ;;
    graspnet-bridge:start) start_graspnet_bridge ;;
    graspnet-bridge:stop) stop_graspnet_bridge ;;
    graspnet-bridge:restart) stop_graspnet_bridge; start_graspnet_bridge ;;
    cameras:start) start_cameras ;;
    cameras:stop) stop_cameras ;;
    cameras:restart) stop_cameras; start_cameras ;;
    image-bridge:start) start_image_bridge ;;
    image-bridge:stop) stop_image_bridge ;;
    image-bridge:restart) stop_image_bridge; start_image_bridge ;;
    web:start) start_web ;;
    web:stop) stop_web ;;
    web:restart) stop_web; start_web ;;
    rviz:start) start_rviz ;;
    rviz:stop) stop_rviz ;;
    rviz:restart) stop_rviz; start_rviz ;;
    handeye-tf:start) start_handeye_tf ;;
    handeye-tf:stop) stop_handeye_tf ;;
    handeye-tf:restart) stop_handeye_tf; start_handeye_tf ;;
    hardware:start) start_hardware ;;
    hardware:stop) stop_hardware ;;
    hardware:restart) stop_hardware; start_hardware ;;
  esac
}

component_pattern()
{
  case "$1" in
    state-machine) echo "${STATE_PATTERN}" ;;
    graspnet-bridge) echo "${BRIDGE_PATTERN}" ;;
    cameras) echo "${CAMERA_PATTERN}" ;;
    image-bridge) echo "${IMAGE_PATTERN}" ;;
    rviz) echo "${RVIZ_PATTERN}" ;;
    handeye-tf) echo "${HANDEYE_PATTERN}" ;;
    hardware) echo "${HARDWARE_PATTERN}" ;;
  esac
}

component_log()
{
  case "$1" in
    planning) echo "/tmp/path_planning_server0807.log" ;;
    state-machine) echo "${STATE_LOG}" ;;
    graspnet-bridge) echo "${BRIDGE_LOG}" ;;
    cameras) echo "${CAMERA_LOG}" ;;
    image-bridge) echo "${IMAGE_LOG}" ;;
    web) echo "${WEB_DIR}/web.log" ;;
    rviz) echo "${RVIZ_LOG}" ;;
    handeye-tf) echo "${HANDEYE_LOG}" ;;
    hardware) echo "${HARDWARE_LOG}" ;;
  esac
}

show_status()
{
  local component="$1"
  if [[ "${component}" == "planning" ]]; then
    "${PLANNING_WS}/restart_path_planning_server.sh" status
    return
  fi
  if [[ "${component}" == "web" ]]; then
    if [[ -f "${WEB_DIR}/web.pid" ]] &&
      kill -0 "$(tr -d '[:space:]' < "${WEB_DIR}/web.pid")" 2>/dev/null
    then
      echo "web: 运行中，PID $(tr -d '[:space:]' < "${WEB_DIR}/web.pid")"
    else
      echo "web: 未运行"
    fi
    return
  fi

  local pattern
  local pids
  pattern="$(component_pattern "${component}")"
  pids="$(matching_pids "${pattern}")"
  if [[ -n "${pids}" ]]; then
    echo "${component}: 运行中，PID ${pids//$'\n'/ }"
  else
    echo "${component}: 未运行"
  fi
}

show_all_status()
{
  for component in planning state-machine graspnet-bridge cameras image-bridge web rviz handeye-tf hardware; do
    show_status "${component}"
  done
}

software_restart()
{
  cancel_active_tasks
  stop_web
  stop_state_machine
  stop_graspnet_bridge
  stop_image_bridge
  planning_action stop

  planning_action start
  start_image_bridge
  start_graspnet_bridge
  start_state_machine
  start_web
}

if [[ "${ACTION}" != "status" && "${ACTION}" != "logs" ]]; then
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    echo "另一个服务启停操作正在进行" >&2
    exit 1
  fi
fi

if ${BUILD_FIRST}; then
  case "${COMPONENT}" in
    planning) ;;
    state-machine|rviz|handeye-tf) build_planning ;;
    graspnet-bridge) build_graspnet ;;
    software) build_planning; build_graspnet ;;
    hardware) build_hardware_workspace ;;
    cameras|image-bridge|web) echo "${COMPONENT} 不需要 colcon 编译" ;;
  esac
fi

if [[ "${ACTION}" == "status" ]]; then
  if [[ "${COMPONENT}" == "all" ]]; then
    show_all_status
  elif [[ "${COMPONENT}" == "software" ]]; then
    for component in planning state-machine graspnet-bridge image-bridge web; do
      show_status "${component}"
    done
  else
    show_status "${COMPONENT}"
  fi
  exit 0
fi

if [[ "${ACTION}" == "logs" ]]; then
  if [[ "${COMPONENT}" == "all" || "${COMPONENT}" == "software" ]]; then
    echo "logs 需要指定单个组件" >&2
    exit 2
  fi
  log_file="$(component_log "${COMPONENT}")"
  touch "${log_file}"
  tail -n 100 -F "${log_file}"
  exit 0
fi

if [[ "${COMPONENT}" == "planning" ]]; then
  planning_action "${ACTION}"
elif [[ "${COMPONENT}" == "software" ]]; then
  if [[ "${ACTION}" != "restart" ]]; then
    echo "software 仅支持 restart/status" >&2
    exit 2
  fi
  software_restart
else
  simple_action "${COMPONENT}" "${ACTION}"
fi

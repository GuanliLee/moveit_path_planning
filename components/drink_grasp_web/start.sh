#!/usr/bin/env bash
set -euo pipefail

WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${DRINK_GRASP_WEB_PID_FILE:-${WEB_DIR}/web.pid}"
LOG_FILE="${DRINK_GRASP_WEB_LOG_FILE:-${WEB_DIR}/web.log}"

set +u
source /opt/ros/jazzy/setup.bash
source /home/ligl/path_planning_service0807/install/setup.bash
source /home/ligl/graspnet_path_planning_bridge0804/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export DRINK_GRASP_HOST="${DRINK_GRASP_HOST:-0.0.0.0}"
export DRINK_GRASP_PORT="${DRINK_GRASP_PORT:-8090}"
export DRINK_GRASP_SERVICE="${DRINK_GRASP_SERVICE:-/execute_named_grasp_task}"
export DRINK_GRASP_PICK_SERVICE="${DRINK_GRASP_PICK_SERVICE:-/execute_named_pick_task}"
export DRINK_GRASP_CANCEL_SERVICE="${DRINK_GRASP_CANCEL_SERVICE:-/cancel_named_grasp_task}"
export DRINK_GRASP_SCENE_ID="${DRINK_GRASP_SCENE_ID:-1}"
export DRINK_GRASP_GRIPPER_WIDTH_M="${DRINK_GRASP_GRIPPER_WIDTH_M:-0.08}"
export DRINK_GRASP_CALL_TIMEOUT_SEC="${DRINK_GRASP_CALL_TIMEOUT_SEC:-240}"
export DRINK_GRASP_RECORDING_ENABLED="${DRINK_GRASP_RECORDING_ENABLED:-1}"
export DRINK_GRASP_RECORD_DIR="${DRINK_GRASP_RECORD_DIR:-${WEB_DIR}/recordings}"
export DRINK_GRASP_RECORD_STORAGE="${DRINK_GRASP_RECORD_STORAGE:-mcap}"
export DRINK_GRASP_RECORD_STORAGE_PRESET="${DRINK_GRASP_RECORD_STORAGE_PRESET:-zstd_fast}"
export DRINK_GRASP_RECORD_TOPICS="${DRINK_GRASP_RECORD_TOPICS:-/cam_high/color/image_raw /cam_high/aligned_depth_to_color/image_raw /cam_high/color/camera_info /cam_left/color/image_raw /cam_left/aligned_depth_to_color/image_raw /cam_left/color/camera_info /cam_right/color/image_raw /cam_right/aligned_depth_to_color/image_raw /cam_right/color/camera_info /joint_states /joint_states_left /joint_states_right /joint_left /joint_right /arm_status_left /arm_status_right /end_pose_left /end_pose_right /end_pose_stamped_left /end_pose_stamped_right /pos_cmd_left /pos_cmd_right /joint_ctrl_cmd_left /joint_ctrl_cmd_right /joint_states_ctrl_left /joint_states_ctrl_right /left/enable_flag /right/enable_flag /tf /tf_static}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(tr -d '[:space:]' < "${PID_FILE}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "Drink grasp web already running, pid=${old_pid}"
    echo "URL: http://127.0.0.1:${DRINK_GRASP_PORT}/"
    exit 0
  fi
fi

cd "${WEB_DIR}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
if command -v setsid >/dev/null 2>&1; then
  nohup setsid /usr/bin/python3 "${WEB_DIR}/server.py" > "${LOG_FILE}" 2>&1 < /dev/null &
else
  nohup /usr/bin/python3 "${WEB_DIR}/server.py" > "${LOG_FILE}" 2>&1 < /dev/null &
fi
pid="$!"
echo "${pid}" > "${PID_FILE}"

for _ in $(seq 1 30); do
  if curl -fsS --max-time 1 "http://127.0.0.1:${DRINK_GRASP_PORT}/api/status" >/dev/null 2>&1; then
    echo "Started drink grasp web, pid=${pid}"
    echo "URL: http://127.0.0.1:${DRINK_GRASP_PORT}/"
    echo "Log: ${LOG_FILE}"
    exit 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "Drink grasp web failed to start. Log: ${LOG_FILE}" >&2
    exit 1
  fi
  sleep 1
done

echo "Drink grasp web start timed out. Log: ${LOG_FILE}" >&2
exit 1

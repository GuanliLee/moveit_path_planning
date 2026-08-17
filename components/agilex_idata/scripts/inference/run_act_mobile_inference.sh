#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
AGILEX_SETUP="${AGILEX_SETUP:-/home/agilex/agilex_ws/install/setup.bash}"
PIPER_SETUP="${PIPER_SETUP:-/home/agilex/piper_ros/install/setup.bash}"
CAMERA_SETUP="${CAMERA_SETUP:-/home/agilex/camera_ros/install/setup.bash}"
CONDA_SETUP="${CONDA_SETUP:-/home/agilex/miniforge3/etc/profile.d/conda.sh}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-aloha}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${SCRIPT:-${SCRIPT_DIR}/run_act_mobile_inference.py}"

if [ ! -f "${ROS_SETUP}" ]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
source "${ROS_SETUP}"

if [ -f "${AGILEX_SETUP}" ]; then
  source "${AGILEX_SETUP}"
fi

if [ -f "${PIPER_SETUP}" ]; then
  source "${PIPER_SETUP}"
fi

if [ -f "${CAMERA_SETUP}" ]; then
  source "${CAMERA_SETUP}"
fi

if [ ! -f "${CONDA_SETUP}" ]; then
  echo "ERROR: conda setup not found: ${CONDA_SETUP}" >&2
  exit 1
fi
source "${CONDA_SETUP}"
conda activate "${CONDA_ENV_NAME}"

set -u
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

exec python "${SCRIPT}" "$@"

#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
AGILEX_SETUP="${AGILEX_SETUP:-/home/agilex/agilex_ws/install/setup.bash}"
PIPER_SETUP="${PIPER_SETUP:-/home/agilex/piper_ros/install/setup.bash}"
CAMERA_SETUP="${CAMERA_SETUP:-/home/agilex/camera_ros/install/setup.bash}"
CONDA_SETUP="${CONDA_SETUP:-/home/agilex/miniforge3/etc/profile.d/conda.sh}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-aloha}"
OPENPI_CLIENT_SRC="${OPENPI_CLIENT_SRC:-/home/agilex/openpi/packages/openpi-client/src}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${SCRIPT:-${SCRIPT_DIR}/run_openpi_agilex_mobile_inference.py}"

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

if [ ! -d "${OPENPI_CLIENT_SRC}" ]; then
  echo "ERROR: OpenPI client source not found: ${OPENPI_CLIENT_SRC}" >&2
  exit 1
fi
export PYTHONPATH="${OPENPI_CLIENT_SRC}:${PYTHONPATH:-}"

export OPENPI_POLICY_HOST="${OPENPI_POLICY_HOST:-192.168.1.154}"
export OPENPI_POLICY_PORT="${OPENPI_POLICY_PORT:-8899}"
export OPENPI_PROMPT="${OPENPI_PROMPT:-Target: Grape Juice. Pick the Grape Juice from the shelf and place it into the cart.}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export NO_PROXY="${OPENPI_POLICY_HOST},127.0.0.1,localhost,${NO_PROXY:-}"
export no_proxy="${OPENPI_POLICY_HOST},127.0.0.1,localhost,${no_proxy:-}"

if ! python - <<'PY' >/dev/null 2>&1
import msgpack
import websockets
from openpi_client import websocket_client_policy
PY
then
  echo "Installing OpenPI client websocket dependencies into conda env: ${CONDA_ENV_NAME}"
  python -m pip install 'websockets>=11.0' 'msgpack>=1.0.5'
fi

python - <<'PY'
import cv2
import numpy
import rclpy
from cv_bridge import CvBridge
from openpi_client import websocket_client_policy
print("OpenPI AgileX mobile client environment OK")
PY

exec python "${SCRIPT}" "$@"

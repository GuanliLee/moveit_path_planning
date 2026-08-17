#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
if [ -f /home/agilex/piper_ros/install/setup.bash ]; then
  source /home/agilex/piper_ros/install/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/piper_slow_return_zero.py" --yes "$@"

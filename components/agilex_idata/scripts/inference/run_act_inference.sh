#!/usr/bin/env bash
# ACT 真机推理（RGB only，不依赖 depth）
set -e

source /opt/ros/humble/setup.bash
source /home/agilex/miniforge3/etc/profile.d/conda.sh
conda activate aloha

CKPT_DIR="${1:-/home/caizj/checkpoint/ACT}"
CKPT_NAME="${2:-policy_epoch_2000_seed_0.ckpt}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python "${SCRIPT_DIR}/run_act_inference_rgb.py" "$CKPT_DIR" "$CKPT_NAME"

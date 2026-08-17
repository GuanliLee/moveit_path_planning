#!/usr/bin/env bash
set -euo pipefail

OPENPI_ROOT="${OPENPI_ROOT:-/home/agilex/openpi}"
UV_BIN="${UV_BIN:-/home/agilex/miniforge3/bin/uv}"
EXP_DIR="${EXP_DIR:-/home/caizj/checkpoint/openpi/pi05_piper_grasp_bottle_bs8_20260529_194731}"
STEP="${STEP:-${1:-20000}}"
PORT="${PORT:-8000}"
CONFIG="${CONFIG:-pi05_aloha_grasp_bottle}"
CHECKPOINT_DIR="${EXP_DIR}/${STEP}"
PYTORCH_DEVICE="${PYTORCH_DEVICE:-cuda}"
PYTORCH_COMPILE_MODE="${PYTORCH_COMPILE_MODE:-none}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_SCRIPT="${SERVER_SCRIPT:-${SCRIPT_DIR}/serve_openpi_policy_piper.py}"

if [ ! -x "${UV_BIN}" ]; then
  echo "ERROR: uv not found or not executable: ${UV_BIN}" >&2
  exit 1
fi

if [ ! -d "${OPENPI_ROOT}" ]; then
  echo "ERROR: openpi repo not found: ${OPENPI_ROOT}" >&2
  exit 1
fi

if [ ! -f "${CHECKPOINT_DIR}/model.safetensors" ]; then
  echo "ERROR: model.safetensors not found under ${CHECKPOINT_DIR}" >&2
  exit 1
fi

if [ ! -f "${CHECKPOINT_DIR}/metadata.pt" ]; then
  echo "ERROR: metadata.pt not found under ${CHECKPOINT_DIR}" >&2
  exit 1
fi

if [ ! -f "${CHECKPOINT_DIR}/assets/local/grasp_bottle/norm_stats.json" ]; then
  echo "ERROR: norm_stats.json not found under ${CHECKPOINT_DIR}/assets/local/grasp_bottle" >&2
  exit 1
fi

LOCAL_NORM_STATS="${CHECKPOINT_DIR}/assets/local/grasp_bottle/norm_stats.json"
POLICY_NORM_STATS_DIR="${CHECKPOINT_DIR}/assets/trossen"
POLICY_NORM_STATS="${POLICY_NORM_STATS_DIR}/norm_stats.json"

if [ ! -f "${POLICY_NORM_STATS}" ]; then
  echo "Preparing checkpoint norm stats for OpenPI ALOHA asset_id=trossen"
  mkdir -p "${POLICY_NORM_STATS_DIR}"
  cp "${LOCAL_NORM_STATS}" "${POLICY_NORM_STATS}"
fi

cd "${OPENPI_ROOT}"

echo "Starting OpenPI policy server"
echo "  config:     ${CONFIG}"
echo "  checkpoint: ${CHECKPOINT_DIR}"
echo "  port:       ${PORT}"
echo "  device:     ${PYTORCH_DEVICE}"
echo "  compile:    ${PYTORCH_COMPILE_MODE}"

export XLA_PYTHON_CLIENT_MEM_FRACTION="${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.85}"

echo "Checking OpenPI Python environment"
"${UV_BIN}" run python - <<'PY'
import transformers
print(f"  transformers: {transformers.__version__}")
PY

TRANSFORMERS_DIR="${OPENPI_ROOT}/.venv/lib/python3.11/site-packages/transformers"
REPLACE_DIR="${OPENPI_ROOT}/src/openpi/models_pytorch/transformers_replace"

if [ ! -d "${TRANSFORMERS_DIR}" ]; then
  echo "ERROR: transformers package dir not found: ${TRANSFORMERS_DIR}" >&2
  exit 1
fi

echo "Applying OpenPI transformers_replace patch"
cp -r "${REPLACE_DIR}/"* "${TRANSFORMERS_DIR}/"

"${UV_BIN}" run python - <<'PY'
from transformers.models.siglip import check
ok = check.check_whether_transformers_replace_is_installed_correctly()
print(f"  transformers_replace ok: {ok}")
raise SystemExit(0 if ok else 1)
PY

# --port is a top-level tyro argument, so it must appear before policy:checkpoint.
exec "${UV_BIN}" run "${SERVER_SCRIPT}" \
  --port="${PORT}" \
  --pytorch-device="${PYTORCH_DEVICE}" \
  --pytorch-compile-mode="${PYTORCH_COMPILE_MODE}" \
  policy:checkpoint \
  --policy.config="${CONFIG}" \
  --policy.dir="${CHECKPOINT_DIR}"

#!/usr/bin/env bash
set -euo pipefail

WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${DRINK_GRASP_WEB_PID_FILE:-${WEB_DIR}/web.pid}"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "Drink grasp web is not running."
  exit 0
fi

pid="$(tr -d '[:space:]' < "${PID_FILE}")"
if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
  kill -INT "${pid}" || true
  for _ in $(seq 1 20); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      rm -f "${PID_FILE}"
      echo "Stopped drink grasp web."
      exit 0
    fi
    sleep 0.2
  done
  echo "Process still running after SIGINT, pid=${pid}" >&2
  exit 1
fi

rm -f "${PID_FILE}"
echo "Drink grasp web is not running."

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_FILE="${CAMERA_STREAM_STATE_FILE:-/tmp/mekk4_camera_stream.state}"
PID_FILE="${CAMERA_STREAM_PID_FILE:-/tmp/mekk4_camera_stream_supervisor.pid}"

if [[ ! -f "${PID_FILE}" || ! -f "${STATE_FILE}" ]]; then
  echo "[camera-reload] Camera supervisor is not running. Start with make pi-bringup first." >&2
  exit 1
fi

supervisor_pid="$(cat "${PID_FILE}")"
if ! kill -0 "${supervisor_pid}" 2>/dev/null; then
  echo "[camera-reload] Camera supervisor PID ${supervisor_pid} is not running." >&2
  rm -f "${PID_FILE}" "${STATE_FILE}"
  exit 1
fi

eval "$(python3 "${SCRIPT_DIR}/camera_config_env.py")"
source "${STATE_FILE}"

new_width="${WIDTH}"
new_height="${HEIGHT}"
new_cam_port="${CAM_PORT}"
old_width="${STREAM_WIDTH}"
old_height="${STREAM_HEIGHT}"
old_cam_port="${STREAM_CAM_PORT}"

if [[ "${new_width}" != "${old_width}" || "${new_height}" != "${old_height}" || "${new_cam_port}" != "${old_cam_port}" ]]; then
  echo "[camera-reload] width/height/port changed. Do a full restart of Pi bringup." >&2
  exit 1
fi

kill -HUP "${supervisor_pid}"
echo "[camera-reload] Reload requested. Stream is restarting with current camera_params.yaml." >&2

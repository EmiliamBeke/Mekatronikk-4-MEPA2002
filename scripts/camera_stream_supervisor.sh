#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_FILE="${CAMERA_STREAM_STATE_FILE:-/tmp/mekk4_camera_stream.state}"
PID_FILE="${CAMERA_STREAM_PID_FILE:-/tmp/mekk4_camera_stream_supervisor.pid}"

child_pid=""
reload_requested=0
stop_requested=0

write_state() {
  cat > "${STATE_FILE}" <<EOF
STREAM_LOCAL_PORT=${LOCAL_PORT:-5600}
STREAM_LOCAL_HOST=${LOCAL_HOST:-127.0.0.1}
STREAM_WIDTH=${WIDTH:-1296}
STREAM_HEIGHT=${HEIGHT:-972}
STREAM_FPS=${FPS:-15}
STREAM_CAM_PORT=${CAM_PORT:-${LOCAL_PORT:-5600}}
EOF
}

start_child() {
  bash "${SCRIPT_DIR}/camera_udp_stream.sh" &
  child_pid=$!
}

stop_child() {
  if [[ -n "${child_pid}" ]]; then
    kill "${child_pid}" 2>/dev/null || true
    wait "${child_pid}" 2>/dev/null || true
    child_pid=""
  fi
}

cleanup() {
  stop_child
  rm -f "${PID_FILE}" "${STATE_FILE}"
}

request_reload() {
  reload_requested=1
  stop_child
}

request_stop() {
  stop_requested=1
  stop_child
}

trap cleanup EXIT
trap request_reload HUP
trap request_stop INT TERM

printf '%s\n' "$$" > "${PID_FILE}"
write_state

while true; do
  start_child
  if wait "${child_pid}"; then
    status=0
  else
    status=$?
  fi
  child_pid=""

  if [[ "${stop_requested}" == "1" ]]; then
    exit 0
  fi

  if [[ "${reload_requested}" == "1" ]]; then
    reload_requested=0
    write_state
    continue
  fi

  exit "${status}"
done

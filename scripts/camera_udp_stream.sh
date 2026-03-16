#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(python3 "${SCRIPT_DIR}/camera_config_env.py")"

WIDTH=${WIDTH:-1296}
HEIGHT=${HEIGHT:-972}
FPS=${FPS:-15}
BITRATE=${BITRATE:-}
INTRA=${INTRA:-}
LOW_LATENCY=${LOW_LATENCY:-0}
FLUSH_OUTPUT=${FLUSH_OUTPUT:-1}
LOCAL_PORT=${LOCAL_PORT:-5600}
LOCAL_HOST=${LOCAL_HOST:-127.0.0.1}
ENABLE_LOCAL=${ENABLE_LOCAL:-1}
REMOTE_HOST=${REMOTE_HOST:-}
REMOTE_PORT=${REMOTE_PORT:-5601}

if ! command -v rpicam-vid >/dev/null 2>&1; then
  echo "[camera-stream] rpicam-vid not found. Install rpicam-apps on the Pi host." >&2
  exit 1
fi

if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "[camera-stream] gst-launch-1.0 not found. Install gstreamer1.0-tools on the Pi host." >&2
  exit 1
fi

supports_low_latency=0
if rpicam-vid --help 2>&1 | grep -q -- '--low-latency'; then
  supports_low_latency=1
fi

supports_flush=0
if rpicam-vid --help 2>&1 | grep -q -- '--flush'; then
  supports_flush=1
fi

if [[ "${ENABLE_LOCAL}" != "1" && -z "${REMOTE_HOST}" ]]; then
  echo "[camera-stream] Nothing to do. Enable local streaming or set REMOTE_HOST." >&2
  exit 1
fi

rpicam_pid=""
gst_pid=""
fifo_path="$(mktemp -u /tmp/mekk4_camera_fifo.XXXXXX)"

cleanup() {
  if [[ -n "${rpicam_pid}" ]]; then
    kill "${rpicam_pid}" 2>/dev/null || true
    wait "${rpicam_pid}" 2>/dev/null || true
  fi
  if [[ -n "${gst_pid}" ]]; then
    kill "${gst_pid}" 2>/dev/null || true
    wait "${gst_pid}" 2>/dev/null || true
  fi
  rm -f "${fifo_path}"
}
trap cleanup EXIT INT TERM

mkfifo "${fifo_path}"

gst_cmd=(gst-launch-1.0 -q fdsrc '!' h264parse '!' tee name=t)

if [[ "${ENABLE_LOCAL}" == "1" ]]; then
  gst_cmd+=(
    t. '!' queue '!' rtph264pay pt=96 config-interval=1 '!'
    udpsink "host=${LOCAL_HOST}" "port=${LOCAL_PORT}" sync=false async=false
  )
fi

if [[ -n "${REMOTE_HOST}" ]]; then
  gst_cmd+=(
    t. '!' queue '!' rtph264pay pt=96 config-interval=1 '!'
    udpsink "host=${REMOTE_HOST}" "port=${REMOTE_PORT}" sync=false async=false
  )
fi

echo "[camera-stream] width=${WIDTH} height=${HEIGHT} fps=${FPS}" >&2
if [[ "${ENABLE_LOCAL}" == "1" ]]; then
  echo "[camera-stream] local udp -> ${LOCAL_HOST}:${LOCAL_PORT}" >&2
fi
if [[ -n "${REMOTE_HOST}" ]]; then
  echo "[camera-stream] remote udp -> ${REMOTE_HOST}:${REMOTE_PORT}" >&2
fi

RPICAM_ARGS=(
  -t 0
  --width "${WIDTH}"
  --height "${HEIGHT}"
  --framerate "${FPS}"
  --codec h264
  --inline
  --libav-format h264
  -n
  --awb "${AWB}"
  --brightness "${BRIGHTNESS}"
  --contrast "${CONTRAST}"
  --saturation "${SATURATION}"
  --sharpness "${SHARPNESS}"
  --ev "${EV}"
  --denoise "${DENOISE}"
  --metering "${METERING}"
)

if [[ -n "${BITRATE}" ]]; then
  RPICAM_ARGS+=(--bitrate "${BITRATE}")
fi

if [[ -n "${INTRA}" ]]; then
  RPICAM_ARGS+=(--intra "${INTRA}")
fi

if [[ "${LOW_LATENCY}" == "1" ]]; then
  if [[ "${supports_low_latency}" == "1" ]]; then
    RPICAM_ARGS+=(--low-latency)
  else
    echo "[camera-stream] LOW_LATENCY requested, but this rpicam-vid build does not support --low-latency." >&2
  fi
fi

if [[ "${FLUSH_OUTPUT}" == "1" ]]; then
  if [[ "${supports_flush}" == "1" ]]; then
    RPICAM_ARGS+=(--flush)
  else
    echo "[camera-stream] FLUSH_OUTPUT requested, but this rpicam-vid build does not support --flush." >&2
  fi
fi

if [[ -n "${AWB_GAINS}" ]]; then
  RPICAM_ARGS+=(--awbgains "${AWB_GAINS}")
fi

if [[ -n "${TUNING_FILE}" ]]; then
  RPICAM_ARGS+=(--tuning-file "${TUNING_FILE}")
fi

"${gst_cmd[@]}" < "${fifo_path}" &
gst_pid=$!

rpicam-vid "${RPICAM_ARGS[@]}" -o "${fifo_path}" &
rpicam_pid=$!

wait -n "${rpicam_pid}" "${gst_pid}"
status=$?

if kill -0 "${rpicam_pid}" 2>/dev/null; then
  kill "${rpicam_pid}" 2>/dev/null || true
fi
if kill -0 "${gst_pid}" 2>/dev/null; then
  kill "${gst_pid}" 2>/dev/null || true
fi
wait "${rpicam_pid}" 2>/dev/null || true
wait "${gst_pid}" 2>/dev/null || true

exit "${status}"

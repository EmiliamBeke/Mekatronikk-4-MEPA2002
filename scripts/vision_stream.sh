#!/usr/bin/env bash
set -euo pipefail

WIDTH=${WIDTH:-1296}
HEIGHT=${HEIGHT:-972}
FPS=${FPS:-15}
PORT=${PORT:-5600}
HOST=${HOST:-127.0.0.1}

if ! command -v rpicam-vid >/dev/null 2>&1; then
  echo "rpicam-vid not found. Install rpicam-apps on host." >&2
  exit 1
fi

if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "gst-launch-1.0 not found. Install gstreamer1.0-tools on host." >&2
  exit 1
fi

cleanup() {
  if [[ -n "${CAM_PID:-}" ]]; then
    kill "${CAM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

rpicam-vid -t 0 --width "${WIDTH}" --height "${HEIGHT}" --framerate "${FPS}" \
  --codec h264 --inline --libav-format h264 -n -o - \
  | gst-launch-1.0 -q fdsrc \
    ! h264parse \
    ! rtph264pay pt=96 config-interval=1 \
    ! udpsink host="${HOST}" port="${PORT}" &
CAM_PID=$!

PIPELINE="udpsrc port=${PORT} caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1 sync=false"

X11_ARGS=()
if [[ -n "${DISPLAY:-}" ]]; then
  X11_ARGS+=("-e" "DISPLAY=${DISPLAY}")
  if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
    X11_ARGS+=("-e" "XAUTHORITY=/tmp/.Xauthority" "-v" "${XAUTHORITY}:/tmp/.Xauthority:ro")
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    X11_ARGS+=("-e" "XAUTHORITY=/tmp/.Xauthority" "-v" "${HOME}/.Xauthority:/tmp/.Xauthority:ro")
  fi
  if [[ -d /tmp/.X11-unix ]]; then
    X11_ARGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix:rw")
  fi
fi

docker compose run --rm \
  "${X11_ARGS[@]}" \
  -e MEKK4_CAM_WIDTH="${WIDTH}" \
  -e MEKK4_CAM_HEIGHT="${HEIGHT}" \
  -e MEKK4_CAM_SOURCE_GST="${PIPELINE}" \
  ros bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch mekk4_bringup vision_stream.launch.py"

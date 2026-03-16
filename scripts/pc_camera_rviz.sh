#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

eval "$(python3 "${SCRIPT_DIR}/camera_config_env.py")"

PI_HOST="${1:-${PI_HOST:-gruppe5pi5}}"
PORT="${PORT:-5601}"
WIDTH="${WIDTH:-1296}"
HEIGHT="${HEIGHT:-972}"
PC_JITTER_MS="${PC_JITTER_MS:-40}"
TOPIC_NAME="${TOPIC_NAME:-/camera}"
FRAME_ID="${FRAME_ID:-camera_link}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${REPO_ROOT}/src/robot_bringup/rviz/pre_odom_lidar.rviz}"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "[pc-camera-rviz] Missing install/setup.bash. Build the local workspace first." >&2
  echo "[pc-camera-rviz] Example: source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" >&2
  exit 1
fi

if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "[pc-camera-rviz] gst-launch-1.0 not found. Install gstreamer1.0-tools on the PC." >&2
  exit 1
fi

if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
  echo "[pc-camera-rviz] gst-inspect-1.0 not found. Install gstreamer1.0-tools on the PC." >&2
  exit 1
fi

for plugin in h264parse rtph264depay decodebin videoconvert; do
  if ! gst-inspect-1.0 "${plugin}" >/dev/null 2>&1; then
    echo "[pc-camera-rviz] Missing GStreamer plugin: ${plugin}" >&2
    echo "[pc-camera-rviz] Install at least: gstreamer1.0-plugins-bad gstreamer1.0-libav" >&2
    exit 1
  fi
done

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

eval "$(bash "${SCRIPT_DIR}/ros_discovery_env.sh" pc "${PI_HOST}")"

cleanup() {
  if [[ -n "${camera_pid:-}" ]]; then
    kill "${camera_pid}" 2>/dev/null || true
    wait "${camera_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 launch mekk4_bringup pc_camera_view.launch.py \
  port:="${PORT}" \
  width:="${WIDTH}" \
  height:="${HEIGHT}" \
  jitter_ms:="${PC_JITTER_MS}" \
  topic_name:="${TOPIC_NAME}" \
  frame_id:="${FRAME_ID}" \
  >/tmp/mekk4_pc_camera.log 2>&1 &
camera_pid=$!

sleep 1
if ! kill -0 "${camera_pid}" 2>/dev/null; then
  echo "[pc-camera-rviz] Camera bridge failed to start. Last log lines:" >&2
  tail -n 60 /tmp/mekk4_pc_camera.log >&2 || true
  exit 1
fi

rviz2 -d "${RVIZ_CONFIG}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

WITH_NAV2="${WITH_NAV2:-1}"
WITH_TEDDY="${WITH_TEDDY:-0}"
PC_HOST="${PC_HOST:-}"
PORT_NAME="${PORT_NAME:-/dev/ttyAMA0}"
PORT_BAUDRATE="${PORT_BAUDRATE:-230400}"
PRODUCT_NAME="${PRODUCT_NAME:-LDLiDAR_LD06}"
LIDAR_FRAME="${LIDAR_FRAME:-base_laser}"
BASE_FRAME="${BASE_FRAME:-chassis}"
MAP_FILE="${MAP_FILE:-/ws/maps/my_map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-/ws/config/nav2_params.yaml}"
WIDTH="${WIDTH:-1296}"
HEIGHT="${HEIGHT:-972}"
FPS="${FPS:-15}"
CAM_PORT="${CAM_PORT:-5600}"
SOURCE_LAUNCH="${REPO_ROOT}/src/robot_bringup/launch/pi_robot.launch.py"
INSTALLED_LAUNCH="${REPO_ROOT}/install/robot_bringup/share/robot_bringup/launch/pi_robot.launch.py"
SOURCE_PKG_XML="${REPO_ROOT}/src/robot_bringup/package.xml"
SOURCE_CMAKE="${REPO_ROOT}/src/robot_bringup/CMakeLists.txt"

needs_ws_build=0
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  needs_ws_build=1
elif [[ ! -f "${INSTALLED_LAUNCH}" ]]; then
  needs_ws_build=1
fi

if [[ "${needs_ws_build}" == "1" ]]; then
  echo "[pi-bringup] Workspace install is missing or stale. Building with make ws..." >&2
  docker compose run --rm ros bash -lc '/ws/scripts/ws_build.sh'
fi

eval "$(bash "${SCRIPT_DIR}/ros_discovery_env.sh" pi "${PC_HOST}")"

camera_pid=""

cleanup() {
  if [[ -n "${camera_pid}" ]]; then
    kill "${camera_pid}" 2>/dev/null || true
    wait "${camera_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "${WITH_TEDDY}" == "1" ]]; then
  if ! command -v rpicam-vid >/dev/null 2>&1; then
    echo "[pi-bringup] WITH_TEDDY=1 requires rpicam-vid on the Pi host." >&2
    exit 1
  fi
  if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
    echo "[pi-bringup] WITH_TEDDY=1 requires gst-launch-1.0 on the Pi host." >&2
    exit 1
  fi

  echo "[pi-bringup] Starting local camera UDP stream for teddy_detector on port ${CAM_PORT}..." >&2
  rpicam-vid -t 0 --width "${WIDTH}" --height "${HEIGHT}" --framerate "${FPS}" \
    --codec h264 --inline --libav-format h264 -n -o - \
    | gst-launch-1.0 -q fdsrc \
      ! h264parse \
      ! rtph264pay pt=96 config-interval=1 \
      ! udpsink host=127.0.0.1 port="${CAM_PORT}" &
  camera_pid=$!
fi

echo "[pi-bringup] Launching robot stack in Docker..." >&2
docker compose run --rm \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE}" \
  -e ROS_STATIC_PEERS="${ROS_STATIC_PEERS}" \
  -e MEKK4_CAM_SOURCE_GST="udpsrc port=${CAM_PORT} caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1 sync=false" \
  -e MEKK4_CAM_WIDTH="${WIDTH}" \
  -e MEKK4_CAM_HEIGHT="${HEIGHT}" \
  ros bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch robot_bringup pi_robot.launch.py use_nav2:=${WITH_NAV2} use_teddy:=${WITH_TEDDY} product_name:=${PRODUCT_NAME} port_name:=${PORT_NAME} port_baudrate:=${PORT_BAUDRATE} frame_id:=${LIDAR_FRAME} base_frame:=${BASE_FRAME} map:=${MAP_FILE} params_file:=${PARAMS_FILE}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

eval "$(python3 "${SCRIPT_DIR}/camera_config_env.py")"

WITH_NAV2="${WITH_NAV2:-1}"
WITH_TEDDY="${WITH_TEDDY:-0}"
WITH_IMU="${WITH_IMU:-0}"
WITH_MEGA_DRIVER="${WITH_MEGA_DRIVER:-0}"
PC_HOST="${PC_HOST:-}"
PORT_NAME="${PORT_NAME:-/dev/ttyAMA0}"
PORT_BAUDRATE="${PORT_BAUDRATE:-230400}"
PRODUCT_NAME="${PRODUCT_NAME:-LDLiDAR_LD06}"
LIDAR_FRAME="${LIDAR_FRAME:-base_laser}"
BASE_FRAME="${BASE_FRAME:-chassis}"
IMU_FRAME="${IMU_FRAME:-imu_link}"
MEGA_PORT="${MEGA_PORT:-/dev/ttyACM0}"
MEGA_BAUDRATE="${MEGA_BAUDRATE:-115200}"
LEFT_CMD_SIGN="${LEFT_CMD_SIGN:-1}"
RIGHT_CMD_SIGN="${RIGHT_CMD_SIGN:-1}"
LEFT_TICK_SIGN="${LEFT_TICK_SIGN:-1}"
RIGHT_TICK_SIGN="${RIGHT_TICK_SIGN:-1}"
LEFT_M_PER_TICK="${LEFT_M_PER_TICK:-0.0}"
RIGHT_M_PER_TICK="${RIGHT_M_PER_TICK:-0.0}"
TRACK_WIDTH_EFF_M="${TRACK_WIDTH_EFF_M:-0.35}"
MAP_FILE="${MAP_FILE:-/ws/maps/my_map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-/ws/config/nav2_params.yaml}"
WIDTH="${WIDTH:-1296}"
HEIGHT="${HEIGHT:-972}"
FPS="${FPS:-15}"
CAM_PORT="${CAM_PORT:-5600}"
WITH_CAMERA_RVIZ="${WITH_CAMERA_RVIZ:-0}"
CAMERA_REMOTE_HOST="${CAMERA_REMOTE_HOST:-}"
CAMERA_REMOTE_PORT="${CAMERA_REMOTE_PORT:-5601}"
DOCKER_LIDAR_GID="${DOCKER_LIDAR_GID:-}"
DOCKER_I2C_GID="${DOCKER_I2C_GID:-}"
DOCKER_GPIO_GID="${DOCKER_GPIO_GID:-}"
COMPOSE_MEGA_DEVICE="${COMPOSE_MEGA_DEVICE:-/dev/null}"
SOURCE_LAUNCH="${REPO_ROOT}/src/robot_bringup/launch/pi_robot.launch.py"
INSTALLED_LAUNCH="${REPO_ROOT}/install/robot_bringup/share/robot_bringup/launch/pi_robot.launch.py"
SOURCE_PKG_XML="${REPO_ROOT}/src/robot_bringup/package.xml"
SOURCE_CMAKE="${REPO_ROOT}/src/robot_bringup/CMakeLists.txt"

if [[ -z "${DOCKER_LIDAR_GID}" && -e "${PORT_NAME}" ]]; then
  DOCKER_LIDAR_GID="$(stat -c '%g' "${PORT_NAME}")"
fi
if [[ -z "${DOCKER_I2C_GID}" && -e /dev/i2c-1 ]]; then
  DOCKER_I2C_GID="$(stat -c '%g' /dev/i2c-1)"
fi
if [[ -z "${DOCKER_GPIO_GID}" && -e /dev/gpiochip0 ]]; then
  DOCKER_GPIO_GID="$(stat -c '%g' /dev/gpiochip0)"
fi
export DOCKER_LIDAR_GID
export DOCKER_I2C_GID
export DOCKER_GPIO_GID

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
trap cleanup EXIT INT TERM

if [[ "${WITH_CAMERA_RVIZ}" == "1" && -z "${CAMERA_REMOTE_HOST}" ]]; then
  CAMERA_REMOTE_HOST="${ROS_STATIC_PEERS}"
fi

if [[ "${WITH_TEDDY}" == "1" && "${MEKK4_DEBUG_STREAM:-0}" == "1" && -z "${CAMERA_REMOTE_HOST}" ]]; then
  CAMERA_REMOTE_HOST="${ROS_STATIC_PEERS}"
fi

if [[ "${WITH_TEDDY}" == "1" || "${WITH_CAMERA_RVIZ}" == "1" ]]; then
  if ! command -v rpicam-vid >/dev/null 2>&1; then
    echo "[pi-bringup] Camera streaming requires rpicam-vid on the Pi host." >&2
    exit 1
  fi
  if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
    echo "[pi-bringup] Camera streaming requires gst-launch-1.0 on the Pi host." >&2
    exit 1
  fi

  bash "${SCRIPT_DIR}/camera_stop.sh" >/dev/null 2>&1 || true
  echo "[pi-bringup] Starting camera UDP stream..." >&2
  ENABLE_LOCAL=0
  ENABLE_REMOTE=0
  if [[ "${WITH_TEDDY}" == "1" ]]; then
    ENABLE_LOCAL=1
  fi
  if [[ "${WITH_CAMERA_RVIZ}" == "1" ]]; then
    ENABLE_REMOTE=1
  fi
  WIDTH="${WIDTH}" HEIGHT="${HEIGHT}" FPS="${FPS}" \
    CAM_PORT="${CAM_PORT}" CAMERA_REMOTE_PORT="${CAMERA_REMOTE_PORT}" \
    LOCAL_PORT="${CAM_PORT}" ENABLE_LOCAL="${ENABLE_LOCAL}" ENABLE_REMOTE="${ENABLE_REMOTE}" \
    REMOTE_HOST="${CAMERA_REMOTE_HOST}" REMOTE_PORT="${CAMERA_REMOTE_PORT}" \
    bash "${SCRIPT_DIR}/camera_stream_supervisor.sh" &
  camera_pid=$!
  sleep 1
  if ! kill -0 "${camera_pid}" 2>/dev/null; then
    echo "[pi-bringup] Camera UDP stream exited early. Check camera logs above." >&2
    exit 1
  fi
  echo "[pi-bringup] Camera color/exposure settings can be reloaded with: make camera-reload" >&2
fi

echo "[pi-bringup] Launching robot stack in Docker..." >&2
docker_run_args=(
  compose run --rm
)

if [[ "${WITH_MEGA_DRIVER}" == "1" ]]; then
  if [[ ! -e "${MEGA_PORT}" ]]; then
    echo "[pi-bringup] Mega serial device not found: ${MEGA_PORT}" >&2
    exit 1
  fi
  COMPOSE_MEGA_DEVICE="${MEGA_PORT}"
fi
export COMPOSE_MEGA_DEVICE

docker "${docker_run_args[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE}" \
  -e ROS_STATIC_PEERS="${ROS_STATIC_PEERS}" \
  -e MEKK4_CAM_SOURCE_GST="udpsrc port=${CAM_PORT} caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1 sync=false" \
  -e MEKK4_CAM_WIDTH="${WIDTH}" \
  -e MEKK4_CAM_HEIGHT="${HEIGHT}" \
  -e MEKK4_CAM_FPS="${FPS}" \
  -e MEKK4_NCNN_MODEL="${MEKK4_NCNN_MODEL}" \
  -e MEKK4_CONF="${MEKK4_CONF}" \
  -e MEKK4_IMGSZ="${MEKK4_IMGSZ}" \
  -e MEKK4_CENTER_TOL="${MEKK4_CENTER_TOL}" \
  -e MEKK4_SHOW="${MEKK4_SHOW}" \
  -e MEKK4_DEBUG_IMAGE="${MEKK4_DEBUG_IMAGE}" \
  -e MEKK4_DEBUG_IMAGE_TOPIC="${MEKK4_DEBUG_IMAGE_TOPIC}" \
  -e MEKK4_DEBUG_IMAGE_SCALE="${MEKK4_DEBUG_IMAGE_SCALE}" \
  -e MEKK4_DEBUG_IMAGE_FPS="${MEKK4_DEBUG_IMAGE_FPS}" \
  -e MEKK4_DEBUG_STREAM="${MEKK4_DEBUG_STREAM}" \
  -e MEKK4_DEBUG_STREAM_HOST="${CAMERA_REMOTE_HOST}" \
  -e MEKK4_DEBUG_STREAM_PORT="${MEKK4_DEBUG_STREAM_PORT}" \
  -e MEKK4_DEBUG_STREAM_SCALE="${MEKK4_DEBUG_STREAM_SCALE}" \
  -e MEKK4_DEBUG_STREAM_FPS="${MEKK4_DEBUG_STREAM_FPS}" \
  -e MEKK4_DEBUG_STREAM_BITRATE="${MEKK4_DEBUG_STREAM_BITRATE}" \
  ros bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch robot_bringup pi_robot.launch.py use_nav2:=${WITH_NAV2} use_teddy:=${WITH_TEDDY} use_imu:=${WITH_IMU} use_mega_driver:=${WITH_MEGA_DRIVER} product_name:=${PRODUCT_NAME} port_name:=${PORT_NAME} port_baudrate:=${PORT_BAUDRATE} frame_id:=${LIDAR_FRAME} base_frame:=${BASE_FRAME} imu_frame:=${IMU_FRAME} mega_port:=${MEGA_PORT} mega_baudrate:=${MEGA_BAUDRATE} left_cmd_sign:=${LEFT_CMD_SIGN} right_cmd_sign:=${RIGHT_CMD_SIGN} left_tick_sign:=${LEFT_TICK_SIGN} right_tick_sign:=${RIGHT_TICK_SIGN} left_m_per_tick:=${LEFT_M_PER_TICK} right_m_per_tick:=${RIGHT_M_PER_TICK} track_width_eff_m:=${TRACK_WIDTH_EFF_M} map:=${MAP_FILE} params_file:=${PARAMS_FILE}"

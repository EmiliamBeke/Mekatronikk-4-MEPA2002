#!/usr/bin/env bash
set -euo pipefail

PORT_NAME="${PORT_NAME:-/dev/ttyAMA0}"
PRODUCT_NAME="${PRODUCT_NAME:-LDLiDAR_LD06}"
TOPIC_NAME="${TOPIC_NAME:-/lidar}"
STARTUP_WAIT_SECS="${STARTUP_WAIT_SECS:-5}"

# setup.bash scripts may reference unset vars; source with nounset temporarily disabled.
set +u
source /opt/ros/jazzy/setup.bash

if [[ ! -f /ws/install/setup.bash ]]; then
  echo "[lidar-test] Missing /ws/install/setup.bash. Build workspace first (make lidar-setup or make ws)."
  exit 1
fi

source /ws/install/setup.bash
set -u

if [[ ! -e "${PORT_NAME}" ]]; then
  echo "[lidar-test] Serial device not found: ${PORT_NAME}"
  echo "[lidar-test] Try PORT_NAME=/dev/serial0 if your UART symlink is there."
  exit 1
fi

echo "[lidar-test] Starting LDLiDAR on ${PORT_NAME} (${PRODUCT_NAME})..."
ros2 launch robot_bringup lidar_nav2_compat.launch.py \
  port_name:="${PORT_NAME}" \
  product_name:="${PRODUCT_NAME}" \
  topic_name:="${TOPIC_NAME}" \
  > /tmp/lidar_smoketest_launch.log 2>&1 &
launch_pid=$!

cleanup() {
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep "${STARTUP_WAIT_SECS}"

echo "[lidar-test] Waiting for one scan on ${TOPIC_NAME}..."
if timeout 12 ros2 topic echo --once "${TOPIC_NAME}" >/tmp/lidar_scan_once.log 2>&1; then
  echo "[lidar-test] OK: Received LaserScan message on ${TOPIC_NAME}."
  echo "[lidar-test] Showing quick rate sample (5s):"
  timeout 5 ros2 topic hz "${TOPIC_NAME}" || true
  exit 0
fi

echo "[lidar-test] ERROR: No scan message received on ${TOPIC_NAME}."
echo "[lidar-test] Last lines from launch log:"
tail -n 60 /tmp/lidar_smoketest_launch.log || true
exit 1

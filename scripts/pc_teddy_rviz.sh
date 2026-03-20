#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

eval "$(python3 "${SCRIPT_DIR}/camera_config_env.py")"

export PORT="${TEDDY_PORT:-${MEKK4_DEBUG_STREAM_PORT}}"
export WIDTH="${TEDDY_WIDTH:-${MEKK4_DEBUG_STREAM_WIDTH}}"
export HEIGHT="${TEDDY_HEIGHT:-${MEKK4_DEBUG_STREAM_HEIGHT}}"
export TOPIC_NAME="${TEDDY_TOPIC_NAME:-${MEKK4_DEBUG_IMAGE_TOPIC}}"
export FRAME_ID="${TEDDY_FRAME_ID:-camera_link}"
export RVIZ_CONFIG="${RVIZ_CONFIG:-${REPO_ROOT}/src/robot_bringup/rviz/pc_teddy_nav.rviz}"

bash "${SCRIPT_DIR}/pc_camera_rviz.sh" "${1:-${PI_HOST:-gruppe5pi5}}"

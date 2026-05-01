#!/usr/bin/env bash
set -euo pipefail

REPO_URL="https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git"
REPO_TAG="v3.0.3"
WS_ROOT="${1:-.}"
SRC_DIR="${WS_ROOT}/src"
PKG_DIR="${SRC_DIR}/ldlidar_stl_ros2"

mkdir -p "${SRC_DIR}"

if [[ -d "${PKG_DIR}/.git" ]]; then
  echo "[lidar-setup] Existing ldlidar_stl_ros2 found, updating..."
  git -C "${PKG_DIR}" fetch --tags origin
  git -C "${PKG_DIR}" checkout -f "${REPO_TAG}"
elif [[ -d "${PKG_DIR}" ]]; then
  if [[ -f "${PKG_DIR}/package.xml" ]]; then
    echo "[lidar-setup] ${PKG_DIR} already exists (non-git). Using existing folder."
  else
    echo "[lidar-setup] Warning: ${PKG_DIR} exists but is invalid. Recreating folder."
    rm -rf "${PKG_DIR}"
    echo "[lidar-setup] Cloning ldlidar_stl_ros2 (${REPO_TAG})..."
    git clone --depth 1 --branch "${REPO_TAG}" "${REPO_URL}" "${PKG_DIR}"
  fi
else
  echo "[lidar-setup] Cloning ldlidar_stl_ros2 (${REPO_TAG})..."
  git clone --depth 1 --branch "${REPO_TAG}" "${REPO_URL}" "${PKG_DIR}"
fi

# Upstream v3.0.3 misses <pthread.h> include in logger module on some toolchains.
LOG_CPP="${PKG_DIR}/ldlidar_driver/src/logger/log_module.cpp"
if [[ -f "${LOG_CPP}" ]] && ! grep -q '^#include <pthread.h>$' "${LOG_CPP}"; then
  echo "[lidar-setup] Patching missing pthread include in log_module.cpp"
  sed -i '1i#include <pthread.h>' "${LOG_CPP}"
fi

echo "[lidar-setup] Building workspace..."
"${WS_ROOT}/scripts/ws_build.sh"

echo "[lidar-setup] Done."

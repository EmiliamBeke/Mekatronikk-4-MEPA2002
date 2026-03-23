#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

PI_HOST="${1:-${PI_HOST:-gruppe5pi5}}"
shift || true

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "[pc-ros-keyboard] Missing install/setup.bash. Build the local workspace first." >&2
  echo "[pc-ros-keyboard] Example: source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

eval "$(bash "${SCRIPT_DIR}/ros_discovery_env.sh" pc "${PI_HOST}")"

ros2 run mekk4_bringup ros_keyboard_teleop "$@"

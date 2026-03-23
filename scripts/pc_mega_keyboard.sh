#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

eval "$(python3 "${SCRIPT_DIR}/robot_calibration_env.py")"

PI_HOST="${1:-${PI_HOST:-gruppe5@gruppe5pi5}}"
MEGA_PORT="${MEGA_PORT:-/dev/ttyACM0}"
MEGA_BAUDRATE="${MEGA_BAUDRATE:-115200}"
DRIVE_SPEED="${DRIVE_SPEED:-90}"
TURN_SPEED="${TURN_SPEED:-55}"
REMOTE_REPO="${REMOTE_REPO:-~/Mekatronikk-4-MEPA2002}"
PI_PASSWORD="${PI_PASSWORD:-}"

if ! python3 -c 'import tkinter' >/dev/null 2>&1; then
  echo "[pc-mega-keyboard] Missing tkinter on this machine." >&2
  echo "[pc-mega-keyboard] Install it with: sudo apt install python3-tk" >&2
  exit 1
fi

if ! command -v sshpass >/dev/null 2>&1; then
  echo "[pc-mega-keyboard] Missing sshpass on this machine." >&2
  echo "[pc-mega-keyboard] Install it with: sudo apt install sshpass" >&2
  exit 1
fi

if [[ -z "${PI_PASSWORD}" ]]; then
  read -rsp "[pc-mega-keyboard] Pi password: " PI_PASSWORD
  echo >&2
fi

export MEGA_PI_PASSWORD="${PI_PASSWORD}"

python3 "${SCRIPT_DIR}/mega_keyboard_gui.py" \
  --host "${PI_HOST}" \
  --port "${MEGA_PORT}" \
  --baudrate "${MEGA_BAUDRATE}" \
  --speed "${DRIVE_SPEED}" \
  --turn-speed "${TURN_SPEED}" \
  --remote-repo "${REMOTE_REPO}" \
  "$(if [[ "${SWAP_SIDES:-1}" == "1" ]]; then echo --swap-sides; else echo --no-swap-sides; fi)" \
  --left-cmd-sign "${LEFT_CMD_SIGN:-1}" \
  --right-cmd-sign "${RIGHT_CMD_SIGN:-1}" \
  --left-cmd-scale "${LEFT_CMD_SCALE:-1.0}" \
  --right-cmd-scale "${RIGHT_CMD_SCALE:-1.0}"

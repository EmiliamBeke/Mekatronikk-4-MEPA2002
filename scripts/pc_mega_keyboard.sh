#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

PI_HOST="${1:-${PI_HOST:-gruppe5pi5}}"
MEGA_PORT="${MEGA_PORT:-/dev/ttyACM0}"
MEGA_BAUDRATE="${MEGA_BAUDRATE:-115200}"
DRIVE_SPEED="${DRIVE_SPEED:-90}"
TURN_SPEED="${TURN_SPEED:-55}"
REMOTE_REPO="${REMOTE_REPO:-\$HOME/Mekatronikk-4-MEPA2002}"

if ! python3 -c 'import tkinter' >/dev/null 2>&1; then
  echo "[pc-mega-keyboard] Missing tkinter on this machine." >&2
  echo "[pc-mega-keyboard] Install it with: sudo apt install python3-tk" >&2
  exit 1
fi

python3 "${SCRIPT_DIR}/mega_keyboard_gui.py" \
  --host "${PI_HOST}" \
  --port "${MEGA_PORT}" \
  --baudrate "${MEGA_BAUDRATE}" \
  --speed "${DRIVE_SPEED}" \
  --turn-speed "${TURN_SPEED}" \
  --remote-repo "${REMOTE_REPO}"

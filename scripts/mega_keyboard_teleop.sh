#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

eval "$(python3 "${SCRIPT_DIR}/robot_calibration_env.py")"

detect_mega_port() {
  local candidate resolved

  for candidate in /dev/serial/by-id/*; do
    [[ -e "${candidate}" ]] || continue
    resolved="$(readlink -f "${candidate}")"
    case "${candidate} ${resolved}" in
      *Arduino*|*arduino*|*Mega*|*mega*|*ttyACM*|*ttyUSB*)
        printf '%s\n' "${resolved}"
        return 0
        ;;
    esac
  done

  for candidate in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "${candidate}" ]] || continue
    printf '%s\n' "${candidate}"
    return 0
  done

  return 1
}

MEGA_PORT="${MEGA_PORT:-}"
MEGA_BAUDRATE="${MEGA_BAUDRATE:-115200}"
DRIVE_SPEED="${DRIVE_SPEED:-90}"
TURN_SPEED="${TURN_SPEED:-75}"

if [[ -z "${MEGA_PORT}" ]]; then
  MEGA_PORT="$(detect_mega_port)" || {
    echo "[mega-keyboard] Could not auto-detect Arduino Mega serial device." >&2
    echo "[mega-keyboard] Set it manually, for example: MEGA_PORT=/dev/ttyACM0 make mega-keyboard" >&2
    exit 1
  }
fi

if [[ ! -e "${MEGA_PORT}" ]]; then
  echo "[mega-keyboard] Serial device not found: ${MEGA_PORT}" >&2
  exit 1
fi

if ! python3 -c 'import serial' >/dev/null 2>&1; then
  echo "[mega-keyboard] Missing python3 serial support on host." >&2
  echo "[mega-keyboard] Install it with: sudo apt install python3-serial" >&2
  exit 1
fi

echo "[mega-keyboard] Using ${MEGA_PORT} @ ${MEGA_BAUDRATE}" >&2
echo "[mega-keyboard] Upload arduino/mega_keyboard_drive/mega_keyboard_drive.ino first." >&2

python3 "${SCRIPT_DIR}/mega_keyboard_teleop.py" \
  --port "${MEGA_PORT}" \
  --baudrate "${MEGA_BAUDRATE}" \
  --speed "${DRIVE_SPEED}" \
  --turn-speed "${TURN_SPEED}" \
  "$(if [[ "${SWAP_SIDES:-1}" == "1" ]]; then echo --swap-sides; else echo --no-swap-sides; fi)" \
  --left-cmd-sign "${LEFT_CMD_SIGN:-1}" \
  --right-cmd-sign "${RIGHT_CMD_SIGN:-1}" \
  --left-cmd-scale "${LEFT_CMD_SCALE:-1.0}" \
  --right-cmd-scale "${RIGHT_CMD_SCALE:-1.0}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

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

MODE="${1:-all}"
MEGA_PORT="${MEGA_PORT:-}"
MEGA_BAUDRATE="${MEGA_BAUDRATE:-115200}"
ARM_HOME_TIMEOUT="${ARM_HOME_TIMEOUT:-180}"

if [[ "${MODE}" != "all" && "${MODE}" != "x" ]]; then
  echo "[mega-arm-cal] Mode must be 'all' or 'x'." >&2
  exit 1
fi

if [[ -z "${MEGA_PORT}" ]]; then
  MEGA_PORT="$(detect_mega_port)" || {
    echo "[mega-arm-cal] Could not auto-detect Arduino Mega serial device." >&2
    echo "[mega-arm-cal] Set it manually, for example: MEGA_PORT=/dev/ttyACM0 make arm-cal" >&2
    exit 1
  }
fi

if [[ ! -e "${MEGA_PORT}" ]]; then
  echo "[mega-arm-cal] Serial device not found: ${MEGA_PORT}" >&2
  exit 1
fi

if ! python3 -c 'import serial' >/dev/null 2>&1; then
  echo "[mega-arm-cal] Missing python3 serial support on host." >&2
  echo "[mega-arm-cal] Install it with: sudo apt install python3-serial" >&2
  exit 1
fi

python3 "${SCRIPT_DIR}/mega_arm_calibrate.py" \
  "${MODE}" \
  --port "${MEGA_PORT}" \
  --baudrate "${MEGA_BAUDRATE}" \
  --timeout "${ARM_HOME_TIMEOUT}"

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

resolve_sketch_dir() {
  local input="${1}"
  local sketch_dir=""
  local sketch_file=""
  local base_name=""

  if [[ -d "${input}" ]]; then
    sketch_dir="${input}"
  elif [[ -f "${input}" ]]; then
    sketch_dir="$(dirname "${input}")"
    sketch_file="$(basename "${input}")"
    base_name="$(basename "${sketch_dir}")"
    if [[ "${sketch_file}" != "${base_name}.ino" ]]; then
      echo "[mega-upload] Sketch file must match its directory name: ${base_name}.ino" >&2
      return 1
    fi
  elif [[ -d "${REPO_ROOT}/arduino/${input}" ]]; then
    sketch_dir="${REPO_ROOT}/arduino/${input}"
  else
    echo "[mega-upload] Sketch not found: ${input}" >&2
    echo "[mega-upload] Try for example: MEGA_SKETCH=mega_keyboard_drive make mega-upload" >&2
    return 1
  fi

  sketch_dir="$(cd "${sketch_dir}" && pwd)"
  base_name="$(basename "${sketch_dir}")"
  sketch_file="${sketch_dir}/${base_name}.ino"

  if [[ ! -f "${sketch_file}" ]]; then
    echo "[mega-upload] Expected sketch file is missing: ${sketch_file}" >&2
    return 1
  fi

  printf '%s\n' "${sketch_dir}"
}

SKETCH_INPUT="${1:-${MEGA_SKETCH:-mega_smoketest}}"
MEGA_PORT="${MEGA_PORT:-}"
MEGA_FQBN="${MEGA_FQBN:-arduino:avr:mega}"
ARDUINO_CLI="${ARDUINO_CLI:-arduino-cli}"

if ! command -v "${ARDUINO_CLI}" >/dev/null 2>&1; then
  echo "[mega-upload] Missing ${ARDUINO_CLI} on host." >&2
  echo "[mega-upload] Install arduino-cli and run once: arduino-cli core install arduino:avr" >&2
  exit 1
fi

SKETCH_DIR="$(resolve_sketch_dir "${SKETCH_INPUT}")"
SKETCH_NAME="$(basename "${SKETCH_DIR}")"

if [[ -z "${MEGA_PORT}" ]]; then
  MEGA_PORT="$(detect_mega_port)" || {
    echo "[mega-upload] Could not auto-detect Arduino Mega serial device." >&2
    echo "[mega-upload] Set it manually, for example: MEGA_PORT=/dev/ttyACM0 make mega-upload" >&2
    exit 1
  }
fi

if [[ ! -e "${MEGA_PORT}" ]]; then
  echo "[mega-upload] Serial device not found: ${MEGA_PORT}" >&2
  exit 1
fi

BUILD_PATH="$(mktemp -d "/tmp/mega-upload-${SKETCH_NAME}.XXXXXX")"
cleanup() {
  rm -rf "${BUILD_PATH}"
}
trap cleanup EXIT INT TERM

echo "[mega-upload] Using ${MEGA_PORT}" >&2
echo "[mega-upload] Sketch ${SKETCH_DIR}/${SKETCH_NAME}.ino" >&2
echo "[mega-upload] FQBN ${MEGA_FQBN}" >&2

"${ARDUINO_CLI}" compile \
  --fqbn "${MEGA_FQBN}" \
  --build-path "${BUILD_PATH}" \
  --upload \
  -p "${MEGA_PORT}" \
  "${SKETCH_DIR}"

echo "[mega-upload] Upload complete." >&2

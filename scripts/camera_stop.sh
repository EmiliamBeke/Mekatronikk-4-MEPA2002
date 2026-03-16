#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "/scripts/camera_stream_supervisor.sh"
  "/scripts/camera_udp_stream.sh"
  "rpicam-vid.*--libav-format h264"
  "gst-launch-1.0 -q fdsrc .*rtph264pay"
)

found=0
for pattern in "${patterns[@]}"; do
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    found=1
    pkill -f "${pattern}" 2>/dev/null || true
  fi
done

if [[ "${found}" == "1" ]]; then
  echo "[camera-stop] Stopped existing camera stream processes." >&2
else
  echo "[camera-stop] No existing camera stream processes found." >&2
fi

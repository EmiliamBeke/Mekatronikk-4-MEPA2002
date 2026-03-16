#!/usr/bin/env bash
set -euo pipefail

cd /ws

# Clear stale Python package build/install state before colcon build.
# ament_python packages copy setup.py/launch data into /ws/build/<pkg>, and a
# deleted launch/data file can remain referenced there even after source files
# are removed from /ws/src.
while IFS= read -r pkg_dir; do
  pkg_name="$(basename "${pkg_dir}")"
  rm -rf "/ws/build/${pkg_name}" "/ws/install/${pkg_name}"
done < <(find /ws/src -maxdepth 2 -name setup.py -printf '%h\n')

# Also clear any lingering setuptools metadata just in case.
find /ws/build -type d -name '*.egg-info' -prune -exec rm -rf {} +

colcon build --symlink-install

python3 - <<'PY'
from pathlib import Path

old = b"#!/usr/bin/python3\n"
new = b"#!/opt/venv/bin/python3\n"

root = Path("/ws/install")
patched = 0

for f in root.rglob("*"):
    if not f.is_file():
        continue
    # vi vil kun treffe /ws/install/<pkg>/lib/<pkg>/<script>
    parts = f.parts
    if "lib" not in parts:
        continue
    i = parts.index("lib")
    if len(parts) < i + 3:
        continue

    try:
        with f.open("rb") as fh:
            first = fh.readline()
        if first == old:
            data = f.read_bytes()
            f.write_bytes(new + data[len(old):])
            patched += 1
    except Exception:
        pass

print(f"shebang patched: {patched} file(s)")
PY

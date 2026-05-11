#!/usr/bin/env bash
# Pi sparse checkout setup
# Run this on Pi for first-time clone to set up sparse checkout.
# Usage: bash pi_sparse_checkout_setup.sh

set -euo pipefail

REPO_URL="${1:-https://github.com/EmiliamBeke/Mekatronikk-4-MEPA2002.git}"
REPO_DIR="${2:-Mekatronikk-4-MEPA2002}"

echo "[pi-sparse-checkout] Cloning repository with sparse checkout..." >&2
echo "[pi-sparse-checkout] URL: ${REPO_URL}" >&2
echo "[pi-sparse-checkout] Directory: ${REPO_DIR}" >&2

# Clone without checking out files
git clone --no-checkout "${REPO_URL}" "${REPO_DIR}"
cd "${REPO_DIR}"

# Initialize sparse checkout (non-cone: supports subdirectory exclusions)
git sparse-checkout init

# Define what gets checked out on Pi.
# Meshes are excluded — robot_state_publisher only needs the URDF text, not STL files.
git sparse-checkout set \
  'src/mekk4_bringup/' \
  'src/mekk4_perception/' \
  'src/robot_bringup/' \
  'src/robot_description/' \
  '!src/robot_description/meshes/' \
  'arduino/' \
  'config/' \
  'scripts/' \
  'docker/' \
  'models/yolo26n_ncnn_model/' \
  'compose.yml' \
  'Makefile' \
  '.gitignore' \
  '.dockerignore' \
  '.env.example'

# Checkout main branch
git checkout main

echo "[pi-sparse-checkout] Setup complete." >&2
echo "[pi-sparse-checkout] Workspace is ready at: ${REPO_DIR}" >&2
echo "[pi-sparse-checkout] To verify sparse checkout:" >&2
echo "[pi-sparse-checkout]   cd ${REPO_DIR}" >&2
echo "[pi-sparse-checkout]   git sparse-checkout list" >&2
echo "[pi-sparse-checkout]" >&2
echo "[pi-sparse-checkout] Contents on Pi:" >&2
ls -la

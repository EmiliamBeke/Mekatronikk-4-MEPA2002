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

# Initialize sparse checkout
git sparse-checkout init --cone

# Define what gets checked out on Pi
# Include only Pi-essential directories and files
git sparse-checkout set \
  src/mekk4_bringup \
  src/mekk4_perception \
  src/robot_bringup \
  src/robot_description \
  arduino \
  config \
  scripts \
  docker \
  models/yolo26n_ncnn_model \
  compose.yml \
  Makefile \
  README.md \
  AGENTS.md \
  LICENSE \
  .gitignore \
  .env.example

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

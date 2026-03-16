#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
Usage:
  bash scripts/ros_discovery_env.sh pi [pc-host-or-ip]
  bash scripts/ros_discovery_env.sh pc [pi-host-or-ip]

Examples:
  eval "$(bash scripts/ros_discovery_env.sh pi)"
  eval "$(bash scripts/ros_discovery_env.sh pc gruppe5pi5)"
EOF
  exit 1
}

first_ipv4_from_host() {
  local target="$1"
  local resolved=""

  if [[ "${target}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    printf '%s\n' "${target}"
    return 0
  fi

  resolved="$(getent ahostsv4 "${target}" 2>/dev/null | awk 'NR == 1 {print $1}')"
  if [[ -n "${resolved}" ]]; then
    printf '%s\n' "${resolved}"
    return 0
  fi

  if [[ "${target}" != *.* ]]; then
    resolved="$(getent ahostsv4 "${target}.local" 2>/dev/null | awk 'NR == 1 {print $1}')"
    if [[ -n "${resolved}" ]]; then
      printf '%s\n' "${resolved}"
      return 0
    fi
  fi

  return 1
}

detect_ssh_client_ip() {
  if [[ -n "${ROS_STATIC_PEERS:-}" ]]; then
    printf '%s\n' "${ROS_STATIC_PEERS}"
    return 0
  fi

  if [[ -n "${ROS_PEER_IP:-}" ]]; then
    printf '%s\n' "${ROS_PEER_IP}"
    return 0
  fi

  if [[ -n "${ROS_PC_IP:-}" ]]; then
    printf '%s\n' "${ROS_PC_IP}"
    return 0
  fi

  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    awk '{print $1}' <<<"${SSH_CONNECTION}"
    return 0
  fi

  if [[ -n "${SSH_CLIENT:-}" ]]; then
    awk '{print $1}' <<<"${SSH_CLIENT}"
    return 0
  fi

  return 1
}

role="${1:-}"
peer_input="${2:-}"

if [[ -z "${role}" ]]; then
  usage
fi

domain_id="${ROS_DOMAIN_ID:-0}"
discovery_range="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
peer_value=""

case "${role}" in
  pi)
    if [[ -n "${peer_input}" ]]; then
      peer_value="$(first_ipv4_from_host "${peer_input}")" || {
        echo "[ros-net] Could not resolve PC host: ${peer_input}" >&2
        exit 1
      }
    else
      peer_value="$(detect_ssh_client_ip)" || {
        echo "[ros-net] Could not detect PC IP automatically. Pass it explicitly." >&2
        exit 1
      }
    fi
    ;;
  pc)
    peer_input="${peer_input:-${ROS_PI_HOST:-gruppe5pi5}}"
    peer_value="$(first_ipv4_from_host "${peer_input}")" || {
      echo "[ros-net] Could not resolve Pi host: ${peer_input}" >&2
      exit 1
    }
    ;;
  *)
    usage
    ;;
esac

cat <<EOF
export ROS_DOMAIN_ID=${domain_id}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${discovery_range}
export ROS_STATIC_PEERS=${peer_value}
EOF

echo "[ros-net] role=${role} peer=${peer_value} domain=${domain_id} discovery=${discovery_range}" >&2

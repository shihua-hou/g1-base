#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

load_ros_env
export START_ODOM_TO_TF=1
export SKIP_ODOM_TO_TF=1

NAV_MANAGER_ARGS=(--auto-ensure)
if [[ -n "${READY_TIMEOUT:-}" ]]; then
  NAV_MANAGER_ARGS+=(--localization-timeout "$READY_TIMEOUT")
fi
if [[ -n "${POINTCLOUD_TOPIC:-}" ]]; then
  NAV_MANAGER_ARGS+=(--pointcloud-topic "$POINTCLOUD_TOPIC")
fi
if [[ -n "${RELOCATION_TOPIC:-}" ]]; then
  NAV_MANAGER_ARGS+=(--relocal-odom-topic "$RELOCATION_TOPIC")
fi

cleanup() {
  if [[ -n "${NAV_MANAGER_PID:-}" ]]; then
    kill "$NAV_MANAGER_PID" 2>/dev/null || true
    wait "$NAV_MANAGER_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

ros2 run g1_base navigation_manager "${NAV_MANAGER_ARGS[@]}" &
NAV_MANAGER_PID=$!

exec ros2 run g1_base g1_control_server "$@"

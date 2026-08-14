#!/usr/bin/env bash
# 启动 iPad 上位机 HTTP 网关 (g1_web_bridge)。
#
# 前置条件: navigation_manager + g1_control_server 已经在运行
# (通常由 start_navigation.sh 启动)。本脚本只负责把 ROS2 服务/动作
# 包装成 HTTP API + 静态网页，监听 --port (默认 8081)。

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

DEFAULT_NET_IF="${DEFAULT_NET_IF:-enP8p1s0}"
DEFAULT_PORT="${WEB_BRIDGE_PORT:-8081}"

load_ros_env

if [[ "$#" -eq 0 ]]; then
    set -- --net-if "$DEFAULT_NET_IF" --port "$DEFAULT_PORT"
fi

exec ros2 run g1_base g1_web_bridge "$@"

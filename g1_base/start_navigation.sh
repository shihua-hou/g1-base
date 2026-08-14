#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

resolve_default_map_file() {
    local maps_dir="$ROOT_DIR/config/maps"
    local default_map="$maps_dir/exhibit_2d_map.yaml"

    if [[ -f "$default_map" ]]; then
        echo "$default_map"
        return
    fi

    local latest_map
    latest_map="$(find "$maps_dir" -maxdepth 1 -type f -name '*_exhibit_2d_map.yaml' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
    if [[ -n "$latest_map" ]]; then
        echo "$latest_map"
        return
    fi

    echo "$default_map"
}

MAP_FILE="${MAP_FILE:-$(resolve_default_map_file)}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$ROOT_DIR/config/nav2_params.yaml}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/lio/cloud_world}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
TARGET_FRAME="${TARGET_FRAME:-base_link}"
SCAN_RANGE_MIN="${SCAN_RANGE_MIN:-0.05}"
SKIP_ODOM_TO_TF="${SKIP_ODOM_TO_TF:-false}"

load_ros_env

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
fi

echo "============================================================"
if $DRY_RUN; then
    echo "  G1 导航系统启动 - 干跑模式"
else
    echo "  G1 导航系统启动"
fi
echo "============================================================"

if $DRY_RUN; then
    echo "[1/2] 启动模拟 TF + 里程计 (cmd_vel_mock)..."
    ros2 run g1_base cmd_vel_mock &
    ODOM_PID=$!
else
    if [[ "$SKIP_ODOM_TO_TF" == "true" || "$SKIP_ODOM_TO_TF" == "1" ]]; then
        echo "[1/2] 复用已启动的 TF 广播 (odom_to_tf)..."
        ODOM_PID=""
    else
        echo "[1/2] 启动 TF 广播 (odom_to_tf)..."
        ros2 run g1_base odom_to_tf &
        ODOM_PID=$!
    fi
fi
sleep 1

echo "[2/2] 启动 Nav2 导航..."
if $DRY_RUN; then
    echo "  另开终端执行 RViz 发送目标点验证 /cmd_vel。"
else
    echo "  另开终端执行: bash $ROOT_DIR/start_nav_script.sh --net-if enP8p1s0 --route $ROOT_DIR/config/routes/waypoint_1.yaml"
fi

cleanup() {
    echo
    echo "正在停止导航系统..."
    if [[ -n "${ODOM_PID:-}" ]]; then
        kill "$ODOM_PID" 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    echo "已停止"
}
trap cleanup EXIT INT TERM

ros2 launch g1_base navigation.launch.py \
    map_file:="$MAP_FILE" \
    nav2_params_file:="$NAV2_PARAMS_FILE" \
    cloud_topic:="$POINTCLOUD_TOPIC" \
    scan_topic:="$SCAN_TOPIC" \
    target_frame:="$TARGET_FRAME" \
    scan_range_min:="$SCAN_RANGE_MIN"

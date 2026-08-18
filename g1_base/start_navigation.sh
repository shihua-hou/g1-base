#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

# 地图目录：必须和 start_pc2_localization.sh / 网关三方一致。
# 之前这里写死 $ROOT_DIR/config/maps（镜像里的出厂地图），而重定位读的是
# 数据卷里现场建的图 —— 于是 Super-LIO 在你的地图坐标系里算位姿，Nav2 却
# 拿另一个场馆的底图做代价地图，界面上位姿画得完全不对，两边还都不报错。
resolve_maps_dir() {
    if [[ -n "${G1_MAPS_DIR:-}" ]]; then
        echo "$G1_MAPS_DIR"
    elif [[ -n "${G1_DATA_DIR:-}" ]]; then
        echo "$G1_DATA_DIR/maps"
    else
        echo "$ROOT_DIR/config/maps"
    fi
}

resolve_default_map_file() {
    local maps_dir
    maps_dir="$(resolve_maps_dir)"
    # exhibit_2d_map.yaml 是「设为当前」时写过去的那张，优先用它
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

    # 兜底：数据卷里一张图都没有时，用包内出厂地图，至少让 Nav2 起得来
    local packaged="$ROOT_DIR/config/maps/exhibit_2d_map.yaml"
    if [[ "$maps_dir" != "$ROOT_DIR/config/maps" && -f "$packaged" ]]; then
        echo "$packaged"
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

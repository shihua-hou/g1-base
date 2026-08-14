#!/usr/bin/env bash
# 一键编译 g1_base_interfaces + g1_centerline_planner + g1_base_perception + g1_base
# 用法: cd /home/unitree/g1_base && bash build.sh
set -euo pipefail

G1_BASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_USER_HOME="${HOME:-/home/$(id -un)}"
G1_WS="${G1_USER_HOME}/g1_ws"
G1_WS_SRC="${G1_WS}/src"
G1_INTERFACES_SRC="${G1_INTERFACES_SRC:-${G1_WS_SRC}/g1_base_interfaces}"
G1_CENTERLINE_PLANNER_SRC="${G1_CENTERLINE_PLANNER_SRC:-${G1_WS_SRC}/g1_centerline_planner}"
G1_BASE_PERCEPTION_SRC="${G1_BASE_PERCEPTION_SRC:-${G1_BASE_ROOT}/../g1_base_perception}"
G1_BASE_CLEAN_BUILD="${G1_BASE_CLEAN_BUILD:-1}"

ensure_workspace_link() {
    local target="$1"
    local link_path="$2"

    if [[ -L "$link_path" && ! -e "$link_path" ]]; then
        rm -f "$link_path"
    fi
    if [[ ! -e "$link_path" && -d "$target" ]]; then
        mkdir -p "$(dirname "$link_path")"
        ln -s "$target" "$link_path"
        echo "[build] linked $link_path -> $target"
    fi
}

require_package() {
    local package_name="$1"
    local package_dir="$2"

    if [[ ! -f "${package_dir}/package.xml" ]]; then
        echo "[build] ERROR: ${package_name} not found at ${package_dir}" >&2
        echo "[build] 请先同步 ${package_name} 到该路径，或设置对应 *_SRC 环境变量。" >&2
        exit 1
    fi
}

clean_g1_base_artifacts() {
    if [[ "${G1_BASE_CLEAN_BUILD}" == "0" || "${G1_BASE_CLEAN_BUILD}" == "false" ]]; then
        echo "[build] 跳过 g1_base 构建缓存清理 (G1_BASE_CLEAN_BUILD=${G1_BASE_CLEAN_BUILD})"
        return
    fi

    echo "[build] 清理 g1_base 包级构建缓存，避免旧资源清单引用已删除文件 ..."
    rm -rf \
        "${G1_BASE_ROOT}/build/g1_base" \
        "${G1_BASE_ROOT}/build/g1_base_perception" \
        "${G1_BASE_ROOT}/install/g1_base" \
        "${G1_BASE_ROOT}/install/g1_base_perception" \
        "${G1_BASE_ROOT}/build/g1_base.egg-info" \
        "${G1_BASE_ROOT}/g1_base.egg-info"
}

mkdir -p "${G1_WS_SRC}"
ensure_workspace_link "${G1_BASE_ROOT}/g1_base_interfaces" "${G1_INTERFACES_SRC}"
ensure_workspace_link "${G1_BASE_ROOT}/../g1_centerline_planner" "${G1_CENTERLINE_PLANNER_SRC}"

require_package "g1_base_interfaces" "${G1_INTERFACES_SRC}"
require_package "g1_centerline_planner" "${G1_CENTERLINE_PLANNER_SRC}"
require_package "g1_base_perception" "${G1_BASE_PERCEPTION_SRC}"

echo "[build] ① 编译 g1_base_interfaces + g1_centerline_planner (in ${G1_WS}) ..."
cd "${G1_WS}"
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select g1_base_interfaces g1_centerline_planner
echo "[build] ① g1_base_interfaces + g1_centerline_planner 编译完成"

echo "[build] ② 编译 g1_base_perception + g1_base (in ${G1_BASE_ROOT}) ..."
cd "${G1_BASE_ROOT}"
clean_g1_base_artifacts
set +u
source ./config/robot_env.sh
load_ros_env
set -u
colcon build --symlink-install \
    --base-paths "${G1_BASE_ROOT}" "${G1_BASE_PERCEPTION_SRC}" \
    --packages-up-to g1_base
echo "[build] ② g1_base_perception + g1_base 编译完成"

set +u
source "${G1_BASE_ROOT}/install/setup.bash"
set -u
echo "[build] 全部编译完成 ✓"

#!/usr/bin/env bash
# g1_base_manager 启动脚本
# 由 bot_mind 的 NavigationManagerService 作为子进程拉起
set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"
load_ros_env

# SDK 子进程需要 unitree_sdk2py，它装在 conda py310 里
CONDA_SH="${CONDA_SH:-/home/unitree/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-py310}"
if [[ -f "$CONDA_SH" ]]; then
    set +u
    source "$CONDA_SH"
    conda activate "$CONDA_ENV"
    set -u
fi

export G1_SDK_PYTHON_BIN="$(which python)"

# 确保仓库根目录在 PYTHONPATH 中，以便 python -m g1_base.xxx 能找到包
case ":${PYTHONPATH:-}:" in
    *":$ROOT_DIR:"*) ;;
    *) export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:${PYTHONPATH}}" ;;
esac

G1_BASE_MANAGER_ARGS=()
if [[ -n "${READY_TIMEOUT:-}" ]]; then
    G1_BASE_MANAGER_ARGS+=(--localization-timeout "$READY_TIMEOUT")
fi
if [[ -n "${POINTCLOUD_TOPIC:-}" ]]; then
    G1_BASE_MANAGER_ARGS+=(--pointcloud-topic "$POINTCLOUD_TOPIC")
fi
if [[ -n "${RELOCATION_TOPIC:-}" ]]; then
    G1_BASE_MANAGER_ARGS+=(--relocal-odom-topic "$RELOCATION_TOPIC")
fi

exec python -m g1_base.g1_base_manager "${G1_BASE_MANAGER_ARGS[@]}" "$@"

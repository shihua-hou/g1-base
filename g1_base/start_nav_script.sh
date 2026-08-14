#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

DEFAULT_NET_IF="${DEFAULT_NET_IF:-enP8p1s0}"
DEFAULT_ROUTE="${DEFAULT_ROUTE:-$ROOT_DIR/config/routes/waypoint_1.yaml}"

load_ros_env

if [[ "$#" -eq 0 ]]; then
    set -- --net-if "$DEFAULT_NET_IF" --route "$DEFAULT_ROUTE"
fi

PYTHON_BIN="${G1_NAV_PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
    if command -v python >/dev/null 2>&1; then
        PYTHON_BIN="$(command -v python)"
    else
        PYTHON_BIN="$(command -v python3)"
    fi
fi

case ":${PYTHONPATH:-}:" in
    *":$ROOT_DIR:"*) ;;
    *)
        export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}$ROOT_DIR"
        ;;
esac

SDK_PYTHON_BIN="${G1_SDK_PYTHON_BIN:-}"
if [[ -z "$SDK_PYTHON_BIN" ]]; then
    for candidate in "$PYTHON_BIN" python python3; do
        if command -v "$candidate" >/dev/null 2>&1; then
            resolved_candidate="$(command -v "$candidate")"
            if "$resolved_candidate" -c "import unitree_sdk2py" >/dev/null 2>&1; then
                SDK_PYTHON_BIN="$resolved_candidate"
                break
            fi
        fi
    done
fi

if [[ -n "$SDK_PYTHON_BIN" ]]; then
    export G1_SDK_PYTHON_BIN="$SDK_PYTHON_BIN"
fi

if "$PYTHON_BIN" -c "import rclpy" >/dev/null 2>&1; then
    echo "使用主 Python 解释器: $PYTHON_BIN"
    if [[ -n "${G1_SDK_PYTHON_BIN:-}" ]]; then
        echo "使用 SDK Python 解释器: $G1_SDK_PYTHON_BIN"
    fi
    exec "$PYTHON_BIN" -m g1_base.nav_script "$@"
fi

echo "警告: $PYTHON_BIN 无法导入 rclpy，回退到 ros2 run" >&2
exec ros2 run g1_base nav_script "$@"

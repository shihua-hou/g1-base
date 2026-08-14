#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

SUPER_LIO_PACKAGE="${SUPER_LIO_PACKAGE:-super_lio}"
SUPER_LIO_LAUNCH="${SUPER_LIO_LAUNCH:-relocation.py}"
SUPER_LIO_CONFIG_FILE="${SUPER_LIO_CONFIG_FILE:-}"
SUPER_LIO_SOURCE_ROOT="${SUPER_LIO_SOURCE_ROOT:-}"
SUPER_LIO_DIRECT_RELOCATION="${SUPER_LIO_DIRECT_RELOCATION:-true}"
RELOCATION_PCD_FILE="${RELOCATION_PCD_FILE:-}"
LIVOX_PACKAGE="${LIVOX_PACKAGE:-livox_ros_driver2}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360_launch.py}"
RVIZ_FLAG="${RVIZ_FLAG:-false}"
START_ODOM_TO_TF="${START_ODOM_TO_TF:-true}"
IMU_STEADY_TIMEOUT="${IMU_STEADY_TIMEOUT:-10.0}"
IMU_STEADY_GYRO_MAX_RAD_S="${IMU_STEADY_GYRO_MAX_RAD_S:-0.05}"
IMU_STEADY_ACCEL_MIN_G="${IMU_STEADY_ACCEL_MIN_G:-0.950}"
IMU_STEADY_ACCEL_MAX_G="${IMU_STEADY_ACCEL_MAX_G:-1.050}"
IMU_STEADY_GYRO_VAR_MAX="${IMU_STEADY_GYRO_VAR_MAX:-1e-4}"
IMU_STEADY_ACCEL_VAR_MAX="${IMU_STEADY_ACCEL_VAR_MAX:-8e-4}"

load_ros_env

RELOCATION_PID=""
LIVOX_PID=""
ODOM_TO_TF_PID=""
RVIZ_PID=""
CLEANED_UP=0

log() {
    printf '[pc2-localization] %s\n' "$*"
}

is_alive() {
    local pid="$1"
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_in_process_group() {
    local __pid_var="$1"
    shift

    setsid bash -c '
        source "$1/config/robot_env.sh"
        shift
        load_ros_env
        exec "$@"
    ' _ "$ROOT_DIR" "$@" &
    printf -v "$__pid_var" '%s' "$!"
}

signal_process_group() {
    local pid="$1"
    local sig="$2"

    [[ -n "$pid" ]] || return 0
    kill -s "$sig" -- "-$pid" 2>/dev/null || kill -s "$sig" "$pid" 2>/dev/null || true
}

wait_until_stopped() {
    local pid="$1"
    local timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))

    while is_alive "$pid" && (( SECONDS < deadline )); do
        sleep 0.5
    done

    ! is_alive "$pid"
}

stop_process_group() {
    local pid="$1"
    local label="$2"

    if ! is_alive "$pid"; then
        return
    fi

    log "stopping ${label} (pid=${pid})"
    signal_process_group "$pid" INT
    if ! wait_until_stopped "$pid" 5; then
        signal_process_group "$pid" TERM
    fi
    if ! wait_until_stopped "$pid" 5; then
        log "${label} did not exit, force killing"
        signal_process_group "$pid" KILL
    fi
    wait "$pid" 2>/dev/null || true
}

resolve_config_map_pcd() {
    local maps_dir="$ROOT_DIR/config/maps"
    local default_map="$maps_dir/map.pcd"

    if [[ -f "$default_map" ]]; then
        echo "$default_map"
        return
    fi

    local latest_map
    latest_map="$(find "$maps_dir" -maxdepth 1 -type f -name '*_map.pcd' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
    if [[ -n "$latest_map" ]]; then
        echo "$latest_map"
        return
    fi

    echo "$default_map"
}

resolve_super_lio_prefix() {
    ros2 pkg prefix "$SUPER_LIO_PACKAGE" 2>/dev/null || true
}

resolve_super_lio_source_root() {
    if [[ -n "$SUPER_LIO_SOURCE_ROOT" ]]; then
        echo "$SUPER_LIO_SOURCE_ROOT"
        return
    fi

    local prefix
    prefix="$(resolve_super_lio_prefix)"
    if [[ -n "$prefix" ]]; then
        local candidate="$prefix/../../src/Super-LIO/src/super_lio"
        if [[ -d "$candidate" ]]; then
            (cd "$candidate" && pwd)
            return
        fi
    fi

    echo "$LIO_WORKSPACE_ROOT/src/Super-LIO/src/super_lio"
}

resolve_super_lio_config_file() {
    if [[ -n "$SUPER_LIO_CONFIG_FILE" ]]; then
        echo "$SUPER_LIO_CONFIG_FILE"
        return
    fi

    local prefix
    prefix="$(resolve_super_lio_prefix)"
    if [[ -n "$prefix" ]]; then
        echo "$prefix/share/$SUPER_LIO_PACKAGE/config/relocation_360.yaml"
        return
    fi

    echo "$LIO_WORKSPACE_ROOT/install/$SUPER_LIO_PACKAGE/share/$SUPER_LIO_PACKAGE/config/relocation_360.yaml"
}

relative_path() {
    python3 - "$1" "$2" <<'PY'
import os
import sys
from pathlib import Path

print(os.path.relpath(Path(sys.argv[2]).resolve(), Path(sys.argv[1]).resolve()))
PY
}

start_relocation() {
    if [[ "$SUPER_LIO_DIRECT_RELOCATION" == "true" || "$SUPER_LIO_DIRECT_RELOCATION" == "1" ]]; then
        local pcd_file="${RELOCATION_PCD_FILE:-$(resolve_config_map_pcd)}"
        if [[ ! -f "$pcd_file" ]]; then
            log "PCD map not found: $pcd_file"
            exit 1
        fi

        local config_file
        config_file="$(resolve_super_lio_config_file)"
        if [[ ! -f "$config_file" ]]; then
            log "Super-LIO config not found: $config_file"
            exit 1
        fi

        local super_lio_root
        super_lio_root="$(resolve_super_lio_source_root)"
        if [[ ! -d "$super_lio_root" ]]; then
            log "Super-LIO source root not found: $super_lio_root"
            exit 1
        fi

        local pcd_dir
        local rel_pcd_dir
        pcd_dir="$(dirname "$pcd_file")"
        rel_pcd_dir="$(relative_path "$super_lio_root" "$pcd_dir")"

        log "using relocation PCD: $pcd_file"
        start_in_process_group RELOCATION_PID \
            ros2 run "$SUPER_LIO_PACKAGE" relocation_node \
            --ros-args --log-level info \
            -r __node:=relocation_node \
            --params-file "$config_file" \
            -p lio.map.save_map_dir:="$rel_pcd_dir" \
            -p lio.map.map_name:="$(basename "$pcd_file")"

        if [[ "$RVIZ_FLAG" == "true" || "$RVIZ_FLAG" == "1" ]]; then
            local prefix
            prefix="$(resolve_super_lio_prefix)"
            if [[ -n "$prefix" && -f "$prefix/share/$SUPER_LIO_PACKAGE/rviz/relocation.rviz" ]]; then
                start_in_process_group RVIZ_PID \
                    ros2 run rviz2 rviz2 \
                    -d "$prefix/share/$SUPER_LIO_PACKAGE/rviz/relocation.rviz" \
                    --ros-args --log-level warn
            fi
        fi
        return
    fi

    start_in_process_group RELOCATION_PID ros2 launch "$SUPER_LIO_PACKAGE" "$SUPER_LIO_LAUNCH" "rviz:=${RVIZ_FLAG}"
}

cleanup() {
    local exit_code=$?

    if [[ "$CLEANED_UP" -eq 1 ]]; then
        exit "$exit_code"
    fi

    CLEANED_UP=1
    trap - EXIT INT TERM

    stop_process_group "$LIVOX_PID" "livox driver"
    stop_process_group "$RELOCATION_PID" "localization process"
    stop_process_group "$ODOM_TO_TF_PID" "odom_to_tf"
    stop_process_group "$RVIZ_PID" "rviz"
    wait 2>/dev/null || true
    exit "$exit_code"
}

trap cleanup EXIT INT TERM

log "starting ${LIVOX_PACKAGE} ${LIVOX_LAUNCH}"
start_in_process_group LIVOX_PID ros2 launch "$LIVOX_PACKAGE" "$LIVOX_LAUNCH"

log "waiting for steady /livox/imu before starting ${SUPER_LIO_PACKAGE}"
if ros2 run g1_base wait_imu_steady --ros-args \
    -p timeout_sec:="$IMU_STEADY_TIMEOUT" \
    -p gyro_max_rad_s:="$IMU_STEADY_GYRO_MAX_RAD_S" \
    -p accel_min_g:="$IMU_STEADY_ACCEL_MIN_G" \
    -p accel_max_g:="$IMU_STEADY_ACCEL_MAX_G" \
    -p gyro_var_max:="$IMU_STEADY_GYRO_VAR_MAX" \
    -p accel_var_max:="$IMU_STEADY_ACCEL_VAR_MAX"; then
    :
else
    imu_gate_rc=$?
    if (( imu_gate_rc == 4 )); then
        log "IMU steady gate failed; 请保持机器人静止后重试"
        exit 4
    fi
    log "IMU steady gate process exited with code ${imu_gate_rc}"
    exit 1
fi

log "starting ${SUPER_LIO_PACKAGE} ${SUPER_LIO_LAUNCH}"
start_relocation

if [[ "$START_ODOM_TO_TF" == "true" || "$START_ODOM_TO_TF" == "1" ]]; then
    log "starting g1_base odom_to_tf"
    start_in_process_group ODOM_TO_TF_PID ros2 run g1_base odom_to_tf
fi

log "localization processes started; navigation_manager will wait for LIO topics"
log "press Ctrl+C to stop child process groups"

while true; do
    if ! is_alive "$RELOCATION_PID"; then
        log "${SUPER_LIO_LAUNCH} exited"
        exit 1
    fi

    if ! is_alive "$LIVOX_PID"; then
        log "${LIVOX_LAUNCH} exited"
        exit 1
    fi

    sleep 2
done

#!/usr/bin/env bash

# start_pc2_mapping.sh — 建图模式启动脚本
# 与 start_pc2_localization.sh 的关键区别：
# cleanup 时先对 super_lio (ros2 launch) 发 SIGINT 并等待保存完成，
# 再清理其他进程。

set -euo pipefail

ROOT_DIR="${G1_BASE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export G1_BASE_ROOT="$ROOT_DIR"

source "$ROOT_DIR/config/robot_env.sh"

SUPER_LIO_PACKAGE="${SUPER_LIO_PACKAGE:-super_lio}"
SUPER_LIO_LAUNCH="${SUPER_LIO_LAUNCH:-Livox_mid360.py}"
LIVOX_PACKAGE="${LIVOX_PACKAGE:-livox_ros_driver2}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360_launch.py}"
RVIZ_FLAG="${RVIZ_FLAG:-false}"
MAP_SAVE_TIMEOUT="${MAP_SAVE_TIMEOUT:-30}"
IMU_STEADY_TIMEOUT="${IMU_STEADY_TIMEOUT:-10.0}"
IMU_STEADY_GYRO_MAX_RAD_S="${IMU_STEADY_GYRO_MAX_RAD_S:-0.05}"
IMU_STEADY_ACCEL_MIN_G="${IMU_STEADY_ACCEL_MIN_G:-0.950}"
IMU_STEADY_ACCEL_MAX_G="${IMU_STEADY_ACCEL_MAX_G:-1.050}"
IMU_STEADY_GYRO_VAR_MAX="${IMU_STEADY_GYRO_VAR_MAX:-1e-4}"
IMU_STEADY_ACCEL_VAR_MAX="${IMU_STEADY_ACCEL_VAR_MAX:-8e-4}"

load_ros_env

MAPPING_PID=""
LIVOX_PID=""
CLEANED_UP=0

log() {
    printf '[pc2-mapping] %s\n' "$*"
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

cleanup() {
    local exit_code=$?

    if [[ "$CLEANED_UP" -eq 1 ]]; then
        exit "$exit_code"
    fi

    CLEANED_UP=1
    trap - EXIT INT TERM

    # 关键：先对 ros2 launch (super_lio) 发 SIGINT，等待地图保存
    if is_alive "$MAPPING_PID"; then
        log "sending SIGINT to mapping process (pid=$MAPPING_PID), waiting for map save..."
        signal_process_group "$MAPPING_PID" INT
        if ! wait_until_stopped "$MAPPING_PID" "$MAP_SAVE_TIMEOUT"; then
            log "mapping process did not exit in ${MAP_SAVE_TIMEOUT}s, force killing"
            signal_process_group "$MAPPING_PID" KILL
        else
            log "mapping process exited"
        fi
        wait "$MAPPING_PID" 2>/dev/null || true
    fi

    # 检查 map.pcd 是否保存成功
    local map_dir
    map_dir="$(ros2 pkg prefix super_lio 2>/dev/null || echo '')/../../src/Super-LIO/src/super_lio/map"
    local map_file="${MAPPING_MAP_DIR:-${map_dir}}/map.pcd"
    if [[ -f "$map_file" ]]; then
        local file_age=$(( $(date +%s) - $(stat -c %Y "$map_file") ))
        if (( file_age < 60 )); then
            log "✓ map saved: $map_file ($(stat -c %s "$map_file") bytes, ${file_age}s ago)"
        else
            log "✗ map.pcd exists but was not updated (last modified ${file_age}s ago)"
        fi
    else
        log "✗ map.pcd not found at $map_file"
    fi

    # 清理其他进程
    stop_process_group "$LIVOX_PID" "livox driver"

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
start_in_process_group MAPPING_PID ros2 launch "$SUPER_LIO_PACKAGE" "$SUPER_LIO_LAUNCH" "rviz:=${RVIZ_FLAG}"

log "mapping processes started; navigation_manager will wait for LIO topics"
log "walk the robot to build the map, then stop with Ctrl+C or stop_mapping service"

while true; do
    if ! is_alive "$MAPPING_PID"; then
        log "mapping process exited"
        exit 1
    fi

    if ! is_alive "$LIVOX_PID"; then
        log "livox driver exited"
        exit 1
    fi

    sleep 2
done

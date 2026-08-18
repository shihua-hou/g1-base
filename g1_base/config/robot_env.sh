#!/usr/bin/env bash

# Shared runtime environment for manual launch and system autostart.

: "${G1_BASE_ROOT:=}"

if [[ -z "${G1_BASE_ROOT}" ]]; then
    G1_BASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

: "${G1_USER_HOME:=${HOME:-/home/$(id -un)}}"
: "${LIO_WORKSPACE_ROOT:=$G1_USER_HOME/ros2_ws}"

: "${ROS_SETUP:=/opt/ros/humble/setup.bash}"
: "${LIO_SETUP:=$LIO_WORKSPACE_ROOT/install/setup.bash}"
: "${WORKSPACE_SETUP:=$G1_BASE_ROOT/install/setup.bash}"
: "${G1_INTERFACES_SETUP:=$G1_USER_HOME/g1_ws/install/setup.bash}"
: "${CYCLONEDDS_HOME:=$G1_USER_HOME/cyclonedds/install}"
# 地图目录：容器里必须落在数据卷上。G1_DATA_DIR 由镜像的 ENV 提供（挂载点），
# 裸机没有这个变量，仍然回落到 ~/g1_maps，行为不变。
# 这里如果直接写死 $G1_USER_HOME/g1_maps，容器里 HOME=/root，地图就存进了
# 容器可写层 —— 每次 docker compose up -d 重建容器，现场建的图全丢，
# 而且网关/navigation_manager/start_pc2_localization.sh 三处都以本变量优先，
# 它们各自的 G1_DATA_DIR 分支会变成永远走不到的死代码。
if [[ -n "${G1_DATA_DIR:-}" ]]; then
    : "${G1_MAPS_DIR:=$G1_DATA_DIR/maps}"
fi
: "${G1_MAPS_DIR:=$G1_USER_HOME/g1_maps}"

: "${ROS_DOMAIN_ID:=42}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
: "${ROS_LOCALHOST_ONLY:=0}"
: "${G1_DDS_INTERFACES:=enP8p1s0,wlp3s0}"

export G1_BASE_ROOT
export G1_USER_HOME
export LIO_WORKSPACE_ROOT
export ROS_SETUP
export LIO_SETUP
export WORKSPACE_SETUP
export G1_INTERFACES_SETUP
export CYCLONEDDS_HOME
export G1_MAPS_DIR
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export ROS_LOCALHOST_ONLY
export G1_DDS_INTERFACES

if [[ -d "${CYCLONEDDS_HOME}/lib" ]]; then
    case ":${LD_LIBRARY_PATH:-}:" in
        *":${CYCLONEDDS_HOME}/lib:"*) ;;
        *)
            export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
            ;;
    esac
fi

if [[ -z "${CYCLONEDDS_URI:-}" && -n "${G1_DDS_INTERFACES}" ]]; then
    IFS=',' read -r -a _g1_dds_ifaces <<< "${G1_DDS_INTERFACES}"
    _g1_cyclonedds_ifaces=''
    _g1_append_iface() {
        local _candidate="$1"
        [[ -n "${_candidate}" && -d "/sys/class/net/${_candidate}" ]] || return 0
        case "${_g1_cyclonedds_ifaces}" in
            *"name=\"${_candidate}\""*) return 0 ;;
        esac
        _g1_cyclonedds_ifaces+="<NetworkInterface name=\"${_candidate}\"/>"
    }
    for _g1_iface in "${_g1_dds_ifaces[@]}"; do
        _g1_iface="${_g1_iface//[[:space:]]/}"
        if [[ -z "${_g1_iface}" ]]; then
            continue
        fi
        # 指定的网卡不存在就跳过；不再做 wl*/en* 同前缀回退，
        # 否则会把无关的 WiFi/有线网卡静默拉进 DDS。
        _g1_append_iface "${_g1_iface}"
    done
    if [[ -n "${_g1_cyclonedds_ifaces}" ]]; then
        # AllowMulticast=spdp：保留参与者发现多播（Nav2 节点数易超过默认
        # MaxAutoParticipantIndex=9，关掉多播会导致发现失败），但禁用数据多播
        # （ROS 2 reliable QoS 默认走单播，关掉不影响业务）。
        export CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast><Interfaces>${_g1_cyclonedds_ifaces}</Interfaces></General></Domain></CycloneDDS>"
    fi
    unset _g1_dds_ifaces
    unset _g1_iface
    unset -f _g1_append_iface
    unset _g1_cyclonedds_ifaces
fi

load_ros_env() {
    set +u
    source "${ROS_SETUP}"
    if [[ -f "${LIO_SETUP}" ]]; then
        source "${LIO_SETUP}"
    fi
    if [[ -f "${WORKSPACE_SETUP}" ]]; then
        source "${WORKSPACE_SETUP}"
    fi
    if [[ -f "${G1_INTERFACES_SETUP}" ]]; then
        source "${G1_INTERFACES_SETUP}"
    fi
    set -u
}

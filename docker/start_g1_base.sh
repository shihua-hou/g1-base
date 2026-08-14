#!/bin/bash
# 容器主进程：起底座三件套 + 网页网关。
#
# 与基础镜像自带的 /start_g1_base.sh 的区别：
#   1. 多起一个 g1_web_bridge（网页控制台）
#   2. 网页网关单独看护、崩了自动重启，不会把整个容器带下去
#      —— 控制台挂掉不该让正在导航的机器人停机
#   3. 首次启动把包内的 routes / maps 播种到数据卷，保证新卷不是空的
set -uo pipefail

G1_LIB=/root/g1_ws/install/g1_base/lib/g1_base
G1_SHARE=/root/g1_ws/install/g1_base/share/g1_base
G1_DATA_DIR="${G1_DATA_DIR:-/data}"
G1_WEB_PORT="${G1_WEB_PORT:-8081}"
# 网页网关只用它显示"机器人 IP"，默认跟 DDS 用同一张网卡
G1_NET_IF="${G1_NET_IF:-${G1_DDS_INTERFACES:-enP8p1s0}}"

ALL_PY_PATHS=$(find /root/g1_ws/install -path "*/lib/python3.10/site-packages" -type d 2>/dev/null | tr "\n" ":")
export PYTHONPATH="${ALL_PY_PATHS}/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
PY="conda run --no-capture-output -n bt_env env PYTHONPATH=${PYTHONPATH} python"

# ── 数据卷播种 ──
# 新挂的空卷如果不播种，现场会看到"没有任何地图和路线"，
# 而包里其实是带了出厂数据的。只在目标不存在时拷，绝不覆盖现场数据。
seed_data() {
    mkdir -p "${G1_DATA_DIR}"
    for item in routes maps movement; do
        if [[ ! -e "${G1_DATA_DIR}/${item}" && -e "${G1_SHARE}/config/${item}" ]]; then
            echo "[start] 播种 ${item} → ${G1_DATA_DIR}/${item}"
            cp -rL "${G1_SHARE}/config/${item}" "${G1_DATA_DIR}/${item}"
        fi
    done
    if [[ ! -e "${G1_DATA_DIR}/walking_mode.yaml" && -e "${G1_SHARE}/config/walking_mode.yaml" ]]; then
        cp -L "${G1_SHARE}/config/walking_mode.yaml" "${G1_DATA_DIR}/walking_mode.yaml"
    fi
}

run_node() {
    if [[ -f "$G1_LIB/$1" ]]; then
        $PY "$G1_LIB/$1" "${@:2}"
    else
        echo "[start] $G1_LIB/$1 not found, fallback to: ros2 run g1_base $1"
        ros2 run g1_base "$@"
    fi
}

seed_data

echo "[start] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} RMW=${RMW_IMPLEMENTATION:-unset}"
echo "[start] 数据目录=${G1_DATA_DIR} 网页端口=${G1_WEB_PORT} 网卡=${G1_NET_IF}"

run_node navigation_manager &
NAV_PID=$!

# 网页网关先起：它不依赖 DDS 桥，而下面那个桥就绪等待最长要 30 秒。
# 放在后面的话，现场重启后要干等半分钟才能打开页面。
# 底座没就绪时页面照样能开，只是显示"导航栈未就绪"——这恰恰是排障时最想看的。
# 崩了就地重启，不拖累底座：控制台挂掉不该让正在导航的机器人停机。
web_bridge_loop() {
    while true; do
        run_node g1_web_bridge --net-if "${G1_NET_IF}" --port "${G1_WEB_PORT}"
        code=$?
        echo "[start] g1_web_bridge 退出 (code=${code})，3 秒后重启"
        sleep 3
    done
}
web_bridge_loop &
WEB_PID=$!

echo "[start] Launching DDS domain bridge (domain 0 -> 42)..."
$PY /usr/local/bin/dds_domain_bridge.py > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
for i in $(seq 1 30); do
    grep -q "Bridge publisher ready on domain 42" /tmp/bridge.log 2>/dev/null && break
    sleep 1
done

run_node g1_control_server &
CTL_PID=$!

stop_all() {
    kill -TERM $NAV_PID $BRIDGE_PID $CTL_PID $WEB_PID 2>/dev/null || true
    # 循环里的子进程要一起收掉
    pkill -TERM -f g1_web_bridge 2>/dev/null || true
}
trap stop_all TERM INT

# 只有底座三件套是"命脉"，任何一个死了就整体退出交给 docker restart 处理
while kill -0 $NAV_PID 2>/dev/null && kill -0 $BRIDGE_PID 2>/dev/null && kill -0 $CTL_PID 2>/dev/null; do
    sleep 2
done

DEAD_NAME=""
DEAD_PID=""
EXIT_CODE=0
if ! kill -0 $NAV_PID 2>/dev/null; then
    DEAD_NAME=navigation_manager; DEAD_PID=$NAV_PID
elif ! kill -0 $BRIDGE_PID 2>/dev/null; then
    DEAD_NAME=bridge; DEAD_PID=$BRIDGE_PID
elif ! kill -0 $CTL_PID 2>/dev/null; then
    DEAD_NAME=g1_control_server; DEAD_PID=$CTL_PID
fi
if [[ -n "$DEAD_PID" ]]; then
    wait $DEAD_PID 2>/dev/null; EXIT_CODE=$?
fi
echo "[start] $DEAD_NAME exited (code=$EXIT_CODE), shutting down others"
stop_all
wait
exit $EXIT_CODE

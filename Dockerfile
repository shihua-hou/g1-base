# G1 底座 + 网页控制台应用镜像
#
# 在厂商给的基础镜像上叠一层：编译本仓库的四个包，并把 g1_web_bridge
# 加进启动列表（基础镜像的 /start_g1_base.sh 只起 navigation_manager /
# dds_domain_bridge / g1_control_server，没有网页网关）。
#
# 必须在 arm64 机器上构建（Jetson 本机或那块 aarch64 开发板）。
# 在 x86 开发机上 build 出来的镜像 Jetson 跑不了。
#
#   docker build -t g1-base:$(git rev-parse --short HEAD) .
#
ARG BASE_IMAGE=hub-nj.iwhalecloud.com/nexrobot/ros2-unitree-g1-base:C_202607311711
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

# 排障用的基础网络工具：基础镜像里连 ip 都没有，现场查网卡很难受
RUN (apt-get update && apt-get install -y --no-install-recommends iproute2 iputils-ping \
     && rm -rf /var/lib/apt/lists/*) || echo "[build] apt 不可用，跳过网络工具（不影响运行）"

# ── LIO 链路：Livox 驱动 + Super-LIO ──
# 放在业务源码之前：这三个第三方包很少动，让它们独占一层，
# 之后改我们自己的代码不用重编 SLAM（省 10+ 分钟）。
# 基础镜像里 glog / tbb / PCL 1.12 / Eigen3 / g++11(C++20) 都齐，无需 apt。
COPY third_party/Livox-SDK2        /root/lio_ws/Livox-SDK2
COPY third_party/livox_ros_driver2 /root/lio_ws/src/livox_ros_driver2
# 保持 src/Super-LIO/src/super_lio 这个层级：start_pc2_mapping.sh 按
# "$(ros2 pkg prefix super_lio)/../../src/Super-LIO/src/super_lio/map" 找地图
COPY third_party/Super-LIO         /root/lio_ws/src/Super-LIO

RUN set -exo pipefail; \
    export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd: -)"; \
    source /opt/ros/humble/setup.bash; \
    # 1) Livox-SDK2：驱动的底层依赖，装到 /usr/local
    cmake -S /root/lio_ws/Livox-SDK2 -B /root/lio_ws/Livox-SDK2/build \
          -DCMAKE_BUILD_TYPE=Release; \
    cmake --build /root/lio_ws/Livox-SDK2/build -j"$(nproc)"; \
    cmake --install /root/lio_ws/Livox-SDK2/build; \
    ldconfig; \
    # 2) livox_ros_driver2 的 ROS2 构建约定：换 package.xml、启用 launch_ROS2
    #    （官方 build.sh 就是干这两件事，这里照做但不调它，免得它顺手 colcon build 整个空间）
    cd /root/lio_ws/src/livox_ros_driver2; \
    cp -f package_ROS2.xml package.xml; \
    rm -rf launch && cp -rf launch_ROS2 launch; \
    cd /root/lio_ws; \
    colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
                 --event-handlers console_direct+; \
    test -f /root/lio_ws/install/super_lio/share/super_lio/config/livox_360.yaml; \
    test -d /root/lio_ws/install/livox_ros_driver2; \
    rm -rf /root/lio_ws/build /root/lio_ws/log

# 本机的 MID360 网络配置（雷达把点云推到 host_net_info 里的 IP）
COPY docker/lio/MID360_config.json /root/lio_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
# Super-LIO 运行参数：上游默认 save_map:false，建完图不落盘，必须覆盖
COPY docker/lio/livox_360.yaml /root/lio_ws/install/super_lio/share/super_lio/config/livox_360.yaml

# 源码放在最后几层：改代码只重建这薄薄一层，8GB 的基础层机器人本地复用
COPY g1_base                /root/g1_ws/src/g1_base
COPY g1_base_perception     /root/g1_ws/src/g1_base_perception
COPY g1_centerline_planner  /root/g1_ws/src/g1_centerline_planner

# 注意：这里不能开 set -u。ROS 的 setup.bash 会引用未定义变量
# （AMENT_TRACE_SETUP_FILES 之类），开了 nounset 直接就挂。
RUN set -exo pipefail; \
    # colcon 认出 g1_base 之后就不再向下递归，嵌套的 interfaces 得单独暴露一次
    ln -sfn /root/g1_ws/src/g1_base/g1_base_interfaces /root/g1_ws/src/g1_base_interfaces; \
    # 关键：PATH 上默认是 conda base(python 3.14)，而 ROS Humble 的 rclpy 是 cp310。
    # 用系统 python3.10 编译，产物落在 lib/python3.10/site-packages，
    # 正好和运行期的 bt_env(3.10) 对得上。
    export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd: -)"; \
    python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version'; \
    source /opt/ros/humble/setup.bash; \
    # 基础镜像里带的是旧版 g1_base，先清干净，避免旧资源清单残留
    rm -rf /root/g1_ws/build /root/g1_ws/log \
           /root/g1_ws/install/g1_base /root/g1_ws/install/g1_base_perception \
           /root/g1_ws/install/g1_base_interfaces /root/g1_ws/install/g1_centerline_planner; \
    cd /root/g1_ws; \
    colcon build \
        --packages-select g1_base_interfaces g1_centerline_planner g1_base_perception g1_base \
        --event-handlers console_direct+; \
    # 编完立刻校验网页资源真的进了 install —— 这是最容易悄悄漏掉的一步
    test -f /root/g1_ws/install/g1_base/share/g1_base/webapp/index.html; \
    test -f /root/g1_ws/install/g1_base/share/g1_base/webapp/assets/g1_meshes.bin; \
    # 镜像里不需要 colcon 的中间产物
    rm -rf /root/g1_ws/build /root/g1_ws/log

COPY docker/start_g1_base.sh /start_g1_base.sh
RUN chmod +x /start_g1_base.sh

# 运行时可写数据统一落到数据卷，容器重建不丢现场的图和路线。
# LIO_WORKSPACE_ROOT 是 config/robot_env.sh 早就留好的钩子：
# load_ros_env 会 source $LIO_WORKSPACE_ROOT/install/setup.bash，
# 这样 start_pc2_mapping.sh 里的 ros2 launch super_lio / livox_ros_driver2 才解析得到。
ENV G1_DATA_DIR=/data \
    G1_WEB_PORT=8081 \
    LIO_WORKSPACE_ROOT=/root/lio_ws

EXPOSE 8081

# ENTRYPOINT 继承自基础镜像（负责按 G1_DDS_INTERFACES 绑定 CycloneDDS + 激活 bt_env）
CMD ["/start_g1_base.sh"]

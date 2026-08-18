"""
g1_base_manager — 统一入口
在同一进程中运行 G1ControlServer + NavigationManager，共享 MultiThreadedExecutor。
bot_mind 作为父进程拉起本模块，管理所有 SDK 调用和导航生命周期。

G1ControlServer:  Unitree SDK (arm + loco) + ROS2 service servers
NavigationManager: 定位 + Nav2 进程生命周期管理 + 健康状态发布
"""
import argparse

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from g1_base.g1_control_server import G1ControlServer, parse_control_server_args
from g1_base.navigation_manager import NavigationManager, parse_args as parse_nav_args


def parse_manager_args(argv=None):
    parser = argparse.ArgumentParser(
        description="G1 Base Manager: control server + navigation manager"
    )
    # G1ControlServer args
    parser.add_argument("--net-if", default="enP8p1s0",
                        help="Unitree SDK network interface")
    parser.add_argument("--control-node-name", default="g1_control_server",
                        help="ROS2 node name for control server")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")

    # NavigationManager args
    parser.add_argument("--auto-ensure", action="store_true", default=True,
                        help="Auto ensure navigation stack on startup")
    parser.add_argument("--nav-node-name", default="navigation_manager",
                        help="ROS2 node name for navigation manager")
    parser.add_argument("--monitor-hz", type=float, default=2.0)
    parser.add_argument("--freshness-window", type=float, default=3.0)
    parser.add_argument("--localization-timeout", type=float, default=60.0)
    parser.add_argument("--navigation-timeout", type=float, default=60.0)
    parser.add_argument("--max-bringup-attempts", type=int, default=2)
    parser.add_argument("--pointcloud-topic", default="/lio/cloud_world")
    parser.add_argument("--relocal-odom-topic", default="/lio/odom")
    parser.add_argument("--odom-topic", default="/odom_2d")

    # Executor args
    parser.add_argument("--threads", type=int, default=6,
                        help="Number of executor threads")

    return parser.parse_known_args(argv)


def _build_control_args(args):
    """Convert unified args to G1ControlServer-compatible namespace."""
    return argparse.Namespace(
        net_if=args.net_if,
        node_name=args.control_node_name,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
    )


def _build_nav_args(args):
    """Convert unified args to NavigationManager-compatible namespace."""
    return argparse.Namespace(
        auto_ensure=args.auto_ensure,
        node_name=args.nav_node_name,
        monitor_hz=args.monitor_hz,
        freshness_window=args.freshness_window,
        localization_timeout=args.localization_timeout,
        navigation_timeout=args.navigation_timeout,
        max_bringup_attempts=args.max_bringup_attempts,
        pointcloud_topic=args.pointcloud_topic,
        relocal_odom_topic=args.relocal_odom_topic,
        odom_topic=args.odom_topic,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
    )


def main(argv=None):
    args, ros_args = parse_manager_args(argv)
    rclpy.init(args=ros_args)

    control_args = _build_control_args(args)
    nav_args = _build_nav_args(args)

    control_server = None
    nav_manager = None
    executor = MultiThreadedExecutor(num_threads=args.threads)

    try:
        control_server = G1ControlServer(control_args)
        executor.add_node(control_server)

        nav_manager = NavigationManager(nav_args)
        executor.add_node(nav_manager)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if nav_manager is not None:
            try:
                nav_manager.shutdown()
            except Exception:
                pass
        if control_server is not None:
            try:
                control_server.shutdown()
            except Exception:
                pass
        try:
            executor.shutdown()
        except Exception:
            pass
        if nav_manager is not None:
            try:
                nav_manager.destroy_node()
            except Exception:
                pass
        if control_server is not None:
            try:
                control_server.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

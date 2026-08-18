import argparse
import json
import os
import select
import sys
import termios
import time
import tty

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from g1_base.common import (
    PACKAGE_NAME,
    package_share_dir,
    quaternion_dict_from_yaw,
    route_file,
    yaw_from_quaternion_msg,
)


DEFAULT_RECORD_OUTPUT = os.path.join(
    package_share_dir(), "config", "routes", "recorded_waypoints.yaml"
)


def yaw_deg_from_quaternion(quaternion):
    return yaw_from_quaternion_msg(quaternion) * 180.0 / 3.141592653589793


def format_stamp(stamp_msg):
    if stamp_msg is None:
        return time.strftime("%Y-%m-%d %H:%M:%S")
    total = stamp_msg.sec + stamp_msg.nanosec / 1e9
    if total <= 0.0:
        return time.strftime("%Y-%m-%d %H:%M:%S")
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(total))


def clear_status_line():
    sys.stdout.write("\r" + (" " * 160) + "\r")
    sys.stdout.flush()


def print_message(text):
    clear_status_line()
    print(text, flush=True)


class CbreakTerminal:
    def __init__(self, stream):
        self.stream = stream
        self.fd = None
        self.old_settings = None

    def __enter__(self):
        if not self.stream.isatty():
            raise RuntimeError("当前终端不是 TTY，无法使用按键采点模式。")
        self.fd = self.stream.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.fd is not None and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


class PoseViewerNode(Node):
    def __init__(self, map_frame, base_frame, odom_topic, robo_odom_topic):
        super().__init__("show_robot_pose")
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom_2d = None
        self.latest_robo_odom = None

        self.create_subscription(Odometry, odom_topic, self._odom_2d_cb, 10)
        self.create_subscription(Odometry, robo_odom_topic, self._robo_odom_cb, 10)

    def _odom_2d_cb(self, msg):
        self.latest_odom_2d = msg

    def _robo_odom_cb(self, msg):
        self.latest_robo_odom = msg

    def get_tf_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except TransformException:
            return None

        q = transform.transform.rotation
        yaw_rad = yaw_from_quaternion_msg(q)
        return {
            "stamp": transform.header.stamp,
            "source": f"tf {self.map_frame}->{self.base_frame}",
            "frame": self.map_frame,
            "x": transform.transform.translation.x,
            "y": transform.transform.translation.y,
            "z": transform.transform.translation.z,
            "yaw_rad": yaw_rad,
            "yaw_deg": yaw_rad * 180.0 / 3.141592653589793,
        }

    def get_topic_pose(self):
        for topic_name, msg in (
            ("/odom_2d", self.latest_odom_2d),
            ("/lio/odom", self.latest_robo_odom),
        ):
            if msg is None:
                continue
            return {
                "stamp": msg.header.stamp,
                "source": f"topic {topic_name}",
                "frame": msg.header.frame_id or "unknown",
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "yaw_rad": yaw_from_quaternion_msg(msg.pose.pose.orientation),
                "yaw_deg": yaw_deg_from_quaternion(msg.pose.pose.orientation),
            }
        return None

    def get_pose(self, allow_fallback=True):
        tf_pose = self.get_tf_pose()
        if tf_pose is not None:
            return tf_pose
        if allow_fallback:
            return self.get_topic_pose()
        return None


def build_waypoint_entry(pose, index, action_id, say_text, digits):
    return {
        "index": index,
        "x": round(pose["x"], digits),
        "y": round(pose["y"], digits),
        "yaw_deg": round(pose["yaw_deg"], digits),
        "action_id": action_id,
        "say_text": say_text,
    }


def save_recorded_waypoints(output_path, route_name, waypoints):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    payload = {"route_name": route_name, "waypoints": waypoints}
    with open(output_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)


def parse_args():
    parser = argparse.ArgumentParser(
        description="终端查看机器人当前位置，优先使用 map->base_link TF。"
    )
    parser.add_argument("--frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-topic", default="/odom_2d")
    parser.add_argument("--relocal-odom-topic", default="/lio/odom")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--record-yaml", nargs="?", const=DEFAULT_RECORD_OUTPUT, default=None)
    parser.add_argument("--route-name", default="default")
    parser.add_argument("--action-id", type=int, default=31)
    parser.add_argument("--say-text", default="到达指定地点")
    parser.add_argument("--precision", type=int, default=3)
    return parser.parse_known_args()


def print_pose(pose):
    print(
        f"{format_stamp(pose['stamp'])} | {pose['source']} | frame={pose['frame']} | "
        f"x={pose['x']:.3f} y={pose['y']:.3f} z={pose['z']:.3f} | "
        f"yaw={pose['yaw_deg']:.1f} deg ({pose['yaw_rad']:.3f} rad)",
        flush=True,
    )


def run_record_mode(args, ros_args):
    rclpy.init(args=ros_args)
    node = PoseViewerNode(
        args.frame, args.base_frame, args.odom_topic, args.relocal_odom_topic
    )
    recorded = []
    start = time.time()
    try:
        while time.time() - start < args.timeout and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_pose() is not None:
                break
        with CbreakTerminal(sys.stdin):
            print_message("按 r / 空格 / 回车记录当前位姿，按 q 退出并保存。")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=max(0.1, 1.0 / max(args.rate, 0.2)))
                pose = node.get_pose()
                if pose is not None:
                    sys.stdout.write(
                        "\r"
                        + (
                            f"{pose['source']} | x={pose['x']:.3f} y={pose['y']:.3f} "
                            f"yaw={pose['yaw_deg']:.1f}deg"
                        ).ljust(120)
                    )
                    sys.stdout.flush()
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not ready:
                    continue
                chars = sys.stdin.read(1)
                if chars.lower() == "q":
                    break
                if chars.lower() in ("r", " ") or chars == "\n":
                    if pose is None:
                        print_message("当前没有可用位姿，稍后再试。")
                        continue
                    entry = build_waypoint_entry(
                        pose, len(recorded) + 1, args.action_id, args.say_text, args.precision
                    )
                    recorded.append(entry)
                    print_message(f"已记录点位 #{entry['index']}: {json.dumps(entry, ensure_ascii=False)}")
    except KeyboardInterrupt:
        print_message("收到 Ctrl-C，退出并保存。")
    finally:
        clear_status_line()
        if recorded:
            save_recorded_waypoints(args.record_yaml, args.route_name, recorded)
            print(f"已保存 {len(recorded)} 个 waypoint -> {args.record_yaml}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    args, ros_args = parse_args()
    if args.record_yaml is not None:
        run_record_mode(args, ros_args)
        return

    rclpy.init(args=ros_args)
    node = PoseViewerNode(
        args.frame, args.base_frame, args.odom_topic, args.relocal_odom_topic
    )
    try:
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0.1, 1.0 / max(args.rate, 0.2)))
            pose = node.get_pose()
            if pose is not None:
                print_pose(pose)
                if args.once:
                    break
            elif args.once and time.time() - start >= args.timeout:
                print("未在超时前拿到位姿。", flush=True)
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

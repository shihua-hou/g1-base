import argparse
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from g1_base.common import load_waypoints_from_yaml, route_file


def quaternion_from_yaw_deg(yaw_deg):
    yaw_rad = math.radians(float(yaw_deg))
    half = yaw_rad / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def make_color(r, g, b, a=1.0):
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


def build_path(frame_id, stamp, waypoints):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for waypoint in waypoints:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.position.z = 0.02
        q = quaternion_from_yaw_deg(waypoint["yaw_deg"])
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        path.poses.append(pose)
    return path


def add_delete_all(marker_array, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)


def add_line_strip(marker_array, frame_id, stamp, waypoints):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "route_line"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.08
    marker.color = make_color(0.10, 0.80, 0.95, 0.95)
    for waypoint in waypoints:
        point = Point()
        point.x = waypoint["x"]
        point.y = waypoint["y"]
        point.z = 0.03
        marker.points.append(point)
    marker_array.markers.append(marker)


def add_point_marker(marker_array, frame_id, stamp, waypoint, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "waypoint_points"
    marker.id = 1000 + waypoint["index"]
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = waypoint["x"]
    marker.pose.position.y = waypoint["y"]
    marker.pose.position.z = 0.12
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.28
    marker.scale.y = 0.28
    marker.scale.z = 0.28
    marker.color = color
    marker_array.markers.append(marker)


def add_heading_arrow(marker_array, frame_id, stamp, waypoint):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "waypoint_heading"
    marker.id = 2000 + waypoint["index"]
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = waypoint["x"]
    marker.pose.position.y = waypoint["y"]
    marker.pose.position.z = 0.18
    q = quaternion_from_yaw_deg(waypoint["yaw_deg"])
    marker.pose.orientation.x = q["x"]
    marker.pose.orientation.y = q["y"]
    marker.pose.orientation.z = q["z"]
    marker.pose.orientation.w = q["w"]
    marker.scale.x = 0.70
    marker.scale.y = 0.10
    marker.scale.z = 0.10
    marker.color = make_color(0.98, 0.55, 0.10, 0.95)
    marker_array.markers.append(marker)


def add_text_label(marker_array, frame_id, stamp, waypoint):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "waypoint_text"
    marker.id = 3000 + waypoint["index"]
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = waypoint["x"]
    marker.pose.position.y = waypoint["y"]
    marker.pose.position.z = 0.62
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.36
    marker.color = make_color(1.0, 1.0, 1.0, 0.95)
    marker.text = (
        f"{waypoint['index']} "
        f"({waypoint['x']:.2f}, {waypoint['y']:.2f}) "
        f"{waypoint['yaw_deg']:.1f}deg"
    )
    marker_array.markers.append(marker)


def build_marker_array(frame_id, stamp, route_name, waypoints):
    marker_array = MarkerArray()
    add_delete_all(marker_array, frame_id, stamp)
    add_line_strip(marker_array, frame_id, stamp, waypoints)

    for waypoint in waypoints:
        if waypoint["index"] == 1:
            color = make_color(0.15, 0.90, 0.25, 0.95)
        elif waypoint["index"] == len(waypoints):
            color = make_color(0.95, 0.20, 0.20, 0.95)
        else:
            color = make_color(0.15, 0.45, 0.98, 0.95)
        add_point_marker(marker_array, frame_id, stamp, waypoint, color)
        add_heading_arrow(marker_array, frame_id, stamp, waypoint)
        add_text_label(marker_array, frame_id, stamp, waypoint)

    title = Marker()
    title.header.frame_id = frame_id
    title.header.stamp = stamp
    title.ns = "route_title"
    title.id = 5000
    title.type = Marker.TEXT_VIEW_FACING
    title.action = Marker.ADD
    title.pose.position.x = waypoints[0]["x"]
    title.pose.position.y = waypoints[0]["y"]
    title.pose.position.z = 1.2
    title.pose.orientation.w = 1.0
    title.scale.z = 0.45
    title.color = make_color(0.95, 0.95, 0.95, 0.95)
    title.text = f"Route: {route_name}"
    marker_array.markers.append(title)
    return marker_array


class PublishWaypointsNode(Node):
    def __init__(self, args):
        super().__init__("publish_waypoints_to_rviz")
        self.frame_id = args.frame
        self.route_path = args.yaml
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 1)
        self.path_pub = self.create_publisher(Path, args.path_topic, 1)
        self.create_timer(max(0.5, args.period), self.publish_once)
        self.get_logger().info(f"已加载路线文件: {self.route_path}")

    def publish_once(self):
        resolved_path, route_name, waypoints = load_waypoints_from_yaml(self.route_path)
        stamp = self.get_clock().now().to_msg()
        path = build_path(self.frame_id, stamp, waypoints)
        marker_array = build_marker_array(self.frame_id, stamp, route_name, waypoints)
        self.path_pub.publish(path)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f"已发布 {len(waypoints)} 个 waypoint 到 RViz ({resolved_path})"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="发布 waypoint 路线到 RViz。")
    parser.add_argument("--yaml", default=route_file("recorded_waypoints.yaml"))
    parser.add_argument("--frame", default="map")
    parser.add_argument("--marker-topic", default="/waypoint_markers")
    parser.add_argument("--path-topic", default="/waypoint_path")
    parser.add_argument("--period", type=float, default=2.0)
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = PublishWaypointsNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

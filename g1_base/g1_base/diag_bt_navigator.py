import argparse

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

from g1_base.bt_tools import DEFAULT_NAV_TO_POSE_BT, inspect_bt_file, missing_bt_plugins
from g1_base.common import package_share_dir


class BtNavigatorDiagNode(Node):
    def __init__(self):
        super().__init__("diag_bt_navigator")
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.param_client = self.create_client(GetParameters, "/bt_navigator/get_parameters")

    def spin_for(self, timeout_sec):
        end = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_future(self, future, timeout_sec):
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and not future.done():
            self.spin_for(0.05)
            if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                raise TimeoutError("等待 future 超时")
        return future.result()

    def wait_for_service_optional(self, client, timeout_sec):
        return client.wait_for_service(timeout_sec=timeout_sec)

    @staticmethod
    def parameter_value_to_text(value):
        if value.type == ParameterType.PARAMETER_BOOL:
            return str(value.bool_value)
        if value.type == ParameterType.PARAMETER_INTEGER:
            return str(value.integer_value)
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return str(value.double_value)
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return str(list(value.byte_array_value))
        if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return str(list(value.bool_array_value))
        if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return str(list(value.integer_array_value))
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return str(list(value.double_array_value))
        if value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return str(list(value.string_array_value))
        return "<unset>"

    def run_report(self, bt_path):
        lines = []
        bt_info = inspect_bt_file(bt_path)

        lines.append("=" * 72)
        lines.append("diag_bt_navigator report")
        lines.append(f"bt_path={bt_info['path']}")
        lines.append(f"bt_exists={'yes' if bt_info['exists'] else 'no'}")
        lines.append(f"bt_resolved={bt_info['resolved']}")
        if bt_info["size"] is not None:
            lines.append(f"bt_size={bt_info['size']} bytes")
        if bt_info["xml_ok"]:
            lines.append(f"bt_xml_parse=ok")
            lines.append(f"bt_tags={bt_info['tags']}")
        else:
            lines.append(f"bt_xml_parse=error detail={bt_info['xml_error'] or 'unknown'}")

        navigate_ready = self.navigate_client.wait_for_server(timeout_sec=0.5)
        lines.append(f"navigate_to_pose_server_ready={'yes' if navigate_ready else 'no'}")

        if self.wait_for_service_optional(self.state_client, 0.5):
            try:
                response = self.wait_for_future(self.state_client.call_async(GetState.Request()), 2.0)
                lines.append(
                    f"bt_navigator_state={response.current_state.label} ({response.current_state.id})"
                )
            except Exception as exc:
                lines.append(f"bt_navigator_state=error detail={exc}")
        else:
            lines.append("bt_navigator_state=service_unavailable")

        plugin_lib_names = []
        if self.wait_for_service_optional(self.param_client, 0.5):
            try:
                request = GetParameters.Request()
                request.names = ["default_nav_to_pose_bt_xml", "plugin_lib_names", "navigators"]
                response = self.wait_for_future(self.param_client.call_async(request), 2.0)
                for name, value in zip(request.names, response.values):
                    lines.append(f"param[{name}]={self.parameter_value_to_text(value)}")
                    if name == "plugin_lib_names" and value.type == ParameterType.PARAMETER_STRING_ARRAY:
                        plugin_lib_names = list(value.string_array_value)
            except Exception as exc:
                lines.append(f"bt_navigator_params=error detail={exc}")
        else:
            lines.append("bt_navigator_params=service_unavailable")

        if bt_info["required_plugins"]:
            lines.append(f"bt_required_plugins={bt_info['required_plugins']}")
            if plugin_lib_names:
                lines.append(
                    f"bt_missing_plugins={missing_bt_plugins(bt_info['required_plugins'], plugin_lib_names)}"
                )
            else:
                lines.append("bt_missing_plugins=unknown")

        lines.append("=" * 72)
        print("\n".join(lines), flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Diagnose bt_navigator BT loading prerequisites.")
    parser.add_argument(
        "--bt",
        default=str(package_share_dir() / "behavior_trees" / DEFAULT_NAV_TO_POSE_BT),
        help="Behavior Tree XML path to verify.",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = BtNavigatorDiagNode()
    try:
        node.run_report(args.bt)
    finally:
        node.destroy_node()
        rclpy.shutdown()

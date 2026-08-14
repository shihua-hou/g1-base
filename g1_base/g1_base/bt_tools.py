from pathlib import Path
import xml.etree.ElementTree as ET


DEFAULT_NAV_TO_POSE_BT = "navigate_to_pose_forward_only.xml"
DEFAULT_NAV_THROUGH_POSES_BT = "navigate_through_poses_forward_only.xml"

# Nav2 BT nodes that are provided via plugin_lib_names rather than being BT.CPP builtins.
BT_NODE_PLUGIN_MAP = {
    "BackUp": "nav2_back_up_action_bt_node",
    "ClearEntireCostmap": "nav2_clear_costmap_service_bt_node",
    "ComputePathThroughPoses": "nav2_compute_path_through_poses_action_bt_node",
    "ComputePathToPose": "nav2_compute_path_to_pose_action_bt_node",
    "FollowPath": "nav2_follow_path_action_bt_node",
    "GoalUpdated": "nav2_goal_updated_condition_bt_node",
    "PipelineSequence": "nav2_pipeline_sequence_bt_node",
    "RateController": "nav2_rate_controller_bt_node",
    "RecoveryNode": "nav2_recovery_node_bt_node",
    "RoundRobin": "nav2_round_robin_node_bt_node",
    "Spin": "nav2_spin_action_bt_node",
    "Wait": "nav2_wait_action_bt_node",
}


def inspect_bt_file(bt_path):
    path = Path(bt_path)
    info = {
        "path": str(path),
        "exists": path.exists(),
        "resolved": str(path.resolve(strict=False)),
        "size": None,
        "xml_ok": False,
        "xml_error": "",
        "tags": [],
        "required_plugins": [],
    }

    if not info["exists"]:
        return info

    try:
        info["size"] = path.stat().st_size
    except Exception:
        pass

    try:
        tree = ET.parse(path)
        tags = sorted({element.tag for element in tree.iter() if isinstance(element.tag, str)})
        required_plugins = sorted(
            {BT_NODE_PLUGIN_MAP[tag] for tag in tags if tag in BT_NODE_PLUGIN_MAP}
        )
        info.update(
            {
                "xml_ok": True,
                "tags": tags,
                "required_plugins": required_plugins,
            }
        )
    except Exception as exc:
        info["xml_error"] = str(exc)

    return info


def missing_bt_plugins(required_plugins, loaded_plugins):
    loaded = set(loaded_plugins)
    return [plugin for plugin in required_plugins if plugin not in loaded]

import ast
import math
import os
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - runtime dependency on target robot
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - allows local import before build
    get_package_share_directory = None


PACKAGE_NAME = "g1_base"
REPO_ROOT = Path(__file__).resolve().parent.parent


def package_share_dir():
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(PACKAGE_NAME))
        except Exception:
            pass
    return REPO_ROOT


def config_file(*parts):
    return str(package_share_dir() / "config" / Path(*parts))


def route_file(name="default.yaml"):
    return config_file("routes", name)


def movement_dir():
    """Return the path to config/movement/ (动作资源目录)."""
    return config_file("movement")


def movement_file(*parts):
    """Return a file path under config/movement/."""
    return config_file("movement", *parts)


def high_level_action_file(name):
    filename = name if str(name).endswith(".jsonl") else f"{name}.jsonl"
    return movement_file("motions", filename)



def yaw_from_quaternion_msg(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def quaternion_dict_from_yaw(yaw):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _resolve_route_path(path):
    path_obj = Path(path)
    if path_obj.is_absolute():
        return str(path_obj)

    cwd_path = (Path.cwd() / path_obj).resolve()
    if cwd_path.exists():
        return str(cwd_path)

    repo_path = REPO_ROOT / path_obj
    if repo_path.exists():
        return str(repo_path)

    share_path = package_share_dir() / path_obj
    if share_path.exists():
        return str(share_path)

    return str(cwd_path)


def _parse_yaml_scalar(value):
    value = value.strip()
    if value == "":
        return None

    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in ("null", "none"):
        return None

    if value[0] in ("'", '"') and value[-1] == value[0]:
        return ast.literal_eval(value)

    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _strip_yaml_comment(line):
    in_single = False
    in_double = False
    for idx, char in enumerate(line):
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            return line[:idx].rstrip()
    return line.rstrip()


def _minimal_yaml_load(route_path):
    root = {}
    current_section = None
    current_item = None

    with open(route_path, "r", encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, start=1):
            line = _strip_yaml_comment(raw_line)
            if not line.strip():
                continue

            indent = len(line) - len(line.lstrip(" "))
            stripped = line.strip()

            if indent == 0:
                current_item = None
                if ":" not in stripped:
                    raise ValueError(f"{route_path}:{line_no} YAML 顶层格式错误")

                key, value = stripped.split(":", 1)
                key = key.strip()
                value = value.strip()
                if value:
                    root[key] = _parse_yaml_scalar(value)
                    current_section = None
                else:
                    root[key] = []
                    current_section = key
                continue

            if current_section is None:
                raise ValueError(f"{route_path}:{line_no} YAML 缩进位置不合法")

            if current_section != "waypoints":
                raise ValueError(
                    f"{route_path}:{line_no} 仅支持顶层标量和 waypoints 列表"
                )

            if indent == 2 and stripped.startswith("- "):
                item = {}
                remainder = stripped[2:].strip()
                if remainder:
                    if ":" not in remainder:
                        raise ValueError(f"{route_path}:{line_no} waypoint 行格式错误")
                    key, value = remainder.split(":", 1)
                    item[key.strip()] = _parse_yaml_scalar(value.strip())
                root[current_section].append(item)
                current_item = item
                continue

            if indent >= 4 and current_item is not None:
                if ":" not in stripped:
                    raise ValueError(f"{route_path}:{line_no} waypoint 字段格式错误")
                key, value = stripped.split(":", 1)
                current_item[key.strip()] = _parse_yaml_scalar(value.strip())
                continue

            raise ValueError(f"{route_path}:{line_no} YAML 结构不受支持")

    return root


def load_waypoints_from_yaml(route_path):
    resolved_path = _resolve_route_path(route_path)
    if not os.path.exists(resolved_path):
        raise FileNotFoundError(f"路线文件不存在: {resolved_path}")

    if yaml is not None:
        with open(resolved_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    else:
        data = _minimal_yaml_load(resolved_path)

    if not data:
        raise ValueError(f"路线文件为空: {resolved_path}")

    if isinstance(data, list):
        route_name = os.path.basename(resolved_path)
        raw_waypoints = data
    elif isinstance(data, dict):
        route_name = data.get("route_name") or data.get("name") or os.path.basename(
            resolved_path
        )
        raw_waypoints = data.get("waypoints")
    else:
        raise ValueError(f"路线文件格式不正确: {resolved_path}")

    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError(f"路线文件中缺少非空 waypoints 列表: {resolved_path}")

    waypoints = []
    for idx, item in enumerate(raw_waypoints, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"第 {idx} 个 waypoint 不是对象: {resolved_path}")

        try:
            x = float(item["x"])
            y = float(item["y"])
        except KeyError as exc:
            raise ValueError(f"第 {idx} 个 waypoint 缺少字段 {exc.args[0]}") from exc

        if "yaw" in item:
            yaw = float(item["yaw"])
        elif "yaw_deg" in item:
            yaw = math.radians(float(item["yaw_deg"]))
        else:
            raise ValueError(f"第 {idx} 个 waypoint 缺少 yaw 或 yaw_deg")

        waypoints.append(
            {
                "index": idx,
                "x": x,
                "y": y,
                "yaw": yaw,
                "yaw_deg": math.degrees(yaw),
                "action_id": int(item.get("action_id", 25)),
                "say_text": str(item.get("say_text", "你好")),
            }
        )

    return resolved_path, route_name, waypoints

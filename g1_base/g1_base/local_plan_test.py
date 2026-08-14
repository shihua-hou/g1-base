"""Offline harness for testing Nav2 global planners against a static map."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


COSTMAP_LETHAL = 254
COSTMAP_NO_INFORMATION = 255


@dataclass
class PoseSpec:
    x: float
    y: float
    yaw: float = 0.0

    @classmethod
    def from_mapping(cls, data: dict) -> "PoseSpec":
        return cls(float(data["x"]), float(data["y"]), float(data.get("yaw", 0.0)))

    @classmethod
    def from_cli(cls, raw: str) -> "PoseSpec":
        parts = [p.strip() for p in raw.split(",")]
        if len(parts) not in (2, 3):
            raise ValueError(f"expected x,y[,yaw], got {raw!r}")
        return cls(float(parts[0]), float(parts[1]), float(parts[2]) if len(parts) == 3 else 0.0)

    def to_pose_stamped(self, frame_id: str, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        half = self.yaw * 0.5
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        return msg


@dataclass
class TestCase:
    name: str
    start: PoseSpec
    goal: PoseSpec
    description: str = ""


class CostmapCache:
    def __init__(self) -> None:
        self.grid: Optional[np.ndarray] = None
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

    def update(self, msg: Costmap) -> None:
        meta = msg.metadata
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        expected = int(meta.size_x) * int(meta.size_y)
        if data.size != expected:
            return
        self.grid = data.reshape((int(meta.size_y), int(meta.size_x)))
        self.resolution = float(meta.resolution)
        self.origin_x = float(meta.origin.position.x)
        self.origin_y = float(meta.origin.position.y)

    @property
    def ready(self) -> bool:
        return self.grid is not None and self.resolution > 0.0

    def world_to_cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        if not self.ready:
            return None
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        height, width = self.grid.shape
        if 0 <= col < width and 0 <= row < height:
            return row, col
        return None


class LocalPlanTester(Node):
    def __init__(self) -> None:
        super().__init__("local_plan_test")
        self._action = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.costmap = CostmapCache()
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Costmap, "/global_costmap/costmap_raw", self._on_costmap, qos)

    def _on_costmap(self, msg: Costmap) -> None:
        self.costmap.update(msg)

    def wait_for_dependencies(self, timeout_s: float) -> bool:
        if not self._action.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().error("compute_path_to_pose action server not available")
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.costmap.ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error("global_costmap/costmap_raw did not publish")
        return False

    def compute_path(
        self, start: PoseSpec, goal: PoseSpec, planner_id: str, timeout_s: float
    ) -> Optional[ComputePathToPose.Result]:
        now = self.get_clock().now().to_msg()
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start.to_pose_stamped("map", now)
        goal_msg.goal = goal.to_pose_stamped("map", now)
        goal_msg.planner_id = planner_id
        goal_msg.use_start = True

        send_future = self._action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        if not send_future.done() or send_future.result() is None:
            return None
        handle = send_future.result()
        if not handle.accepted:
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            return None
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        return wrapped.result


def load_tests(path: Path) -> list[TestCase]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    tests = data.get("tests") if isinstance(data, dict) else None
    if not tests:
        raise ValueError(f"no tests list in {path}")
    return [
        TestCase(
            name=str(item["name"]),
            description=str(item.get("description", "")),
            start=PoseSpec.from_mapping(item["start"]),
            goal=PoseSpec.from_mapping(item["goal"]),
        )
        for item in tests
    ]


def path_to_xy(path_msg) -> np.ndarray:
    return np.array(
        [[pose.pose.position.x, pose.pose.position.y] for pose in path_msg.poses],
        dtype=np.float64,
    )


def path_length(xy: np.ndarray) -> float:
    if xy.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def lethal_distance_map(grid: np.ndarray, resolution: float) -> Optional[np.ndarray]:
    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:
        return None
    free = grid < COSTMAP_LETHAL
    if not free.any() or free.all():
        return None
    return distance_transform_edt(free) * resolution


def summarize_path(xy: np.ndarray, costmap: CostmapCache) -> dict:
    summary = {"num_poses": int(xy.shape[0]), "length_m": path_length(xy)}
    if xy.shape[0] == 0 or not costmap.ready:
        return summary

    costs = []
    for x, y in xy:
        cell = costmap.world_to_cell(float(x), float(y))
        if cell is not None:
            costs.append(int(costmap.grid[cell]))
    if costs:
        arr = np.asarray(costs, dtype=np.int32)
        known = arr != COSTMAP_NO_INFORMATION
        if known.any():
            summary["mean_cost"] = float(arr[known].mean())
            summary["max_cost"] = int(arr[known].max())
        summary["num_lethal_cells_on_path"] = int((arr >= COSTMAP_LETHAL).sum())
        summary["num_unknown_cells_on_path"] = int((arr == COSTMAP_NO_INFORMATION).sum())
        summary["num_inflated_cells_on_path"] = int(((arr > 0) & (arr < COSTMAP_LETHAL)).sum())

    dist = lethal_distance_map(costmap.grid, costmap.resolution)
    if dist is not None:
        clearances = []
        for x, y in xy:
            cell = costmap.world_to_cell(float(x), float(y))
            if cell is not None:
                clearances.append(float(dist[cell]))
        if clearances:
            arr = np.asarray(clearances, dtype=np.float64)
            summary["min_obstacle_distance_m"] = float(arr.min())
            summary["mean_obstacle_distance_m"] = float(arr.mean())
    return summary


def write_path_csv(path: Path, xy: np.ndarray) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["idx", "x", "y"])
        for i, (x, y) in enumerate(xy):
            writer.writerow([i, f"{x:.6f}", f"{y:.6f}"])


def render_png(
    out_path: Path,
    costmap: CostmapCache,
    xy: np.ndarray,
    start: PoseSpec,
    goal: PoseSpec,
    title: str,
) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    if not costmap.ready:
        return False

    grid = costmap.grid
    height, width = grid.shape
    extent = (
        costmap.origin_x,
        costmap.origin_x + width * costmap.resolution,
        costmap.origin_y,
        costmap.origin_y + height * costmap.resolution,
    )
    rgb = np.full((height, width, 3), 255, dtype=np.uint8)
    inflated = (grid > 0) & (grid < COSTMAP_LETHAL)
    if inflated.any():
        scale = grid[inflated].astype(np.float32) / float(COSTMAP_LETHAL - 1)
        rgb[inflated, 0] = 255
        rgb[inflated, 1] = (255 * (1.0 - scale)).astype(np.uint8)
        rgb[inflated, 2] = (255 * (1.0 - scale)).astype(np.uint8)
    rgb[grid >= COSTMAP_LETHAL] = (0, 0, 0)
    rgb[grid == COSTMAP_NO_INFORMATION] = (200, 200, 200)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(rgb, extent=extent, origin="lower", interpolation="nearest")
    if xy.shape[0]:
        ax.plot(xy[:, 0], xy[:, 1], color="#1f77b4", linewidth=1.8, label="path")
    ax.scatter([start.x], [start.y], c="green", s=60, label="start", zorder=5)
    ax.scatter([goal.x], [goal.y], c="red", marker="x", s=80, label="goal", zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return True


def build_tests(args) -> list[TestCase]:
    if args.tests_file:
        return load_tests(Path(args.tests_file).expanduser())
    if not args.start or not args.goal:
        raise SystemExit("provide --tests-file or both --start and --goal")
    return [
        TestCase(
            name=args.case_name,
            description="CLI ad-hoc case",
            start=PoseSpec.from_cli(args.start),
            goal=PoseSpec.from_cli(args.goal),
        )
    ]


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tests-file")
    parser.add_argument("--start", help="Inline start as x,y[,yaw].")
    parser.add_argument("--goal", help="Inline goal as x,y[,yaw].")
    parser.add_argument("--case-name", default="cli_case")
    parser.add_argument("--planners", default="CenterlinePlanner")
    parser.add_argument("--output-dir", default="./out/local_plan")
    parser.add_argument("--wait-timeout", type=float, default=15.0)
    parser.add_argument("--plan-timeout", type=float, default=20.0)
    args = parser.parse_args(argv)

    tests = build_tests(args)
    planners = [item.strip() for item in args.planners.split(",") if item.strip()]
    out_dir = Path(args.output_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    tester = LocalPlanTester()
    summary = {"planners": planners, "cases": []}
    try:
        if not tester.wait_for_dependencies(args.wait_timeout):
            return 2

        for case in tests:
            record = {
                "name": case.name,
                "description": case.description,
                "start": case.start.__dict__,
                "goal": case.goal.__dict__,
                "results": {},
            }
            print(f"\n=== {case.name}: {case.start} -> {case.goal} ===")
            for planner in planners:
                started = time.monotonic()
                result = tester.compute_path(case.start, case.goal, planner, args.plan_timeout)
                wall_ms = (time.monotonic() - started) * 1000.0
                if result is None:
                    record["results"][planner] = {"success": False, "wall_ms": wall_ms}
                    print(f"  [{planner}] FAIL wall={wall_ms:.0f}ms")
                    continue

                xy = path_to_xy(result.path)
                stats = summarize_path(xy, tester.costmap)
                stats["success"] = True
                stats["wall_ms"] = wall_ms
                stats["server_planning_ms"] = (
                    result.planning_time.sec * 1000.0 + result.planning_time.nanosec / 1e6
                )
                record["results"][planner] = stats

                stem = f"{case.name}__{planner}"
                write_path_csv(out_dir / f"{stem}.csv", xy)
                rendered = render_png(
                    out_dir / f"{stem}.png",
                    tester.costmap,
                    xy,
                    case.start,
                    case.goal,
                    f"{case.name} / {planner}",
                )
                print(
                    f"  [{planner}] OK poses={stats['num_poses']} "
                    f"len={stats['length_m']:.2f}m "
                    f"mean_cost={stats.get('mean_cost', 'NA')} "
                    f"max_cost={stats.get('max_cost', 'NA')} "
                    f"min_clear={stats.get('min_obstacle_distance_m', 'NA')} "
                    f"wall={wall_ms:.0f}ms"
                    + ("" if rendered else " (png skipped)")
                )
            summary["cases"].append(record)

        summary_path = out_dir / "summary.json"
        summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\nsummary written to {summary_path}")
        return 0
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

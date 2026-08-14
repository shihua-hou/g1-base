"""3D PCD → 2D 占据栅格地图转换器

纯 Python 实现，仅依赖 numpy。
读取 Super-LIO 输出的 PCD v0.7 binary 文件，默认按离地高度过滤后投影到 XY 平面，
生成 Nav2 兼容的 PGM (P5) + YAML 地图文件。

等价于原 ROS1 方案：
  pcd_to_pointcloud → octomap_server (height filter) → map_saver

用法:
  python3 -m g1_base.pcd_to_2d_map /path/to/map.pcd [--output-dir /path/to/output]
"""

import argparse
import math
import struct
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None


GROUND_CHECK_CELL_SIZE_M = 0.5
MAX_WORLD_TILT_DEG = 1.5
DEFAULT_OBSTACLE_MIN_HEIGHT = 0.15
DEFAULT_OBSTACLE_MAX_HEIGHT = 1.6
LEGACY_Z_MIN = -0.8
LEGACY_Z_MAX = 0.5


@dataclass(frozen=True)
class GroundAlignment:
    point: object
    normal: object
    tilt_deg: float
    candidate_count: int


class TiltedWorldError(Exception):
    def __init__(self, tilt_deg, max_tilt_deg, normal):
        self.tilt_deg = float(tilt_deg)
        self.max_tilt_deg = float(max_tilt_deg)
        self.normal = normal
        super().__init__(
            "PCD world frame is tilted "
            f"{self.tilt_deg:.2f}° from +Z; max allowed is "
            f"{self.max_tilt_deg:.2f}°. Rebuild the map with the robot still "
            "during LIO initialization."
        )


# ── PCD 解析 ──


def _parse_pcd_header(f):
    """解析 PCD v0.7 header，返回字段信息和数据偏移。"""
    header = {}
    while True:
        line = f.readline()
        if not line:
            raise ValueError("PCD header 不完整")
        line = line.decode("ascii", errors="replace").strip()
        if line.startswith("#") or not line:
            continue
        parts = line.split()
        key = parts[0].upper()
        header[key] = parts[1:] if len(parts) > 1 else []
        if key == "DATA":
            break

    # 解析关键字段
    fields = [f.lower() for f in header.get("FIELDS", [])]
    sizes = [int(s) for s in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(c) for c in header.get("COUNT", [1] * len(fields))]
    n_points = int(header.get("POINTS", [0])[0])
    data_type = header.get("DATA", ["ascii"])[0].lower()

    if "x" not in fields or "y" not in fields or "z" not in fields:
        raise ValueError(f"PCD 文件缺少 x/y/z 字段，找到: {fields}")

    return {
        "fields": fields,
        "sizes": sizes,
        "types": types,
        "counts": counts,
        "n_points": n_points,
        "data_type": data_type,
        "point_size": sum(s * c for s, c in zip(sizes, counts)),
    }


def _read_pcd_binary(pcd_path):
    """读取 binary PCD 文件，返回 (N, 3) 的 xyz numpy 数组。"""
    if np is None:
        raise ImportError("numpy 未安装，请运行: pip3 install numpy")

    with open(pcd_path, "rb") as f:
        info = _parse_pcd_header(f)

        if info["data_type"] != "binary":
            raise ValueError(
                f"仅支持 binary PCD，当前: {info['data_type']}。"
                f"如需支持 ascii 格式请扩展此模块。"
            )

        # 构建 numpy dtype
        np_type_map = {"F": "f", "U": "u", "I": "i"}
        dtype_list = []
        for field, size, typ, count in zip(
            info["fields"], info["sizes"], info["types"], info["counts"]
        ):
            np_char = np_type_map.get(typ, "f")
            dt = f"<{np_char}{size}"  # little-endian
            if count == 1:
                dtype_list.append((field, dt))
            else:
                dtype_list.append((field, dt, (count,)))

        dt = np.dtype(dtype_list)
        raw = f.read(info["n_points"] * dt.itemsize)

    points = np.frombuffer(raw, dtype=dt, count=info["n_points"])
    xyz = np.column_stack([points["x"], points["y"], points["z"]])
    return xyz


def _read_pcd_ascii(pcd_path):
    """读取 ascii PCD 文件，返回 (N, 3) 的 xyz numpy 数组。"""
    if np is None:
        raise ImportError("numpy 未安装，请运行: pip3 install numpy")

    with open(pcd_path, "rb") as f:
        info = _parse_pcd_header(f)

        if info["data_type"] != "ascii":
            raise ValueError(f"预期 ascii PCD，实际: {info['data_type']}")

        x_idx = info["fields"].index("x")
        y_idx = info["fields"].index("y")
        z_idx = info["fields"].index("z")

        points = []
        for line in f:
            parts = line.decode("ascii", errors="replace").split()
            if len(parts) < max(x_idx, y_idx, z_idx) + 1:
                continue
            points.append(
                (float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx]))
            )

    return np.array(points, dtype=np.float32)


def read_pcd(pcd_path):
    """自动检测格式并读取 PCD 文件，返回 (N, 3) xyz 数组。"""
    with open(pcd_path, "rb") as f:
        info = _parse_pcd_header(f)

    if info["data_type"] == "binary":
        return _read_pcd_binary(pcd_path)
    elif info["data_type"] == "ascii":
        return _read_pcd_ascii(pcd_path)
    else:
        raise ValueError(f"不支持的 PCD DATA 类型: {info['data_type']}")


def validate_ground_alignment(
    points,
    cell_size=GROUND_CHECK_CELL_SIZE_M,
    max_tilt_deg=MAX_WORLD_TILT_DEG,
):
    if np is None:
        raise ImportError("numpy 未安装，请运行: pip3 install numpy")

    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("点云必须是 Nx3 xyz 数组")

    xyz = xyz[:, :3]
    finite_mask = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite_mask]
    if len(xyz) < 3:
        raise ValueError("地面校验需要至少 3 个有效点")

    cells = np.floor(xyz[:, :2] / float(cell_size)).astype(np.int64)
    order = np.lexsort((xyz[:, 2], cells[:, 1], cells[:, 0]))
    sorted_cells = cells[order]
    first_in_cell = np.empty(len(order), dtype=bool)
    first_in_cell[0] = True
    first_in_cell[1:] = np.any(sorted_cells[1:] != sorted_cells[:-1], axis=1)
    ground = xyz[order[first_in_cell]]

    if len(ground) < 3:
        raise ValueError("地面校验需要至少 3 个网格候选点")

    plane_point = ground.mean(axis=0)
    _, _, vh = np.linalg.svd(ground - plane_point, full_matrices=False)
    normal = vh[-1].astype(np.float64)
    normal_norm = np.linalg.norm(normal)
    if not math.isfinite(float(normal_norm)) or normal_norm <= 0.0:
        raise ValueError("地面法线计算失败")

    normal = normal / normal_norm
    if normal[2] < 0.0:
        normal = -normal

    tilt_deg = math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))
    alignment = GroundAlignment(
        point=plane_point,
        normal=normal,
        tilt_deg=tilt_deg,
        candidate_count=len(ground),
    )
    if tilt_deg > float(max_tilt_deg):
        raise TiltedWorldError(tilt_deg, max_tilt_deg, normal)
    return alignment


# ── 2D 地图生成 ──


def convert_pcd_to_2d_map(
    pcd_path,
    output_dir,
    output_name=None,
    resolution=0.05,
    obstacle_min_height=DEFAULT_OBSTACLE_MIN_HEIGHT,
    obstacle_max_height=DEFAULT_OBSTACLE_MAX_HEIGHT,
    z_min=None,
    z_max=None,
    padding=1.0,
):
    """将 3D PCD 点云转换为 2D 占据栅格地图。

    Args:
        pcd_path: PCD 文件路径
        output_dir: 输出目录
        output_name: 输出文件名（不含扩展名），默认 map_YYYYMMDD_HHMMSS
        resolution: 栅格分辨率（米/像素），默认 0.05
        obstacle_min_height: 离地障碍高度下限（米），默认 0.15
        obstacle_max_height: 离地障碍高度上限（米），默认 1.6
        z_min: 兼容旧逻辑的 world z 过滤下限；显式传入后启用旧路径
        z_max: 兼容旧逻辑的 world z 过滤上限；显式传入后启用旧路径
        padding: 地图边缘留白（米），默认 1.0

    Returns:
        (pgm_path, yaml_path) 元组
    """
    if np is None:
        raise ImportError("numpy 未安装，请运行: pip3 install numpy")

    pcd_path = Path(pcd_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_name = output_name or datetime.now().strftime("map_%Y%m%d_%H%M%S")

    # 1. 读取点云
    xyz = read_pcd(str(pcd_path))
    total_points = len(xyz)

    if total_points == 0:
        raise ValueError(f"PCD 文件为空: {pcd_path}")

    alignment = validate_ground_alignment(xyz)

    # 2. 高度过滤。默认使用相对地面平面的离地高度；显式 z_min/z_max 保留旧调试路径。
    use_legacy_z_filter = z_min is not None or z_max is not None
    if use_legacy_z_filter:
        z_filter_min = LEGACY_Z_MIN if z_min is None else float(z_min)
        z_filter_max = LEGACY_Z_MAX if z_max is None else float(z_max)
        mask = (xyz[:, 2] >= z_filter_min) & (xyz[:, 2] <= z_filter_max)
        filter_label = f"z 过滤: [{z_filter_min}, {z_filter_max}]"
        empty_message = (
            f"z 轴过滤后无点（z_min={z_filter_min}, z_max={z_filter_max}）。"
            f"原始 z 范围: [{xyz[:, 2].min():.2f}, {xyz[:, 2].max():.2f}]"
        )
    else:
        heights = (xyz - alignment.point) @ alignment.normal
        height_min = float(obstacle_min_height)
        height_max = float(obstacle_max_height)
        mask = (heights >= height_min) & (heights <= height_max)
        filter_label = f"离地高度过滤: [{height_min}, {height_max}]"
        empty_message = (
            "离地高度过滤后无点"
            f"（obstacle_min_height={height_min}, obstacle_max_height={height_max}）。"
            f"原始离地高度范围: [{heights.min():.2f}, {heights.max():.2f}]"
        )
    filtered = xyz[mask]
    filtered_count = len(filtered)

    if filtered_count == 0:
        raise ValueError(empty_message)

    # 3. 计算边界和栅格尺寸
    x_min_world = filtered[:, 0].min() - padding
    x_max_world = filtered[:, 0].max() + padding
    y_min_world = filtered[:, 1].min() - padding
    y_max_world = filtered[:, 1].max() + padding

    width = int(np.ceil((x_max_world - x_min_world) / resolution))
    height = int(np.ceil((y_max_world - y_min_world) / resolution))

    # 4. 投影到 2D 栅格
    # 计算每个点落入的栅格坐标
    col = ((filtered[:, 0] - x_min_world) / resolution).astype(np.int32)
    row = ((filtered[:, 1] - y_min_world) / resolution).astype(np.int32)

    # 裁剪到有效范围
    col = np.clip(col, 0, width - 1)
    row = np.clip(row, 0, height - 1)

    # 创建栅格：205 = unknown(灰色), 254 = free(白色), 0 = occupied(黑色)
    # Nav2 map_server 格式：0=occupied, 254=free, 205=unknown
    grid = np.full((height, width), 205, dtype=np.uint8)  # 默认未知，不能把未观测区域当自由

    # 标记占据区域
    grid[row, col] = 0

    # 翻转 Y 轴（PGM 从上到下，地图 Y 从下到上）
    grid = np.flipud(grid)

    # 5. 保存 PGM (P5 binary)
    pgm_path = output_dir / f"{output_name}.pgm"
    _save_pgm(pgm_path, grid)

    # 6. 保存 YAML
    yaml_path = output_dir / f"{output_name}.yaml"
    origin_x = x_min_world
    origin_y = y_min_world
    _save_yaml(yaml_path, f"{output_name}.pgm", resolution, origin_x, origin_y)

    print(f"转换完成:")
    print(f"  输入: {pcd_path} ({total_points} 点)")
    print(
        f"  地面倾斜: {alignment.tilt_deg:.2f}° "
        f"({alignment.candidate_count} candidates)"
    )
    print(f"  {filter_label} → {filtered_count} 点")
    print(f"  栅格: {width} x {height} @ {resolution}m/px")
    print(f"  输出: {pgm_path}")
    print(f"  输出: {yaml_path}")
    print(f"  origin: ({origin_x:.3f}, {origin_y:.3f})")

    return str(pgm_path), str(yaml_path)


def _save_pgm(path, grid):
    """保存 P5 (binary) PGM 文件。"""
    height, width = grid.shape
    with open(path, "wb") as f:
        # PGM P5 header
        header = f"P5\n{width} {height}\n255\n"
        f.write(header.encode("ascii"))
        f.write(grid.tobytes())


def _save_yaml(path, image_filename, resolution, origin_x, origin_y):
    """保存 Nav2 map_server 兼容的 YAML 文件。"""
    content = (
        f"image: {image_filename}\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


# ── CLI 入口 ──


def main():
    parser = argparse.ArgumentParser(
        description="3D PCD 点云 → 2D 占据栅格地图转换器"
    )
    parser.add_argument("pcd_path", help="输入 PCD 文件路径")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="输出目录（默认：与 PCD 同目录）",
    )
    parser.add_argument(
        "--output-name",
        default=None,
        help="输出文件名（不含扩展名，默认: map_YYYYMMDD_HHMMSS）",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="栅格分辨率（米/像素，默认: 0.05）",
    )
    parser.add_argument(
        "--obstacle-min-height",
        type=float,
        default=DEFAULT_OBSTACLE_MIN_HEIGHT,
        help="离地障碍高度下限（米，默认: 0.15）",
    )
    parser.add_argument(
        "--obstacle-max-height",
        type=float,
        default=DEFAULT_OBSTACLE_MAX_HEIGHT,
        help="离地障碍高度上限（米，默认: 1.6）",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=None,
        help="兼容旧逻辑的 world z 过滤下限；传入后启用 z 过滤",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=None,
        help="兼容旧逻辑的 world z 过滤上限；传入后启用 z 过滤",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=1.0,
        help="地图边缘留白（米，默认: 1.0）",
    )
    args = parser.parse_args()

    pcd_path = Path(args.pcd_path)
    if not pcd_path.is_file():
        print(f"错误: PCD 文件不存在: {pcd_path}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output_dir or str(pcd_path.parent)
    output_name = args.output_name or datetime.now().strftime("map_%Y%m%d_%H%M%S")

    try:
        pgm, yaml_f = convert_pcd_to_2d_map(
            pcd_path=str(pcd_path),
            output_dir=output_dir,
            output_name=output_name,
            resolution=args.resolution,
            obstacle_min_height=args.obstacle_min_height,
            obstacle_max_height=args.obstacle_max_height,
            z_min=args.z_min,
            z_max=args.z_max,
            padding=args.padding,
        )
    except Exception as exc:
        print(f"转换失败: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

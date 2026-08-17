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
# 稳健拟合之后仍然超过这个角度，才认为世界系真的歪了。
#
# 现场那次"倾斜 9.6°"是误报：候选地面点里 47% 是野点（有的跑到地面下
# 7.6m），普通最小二乘被拽歪了。换成 RANSAC 后同一份点云量出来是 0.84°。
#
# 阈值定在 6°：室内地面基本不可能超过（无障碍坡道国标上限 1:12 ≈ 4.8°），
# 而实测一张正常的图只有 0.84°，余量足够。
# 不敢再放大是因为拟合已经稳健、角度可信了——真让一张歪了 8°（重力初始化
# 失败）的图静默通过，之后导航里每个位姿都带系统性偏差，那才难查。
MAX_WORLD_TILT_DEG = 6.0
# RANSAC：地面点里混进野点是常态，不能用普通最小二乘
GROUND_RANSAC_ITERS = 240
GROUND_RANSAC_TOL_M = 0.08
# 内点率低于此值说明"地面"根本不是一个平面（多层、斜坡，或 LIO 漂了）
GROUND_MIN_INLIER_RATIO = 0.45
# 只用离原点这个半径内的点拟合地面。
# MID360 量程 70m，半分钟不动也能扫出 50×70m 的图，但远处地面是以极小的
# 掠射角打到的，仰角误差被距离放大得厉害。而"每格取最低点"恰好专挑这些
# 远场坏点，杠杆又长 —— 35m 外差 3.8m 就能把平面撬起 6°。
# 近场地面点又准又密，够拟合了。0 表示不限制。
GROUND_CHECK_MAX_RADIUS_M = 20.0
DEFAULT_OBSTACLE_MIN_HEIGHT = 0.15
DEFAULT_OBSTACLE_MAX_HEIGHT = 1.6
# 离地 ±这个厚度内的点算"看到了地板"，据此标出可通行区域。
# 激光能打到地板，说明那条光路上没有障碍——这是自由空间最可靠的证据，
# 比从轨迹做光线投射简单得多，也不需要额外保存轨迹。
DEFAULT_GROUND_BAND = 0.12
# 自由区膨胀半径（像素）。地面点再密也是离散采样，5cm 栅格下会留下
# 一格一格的空洞；补上这一圈，Nav2 的代价地图才不会到处是"未知"小孔。
# 只膨胀自由区，障碍物最后画，不会被吃掉。
DEFAULT_FREE_FILL_PX = 2
LEGACY_Z_MIN = -0.8
LEGACY_Z_MAX = 0.5


@dataclass(frozen=True)
class GroundAlignment:
    point: object
    normal: object
    tilt_deg: float
    candidate_count: int
    inlier_ratio: float = 1.0
    residual_rms: float = 0.0


class TiltedWorldError(Exception):
    def __init__(self, tilt_deg, max_tilt_deg, normal,
                 inlier_ratio=None, residual_rms=None):
        self.tilt_deg = float(tilt_deg)
        self.max_tilt_deg = float(max_tilt_deg)
        self.normal = normal
        self.inlier_ratio = inlier_ratio
        self.residual_rms = residual_rms
        detail = ""
        if inlier_ratio is not None and residual_rms is not None:
            detail = (f" (ground inliers {inlier_ratio * 100:.0f}%, "
                      f"residual RMS {residual_rms:.2f} m)")
        super().__init__(
            "PCD world frame is tilted "
            f"{self.tilt_deg:.2f}° from +Z; max allowed is "
            f"{self.max_tilt_deg:.2f}°{detail}."
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
    max_radius_m=GROUND_CHECK_MAX_RADIUS_M,
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

    # 拟合地面只用近场点，远场掠射点误差被距离放大（见 GROUND_CHECK_MAX_RADIUS_M）。
    # 注意只影响"拟合"，高度过滤和投影仍然用全部点，地图范围不会被裁掉。
    fit_src = xyz
    if max_radius_m and float(max_radius_m) > 0.0:
        center = np.median(xyz[:, :2], axis=0)
        near = np.linalg.norm(xyz[:, :2] - center, axis=1) <= float(max_radius_m)
        if int(near.sum()) >= 32:      # 近场点太少就退回用全部，总比拟合不出来强
            fit_src = xyz[near]

    cells = np.floor(fit_src[:, :2] / float(cell_size)).astype(np.int64)
    order = np.lexsort((fit_src[:, 2], cells[:, 1], cells[:, 0]))
    sorted_cells = cells[order]
    first_in_cell = np.empty(len(order), dtype=bool)
    first_in_cell[0] = True
    first_in_cell[1:] = np.any(sorted_cells[1:] != sorted_cells[:-1], axis=1)
    ground = fit_src[order[first_in_cell]]

    if len(ground) < 3:
        raise ValueError("地面校验需要至少 3 个网格候选点")

    # 每格取最低点当地面候选，恰恰是野点最爱待的位置：LIO 未收敛时甩出的
    # 点、玻璃/镜面的穿透点，全都比真地面低。普通最小二乘对它们毫无抵抗力，
    # 几个点就能把平面拽歪十几度。所以先 RANSAC 挑出真正共面的那批点再拟合。
    inliers = _ransac_plane_inliers(ground, GROUND_RANSAC_TOL_M, GROUND_RANSAC_ITERS)
    fit_pts = ground[inliers] if inliers is not None and int(inliers.sum()) >= 3 else ground
    inlier_ratio = float(len(fit_pts)) / float(len(ground))

    plane_point = fit_pts.mean(axis=0)
    _, _, vh = np.linalg.svd(fit_pts - plane_point, full_matrices=False)
    normal = vh[-1].astype(np.float64)
    normal_norm = np.linalg.norm(normal)
    if not math.isfinite(float(normal_norm)) or normal_norm <= 0.0:
        raise ValueError("地面法线计算失败")

    normal = normal / normal_norm
    if normal[2] < 0.0:
        normal = -normal

    # 内点到平面的 RMS：区分"整体歪了"和"地面根本不平"。
    # 前者 RMS 小、单纯是个倾斜，后者说明 LIO 漂了或场地本来就有高差。
    residual_rms = float(np.sqrt(np.mean(((fit_pts - plane_point) @ normal) ** 2)))

    tilt_deg = math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))
    alignment = GroundAlignment(
        point=plane_point,
        normal=normal,
        tilt_deg=tilt_deg,
        candidate_count=len(ground),
        inlier_ratio=inlier_ratio,
        residual_rms=residual_rms,
    )
    if tilt_deg > float(max_tilt_deg):
        raise TiltedWorldError(tilt_deg, max_tilt_deg, normal,
                               inlier_ratio=inlier_ratio, residual_rms=residual_rms)
    return alignment


def _ransac_plane_inliers(pts, tol, iters, seed=12345):
    """在候选地面点里找出最大的共面子集，返回布尔掩码（找不到返回 None）。

    固定随机种子：同一份点云每次跑出来的地图必须一模一样，
    否则同一张图重存两次得到两个略有差别的 pgm，排障时会怀疑人生。
    """
    n = len(pts)
    if n < 3:
        return None
    rng = np.random.default_rng(seed)
    best_mask = None
    best_count = 0
    for _ in range(int(iters)):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        nlen = float(np.linalg.norm(nrm))
        if nlen < 1e-9:
            continue           # 三点共线，这次采样作废
        nrm = nrm / nlen
        mask = np.abs((pts - p0) @ nrm) <= float(tol)
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    return best_mask


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
    ground_band=DEFAULT_GROUND_BAND,
    free_fill_px=DEFAULT_FREE_FILL_PX,
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

    # 沿地面法线的离地高度：障碍物过滤和地面识别都用它，和 LIO 原点在哪无关
    heights = (xyz - alignment.point) @ alignment.normal

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

    # 3. 地面点 = 可通行的证据。
    # 激光能打到某处的地板，说明那条光路上没东西挡着，那一格就是自由空间。
    # 不这么做的话整张图只有"障碍"和"未知"两种值，Nav2 无处可规划 ——
    # 之前生成的图全是灰底黑点，就是因为压根没写过自由区。
    ground = xyz[np.abs(heights) <= float(ground_band)]

    # 4. 计算边界和栅格尺寸（要把地面点也算进去，否则自由区会被裁掉）
    span_src = np.vstack([filtered[:, :2], ground[:, :2]]) if len(ground) else filtered[:, :2]
    x_min_world = span_src[:, 0].min() - padding
    x_max_world = span_src[:, 0].max() + padding
    y_min_world = span_src[:, 1].min() - padding
    y_max_world = span_src[:, 1].max() + padding

    width = int(np.ceil((x_max_world - x_min_world) / resolution))
    height = int(np.ceil((y_max_world - y_min_world) / resolution))

    def _to_grid(pts):
        c = ((pts[:, 0] - x_min_world) / resolution).astype(np.int32)
        r = ((pts[:, 1] - y_min_world) / resolution).astype(np.int32)
        return np.clip(r, 0, height - 1), np.clip(c, 0, width - 1)

    # 5. 投影到 2D 栅格
    # Nav2 map_server 约定：0=occupied(黑), 254=free(白), 205=unknown(灰)
    grid = np.full((height, width), 205, dtype=np.uint8)   # 默认未知

    free_mask = np.zeros((height, width), dtype=bool)
    if len(ground):
        gr, gc = _to_grid(ground)
        free_mask[gr, gc] = True
        # 地面采样是离散的，5cm 栅格下会留下一格格的空洞。补一圈，
        # 免得代价地图里到处是"未知"小孔把路径卡死。
        free_mask = _dilate(free_mask, int(free_fill_px))
    grid[free_mask] = 254

    # 障碍物最后画，压过自由区：同一格既看到地板又看到障碍时，按障碍算
    orow, ocol = _to_grid(filtered)
    grid[orow, ocol] = 0

    free_cells = int(free_mask.sum())
    occ_cells = int((grid == 0).sum())

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
    print(f"  地面点: {len(ground)} → 自由区 {free_cells} 格；障碍 {occ_cells} 格")
    print(f"  栅格: {width} x {height} @ {resolution}m/px "
          f"（自由 {free_cells * 100.0 / (width * height):.1f}%）")
    print(f"  输出: {pgm_path}")
    print(f"  输出: {yaml_path}")
    print(f"  origin: ({origin_x:.3f}, {origin_y:.3f})")

    return str(pgm_path), str(yaml_path)


def _dilate(mask, radius):
    """布尔掩码的方形膨胀。

    只为了补自由区里的采样空洞，用不着 scipy —— 沿两个轴各做几次
    邻位取或就够了，代价是 O(radius)，栅格再大也不心疼。
    """
    if radius <= 0:
        return mask
    out = mask
    for _ in range(int(radius)):
        padded = np.zeros_like(out)
        padded[:, :] = out
        padded[1:, :] |= out[:-1, :]
        padded[:-1, :] |= out[1:, :]
        out = padded
        padded = np.zeros_like(out)
        padded[:, :] = out
        padded[:, 1:] |= out[:, :-1]
        padded[:, :-1] |= out[:, 1:]
        out = padded
    return out


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


def _inspect_ground(pcd_path):
    """诊断地面拟合：不同拟合半径下的倾斜角/内点率/残差，外加 z 分布。

    「地面不水平」到底是真倾斜、是野点、还是 LIO 漂了，看这张表就能分辨：
      · 各半径下倾斜角都很小        → 地面本来就是平的，之前是误报
      · 半径越大倾斜角越大          → 远场点不可信（掠射角误差 / 漂移）
      · 内点率低、残差大            → 地面根本不是一个平面，多半是 LIO 漂了
      · 各半径下倾斜角一致且很大    → 场地是真的有坡
    """
    xyz = read_pcd(str(pcd_path))
    print(f"点数 {len(xyz)}")
    print("z 分位数 " + "  ".join(
        f"p{p}={np.percentile(xyz[:, 2], p):.2f}" for p in (0.1, 1, 50, 99, 99.9)))
    span = xyz[:, :2].max(axis=0) - xyz[:, :2].min(axis=0)
    print(f"平面范围 {span[0]:.1f} × {span[1]:.1f} m")
    print(f"{'拟合半径':>10}  {'倾斜角':>8}  {'内点率':>8}  {'残差RMS':>9}  {'候选点':>8}")
    for radius in (5.0, 10.0, 20.0, 40.0, 0.0):
        label = "全部" if radius == 0.0 else f"{radius:.0f} m"
        try:
            a = validate_ground_alignment(xyz, max_tilt_deg=90.0, max_radius_m=radius)
            print(f"{label:>10}  {a.tilt_deg:7.2f}°  {a.inlier_ratio * 100:7.0f}%  "
                  f"{a.residual_rms:8.3f} m  {a.candidate_count:8d}")
        except Exception as exc:
            print(f"{label:>10}  失败: {exc}")
    print(f"\n当前生效阈值 MAX_WORLD_TILT_DEG = {MAX_WORLD_TILT_DEG}°，"
          f"拟合半径 = {GROUND_CHECK_MAX_RADIUS_M} m")
    return 0


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
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="只诊断不出图：打印地面拟合的倾斜角/内点率/残差，"
             "并对比不同拟合半径的结果，用来判断「地面不水平」是真是假",
    )
    args = parser.parse_args()

    if args.inspect:
        return _inspect_ground(args.pcd_path)

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

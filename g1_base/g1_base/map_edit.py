"""地图编辑：在已生成的 2D 栅格上涂改、划禁行区、裁剪旋转。

为什么需要它：pcd_to_2d_map 是从点云一次性投出来的，现场总有它处理不了的
情况——玻璃门和镂空栏杆雷达打不到，于是地图上是通的，机器人会径直撞过去；
反过来，走动的人和临时堆放的箱子会被扫成永久障碍，把通道堵死。这些只能靠
人工修。

三种编辑的存储策略不同，因为可逆性要求不同：

  涂改     直接改 pgm 像素，但首次编辑前把原图另存为 <pgm>.orig.pgm，
           所以「还原」永远可用。
  禁行区   存成世界坐标的矩形列表（<base>_edits.json），保存时再烘焙进
           pgm。这样禁行区随时可以增删改，而不是涂上去就洗不掉。
           之所以不走 Nav2 的 KeepoutFilter，是因为那需要额外的
           costmap_filter_info_server + 一套 mask 地图 + 两个 costmap 都
           挂插件；烘焙进底图效果一样，而且重定位用的 pcd 不受影响。
  裁剪旋转 改变栅格尺寸和 origin，是破坏性的，同样靠 .orig 兜底。

像素约定沿用 Nav2 map_server 的默认（negate:0, occupied>0.65, free<0.196）：
    0 = 占用    205 = 未知    254 = 空闲
"""

import json
import math
import shutil
from pathlib import Path

import numpy as np


OCCUPIED = 0
UNKNOWN = 205
FREE = 254

# 画笔只允许这三种值。写别的进去，map_server 按阈值会把它归到意想不到的一类。
BRUSH_VALUES = {"occupied": OCCUPIED, "unknown": UNKNOWN, "free": FREE}

EDITS_SUFFIX = "_edits.json"
ORIG_SUFFIX = ".orig.pgm"
BASE_SUFFIX = ".base.pgm"

# 三层文件，缺一不可：
#
#   <name>.pgm            Nav2 实际加载的那张 = 底图 + 烘焙好的禁行区
#   <name>.pgm.base.pgm   底图：涂改的结果，但【不含】禁行区
#   <name>.pgm.orig.pgm   最初那张，一次都没编辑过
#
# 为什么要中间那层：禁行区如果直接烧进 .pgm，删掉某个禁行区就没法还原了
# ——已经涂黑的像素分不清是禁行区涂的还是本来就有障碍。有了底图，每次
# 禁行区增删都从底图重新烘焙一遍，增删自如。


class MapEditError(Exception):
    """编辑失败。调用方负责转成 HTTP 4xx/5xx。"""


# ── PGM 读写 ──

def read_pgm(path):
    """读 P5 (binary) PGM，返回 uint8 的 (height, width) 数组。

    只认 P5：pcd_to_2d_map 写的就是 P5，Nav2 也只认它。遇到 P2(ASCII)
    直接报错而不是猜着解析——猜错了整张地图会变成噪声。
    """
    data = Path(path).read_bytes()
    if not data.startswith(b"P5"):
        raise MapEditError("不是 P5 格式的 PGM: {}".format(path))

    # 头部是三个数字（宽 高 最大值），中间允许注释行和任意空白
    fields = []
    pos = 2
    while len(fields) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":                 # 注释吃到行尾
            while pos < len(data) and data[pos:pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        if start == pos:
            raise MapEditError("PGM 头部不完整: {}".format(path))
        fields.append(int(data[start:pos]))
    pos += 1                                          # 头部与像素之间正好一个空白字符

    width, height, maxval = fields
    if maxval != 255:
        raise MapEditError("只支持 8 位 PGM（maxval=255），实际 {}".format(maxval))
    expected = width * height
    body = data[pos:pos + expected]
    if len(body) != expected:
        raise MapEditError(
            "PGM 数据长度不符：期望 {}，实际 {}".format(expected, len(body))
        )
    return np.frombuffer(body, dtype=np.uint8).reshape(height, width).copy()


def write_pgm(path, grid):
    grid = np.ascontiguousarray(grid, dtype=np.uint8)
    height, width = grid.shape
    with open(path, "wb") as fp:
        fp.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
        fp.write(grid.tobytes())


def read_map_yaml(path):
    """极简 yaml 读取，只取需要的三个字段。

    不引 pyyaml：这个模块也被独立命令行工具用，不能假设装了它。
    字段格式由 pcd_to_2d_map._save_yaml 固定产出，可控。
    """
    out = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        out[key.strip()] = value.strip()
    try:
        resolution = float(out["resolution"])
        origin = [float(v) for v in out["origin"].strip("[]").split(",")[:2]]
    except (KeyError, ValueError) as exc:
        raise MapEditError("地图 yaml 缺少 resolution/origin: {} ({})".format(path, exc))
    return {
        "image": out.get("image", ""),
        "resolution": resolution,
        "origin_x": origin[0],
        "origin_y": origin[1],
    }


def write_map_yaml(path, image_filename, resolution, origin_x, origin_y):
    Path(path).write_text(
        "image: {}\n".format(image_filename)
        + "resolution: {:.6f}\n".format(resolution)
        + "origin: [{:.6f}, {:.6f}, 0]\n".format(origin_x, origin_y)
        + "negate: 0\n"
        + "occupied_thresh: 0.65\n"
        + "free_thresh: 0.196\n",
        encoding="utf-8",
    )


# ── 世界坐标 <-> 像素 ──
#
# map_server 的约定：origin 指【左下角】，栅格行号从下往上数。
# 而图像的第 0 行在【顶部】。所以纵向要翻一次——这里是最容易搞反的地方，
# 反了的话地图看着正常，涂改的位置会上下镜像。

def _floor_cells(distance, resolution):
    """把距离换算成格子数，做浮点边界修正。

    直接 floor 会出错：裁剪之后 origin 正好落在格子边界上，于是
    0.3 - 0.1 算出 0.19999999999999998，除以 0.05 得 3.9999999999999996，
    floor 掉到第 3 格——比实际位置少一格。表现为裁剪后涂改错位、
    世界坐标和像素对不上，而且不会报任何错。
    """
    q = distance / resolution
    nearest = round(q)
    if abs(q - nearest) < 1e-6:
        q = nearest
    return int(math.floor(q))


def world_to_pixel(x, y, geo, height):
    col = _floor_cells(x - geo["origin_x"], geo["resolution"])
    row_from_bottom = _floor_cells(y - geo["origin_y"], geo["resolution"])
    return col, height - 1 - row_from_bottom


def pixel_to_world(col, row, geo, height):
    x = geo["origin_x"] + (col + 0.5) * geo["resolution"]
    y = geo["origin_y"] + (height - 1 - row + 0.5) * geo["resolution"]
    return x, y


# ── 涂改 ──

def paint_strokes(grid, geo, strokes, brush, radius_m):
    """按世界坐标的笔画涂色，返回实际改动的像素数。

    每个 stroke 是一串点；相邻点之间做插值，否则鼠标快速拖动时会画成
    一串断开的圆点。
    """
    if brush not in BRUSH_VALUES:
        raise MapEditError("未知画笔类型: {}".format(brush))
    value = BRUSH_VALUES[brush]
    height, width = grid.shape
    radius_px = max(1, int(round(float(radius_m) / geo["resolution"])))

    # 预先算好圆形笔尖的偏移量，避免每个点重算一遍
    span = np.arange(-radius_px, radius_px + 1)
    dy, dx = np.meshgrid(span, span, indexing="ij")
    disc = (dx * dx + dy * dy) <= radius_px * radius_px
    offs_y, offs_x = dy[disc], dx[disc]

    before = grid.copy()
    for stroke in strokes:
        pts = [world_to_pixel(p[0], p[1], geo, height) for p in stroke if len(p) >= 2]
        if not pts:
            continue
        dense = [pts[0]]
        for (c0, r0), (c1, r1) in zip(pts, pts[1:]):
            steps = max(abs(c1 - c0), abs(r1 - r0))
            for i in range(1, steps + 1):
                dense.append((c0 + (c1 - c0) * i // steps, r0 + (r1 - r0) * i // steps))
        for col, row in dense:
            cc = np.clip(col + offs_x, 0, width - 1)
            rr = np.clip(row + offs_y, 0, height - 1)
            grid[rr, cc] = value
    return int(np.count_nonzero(before != grid))


# ── 禁行区 ──

def bake_zones(grid, geo, zones):
    """把禁行区矩形烧成占用像素。zones 用世界坐标 {x, y, w, h}，x/y 是左下角。"""
    height, width = grid.shape
    painted = 0
    for zone in zones:
        try:
            x, y = float(zone["x"]), float(zone["y"])
            w, h = abs(float(zone["w"])), abs(float(zone["h"]))
        except (KeyError, TypeError, ValueError):
            continue
        if w <= 0 or h <= 0:
            continue
        c0, r0 = world_to_pixel(x, y + h, geo, height)      # 左上角
        c1, r1 = world_to_pixel(x + w, y, geo, height)      # 右下角
        c0, c1 = sorted((c0, c1))
        r0, r1 = sorted((r0, r1))
        c0, c1 = max(0, c0), min(width - 1, c1)
        r0, r1 = max(0, r0), min(height - 1, r1)
        if c1 < c0 or r1 < r0:
            continue
        painted += int((r1 - r0 + 1) * (c1 - c0 + 1))
        grid[r0:r1 + 1, c0:c1 + 1] = OCCUPIED
    return painted


def load_edits(base_path):
    path = Path(str(base_path) + EDITS_SUFFIX)
    if not path.is_file():
        return {"zones": []}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {"zones": []}
    zones = data.get("zones")
    return {"zones": zones if isinstance(zones, list) else []}


def save_edits(base_path, edits):
    path = Path(str(base_path) + EDITS_SUFFIX)
    path.write_text(json.dumps(edits, ensure_ascii=False, indent=2), encoding="utf-8")


# ── 裁剪 / 旋转 ──

def crop(grid, geo, bbox):
    """按世界坐标 bbox {x, y, w, h} 裁剪，返回 (新栅格, 新 geo)。

    裁剪必须同步改 origin：origin 指左下角，裁掉左边和下边就得往里挪。
    漏了这一步地图看着正常，机器人位姿会整体偏移。
    """
    height, width = grid.shape
    x, y = float(bbox["x"]), float(bbox["y"])
    w, h = float(bbox["w"]), float(bbox["h"])
    c0, r0 = world_to_pixel(x, y + h, geo, height)
    c1, r1 = world_to_pixel(x + w, y, geo, height)
    c0, c1 = sorted((c0, c1))
    r0, r1 = sorted((r0, r1))
    c0, c1 = max(0, c0), min(width - 1, c1)
    r0, r1 = max(0, r0), min(height - 1, r1)
    if c1 <= c0 or r1 <= r0:
        raise MapEditError("裁剪区域太小或完全在地图之外")
    return _slice_with_origin(grid, geo, r0, r1, c0, c1)


def autocrop(grid, geo, margin_px=10):
    """去掉四周成片的未知区域，只保留有内容的部分再留一圈边距。"""
    known = grid != UNKNOWN
    if not known.any():
        raise MapEditError("整张地图都是未知区域，没有可保留的内容")
    rows = np.where(known.any(axis=1))[0]
    cols = np.where(known.any(axis=0))[0]
    height, width = grid.shape
    r0 = max(0, int(rows[0]) - margin_px)
    r1 = min(height - 1, int(rows[-1]) + margin_px)
    c0 = max(0, int(cols[0]) - margin_px)
    c1 = min(width - 1, int(cols[-1]) + margin_px)
    return _slice_with_origin(grid, geo, r0, r1, c0, c1)


def _slice_with_origin(grid, geo, r0, r1, c0, c1):
    height = grid.shape[0]
    out = grid[r0:r1 + 1, c0:c1 + 1].copy()
    new_geo = dict(geo)
    new_geo["origin_x"] = geo["origin_x"] + c0 * geo["resolution"]
    # 新图底边在原图里是第 r1 行，它距原图底部还有 (height-1-r1) 行
    new_geo["origin_y"] = geo["origin_y"] + (height - 1 - r1) * geo["resolution"]
    return out, new_geo


def rotate(grid, geo, degrees):
    """只支持 90 度的整数倍。

    ⚠ 旋转会让地图坐标系和 Super-LIO 重定位用的 pcd 对不上——pcd 不跟着转。
    所以这只适合"地图还没投入使用、只是想摆正了看"的场景。调用方必须提醒
    用户：旋转后要么重新建图，要么接受定位对不上。
    """
    if int(degrees) % 90 != 0:
        raise MapEditError("只支持 90 度的整数倍旋转")
    turns = int(degrees) // 90 % 4
    if turns == 0:
        return grid.copy(), dict(geo)
    out = np.rot90(grid, k=-turns)   # 负号：屏幕上看是顺时针
    height, width = grid.shape
    res = geo["resolution"]
    # 绕地图中心转，转完重新算左下角
    cx = geo["origin_x"] + width * res / 2.0
    cy = geo["origin_y"] + height * res / 2.0
    new_h, new_w = out.shape
    new_geo = dict(geo)
    new_geo["origin_x"] = cx - new_w * res / 2.0
    new_geo["origin_y"] = cy - new_h * res / 2.0
    return np.ascontiguousarray(out), new_geo


# ── 原图备份 / 还原 ──

def ensure_backup(pgm_path):
    """首次编辑前备份原图。已存在就不动——备份的永远是"最初"那张。"""
    backup = Path(str(pgm_path) + ORIG_SUFFIX)
    if not backup.exists():
        shutil.copy2(pgm_path, backup)
    return backup


def base_path_for(pgm_path):
    return Path(str(pgm_path) + BASE_SUFFIX)


def ensure_base(pgm_path):
    """底图不存在就从当前 pgm 建一份。

    首次编辑时当前 pgm 里还没有禁行区，直接拷过来就是干净底图。
    """
    base = base_path_for(pgm_path)
    if not base.exists():
        shutil.copy2(pgm_path, base)
    return base


def rebuild(pgm_path, geo, zones):
    """从底图重新生成 Nav2 用的那张：底图 + 烘焙禁行区。

    每次禁行区或涂改改动后都要调一次。返回禁行区覆盖的像素数。
    """
    ensure_base(pgm_path)
    grid = read_pgm(base_path_for(pgm_path))
    painted = bake_zones(grid, geo, zones or [])
    write_pgm(pgm_path, grid)
    return painted


def apply_transform_to_all(pgm_path, transform_fn):
    """把同一个几何变换（裁剪/旋转）作用到三层文件上。

    只改 .pgm 不改底图的话，下次禁行区一变动就会从旧尺寸的底图重建，
    地图尺寸突然变回去——这种错很难查，所以三层必须一起动。
    返回新的 geo。
    """
    ensure_backup(pgm_path)
    ensure_base(pgm_path)
    new_geo = None
    for path in (Path(pgm_path), base_path_for(pgm_path),
                 Path(str(pgm_path) + ORIG_SUFFIX)):
        if not path.is_file():
            continue
        out, new_geo = transform_fn(read_pgm(path))
        write_pgm(path, out)
    if new_geo is None:
        raise MapEditError("没有可变换的地图文件")
    return new_geo


def has_backup(pgm_path):
    return Path(str(pgm_path) + ORIG_SUFFIX).is_file()


def revert(pgm_path):
    """回到最初那张：三层同时复位，禁行区也一并清掉。"""
    backup = Path(str(pgm_path) + ORIG_SUFFIX)
    if not backup.is_file():
        raise MapEditError("没有原图备份，无法还原")
    shutil.copy2(backup, pgm_path)
    # 底图也要跟着回去，否则下次禁行区一改就从旧底图重建，编辑又回来了
    shutil.copy2(backup, base_path_for(pgm_path))
    return True

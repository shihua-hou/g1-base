"""地图编辑的几何与像素约定。

这块最容易出的错是纵向翻转：map_server 的 origin 指左下角、行号从下往上，
而图像第 0 行在顶部。搞反了地图看着完全正常，涂改的位置却上下镜像，
裁剪之后机器人位姿整体偏移——都不会报错，只会在现场表现为"定位不准"。
所以坐标换算、裁剪后的 origin、旋转后的 origin 全部钉死。
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_base import map_edit as me  # noqa: E402


def _geo(res=0.05, ox=-1.0, oy=-2.0):
    return {"resolution": res, "origin_x": ox, "origin_y": oy}


def _grid(h=20, w=30, fill=me.UNKNOWN):
    return np.full((h, w), fill, dtype=np.uint8)


# ── PGM 读写 ──

def test_pgm_roundtrip(tmp_path):
    grid = _grid()
    grid[3, 4] = me.OCCUPIED
    grid[10, 20] = me.FREE
    path = tmp_path / "m.pgm"
    me.write_pgm(path, grid)
    back = me.read_pgm(path)
    assert back.shape == grid.shape
    assert np.array_equal(back, grid)


def test_pgm_header_with_comment(tmp_path):
    """有些工具会在头里写注释行，不能因此解析失败。"""
    path = tmp_path / "c.pgm"
    path.write_bytes(b"P5\n# made by something\n4 2\n255\n" + bytes(range(8)))
    grid = me.read_pgm(path)
    assert grid.shape == (2, 4)
    assert grid[0, 0] == 0 and grid[1, 3] == 7


def test_pgm_rejects_truncated_body(tmp_path):
    path = tmp_path / "t.pgm"
    path.write_bytes(b"P5\n4 4\n255\n" + b"\x00" * 5)
    try:
        me.read_pgm(path)
    except me.MapEditError:
        return
    raise AssertionError("截断的 PGM 必须报错，不能当成有效地图")


# ── 坐标换算 ──

def test_world_pixel_roundtrip_is_vertically_correct():
    geo, height = _geo(), 20
    # 世界坐标 y 越大 -> 行号越小（越靠图像顶部）
    _, row_low = me.world_to_pixel(0.0, -1.9, geo, height)
    _, row_high = me.world_to_pixel(0.0, -1.1, geo, height)
    assert row_high < row_low, "y 增大时行号必须减小，否则地图上下翻转"

    for col, row in ((0, 0), (7, 3), (29, 19)):
        x, y = me.pixel_to_world(col, row, geo, height)
        assert me.world_to_pixel(x, y, geo, height) == (col, row)


def test_boundary_coordinates_do_not_fall_into_the_previous_cell():
    """正好落在格子边界的世界坐标不能因为浮点误差掉进前一格。

    裁剪之后 origin 恰好是格子边界，0.3-0.1 会算出 0.19999999999999998，
    直接 floor 就少一格——裁剪后涂改错位，而且不报任何错。
    """
    geo = {"resolution": 0.05, "origin_x": 0.1, "origin_y": -1.5}
    assert me.world_to_pixel(0.3, -1.5, geo, 13)[0] == 4
    assert me.world_to_pixel(0.1, -1.2, geo, 13)[0] == 0
    # 负方向同样要对
    geo2 = {"resolution": 0.05, "origin_x": -1.0, "origin_y": -2.0}
    assert me.world_to_pixel(-0.9, -2.0, geo2, 40)[0] == 2


def test_origin_pixel_is_bottom_left():
    geo, height = _geo(), 20
    col, row = me.world_to_pixel(geo["origin_x"] + 0.01, geo["origin_y"] + 0.01, geo, height)
    assert (col, row) == (0, height - 1)


# ── 涂改 ──

def test_paint_marks_expected_cell():
    grid, geo = _grid(), _geo()
    # 栅格 20x30 @0.05 覆盖 x[-1.0,0.5) y[-2.0,-1.0)，取正中间免得踩边界
    changed = me.paint_strokes(grid, geo, [[(0.0, -1.5)]], "occupied", radius_m=0.05)
    assert changed > 0
    col, row = me.world_to_pixel(0.0, -1.5, geo, grid.shape[0])
    assert grid[row, col] == me.OCCUPIED


def test_paint_interpolates_between_points():
    """鼠标快速拖动时两个采样点隔很远，中间必须补上，否则画成断续圆点。"""
    grid, geo = _grid(), _geo()
    me.paint_strokes(grid, geo, [[(-0.5, -1.0), (0.5, -1.0)]], "occupied", radius_m=0.05)
    cols = np.where((grid == me.OCCUPIED).any(axis=0))[0]
    # 覆盖的列必须是连续的
    assert np.array_equal(cols, np.arange(cols[0], cols[-1] + 1))


def test_paint_clips_at_border_without_wrapping():
    """画到边界外不能绕到另一侧去。"""
    grid, geo = _grid(), _geo()
    me.paint_strokes(grid, geo, [[(-5.0, -5.0)]], "occupied", radius_m=0.2)
    assert grid[0, -1] != me.OCCUPIED, "左下角的笔画绕到了右上角"


def test_paint_rejects_unknown_brush():
    grid, geo = _grid(), _geo()
    try:
        me.paint_strokes(grid, geo, [[(0.0, 0.0)]], "rainbow", 0.1)
    except me.MapEditError:
        return
    raise AssertionError("未知画笔必须报错，不能往地图里写任意灰度")


# ── 禁行区 ──

def test_bake_zone_fills_rectangle():
    grid, geo = _grid(), _geo()
    me.bake_zones(grid, geo, [{"x": 0.0, "y": -1.6, "w": 0.2, "h": 0.2}])
    occupied = np.argwhere(grid == me.OCCUPIED)
    assert len(occupied) > 0
    # 矩形四角都应落在涂黑区域内
    for cx, cy in ((0.01, -1.59), (0.19, -1.59), (0.01, -1.41), (0.19, -1.41)):
        col, row = me.world_to_pixel(cx, cy, geo, grid.shape[0])
        assert grid[row, col] == me.OCCUPIED


def test_bake_zone_ignores_degenerate_and_bad_input():
    grid, geo = _grid(), _geo()
    painted = me.bake_zones(grid, geo, [
        {"x": 0, "y": 0, "w": 0, "h": 1},        # 零宽
        {"x": 0, "y": 0},                        # 缺字段
        {"x": "a", "y": 0, "w": 1, "h": 1},      # 非数字
        {"x": 99, "y": 99, "w": 1, "h": 1},      # 完全在图外
    ])
    assert painted == 0
    assert not (grid == me.OCCUPIED).any()


def test_edits_roundtrip(tmp_path):
    base = tmp_path / "map"
    zones = [{"x": 1.0, "y": 2.0, "w": 0.5, "h": 0.5}]
    me.save_edits(base, {"zones": zones})
    assert me.load_edits(base)["zones"] == zones


def test_load_edits_survives_corrupt_file(tmp_path):
    base = tmp_path / "map"
    Path(str(base) + me.EDITS_SUFFIX).write_text("{ not json", encoding="utf-8")
    assert me.load_edits(base) == {"zones": []}


# ── 裁剪 ──

def test_crop_shifts_origin_so_world_points_stay_put():
    """裁剪后同一个世界坐标必须还指向同一块内容。"""
    grid, geo = _grid(h=40, w=40), _geo()
    mark_world = (0.3, -1.2)
    col, row = me.world_to_pixel(*mark_world, geo, grid.shape[0])
    grid[row, col] = me.OCCUPIED

    out, new_geo = me.crop(grid, geo, {"x": 0.1, "y": -1.5, "w": 0.6, "h": 0.6})
    ncol, nrow = me.world_to_pixel(*mark_world, new_geo, out.shape[0])
    assert out[nrow, ncol] == me.OCCUPIED, "裁剪后 origin 没跟着挪，世界坐标对不上了"


def test_autocrop_keeps_content_and_shifts_origin():
    grid, geo = _grid(h=60, w=60), _geo()
    grid[30, 30] = me.FREE
    out, new_geo = me.autocrop(grid, geo, margin_px=2)
    assert out.shape == (5, 5)
    x, y = me.pixel_to_world(30, 30, geo, grid.shape[0])
    ncol, nrow = me.world_to_pixel(x, y, new_geo, out.shape[0])
    assert out[nrow, ncol] == me.FREE


def test_autocrop_refuses_all_unknown():
    try:
        me.autocrop(_grid(), _geo())
    except me.MapEditError:
        return
    raise AssertionError("全未知的地图没有可裁剪内容，必须报错")


# ── 旋转 ──

def test_rotate_90_swaps_dimensions_and_recenters():
    grid, geo = _grid(h=20, w=30), _geo()
    out, new_geo = me.rotate(grid, geo, 90)
    assert out.shape == (30, 20)
    res = geo["resolution"]
    old_center = (geo["origin_x"] + 30 * res / 2, geo["origin_y"] + 20 * res / 2)
    new_center = (new_geo["origin_x"] + 20 * res / 2, new_geo["origin_y"] + 30 * res / 2)
    assert abs(old_center[0] - new_center[0]) < 1e-9
    assert abs(old_center[1] - new_center[1]) < 1e-9


def test_rotate_360_is_identity():
    grid, geo = _grid(h=8, w=12), _geo()
    grid[2, 3] = me.OCCUPIED
    out, new_geo = me.rotate(grid, geo, 360)
    assert np.array_equal(out, grid)
    assert new_geo == geo


def test_rotate_rejects_non_multiples_of_90():
    try:
        me.rotate(_grid(), _geo(), 45)
    except me.MapEditError:
        return
    raise AssertionError("非 90 度倍数必须报错")


# ── 备份 / 还原 ──

def test_backup_keeps_the_original_not_the_latest(tmp_path):
    """连续编辑两次后还原，必须回到最初那张，而不是上一次编辑的结果。"""
    path = tmp_path / "m.pgm"
    original = _grid()
    me.write_pgm(path, original)

    me.ensure_backup(path)
    edited = original.copy()
    edited[0, 0] = me.OCCUPIED
    me.write_pgm(path, edited)

    me.ensure_backup(path)          # 第二次编辑前再调一次，不该覆盖备份
    edited2 = edited.copy()
    edited2[1, 1] = me.FREE
    me.write_pgm(path, edited2)

    me.revert(path)
    assert np.array_equal(me.read_pgm(path), original)


def test_revert_without_backup_errors(tmp_path):
    path = tmp_path / "m.pgm"
    me.write_pgm(path, _grid())
    try:
        me.revert(path)
    except me.MapEditError:
        return
    raise AssertionError("没有备份时还原必须报错，不能静默成功")


# ── yaml ──

def test_map_yaml_roundtrip(tmp_path):
    path = tmp_path / "m.yaml"
    me.write_map_yaml(path, "m.pgm", 0.05, -1.25, 2.5)
    got = me.read_map_yaml(path)
    assert got["image"] == "m.pgm"
    assert abs(got["resolution"] - 0.05) < 1e-9
    assert abs(got["origin_x"] + 1.25) < 1e-9
    assert abs(got["origin_y"] - 2.5) < 1e-9


# ── 三层文件模型 ──

def _setup_layers(tmp_path):
    path = tmp_path / "m.pgm"
    grid = _grid(h=40, w=40, fill=me.FREE)
    me.write_pgm(path, grid)
    me.ensure_backup(path)
    me.ensure_base(path)
    return path, _geo()


def test_zone_removal_restores_pixels(tmp_path):
    """删掉禁行区必须能还原——这正是要中间那层底图的原因。"""
    path, geo = _setup_layers(tmp_path)
    zone = {"x": 0.0, "y": -1.6, "w": 0.2, "h": 0.2}

    me.rebuild(path, geo, [zone])
    assert (me.read_pgm(path) == me.OCCUPIED).any()

    me.rebuild(path, geo, [])          # 删掉禁行区
    assert not (me.read_pgm(path) == me.OCCUPIED).any(), "禁行区删了却洗不掉"


def test_paint_survives_zone_changes(tmp_path):
    """涂改进底图，禁行区增删不该把涂改一起抹掉。"""
    path, geo = _setup_layers(tmp_path)
    base = me.base_path_for(path)

    grid = me.read_pgm(base)
    me.paint_strokes(grid, geo, [[(0.0, -1.0)]], "occupied", 0.05)
    me.write_pgm(base, grid)

    col, row = me.world_to_pixel(0.0, -1.0, geo, 40)
    me.rebuild(path, geo, [{"x": 0.5, "y": -1.6, "w": 0.2, "h": 0.2}])
    assert me.read_pgm(path)[row, col] == me.OCCUPIED
    me.rebuild(path, geo, [])
    assert me.read_pgm(path)[row, col] == me.OCCUPIED, "删禁行区把涂改也抹了"


def test_revert_also_resets_the_base_layer(tmp_path):
    """还原只复位 .pgm 不复位底图的话，下次禁行区一改编辑就全回来了。"""
    path, geo = _setup_layers(tmp_path)
    base = me.base_path_for(path)

    grid = me.read_pgm(base)
    me.paint_strokes(grid, geo, [[(0.0, -1.0)]], "occupied", 0.05)
    me.write_pgm(base, grid)
    me.rebuild(path, geo, [])

    me.revert(path)
    me.rebuild(path, geo, [])          # 再走一次重建
    assert not (me.read_pgm(path) == me.OCCUPIED).any(), "还原后涂改又冒出来了"


def test_transform_applies_to_all_three_layers(tmp_path):
    """只裁 .pgm 不裁底图，下次重建地图尺寸会突然变回去。"""
    path, geo = _setup_layers(tmp_path)
    new_geo = me.apply_transform_to_all(
        path, lambda g: me.crop(g, geo, {"x": -0.5, "y": -1.5, "w": 0.8, "h": 0.8})
    )
    shapes = {
        me.read_pgm(path).shape,
        me.read_pgm(me.base_path_for(path)).shape,
        me.read_pgm(Path(str(path) + me.ORIG_SUFFIX)).shape,
    }
    assert len(shapes) == 1, "三层尺寸不一致: {}".format(shapes)
    me.rebuild(path, new_geo, [])
    assert me.read_pgm(path).shape == shapes.pop()

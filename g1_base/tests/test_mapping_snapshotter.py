"""MappingSnapshotter 单测：不依赖 ROS，通过 ingest_points + render_now 直接验证。"""

from collections import Counter
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_base.mapping_snapshotter import MappingSnapshotter


def _read_pgm_pixels(path):
    with open(path, "rb") as f:
        assert f.readline() == b"P5\n"
        width, height = [int(value) for value in f.readline().split()]
        assert f.readline() == b"255\n"
        return width, height, f.read()


def _make_snapshotter(tmp_path, base_name="20260430_120000", **overrides):
    return MappingSnapshotter(
        node=None,
        maps_dir=tmp_path,
        base_name=base_name,
        resolution=overrides.get("resolution", 0.05),
        z_min=overrides.get("z_min", -0.8),
        z_max=overrides.get("z_max", 0.5),
        padding=overrides.get("padding", 0.1),
        snapshot_period_sec=overrides.get("snapshot_period_sec", 20.0),
        manifest_writer=overrides.get("manifest_writer"),
        hard_cap_cells=overrides.get("hard_cap_cells"),
    )


def test_render_marks_occupied_and_unknown(tmp_path):
    snap = _make_snapshotter(tmp_path)
    points = np.array(
        [
            [0.00, 0.00, 0.0],  # in z range
            [0.20, 0.10, 0.0],  # in z range
            [1.00, 1.00, 1.0],  # OUT of z range (z>0.5) — should be filtered
            [0.50, -0.50, -0.9],  # OUT of z range (z<-0.8)
        ],
        dtype=np.float32,
    )
    snap.ingest_points(points)
    out = snap.render_now()
    assert out is not None
    pgm_path, yaml_path = out
    assert Path(pgm_path).is_file()
    assert Path(yaml_path).is_file()

    width, height, pixels = _read_pgm_pixels(pgm_path)
    counts = Counter(pixels)
    # 两个有效点落在不同 cell（resolution 0.05，(0,0) 和 (4,2)）
    assert counts[0] == 2
    assert counts[205] == width * height - 2
    assert counts[254] == 0


def test_render_returns_none_when_empty(tmp_path):
    snap = _make_snapshotter(tmp_path)
    assert snap.render_now() is None


def test_bbox_expands_with_new_points(tmp_path):
    snap = _make_snapshotter(tmp_path)
    snap.ingest_points(np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    pgm1, _ = snap.render_now()
    width1, height1, _ = _read_pgm_pixels(pgm1)

    # 加入更远的点，bbox 应扩大
    snap.ingest_points(np.array([[5.0, 5.0, 0.0]], dtype=np.float32))
    pgm2, _ = snap.render_now()
    width2, height2, pixels2 = _read_pgm_pixels(pgm2)

    assert width2 > width1
    assert height2 > height1
    counts = Counter(pixels2)
    # 仍然两个占据点
    assert counts[0] == 2


def test_hard_cap_stops_new_cells(tmp_path):
    snap = _make_snapshotter(tmp_path, hard_cap_cells=3)
    # 喂足够多不同 cell 的点
    pts = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [0.3, 0.0, 0.0],  # 这一帧应该全被拒（已到 cap）
            [0.4, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    # 分两次喂以触发 cap 检查（第二次 ingest 时 len(cells)>=cap 就完全拒绝）
    snap.ingest_points(pts[:3])
    assert len(snap.cells) == 3
    snap.ingest_points(pts[3:])
    assert len(snap.cells) == 3
    assert snap.cap_warned


def test_manifest_writer_called(tmp_path):
    calls = []
    snap = _make_snapshotter(
        tmp_path, manifest_writer=lambda ts: calls.append(ts)
    )
    snap.ingest_points(np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    snap.render_now()
    assert len(calls) == 1
    # 是 ISO 格式的时间字符串
    assert "T" in calls[0]


def test_atomic_write_no_tmp_residue(tmp_path):
    snap = _make_snapshotter(tmp_path)
    snap.ingest_points(np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    snap.render_now()
    residues = list(tmp_path.glob("*.tmp"))
    assert residues == []


def test_stop_is_idempotent(tmp_path):
    snap = _make_snapshotter(tmp_path)
    snap.stop()
    snap.stop()  # 第二次不应抛

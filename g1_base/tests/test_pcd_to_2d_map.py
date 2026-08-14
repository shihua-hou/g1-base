from collections import Counter
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_base import pcd_to_2d_map as pcd_map
from g1_base.pcd_to_2d_map import (
    TiltedWorldError,
    convert_pcd_to_2d_map,
    validate_ground_alignment,
)


def _write_ascii_pcd(path, points):
    rows = "\n".join(f"{x} {y} {z}" for x, y, z in points)
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                f"WIDTH {len(points)}",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                f"POINTS {len(points)}",
                "DATA ascii",
                rows,
                "",
            ]
        ),
        encoding="ascii",
    )


def _read_pgm_pixels(path):
    with open(path, "rb") as f:
        assert f.readline() == b"P5\n"
        width, height = [int(value) for value in f.readline().split()]
        assert f.readline() == b"255\n"
        return width, height, f.read()


def _plane_points(angle_deg=0.0, noise=False):
    slope = math.tan(math.radians(angle_deg))
    points = []
    for ix in range(-5, 6):
        for iy in range(-5, 6):
            x = ix * 0.5
            y = iy * 0.5
            z = x * slope
            if noise:
                z += (((ix * 17 + iy * 31) % 5) - 2) * 0.001
            points.append((x, y, z))
    return points


def _as_points_array(points):
    return pcd_map.np.array(points, dtype=pcd_map.np.float64)


def test_unobserved_cells_are_unknown_not_free(tmp_path):
    pcd_path = tmp_path / "map.pcd"
    _write_ascii_pcd(
        pcd_path,
        [
            (-0.1, -0.1, 0.0),
            (0.1, -0.1, 0.0),
            (-0.1, 0.1, 0.0),
            (0.1, 0.1, 0.0),
        ],
    )

    pgm_path, _yaml_path = convert_pcd_to_2d_map(
        pcd_path=pcd_path,
        output_dir=tmp_path,
        output_name="test_map",
        resolution=0.1,
        z_min=-0.1,
        z_max=0.1,
        padding=0.1,
    )

    width, height, pixels = _read_pgm_pixels(pgm_path)
    counts = Counter(pixels)

    assert width == 4
    assert height == 4
    assert counts[0] == 4
    assert counts[205] == (width * height) - 4
    assert counts[254] == 0


def test_horizontal_noisy_plane_passes_ground_alignment(tmp_path):
    points = _plane_points(angle_deg=0.0, noise=True)
    alignment = validate_ground_alignment(_as_points_array(points))
    assert alignment.tilt_deg < 0.2

    pcd_path = tmp_path / "level_map.pcd"
    _write_ascii_pcd(pcd_path, points)

    pgm_path, yaml_path = convert_pcd_to_2d_map(
        pcd_path=pcd_path,
        output_dir=tmp_path,
        output_name="level_map",
        resolution=0.1,
        z_min=-0.1,
        z_max=0.1,
        padding=0.1,
    )

    assert Path(pgm_path).is_file()
    assert Path(yaml_path).is_file()


def test_ground_relative_height_filter_drops_floor_keeps_obstacle_plane(tmp_path):
    floor_points = [
        (float(ix), float(iy), 0.05)
        for ix in range(3)
        for iy in range(3)
    ]
    obstacle_plane = [
        (0.0, 0.0, 0.35),
        (2.0, 0.0, 0.35),
        (0.0, 2.0, 0.35),
        (2.0, 2.0, 0.35),
    ]

    pcd_path = tmp_path / "height_filter_map.pcd"
    _write_ascii_pcd(pcd_path, floor_points + obstacle_plane)

    pgm_path, _yaml_path = convert_pcd_to_2d_map(
        pcd_path=pcd_path,
        output_dir=tmp_path,
        output_name="height_filter_map",
        resolution=1.0,
        padding=0.5,
    )

    width, height, pixels = _read_pgm_pixels(pgm_path)
    counts = Counter(pixels)

    assert width == 3
    assert height == 3
    assert counts[0] == len(obstacle_plane)
    assert counts[205] == (width * height) - len(obstacle_plane)
    assert counts[254] == 0


def test_tilted_plane_rejected_before_projection(tmp_path):
    pcd_path = tmp_path / "tilted_map.pcd"
    _write_ascii_pcd(pcd_path, _plane_points(angle_deg=10.0))

    try:
        convert_pcd_to_2d_map(
            pcd_path=pcd_path,
            output_dir=tmp_path,
            output_name="tilted_map",
            resolution=0.1,
            z_min=-1.0,
            z_max=1.0,
            padding=0.1,
        )
    except TiltedWorldError as exc:
        assert 9.9 < exc.tilt_deg < 10.1
    else:
        assert False, "expected tilted PCD to be rejected"

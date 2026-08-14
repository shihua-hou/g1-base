"""建图过程中按 /lio/cloud_world 累积占据栅格，每 N 秒落盘一张 PGM/YAML 快照。

设计要点:
- 累积结构: set[(col, row)]，bbox 自适应；每帧 numpy 量化后 set.update。
- 落盘命名与最终图同名（覆盖式），bot_mind 通过 current_map.json 即可取到。
- 失败隔离: timer/callback 内部异常 catch 后 log，不影响建图主流程。
- 内存上限: HARD_CAP_CELLS 兜底，避免野外失控；单次会话内 GC 自然释放。
"""

import os
import threading
from datetime import datetime
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None

try:
    from rclpy.qos import QoSPresetProfiles
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # 允许单元测试在无 ROS 环境 import 本模块（仅测 ingest_points/render_now）
    QoSPresetProfiles = None
    PointCloud2 = None
    pc2 = None

from g1_base.pcd_to_2d_map import LEGACY_Z_MAX, LEGACY_Z_MIN, _save_pgm, _save_yaml


class MappingSnapshotter:
    """建图阶段的 2D 占据栅格累积 + 周期性快照。"""

    HARD_CAP_CELLS = 5_000_000  # 兜底：超此 cell 数停止收新格（防野外失控，对应 ~400MB）

    def __init__(
        self,
        node,
        maps_dir,
        base_name,
        cloud_topic="/lio/cloud_world",
        resolution=0.05,
        z_min=LEGACY_Z_MIN,
        z_max=LEGACY_Z_MAX,
        padding=1.0,
        snapshot_period_sec=20.0,
        manifest_writer=None,
        callback_group=None,
        hard_cap_cells=None,
    ):
        if np is None:
            raise ImportError("numpy 未安装，请运行: pip3 install numpy")

        self.node = node
        self.maps_dir = Path(maps_dir)
        self.base_name = base_name
        self.cloud_topic = cloud_topic
        self.resolution = float(resolution)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.padding = float(padding)
        self.snapshot_period_sec = float(snapshot_period_sec)
        self.manifest_writer = manifest_writer
        self.hard_cap_cells = (
            int(hard_cap_cells) if hard_cap_cells is not None else self.HARD_CAP_CELLS
        )

        self.cells = set()
        self.cap_warned = False
        self.lock = threading.Lock()
        self._stopped = False

        self.maps_dir.mkdir(parents=True, exist_ok=True)

        self._sub = None
        self._timer = None
        if node is not None and PointCloud2 is not None:
            qos = QoSPresetProfiles.SENSOR_DATA.value
            sub_kwargs = {}
            timer_kwargs = {}
            if callback_group is not None:
                sub_kwargs["callback_group"] = callback_group
                timer_kwargs["callback_group"] = callback_group
            self._sub = node.create_subscription(
                PointCloud2, cloud_topic, self._on_cloud, qos, **sub_kwargs
            )
            self._timer = node.create_timer(
                self.snapshot_period_sec, self._on_timer, **timer_kwargs
            )

    # ── ROS 回调 ──

    def _on_cloud(self, msg):
        try:
            xyz = self._extract_xyz(msg)
            if xyz is None or len(xyz) == 0:
                return
            self.ingest_points(xyz)
        except Exception as exc:
            self._log_warn(f"snapshotter cloud cb error: {exc}")

    def _extract_xyz(self, msg):
        # sensor_msgs_py.point_cloud2.read_points 在 humble 上 numpy 接口稳定
        arr = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        # 兼容两种返回：structured numpy 数组 或 generator/list of tuples
        if isinstance(arr, np.ndarray):
            xyz = np.column_stack([arr["x"], arr["y"], arr["z"]])
        else:
            xyz = np.fromiter(
                (v for pt in arr for v in pt),
                dtype=np.float32,
            ).reshape(-1, 3)
        return xyz

    def _on_timer(self):
        try:
            self.render_now()
        except Exception as exc:
            self._log_warn(f"snapshotter render error: {exc}")

    # ── 测试钩子 ──

    def ingest_points(self, xyz):
        """喂 (N, 3) numpy 数组到累积器。z 过滤 + 量化 + set.update。"""
        if np is None:
            raise ImportError("numpy 未安装")
        xyz = np.asarray(xyz, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] == 0:
            return

        mask = (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)
        filtered = xyz[mask]
        if filtered.shape[0] == 0:
            return

        cols = np.floor(filtered[:, 0] / self.resolution).astype(np.int32)
        rows = np.floor(filtered[:, 1] / self.resolution).astype(np.int32)
        new_pairs = set(zip(cols.tolist(), rows.tolist()))

        with self.lock:
            if len(self.cells) >= self.hard_cap_cells:
                if not self.cap_warned:
                    self._log_warn(
                        f"snapshotter hit HARD_CAP_CELLS={self.hard_cap_cells}, "
                        f"stopping new-cell ingestion"
                    )
                    self.cap_warned = True
                return
            self.cells.update(new_pairs)

    def render_now(self):
        """同步渲染一张快照到 maps_dir。返回 (pgm_path, yaml_path) 或 None。

        若累积为空（无任何命中格），不落盘并返回 None。
        """
        with self.lock:
            if not self.cells:
                return None
            cells_snapshot = self.cells.copy()

        cols = np.fromiter((c for c, _ in cells_snapshot), dtype=np.int32)
        rows = np.fromiter((r for _, r in cells_snapshot), dtype=np.int32)

        pad_cells = int(np.ceil(self.padding / self.resolution))
        col_min = int(cols.min()) - pad_cells
        col_max = int(cols.max()) + pad_cells
        row_min = int(rows.min()) - pad_cells
        row_max = int(rows.max()) + pad_cells

        width = col_max - col_min + 1
        height = row_max - row_min + 1

        grid = np.full((height, width), 205, dtype=np.uint8)
        local_cols = cols - col_min
        local_rows = rows - row_min
        grid[local_rows, local_cols] = 0
        grid = np.flipud(grid)

        origin_x = col_min * self.resolution
        origin_y = row_min * self.resolution

        pgm_path = self.maps_dir / f"{self.base_name}_exhibit_2d_map.pgm"
        yaml_path = self.maps_dir / f"{self.base_name}_exhibit_2d_map.yaml"

        # 原子写：先写 .tmp，再 os.replace
        pgm_tmp = pgm_path.with_name(pgm_path.name + ".tmp")
        _save_pgm(pgm_tmp, grid)
        os.replace(pgm_tmp, pgm_path)

        yaml_tmp = yaml_path.with_name(yaml_path.name + ".tmp")
        _save_yaml(yaml_tmp, pgm_path.name, self.resolution, origin_x, origin_y)
        os.replace(yaml_tmp, yaml_path)

        if self.manifest_writer is not None:
            try:
                self.manifest_writer(datetime.now().isoformat(timespec="seconds"))
            except Exception as exc:
                self._log_warn(f"manifest_writer failed: {exc}")

        return str(pgm_path), str(yaml_path)

    # ── 生命周期 ──

    def stop(self):
        """幂等停止：取消订阅与 timer。"""
        if self._stopped:
            return
        self._stopped = True
        if self._sub is not None and self.node is not None:
            try:
                self.node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        if self._timer is not None and self.node is not None:
            try:
                self._timer.cancel()
                self.node.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None

    # ── 日志 ──

    def _log_warn(self, msg):
        if self.node is not None and hasattr(self.node, "get_logger"):
            try:
                self.node.get_logger().warning(f"[snapshotter] {msg}")
                return
            except Exception:
                pass
        print(f"[snapshotter] WARN: {msg}")

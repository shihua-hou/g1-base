"""iPad 上位机 HTTP 网关：把 g1_control_server / navigation_manager 的 ROS2
服务与动作，包装成一套 JSON REST API + 静态网页服务，供 iPad Safari 直接访问。

不引入 FastAPI/Flask 等三方依赖，全部基于标准库 http.server 实现，
方便在 JetPack/Ubuntu 目标机上零安装运行。

用法::

    ros2 run g1_base g1_web_bridge --net-if enP8p1s0 --port 8081

iPad 端浏览器打开 http://<机器人IP>:8081/ 即可。
"""

import argparse
import json
import math
import sys
from array import array
import os
import re
import shutil
import struct
import threading
import time
import zlib
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from sensor_msgs.msg import CompressedImage, PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover - 缺这些只影响实时地图与相机
    CompressedImage = None
    PointCloud2 = None
    pc2 = None

try:
    # 注意别和 pathlib.Path 撞名
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as NavPath
except ImportError:  # pragma: no cover - 缺它只影响地图上的规划路径与建图位姿
    NavPath = None
    Odometry = None

from g1_base.common import config_file, movement_dir, package_share_dir
from g1_base_interfaces.action import NavigateToTarget
from g1_base_interfaces.srv import (
    ExecuteArmAction,
    ExecuteCustomAction,
    GetFsmId,
    MoveRobot,
    PlayNamedAction,
    Relocalize,
    RotateRobot,
    RunMovementScript,
    SetFsmId,
    SquatRobot,
    StopRobot,
)

def _webapp_dir():
    configured = os.environ.get("G1_WEBAPP_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path(package_share_dir()) / "webapp"


WEBAPP_DIR = _webapp_dir()
MAP_NAME_SUFFIX = "exhibit_2d_map"

# 摇杆遥操作话题：g1_control_server 订阅它并转成 set_manual_velocity
TELEOP_TOPIC = "/g1_control/teleop_cmd_vel"

_ASSET_REF_RE = re.compile(rb'(src|href)="/([^"?]+\.(?:js|css))"')


def _assets_version():
    """webapp/assets/ 下所有文件的最新 mtime，作为整包资源的版本号。"""
    newest = 0
    assets_dir = WEBAPP_DIR / "assets"
    try:
        for item in assets_dir.iterdir():
            if item.is_file():
                newest = max(newest, int(item.stat().st_mtime))
    except OSError:
        pass
    return newest


def _stamp_asset_versions(html):
    """给 index.html 里引用的本地 js/css 打上文件 mtime 版本号。

    Cache-Control 只约束听话的客户端，中间代理 / 平板 WebView 仍可能发回
    旧脚本，表现是更新后界面没变、或新旧代码混跑。带上 ?v=<mtime> 后 URL
    本身会变，任何一层缓存都绕不过去。

    同时注入 window.__G1_ASSET_V：模型与动作包是 robot3d.js 在运行时 fetch
    的，不经过 HTML，只能靠这个版本号让它们跟着一起失效。
    """
    def repl(match):
        attr, rel = match.group(1), match.group(2).decode("utf-8")
        target = WEBAPP_DIR / rel
        try:
            stamp = int(target.stat().st_mtime)
        except OSError:
            return match.group(0)
        return b'%s="/%s?v=%d"' % (attr, rel.encode("utf-8"), stamp)

    html = _ASSET_REF_RE.sub(repl, html)
    inject = b'<script>window.__G1_ASSET_V="%d";</script>' % _assets_version()
    return html.replace(b"</head>", inject + b"\n</head>", 1)


def _quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _yaw_from_quaternion(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class ApiError(Exception):
    def __init__(self, message, code=400):
        super().__init__(message)
        self.message = message
        self.code = code


# ── 建图实时视图 ──

class LiveMapView:
    """把 super-lio 的 /lio/cloud_world 累积成 2D 栅格，供建图页实时预览。

    和 mapping_snapshotter 的 20 秒落盘快照不同，这里只在内存里攒，按需渲染
    PNG，建图时能看着地图一格一格长出来。栅格数有硬上限，防止走大场地时把
    内存吃光。
    """

    HARD_CAP_CELLS = 3_000_000
    # 3D 体素上限：一格 12 字节，40 万格 ≈ 4.8MB，浏览器一次性吃得下
    HARD_CAP_VOXELS = 400_000
    # 体素坐标打包进 int64 时每轴的偏移（±100 万格，8cm 下 ±80km，够用了）
    _VOXEL_BIAS = 1 << 20

    def __init__(self, node, cloud_topic="/lio/cloud_world", resolution=0.05,
                 z_min=-0.35, z_max=1.60, voxel_size=0.08, map_frame="map",
                 frame_override="", voxel_z_min=-6.0, voxel_z_max=6.0,
                 callback_group=None):
        self.node = node
        self.topic = cloud_topic
        self.resolution = float(resolution)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.voxel_size = float(voxel_size)
        # 3D 视图的离群带。比 2D 的 [z_min, z_max] 宽得多 —— 天花板、
        # 货架、二层挑空都要留住，只拦明显是脏数据的点。
        # LIO 刚起步时位姿还没收敛，偶尔会甩出 z=-33m 这种点，
        # 它们会把前端高度配色的归一化区间撑爆（真实结构全挤进色带顶端
        # 几个百分点，看上去就是一片糊的单色），所以必须在入口就拦掉。
        self.voxel_z_min = float(voxel_z_min)
        self.voxel_z_max = float(voxel_z_max)
        self.dropped_outlier = 0
        # 点云未必是世界系的：G1 上 /lio/cloud_world 实际是 DDS 域桥把
        # /utlidar/cloud_livox_mid360（传感器系）改了个名字，直接堆会糊成一团。
        # 这里统一按 TF 变换到 map 系再累积；变换不了就不堆，并如实上报原因。
        self.map_frame = map_frame
        self.frame_override = (frame_override or "").strip()
        self.cloud_frame = ""
        self.tf_ok = False
        self.dropped_no_tf = 0
        self.tf_error = ""
        self.cells = set()
        # 3D 点云：按体素去重后按到达顺序追加，前端用下标做游标增量拉取
        self._voxel_keys = set()
        self._voxel_xyz = array("f")
        self._z_range = [None, None]
        self._ground_cache = None      # (体素数, 地面 z)，见 ground_level()
        self.lock = threading.Lock()
        self.last_cloud_time = 0.0
        self.cloud_count = 0
        self._sub = None

        if PointCloud2 is None or pc2 is None:
            node.get_logger().warning("sensor_msgs_py 不可用，建图实时预览关闭")
            return
        kwargs = {"callback_group": callback_group} if callback_group is not None else {}
        self._sub = node.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud,
            QoSPresetProfiles.SENSOR_DATA.value, **kwargs,
        )

    def _on_cloud(self, msg):
        try:
            arr = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            res = self.resolution
            new_cells = set()
            xyz_all = None
            if np is not None and isinstance(arr, np.ndarray):
                xyz_all = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
                xyz_all = self._to_map_frame(xyz_all, msg.header)
                if xyz_all is None:
                    with self.lock:
                        self.last_cloud_time = time.time()
                        self.cloud_count += 1
                    return
                # 2D 栅格只要机器人能撞到的高度层，3D 视图用整帧（见 _accumulate_voxels）
                xyz = xyz_all[(xyz_all[:, 2] >= self.z_min) & (xyz_all[:, 2] <= self.z_max)]
                if xyz.shape[0]:
                    cols = np.floor(xyz[:, 0] / res).astype(np.int32)
                    rows = np.floor(xyz[:, 1] / res).astype(np.int32)
                    new_cells = set(zip(cols.tolist(), rows.tolist()))
            else:
                for x, y, z in arr:
                    if self.z_min <= z <= self.z_max:
                        new_cells.add((int(math.floor(x / res)), int(math.floor(y / res))))
            with self.lock:
                self.last_cloud_time = time.time()
                self.cloud_count += 1
                if len(self.cells) < self.HARD_CAP_CELLS:
                    self.cells.update(new_cells)
            self._accumulate_voxels(xyz_all)
        except Exception as exc:
            self.node.get_logger().warning(f"live map cloud error: {exc}", throttle_duration_sec=5.0)

    def _to_map_frame(self, xyz, header):
        """把点云变换到 map 系。返回 None 表示这一帧不能用，别往地图上堆。

        没有 TF 就意味着没有定位——这种时候把传感器系的点直接累积，
        机器人一转整张图就糊了。宁可不画，也要让界面说清楚为什么。
        """
        frame = self.frame_override or (header.frame_id or "").strip()
        self.cloud_frame = frame or "(空)"
        if not frame or frame == self.map_frame:
            self.tf_ok = True
            self.tf_error = ""
            return xyz

        tf_buffer = getattr(self.node, "tf_buffer", None)
        if tf_buffer is None:
            self.tf_ok = False
            self.tf_error = "网关没有 TF 缓存"
            self.dropped_no_tf += 1
            return None

        tr = None
        for stamp in (rclpy.time.Time.from_msg(header.stamp), rclpy.time.Time()):
            try:
                tr = tf_buffer.lookup_transform(self.map_frame, frame, stamp)
                break
            except TransformException:
                continue
        if tr is None:
            self.tf_ok = False
            self.tf_error = f"查不到 TF {self.map_frame}<-{frame}（定位没起来？）"
            self.dropped_no_tf += 1
            return None

        q = tr.transform.rotation
        t = tr.transform.translation
        # 四元数转旋转矩阵
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        self.tf_ok = True
        self.tf_error = ""
        return xyz @ rot.T + np.array([t.x, t.y, t.z])

    def _accumulate_voxels(self, xyz):
        """把整帧点云按体素去重后追加进 3D 缓冲。

        和 2D 栅格不同，这里不按机器人高度过滤 —— 3D 视图就是要看天花板、
        货架、桌面这些立体结构，滤掉就退化成平面图了。
        只用一条宽松的离群带拦掉脏点，见 voxel_z_min/voxel_z_max。
        """
        if xyz is None or np is None or not len(xyz):
            return
        keep = (xyz[:, 2] >= self.voxel_z_min) & (xyz[:, 2] <= self.voxel_z_max)
        n_drop = int(len(xyz) - int(keep.sum()))
        if n_drop:
            with self.lock:
                self.dropped_outlier += n_drop
            xyz = xyz[keep]
            if not len(xyz):
                return
        vs = self.voxel_size
        bias = self._VOXEL_BIAS
        q = np.floor(xyz / vs).astype(np.int64) + bias
        if q.min() < 0 or q.max() >= (bias << 1):
            return   # 坐标离谱，多半是脏数据，整帧丢掉
        keys = q[:, 0] | (q[:, 1] << 21) | (q[:, 2] << 42)
        uniq, first_idx = np.unique(keys, return_index=True)

        with self.lock:
            if len(self._voxel_keys) >= self.HARD_CAP_VOXELS:
                return
            known = self._voxel_keys
            fresh = [(int(k), i) for k, i in zip(uniq.tolist(), first_idx.tolist()) if int(k) not in known]
            if not fresh:
                return
            room = self.HARD_CAP_VOXELS - len(known)
            fresh = fresh[:room]
            # 存体素中心，前端拿到就是可直接画的点
            for key, idx in fresh:
                known.add(key)
                px, py, pz = xyz[idx]
                self._voxel_xyz.append(float((math.floor(px / vs) + 0.5) * vs))
                self._voxel_xyz.append(float((math.floor(py / vs) + 0.5) * vs))
                self._voxel_xyz.append(float((math.floor(pz / vs) + 0.5) * vs))
            zs = xyz[[i for _, i in fresh], 2]
            lo, hi = float(zs.min()), float(zs.max())
            self._z_range[0] = lo if self._z_range[0] is None else min(self._z_range[0], lo)
            self._z_range[1] = hi if self._z_range[1] is None else max(self._z_range[1], hi)

    def ground_level(self):
        """估计地面在世界系里的 z，用来把 3D 视图的高度显示改成"离地高度"。

        LIO 的世界原点在雷达上（雷达装在头上，离地一米多），所以点云里
        z=0 是雷达高度而不是地面 —— 看图的人会以为机器人陷在地里。
        取 z 的 2% 分位数当地面：比最小值稳（挡得住零星穿地的野点），
        又比中位数低得多（中位数会落在墙面上）。
        """
        if np is None:
            return None
        with self.lock:
            n = len(self._voxel_xyz) // 3
            if n < 200:
                return None
            # 体素只增不减，涨幅不到 5% 就沿用上次结果，别每 700ms 排一次序
            if self._ground_cache and n - self._ground_cache[0] < max(200, n * 0.05):
                return self._ground_cache[1]
            zs = np.frombuffer(memoryview(self._voxel_xyz), dtype=np.float32)[2::3]
            ground = float(np.percentile(zs, 2.0))
            self._ground_cache = (n, ground)
            return ground

    def cloud_since(self, since=0, max_points=60000):
        """增量取体素：返回 (float32 小端字节, 本次起始下标, 总数)。

        前端把 since 一路往后推，就能像贴瓷砖一样把点云攒起来，
        不必每次重传整片地图。
        """
        with self.lock:
            total = len(self._voxel_xyz) // 3
            since = max(0, min(int(since), total))
            end = min(total, since + int(max_points))
            chunk = self._voxel_xyz[since * 3:end * 3]
            zlo, zhi = self._z_range
        payload = chunk.tobytes()
        if sys.byteorder != "little":       # 前端按小端解析
            payload = array("f", chunk).byteswap().tobytes()
        return payload, since, end, total, zlo, zhi

    def reset(self):
        with self.lock:
            self.cells = set()
            self.cloud_count = 0
            self._voxel_keys = set()
            self._voxel_xyz = array("f")
            self._z_range = [None, None]
            self.dropped_no_tf = 0
        return {"success": True, "message": "实时地图已清空"}

    def info(self):
        with self.lock:
            count = len(self.cells)
            age = (time.time() - self.last_cloud_time) if self.last_cloud_time else None
            clouds = self.cloud_count
            voxels = len(self._voxel_xyz) // 3
            zlo, zhi = self._z_range
        return {
            "topic": self.topic,
            "subscribed": self._sub is not None,
            "cell_count": count,
            "cloud_count": clouds,
            "last_cloud_age_sec": round(age, 2) if age is not None else None,
            "streaming": age is not None and age < 3.0,
            "resolution": self.resolution,
            "voxel_size": self.voxel_size,
            "voxel_count": voxels,
            "z_min": zlo,
            "z_max": zhi,
            # 地面在世界系里的 z。前端拿它把高度显示换算成离地高度，
            # 也是标定 lio.extrinsic.odom_robo 时"雷达离地多高"的现成读数。
            "ground_z": self.ground_level(),
            # 建图为什么没数据 / 为什么糊，全靠这几项说清楚
            "cloud_frame": self.cloud_frame,
            "map_frame": self.map_frame,
            "tf_ok": self.tf_ok,
            "tf_error": self.tf_error,
            "dropped_no_tf": self.dropped_no_tf,
            # 被离群带拦掉的点数。持续猛涨说明 LIO 在发散，不是显示问题
            "dropped_outlier": self.dropped_outlier,
        }

    def render(self, max_dim=900):
        """渲染成 PNG，返回 (png_bytes, geometry)。没有点云时返回 (None, None)。"""
        with self.lock:
            cells = self.cells.copy()
        if not cells:
            return None, None

        cols = [c for c, _ in cells]
        rows = [r for _, r in cells]
        pad = int(round(1.0 / self.resolution))
        col_min, col_max = min(cols) - pad, max(cols) + pad
        row_min, row_max = min(rows) - pad, max(rows) + pad
        width = col_max - col_min + 1
        height = row_max - row_min + 1

        scale = max(1, math.ceil(max(width, height) / max_dim))
        out_w = max(1, width // scale)
        out_h = max(1, height // scale)

        # 未知=205（与 nav2 的 PGM 约定一致），占据=0
        buf = bytearray(b"\xcd" * (out_w * out_h))
        for c, r in cells:
            ox = (c - col_min) // scale
            oy = (row_max - r) // scale          # 行翻转：世界 y 向上，图像 y 向下
            if 0 <= ox < out_w and 0 <= oy < out_h:
                buf[oy * out_w + ox] = 0

        geometry = {
            "width": width,
            "height": height,
            "resolution": self.resolution,
            "origin": [col_min * self.resolution, row_min * self.resolution, 0.0],
            "render_scale": scale,
            "render_width": out_w,
            "render_height": out_h,
        }
        return _gray_png_bytes(out_w, out_h, bytes(buf)), geometry


# ── 相机中继 ──

class CameraRelay:
    """转发 sensor_msgs/CompressedImage 的 JPEG 原始字节。

    直接转发压缩帧，Python 侧零编码开销；话题不存在时接口如实返回
    available=false，界面显示占位而不是假画面。
    """

    def __init__(self, node, topic="/camera/color/image_raw/compressed", callback_group=None):
        self.node = node
        self.topic = topic
        self._lock = threading.Lock()
        self._frame = None
        self._format = ""
        self._stamp = 0.0
        self._count = 0
        self._sub = None

        if CompressedImage is None:
            node.get_logger().warning("sensor_msgs.CompressedImage 不可用，相机中继关闭")
            return
        if not topic:
            return
        kwargs = {"callback_group": callback_group} if callback_group is not None else {}
        self._sub = node.create_subscription(
            CompressedImage, topic, self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value, **kwargs,
        )

    def _on_image(self, msg):
        with self._lock:
            self._frame = bytes(msg.data)
            self._format = str(msg.format or "")
            self._stamp = time.time()
            self._count += 1

    def info(self):
        with self._lock:
            age = (time.time() - self._stamp) if self._stamp else None
            size = len(self._frame) if self._frame else 0
            fmt, count = self._format, self._count
        return {
            "topic": self.topic,
            "subscribed": self._sub is not None,
            "available": age is not None and age < 3.0,
            "age_sec": round(age, 2) if age is not None else None,
            "frame_bytes": size,
            "format": fmt,
            "frame_count": count,
        }

    def frame(self):
        with self._lock:
            if not self._frame:
                return None, ""
            return self._frame, self._format


# ── 主机运行指标（读 /proc，无三方依赖；读不到一律返回 None） ──

_cpu_prev = {"idle": 0, "total": 0}


def _cpu_percent():
    try:
        with open("/proc/stat", "r") as fp:
            parts = fp.readline().split()[1:]
        values = [int(v) for v in parts]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        d_idle = idle - _cpu_prev["idle"]
        d_total = total - _cpu_prev["total"]
        _cpu_prev["idle"] = idle
        _cpu_prev["total"] = total
        if d_total <= 0:
            return None
        return round(100.0 * (1.0 - d_idle / d_total), 1)
    except Exception:
        return None


def _mem_percent():
    try:
        info = {}
        with open("/proc/meminfo", "r") as fp:
            for line in fp:
                key, _, rest = line.partition(":")
                info[key] = int(rest.split()[0])
        total = info.get("MemTotal", 0)
        available = info.get("MemAvailable", 0)
        if total <= 0:
            return None
        return round(100.0 * (1.0 - available / total), 1)
    except Exception:
        return None


def _primary_ip(preferred_if=None):
    import socket

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 53))
            return sock.getsockname()[0]
        finally:
            sock.close()
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return None


def _hostname():
    import socket

    try:
        return socket.gethostname()
    except Exception:
        return None


# ─────────────────────────────────────────────────────────────
# ROS2 桥接节点：把服务/动作调用封装成同步方法，供 HTTP handler 调用
# ─────────────────────────────────────────────────────────────

class BridgeNode(Node):
    def __init__(self, args):
        super().__init__("g1_web_bridge")
        self.args = args
        self._start_time = time.time()
        self._group = ReentrantCallbackGroup()

        self._control_status = {}
        self._nav_manager_status = {}
        self._status_lock = threading.Lock()

        self.create_subscription(
            String, "/g1_control/status", self._on_control_status,
            QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=self._group,
        )
        self.create_subscription(
            String, "/navigation_manager/detail", self._on_nav_manager_detail,
            QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=self._group,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # ── g1_control_server 客户端 ──
        self.cli_arm_action = self.create_client(ExecuteArmAction, "/g1_control/execute_arm_action", callback_group=self._group)
        self.cli_named_action = self.create_client(PlayNamedAction, "/g1_control/play_named_action", callback_group=self._group)
        self.cli_move = self.create_client(MoveRobot, "/g1_control/move_robot", callback_group=self._group)
        self.cli_rotate = self.create_client(RotateRobot, "/g1_control/rotate_robot", callback_group=self._group)
        self.cli_stop = self.create_client(StopRobot, "/g1_control/stop_robot", callback_group=self._group)
        self.cli_custom_action = self.create_client(ExecuteCustomAction, "/g1_control/execute_custom_action", callback_group=self._group)
        self.cli_movement_script = self.create_client(RunMovementScript, "/g1_control/run_movement_script", callback_group=self._group)
        self.cli_squat = self.create_client(SquatRobot, "/g1_control/squat_robot", callback_group=self._group)
        self.cli_set_fsm = self.create_client(SetFsmId, "/g1_control/set_fsm_id", callback_group=self._group)
        self.cli_get_fsm = self.create_client(GetFsmId, "/g1_control/get_fsm_id", callback_group=self._group)
        self.act_navigate = ActionClient(self, NavigateToTarget, "/g1_control/navigate_to_target", callback_group=self._group)

        # ── navigation_manager 客户端 ──
        self.cli_ensure_ready = self.create_client(Trigger, "/navigation_manager/ensure_ready", callback_group=self._group)
        self.cli_restart_all = self.create_client(Trigger, "/navigation_manager/restart_all", callback_group=self._group)
        self.cli_stop_all = self.create_client(Trigger, "/navigation_manager/stop_all", callback_group=self._group)
        self.cli_start_mapping = self.create_client(Trigger, "/navigation_manager/start_mapping", callback_group=self._group)
        self.cli_stop_mapping = self.create_client(Trigger, "/navigation_manager/stop_mapping", callback_group=self._group)
        self.cli_generate_2d_map = self.create_client(Trigger, "/navigation_manager/generate_2d_map", callback_group=self._group)
        self.cli_relocalize = self.create_client(Relocalize, "/navigation_manager/relocalize", callback_group=self._group)

        # ── 巡航（多点导航）运行状态 ──
        self._patrol_lock = threading.Lock()
        self._patrol_thread = None
        self._patrol_cancel = threading.Event()
        self._patrol_state = {"running": False, "route_name": None, "index": 0, "total": 0, "message": ""}

        # ── 单点导航反馈 ──
        self._nav_goal_handle = None
        self._nav_feedback = {"active": False, "waypoint_name": "", "phase": "", "distance_to_goal": None}

        # ── 摇杆遥操作：往 g1_control_server 发 Twist，由那边的
        #    set_manual_velocity(timeout) 兜住看门狗，网页断了机器人自己停 ──
        self.pub_teleop = self.create_publisher(Twist, TELEOP_TOPIC, 10)

        # ── 建图时的机器人位姿 ──
        # 这时候 Nav2 那套还没起，TF 里没有 base_link，只能从 LIO 直接拿
        self._lio_pose = None
        self._lio_pose_time = 0.0
        if Odometry is not None:
            self.create_subscription(
                Odometry, args.lio_odom_topic, self._on_lio_odom,
                QoSPresetProfiles.SENSOR_DATA.value, callback_group=self._group,
            )

        # ── 建图实时视图：直接吃 super-lio 的世界点云，自己攒栅格 ──
        self.live_map = LiveMapView(
            self, cloud_topic=args.cloud_topic, resolution=args.live_map_resolution,
            voxel_size=args.live_cloud_voxel, map_frame=args.map_frame,
            frame_override=args.cloud_frame_override,
            voxel_z_min=args.live_cloud_z_min, voxel_z_max=args.live_cloud_z_max,
            callback_group=self._group,
        )

        # ── 相机中继（有 CompressedImage 话题就转发 JPEG，没有就显示占位） ──
        self.camera = CameraRelay(self, topic=args.camera_topic, callback_group=self._group)

        # ── Nav2 规划路径：网页把它画在地图上，让人看得见机器人打算怎么走 ──
        self._plan_lock = threading.Lock()
        self._plan_points = []
        self._plan_stamp = 0.0
        if NavPath is not None:
            self.create_subscription(
                NavPath, args.plan_topic, self._on_plan,
                QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=self._group,
            )

    def _on_plan(self, msg):
        # 路径动辄上千个点，按最小间距抽稀后再给前端，省带宽也省画布
        pts = []
        last = None
        for ps in msg.poses:
            x, y = ps.pose.position.x, ps.pose.position.y
            if last is None or math.hypot(x - last[0], y - last[1]) >= 0.12:
                pts.append([round(x, 3), round(y, 3)])
                last = (x, y)
        if msg.poses and (not pts or pts[-1] != [round(msg.poses[-1].pose.position.x, 3),
                                                 round(msg.poses[-1].pose.position.y, 3)]):
            last_pose = msg.poses[-1].pose.position
            pts.append([round(last_pose.x, 3), round(last_pose.y, 3)])
        with self._plan_lock:
            self._plan_points = pts
            self._plan_stamp = time.time()

    def plan_snapshot(self):
        with self._plan_lock:
            pts, stamp = list(self._plan_points), self._plan_stamp
        age = (time.time() - stamp) if stamp else None
        # 超过 5 秒没更新就当路径已经失效，前端别画一条陈旧的线误导人
        fresh = age is not None and age < 5.0
        return {
            "available": NavPath is not None,
            "fresh": fresh,
            "age_sec": round(age, 2) if age is not None else None,
            "points": pts if fresh else [],
            "count": len(pts) if fresh else 0,
        }

    # ── 状态订阅回调 ──
    def _on_control_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._status_lock:
            self._control_status = data

    def _on_nav_manager_detail(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._status_lock:
            self._nav_manager_status = data

    def _on_lio_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._lio_pose = {
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw": _yaw_from_quaternion(q.z, q.w),
        }
        self._lio_pose_time = time.time()

    def current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, rclpy.time.Time()
            )
            # lookup 用的是"最新可用"，导航停了之后缓存里那条旧变换还会被返回，
            # 页面上就是一个不动的幽灵机器人。按时间戳判一下新鲜度。
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if stamp <= 0.0 or (self.get_clock().now().nanoseconds * 1e-9 - stamp) < 5.0:
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = _yaw_from_quaternion(q.z, q.w)
                return {"x": t.x, "y": t.y, "yaw": yaw,
                        "yaw_deg": math.degrees(yaw), "source": "tf"}
        except TransformException:
            pass
        # 建图阶段没有 odom_to_tf，TF 树只有 map->world->imu，查不到 base_link。
        # 页面上会一直显示"无 TF"，3D 视图里也画不出机器人。
        # 直接拿 super-lio 的机器人里程计兜底 —— 建图时它就是唯一的位姿来源。
        pose = self._lio_pose
        if pose and (time.time() - self._lio_pose_time) < 3.0:
            return {**pose, "yaw_deg": math.degrees(pose["yaw"]), "source": "lio"}
        return None

    def snapshot_status(self):
        with self._status_lock:
            control = dict(self._control_status)
            nav_manager = dict(self._nav_manager_status)
        with self._patrol_lock:
            patrol = dict(self._patrol_state)
        return {
            "control": control,
            "navigation_manager": nav_manager,
            "patrol": patrol,
            "navigate": self.navigate_snapshot(),
            "pose": self.current_pose(),
            "current_map": read_current_map_manifest(resolve_maps_dir()),
            "system": self.system_info(),
            "timestamp": time.time(),
        }

    def publish_teleop(self, vx, vy, wz):
        """摇杆速度下发。

        安全上做三件事：① 导航/巡航进行中直接拒绝，避免两个控制源抢 cmd_vel；
        ② 速度按 CLI 上限截断；③ 不主动解急停锁 —— 锁着就让它锁着，
        由 g1_control_server 那边判定。看门狗在下游（set_manual_velocity 的
        timeout），网页断连后机器人会自己停。
        """
        if self._nav_feedback.get("active"):
            raise ApiError("导航进行中，先取消导航再用摇杆", 409)
        with self._patrol_lock:
            if self._patrol_state.get("running"):
                raise ApiError("巡航进行中，先停止巡航再用摇杆", 409)

        def clamp(value, limit):
            try:
                value = float(value)
            except (TypeError, ValueError):
                return 0.0
            if value != value:      # NaN
                return 0.0
            return max(-limit, min(limit, value))

        vx = clamp(vx, self.args.teleop_max_vx)
        vy = clamp(vy, self.args.teleop_max_vy)
        wz = clamp(wz, self.args.teleop_max_wz)

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub_teleop.publish(msg)
        return {"success": True, "vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}

    def navigate_snapshot(self):
        """导航面板数据：目标位姿 + 用时 + 阶段 + 上次结果。

        用时在读取时算，避免为了刷新计时去开定时器。
        """
        out = dict(self._nav_feedback)
        started = out.get("started_at")
        if started:
            end = out.get("finished_at") or time.time()
            out["elapsed_sec"] = round(end - started, 1)
        else:
            out["elapsed_sec"] = None
        return out

    def system_info(self):
        """主机侧运行指标。取不到的字段返回 None，前端显示 '—'，不编造数值。"""
        return {
            "cpu_percent": _cpu_percent(),
            "mem_percent": _mem_percent(),
            "ip": _primary_ip(self.args.net_if),
            "hostname": _hostname(),
            "uptime_sec": round(time.time() - self._start_time, 1),
            "battery_percent": None,  # 需 SDK lowstate 订阅，当前未接入
        }

    # ── 通用同步调用助手 ──
    def call_service(self, client, request, timeout=8.0, name=""):
        if not client.wait_for_service(timeout_sec=2.0):
            raise ApiError(f"服务不可用: {name or client.srv_name}", 503)
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise ApiError(f"服务调用超时: {name or client.srv_name}", 504)
        exc = future.exception()
        if exc is not None:
            raise ApiError(f"服务调用异常: {exc}", 500)
        return future.result()

    @staticmethod
    def response_to_dict(resp, extra_fields=()):
        out = {
            "success": bool(getattr(resp, "success", False)),
            "status": getattr(resp, "status", None),
            "message": getattr(resp, "message", ""),
        }
        for field in extra_fields:
            out[field] = getattr(resp, field, None)
        return out

    # ── 单点导航（动作） ──
    def start_navigate(self, waypoint_name, x, y, yaw, align_final_yaw=True):
        if not self.act_navigate.wait_for_server(timeout_sec=2.0):
            raise ApiError("导航动作服务不可用", 503)

        goal = NavigateToTarget.Goal()
        goal.waypoint_name = str(waypoint_name)
        pose = PoseStamped()
        pose.header.frame_id = self.args.map_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        _, _, qz, qw = _quaternion_from_yaw(float(yaw))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.target_pose = pose
        goal.align_final_yaw = bool(align_final_yaw)

        self._nav_feedback = {
            "active": True, "waypoint_name": waypoint_name, "phase": "sending", "distance_to_goal": None,
            "target": {"x": round(float(x), 3), "y": round(float(y), 3), "yaw_deg": round(math.degrees(float(yaw)), 1)},
            "started_at": time.time(), "finished_at": None,
            "result_status": "", "result_message": "",
        }

        result_holder = {"done": False, "result": None}
        result_event = threading.Event()

        def _feedback_cb(feedback_msg):
            fb = feedback_msg.feedback
            self._nav_feedback.update({
                "phase": fb.phase, "distance_to_goal": fb.distance_to_goal,
            })

        def _goal_response_cb(future):
            handle = future.result()
            if not handle.accepted:
                self._nav_feedback.update({
                    "active": False, "phase": "rejected", "finished_at": time.time(),
                    "result_status": "error", "result_message": "导航目标被拒绝",
                })
                result_holder["result"] = {"success": False, "status": "error", "message": "导航目标被拒绝"}
                result_holder["done"] = True
                result_event.set()
                return
            self._nav_goal_handle = handle

            def _result_cb(fut):
                res = fut.result().result
                self._nav_feedback.update({
                    "active": False, "finished_at": time.time(),
                    "result_status": res.status, "result_message": res.message,
                })
                result_holder["result"] = {
                    "success": bool(res.success), "status": res.status, "message": res.message,
                }
                result_holder["done"] = True
                result_event.set()

            handle.get_result_async().add_done_callback(_result_cb)

        send_future = self.act_navigate.send_goal_async(goal, feedback_callback=_feedback_cb)
        send_future.add_done_callback(_goal_response_cb)
        return {"accepted": True, "waypoint_name": waypoint_name}

    def cancel_navigate(self):
        with self._patrol_lock:
            self._patrol_cancel.set()
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
        return {"success": True, "message": "已请求取消导航"}

    def navigate_blocking(self, waypoint_name, x, y, yaw, align_final_yaw=True, timeout=180.0):
        """阻塞版本，供巡航线程按顺序调用各个路点。"""
        if not self.act_navigate.wait_for_server(timeout_sec=5.0):
            raise ApiError("导航动作服务不可用", 503)

        goal = NavigateToTarget.Goal()
        goal.waypoint_name = str(waypoint_name)
        pose = PoseStamped()
        pose.header.frame_id = self.args.map_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        _, _, qz, qw = _quaternion_from_yaw(float(yaw))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.target_pose = pose
        goal.align_final_yaw = bool(align_final_yaw)

        send_future = self.act_navigate.send_goal_async(goal)
        done = threading.Event()
        send_future.add_done_callback(lambda _f: done.set())
        if not done.wait(10.0):
            raise ApiError("发送导航目标超时", 504)
        handle = send_future.result()
        if not handle.accepted:
            return {"success": False, "status": "error", "message": "导航目标被拒绝"}
        self._nav_goal_handle = handle

        result_future = handle.get_result_async()
        result_done = threading.Event()
        result_future.add_done_callback(lambda _f: result_done.set())
        if not result_done.wait(timeout):
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
            return {"success": False, "status": "error", "message": "导航超时"}
        res = result_future.result().result
        return {"success": bool(res.success), "status": res.status, "message": res.message}

    # ── 巡航（多点导航） ──
    def start_patrol(self, route_name, waypoints):
        with self._patrol_lock:
            if self._patrol_state["running"]:
                raise ApiError("已有巡航任务在执行，请先停止", 409)
            self._patrol_cancel.clear()
            self._patrol_state = {
                "running": True, "route_name": route_name, "index": 0,
                "total": len(waypoints), "message": "starting",
            }

        def _run():
            try:
                for idx, wp in enumerate(waypoints, start=1):
                    if self._patrol_cancel.is_set():
                        with self._patrol_lock:
                            self._patrol_state.update({"message": "已取消"})
                        break
                    with self._patrol_lock:
                        self._patrol_state.update({"index": idx, "message": f"前往路点 {idx}/{len(waypoints)}"})
                    result = self.navigate_blocking(
                        wp.get("name", f"wp{idx}"), wp["x"], wp["y"], wp["yaw"], True,
                    )
                    if not result.get("success"):
                        with self._patrol_lock:
                            self._patrol_state.update({
                                "message": f"路点 {idx} 失败: {result.get('message')}"
                            })
                        break
                else:
                    with self._patrol_lock:
                        self._patrol_state.update({"message": "巡航完成"})
            except Exception as exc:  # noqa: BLE001
                with self._patrol_lock:
                    self._patrol_state.update({"message": f"巡航异常: {exc}"})
            finally:
                with self._patrol_lock:
                    self._patrol_state["running"] = False

        self._patrol_thread = threading.Thread(target=_run, daemon=True)
        self._patrol_thread.start()
        return dict(self._patrol_state)

    def stop_patrol(self):
        with self._patrol_lock:
            was_running = self._patrol_state["running"]
        self._patrol_cancel.set()
        self.cancel_navigate()
        return {"success": True, "was_running": was_running}

    def patrol_status(self):
        with self._patrol_lock:
            return dict(self._patrol_state)


# ─────────────────────────────────────────────────────────────
# 文件系统辅助：地图 / 路线 / 动作 / 设置 / 日志
# ─────────────────────────────────────────────────────────────

def data_dir(*parts):
    """运行时会被写入的数据（地图 / 路线 / 行走参数）落在哪里。

    容器部署时这些必须落在挂载卷上：包内路径在镜像里，容器一重建
    现场建的图和存的路线就没了。设了 G1_DATA_DIR 就用它，没设则退回
    包内路径，裸机部署行为不变。
    """
    root = os.environ.get("G1_DATA_DIR", "").strip()
    if root:
        return Path(root).expanduser().joinpath(*parts)
    return Path(package_share_dir()) / "config" / Path(*parts)


def resolve_maps_dir():
    configured = os.environ.get("G1_MAPS_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    root = os.environ.get("G1_BASE_ROOT", "").strip()
    if root:
        return Path(root).expanduser() / "config" / "maps"
    return data_dir("maps")


def halls_dir():
    # 场馆预设是随镜像发布的静态资源，不进数据卷
    return Path(package_share_dir()) / "config" / "halls"


# 地图名允许的字符：要当文件名用，也要能安全塞进 URL
_MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,48}$")


def lio_map_dir():
    """Super-LIO 的存图目录。

    Super-LIO 把 map.pcd 写到编译期宏 ROOT 指定的位置（见
    docker/lio/livox_360.yaml 的注释），容器里 start_g1_base.sh
    已把它软链到数据卷。这里按 LIO_WORKSPACE_ROOT 优先解析，
    解析不到再退回裸机上的两个历史位置。
    """
    candidates = []
    root = os.environ.get("LIO_WORKSPACE_ROOT", "").strip()
    if root:
        candidates.append(Path(root).expanduser() / "src" / "Super-LIO" / "src" / "super_lio" / "map")
    candidates += [
        Path.home() / "ros2_ws" / "src" / "Super-LIO" / "src" / "super_lio" / "map",
        Path.home() / "Super-LIO" / "src" / "super_lio" / "map",
    ]
    for c in candidates:
        if c.is_dir():
            return c
    return candidates[0]


def save_live_map(name):
    """把本次建图的 map.pcd 转成 2D 栅格快照，按用户起的名字落进地图目录。

    命名沿用地图列表的约定（<name>_exhibit_2d_map.yaml/.pgm + <name>_map.pcd），
    存完立刻能在「地图列表」里看到并激活。
    """
    name = (name or "").strip()
    if not name:
        raise ApiError("请填写地图名称", 400)
    if not _MAP_NAME_RE.match(name):
        raise ApiError("地图名称只能用字母、数字、下划线、连字符，最长 48 位", 400)

    src = lio_map_dir() / "map.pcd"
    if not src.is_file():
        raise ApiError("没找到 map.pcd。先「开始建图」走一圈，再「停止建图」把点云落盘", 404)

    maps_dir = resolve_maps_dir()
    maps_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = maps_dir / f"{name}_{MAP_NAME_SUFFIX}.yaml"
    if yaml_path.exists():
        raise ApiError(f"地图「{name}」已存在，换个名字", 409)

    from g1_base.pcd_to_2d_map import TiltedWorldError, convert_pcd_to_2d_map

    # 先把 PCD 收进地图目录再转换：转换器按 pcd 所在目录写产物，
    # 而且原始点云要跟快照一起留档（重定位和以后重新投影都要用）
    pcd_target = maps_dir / f"{name}_map.pcd"
    shutil.copy2(src, pcd_target)
    try:
        # 不传 z_min/z_max —— 那会退回按绝对 z 切片的旧路径。
        # 默认路径按"沿地面法线的离地高度"过滤，既不在乎 LIO 原点在雷达
        # 还是在基座（odom_robo 标不标定都一样），也不在乎地面有点倾斜。
        pgm, yaml_f = convert_pcd_to_2d_map(
            pcd_path=str(pcd_target),
            output_dir=str(maps_dir),
            output_name=f"{name}_{MAP_NAME_SUFFIX}",
        )
    except TiltedWorldError as exc:
        pcd_target.unlink(missing_ok=True)
        # 倾斜大 + 残差小 = 整体歪了；残差也大 = 地面根本不平（LIO 漂了）
        drifted = (exc.residual_rms or 0.0) > 0.25
        why = ("地面拟合残差 %.2f m，说明地面本身就不平 —— 多半是 LIO 漂了，"
               "走短一点、慢一点、尽量回到起点闭环再试" % exc.residual_rms) if drifted else \
              "地面本身是平的，只是整体歪了；若确认场地无坡，把 MAX_WORLD_TILT_DEG 调大即可"
        raise ApiError(f"地面倾斜 {exc.tilt_deg:.1f}°（上限 {exc.max_tilt_deg:.0f}°）。{why}", 422)
    except Exception as exc:
        pcd_target.unlink(missing_ok=True)
        raise ApiError(f"生成 2D 地图失败：{exc}", 500)

    return {
        "ok": True,
        "id": f"snapshot:{name}",
        "name": name,
        "pgm": str(pgm),
        "yaml": str(yaml_f),
        "pcd": str(pcd_target),
    }


def clear_lio_pcd():
    """删掉本次建图产生的点云文件（map.pcd + PCD/ 分片）。

    只动 Super-LIO 的工作目录，已保存进地图目录的快照不受影响。
    """
    d = lio_map_dir()
    removed = []
    f = d / "map.pcd"
    if f.is_file():
        f.unlink()
        removed.append("map.pcd")
    frag_dir = d / "PCD"
    if frag_dir.is_dir():
        n = 0
        for frag in frag_dir.glob("*.pcd"):
            frag.unlink()
            n += 1
        if n:
            removed.append(f"PCD 分片 × {n}")
    return removed


def routes_dir():
    return data_dir("routes")


def walking_mode_file():
    return data_dir("walking_mode.yaml")


def read_current_map_manifest(maps_dir):
    manifest_path = maps_dir / "current_map.json"
    if not manifest_path.is_file():
        return None
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _read_map_yaml(yaml_path):
    if yaml is not None:
        return yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    data = {}
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def list_maps():
    """扫描 config/maps（时间戳快照 + 当前激活地图）和 config/halls/*/maps（场馆预设地图）。"""
    maps = []
    maps_dir = resolve_maps_dir()
    active_yaml = maps_dir / f"{MAP_NAME_SUFFIX}.yaml"
    active_mtime = active_yaml.stat().st_mtime if active_yaml.is_file() else None

    if maps_dir.is_dir():
        for yaml_path in sorted(maps_dir.glob(f"*_{MAP_NAME_SUFFIX}.yaml"), reverse=True):
            base_name = yaml_path.name[: -len(f"_{MAP_NAME_SUFFIX}.yaml")]
            pgm_path = yaml_path.with_suffix(".pgm")
            is_active = active_mtime is not None and abs(pgm_path.stat().st_mtime - active_mtime) < 2.0 if pgm_path.is_file() else False
            maps.append({
                "id": f"snapshot:{base_name}",
                "source": "snapshot",
                "base_name": base_name,
                "label": base_name,
                "yaml_path": str(yaml_path),
                "pgm_exists": pgm_path.is_file(),
                "is_active": is_active,
                "mtime": yaml_path.stat().st_mtime,
            })
        if active_yaml.is_file():
            maps.insert(0, {
                "id": "active",
                "source": "active",
                "base_name": MAP_NAME_SUFFIX,
                "label": "当前使用中的地图",
                "yaml_path": str(active_yaml),
                "pgm_exists": (maps_dir / f"{MAP_NAME_SUFFIX}.pgm").is_file(),
                "is_active": True,
                "mtime": active_yaml.stat().st_mtime,
            })

    hd = halls_dir()
    if hd.is_dir():
        for hall_path in sorted(hd.iterdir()):
            hall_maps_dir = hall_path / "maps"
            if not hall_maps_dir.is_dir():
                continue
            for yaml_path in sorted(hall_maps_dir.glob("*.yaml")):
                pgm_path = yaml_path.with_suffix(".pgm")
                maps.append({
                    "id": f"hall:{hall_path.name}:{yaml_path.stem}",
                    "source": "hall",
                    "hall": hall_path.name,
                    "base_name": yaml_path.stem,
                    "label": f"{hall_path.name} / {yaml_path.stem}",
                    "yaml_path": str(yaml_path),
                    "pgm_exists": pgm_path.is_file(),
                    "is_active": False,
                    "mtime": yaml_path.stat().st_mtime,
                })
    return maps


def resolve_map_entry(map_id):
    for entry in list_maps():
        if entry["id"] == map_id:
            return entry
    raise ApiError(f"地图不存在: {map_id}", 404)


def activate_map(map_id):
    entry = resolve_map_entry(map_id)
    if entry["source"] == "active":
        return {"success": True, "message": "该地图已是当前地图"}

    maps_dir = resolve_maps_dir()
    maps_dir.mkdir(parents=True, exist_ok=True)
    src_yaml = Path(entry["yaml_path"])
    src_pgm = src_yaml.with_suffix(".pgm")
    if not src_pgm.is_file():
        raise ApiError(f"地图缺少 pgm 文件: {src_pgm}", 400)

    dst_yaml = maps_dir / f"{MAP_NAME_SUFFIX}.yaml"
    dst_pgm = maps_dir / f"{MAP_NAME_SUFFIX}.pgm"

    data = _read_map_yaml(src_yaml)
    if isinstance(data, dict):
        data["image"] = dst_pgm.name
        text_lines = [f"image: {dst_pgm.name}"]
        for key, value in data.items():
            if key == "image":
                continue
            text_lines.append(f"{key}: {value}")
        dst_yaml.write_text("\n".join(text_lines) + "\n", encoding="utf-8")
    else:
        shutil.copy2(src_yaml, dst_yaml)
    shutil.copy2(src_pgm, dst_pgm)

    return {
        "success": True,
        "message": f"已将 {entry['label']} 设为当前地图，需要在「设置」中重启导航栈生效",
    }


def delete_map(map_id):
    entry = resolve_map_entry(map_id)
    if entry["source"] == "active":
        raise ApiError("当前使用中的地图不能删除", 400)
    yaml_path = Path(entry["yaml_path"])
    pgm_path = yaml_path.with_suffix(".pgm")
    pcd_path = yaml_path.parent / f"{entry['base_name']}_map.pcd"
    removed = []
    for path in (yaml_path, pgm_path, pcd_path):
        try:
            if path.is_file():
                path.unlink()
                removed.append(path.name)
        except Exception as exc:
            raise ApiError(f"删除失败: {exc}", 500) from exc
    return {"success": True, "removed": removed}


def _read_pgm_header(pgm_path):
    """只读 PGM(P5) 头部，返回 (width, height, maxval, data_start_offset, raw_bytes)。"""
    with open(pgm_path, "rb") as fp:
        data = fp.read()

    pos = 0

    def _next_token():
        nonlocal pos
        while pos < len(data) and data[pos : pos + 1].isspace():
            pos += 1
        if pos < len(data) and data[pos : pos + 1] == b"#":
            while pos < len(data) and data[pos : pos + 1] != b"\n":
                pos += 1
            return _next_token()
        start = pos
        while pos < len(data) and not data[pos : pos + 1].isspace():
            pos += 1
        return data[start:pos]

    magic = _next_token()
    if magic != b"P5":
        raise ApiError("不支持的 PGM 格式（需要 P5）", 500)
    width = int(_next_token())
    height = int(_next_token())
    maxval = int(_next_token())
    pos += 1  # 跳过 token 后的单个空白符
    return width, height, maxval, pos, data


def render_scale_for(width, height, max_dim=1600):
    if max(width, height) > max_dim:
        return math.ceil(max(width, height) / max_dim)
    return 1


def _pgm_to_png_bytes(pgm_path, max_dim=1600):
    """极简 PGM(P5) -> PNG 编码，避免依赖 Pillow。"""
    width, height, maxval, pos, data = _read_pgm_header(pgm_path)
    pixels = data[pos : pos + width * height]
    if len(pixels) < width * height:
        raise ApiError("PGM 文件数据不完整", 500)

    scale = render_scale_for(width, height, max_dim)

    if scale > 1:
        out_w = width // scale
        out_h = height // scale
        scaled = bytearray(out_w * out_h)
        for oy in range(out_h):
            sy = oy * scale
            row_off = sy * width
            for ox in range(out_w):
                scaled[oy * out_w + ox] = pixels[row_off + ox * scale]
        width, height, pixels = out_w, out_h, bytes(scaled)

    if maxval != 255:
        pixels = bytes(int(p * 255 / maxval) for p in pixels)

    return _gray_png_bytes(width, height, pixels)


def _gray_png_bytes(width, height, pixels):
    """8 位灰度 PNG 编码，只用标准库 zlib/struct，不依赖 Pillow。"""
    def _chunk(tag, payload):
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    raw = bytearray()
    for row in range(height):
        raw.append(0)
        raw.extend(pixels[row * width : (row + 1) * width])
    compressed = zlib.compress(bytes(raw), level=6)

    png = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    png += _chunk(b"IHDR", ihdr)
    png += _chunk(b"IDAT", compressed)
    png += _chunk(b"IEND", b"")
    return png


def render_map_png(map_id):
    entry = resolve_map_entry(map_id)
    pgm_path = Path(entry["yaml_path"]).with_suffix(".pgm")
    if not pgm_path.is_file():
        raise ApiError("地图图片不存在", 404)
    return _pgm_to_png_bytes(pgm_path)


def map_yaml_info(map_id):
    entry = resolve_map_entry(map_id)
    data = _read_map_yaml(Path(entry["yaml_path"])) or {}
    pgm_path = Path(entry["yaml_path"]).with_suffix(".pgm")
    geometry = None
    if pgm_path.is_file():
        width, height, _maxval, _pos, _data = _read_pgm_header(pgm_path)
        scale = render_scale_for(width, height)
        geometry = {
            "width": width,
            "height": height,
            "resolution": float(data.get("resolution", 0.05)),
            "origin": data.get("origin", [0.0, 0.0, 0.0]),
            "render_scale": scale,
            "render_width": width // scale,
            "render_height": height // scale,
        }
    return {**entry, "yaml": data, "geometry": geometry}


# ── 路线 / 巡航点 ──

def _route_path(name):
    safe = re.sub(r"[^A-Za-z0-9_\-]", "_", name)
    return routes_dir() / f"{safe}.yaml"


def list_routes():
    rdir = routes_dir()
    if not rdir.is_dir():
        return []
    out = []
    for path in sorted(rdir.glob("*.yaml")):
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) if yaml else {}
        except Exception:
            data = {}
        waypoints = (data or {}).get("waypoints") or []
        out.append({
            "name": path.stem,
            "route_name": (data or {}).get("route_name", path.stem),
            "waypoint_count": len(waypoints),
            "mtime": path.stat().st_mtime,
        })
    return out


def get_route(name):
    path = _route_path(name)
    if not path.is_file():
        raise ApiError(f"路线不存在: {name}", 404)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) if yaml else {}
    data = data or {}
    waypoints = []
    for wp in data.get("waypoints") or []:
        yaw = wp.get("yaw")
        if yaw is None and "yaw_deg" in wp:
            yaw = math.radians(float(wp["yaw_deg"]))
        waypoints.append({
            "name": wp.get("name", ""),
            "x": float(wp.get("x", 0.0)),
            "y": float(wp.get("y", 0.0)),
            "yaw": float(yaw or 0.0),
            "yaw_deg": math.degrees(float(yaw or 0.0)),
            "action_id": int(wp.get("action_id", 25)),
            "say_text": str(wp.get("say_text", "")),
        })
    return {"name": name, "route_name": data.get("route_name", name), "waypoints": waypoints}


def save_route(name, route_name, waypoints):
    if not name or not re.match(r"^[A-Za-z0-9_\-]+$", name):
        raise ApiError("路线名称仅支持字母/数字/下划线/横杠", 400)
    routes_dir().mkdir(parents=True, exist_ok=True)
    lines = [f'route_name: "{route_name or name}"', "waypoints:"]
    for wp in waypoints:
        lines.append(f"  - x: {float(wp['x'])}")
        lines.append(f"    y: {float(wp['y'])}")
        lines.append(f"    yaw_deg: {float(wp.get('yaw_deg', 0.0))}")
        lines.append(f"    action_id: {int(wp.get('action_id', 25))}")
        say_text = str(wp.get("say_text", "")).replace('"', '\\"')
        lines.append(f'    say_text: "{say_text}"')
        if wp.get("name"):
            safe_name = str(wp["name"]).replace('"', '\\"')
            lines.append(f'    name: "{safe_name}"')
    _route_path(name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"success": True}


def delete_route(name):
    path = _route_path(name)
    if not path.is_file():
        raise ApiError(f"路线不存在: {name}", 404)
    path.unlink()
    return {"success": True}


# ── 动作资源 ──

def list_preset_actions():
    path = Path(config_file("preset_actions.json"))
    if not path.is_file():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    out = []
    for action_id, info in data.items():
        if action_id.startswith("_"):
            continue
        out.append({"action_id": int(action_id), **info})
    out.sort(key=lambda item: item["action_id"])
    return out


def list_named_actions():
    mdir = Path(movement_dir()) / "motions"
    if not mdir.is_dir():
        return []
    return sorted(p.stem for p in mdir.glob("*.jsonl"))


def list_movement_scripts():
    sdir = Path(movement_dir()) / "scripts"
    if not sdir.is_dir():
        return []
    return sorted(p.name for p in sdir.glob("*.json"))


# ── 设置：速度 / 走路模式 ──

def get_walking_mode_settings():
    path = walking_mode_file()
    if not path.is_file() or yaml is None:
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def set_walking_mode(mode=None, linear_speed=None):
    path = walking_mode_file()
    if yaml is None:
        raise ApiError("服务器缺少 pyyaml，无法修改设置", 500)
    data = get_walking_mode_settings()
    if mode is not None:
        if mode not in ("locked_waist", "unlocked_waist"):
            raise ApiError("mode 仅支持 locked_waist / unlocked_waist", 400)
        data["walking_mode"] = mode
    if linear_speed is not None:
        active_mode = data.get("walking_mode", "unlocked_waist")
        data.setdefault(active_mode, {})["linear_speed"] = float(linear_speed)
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return {"success": True, "settings": data}


def tail_log(lines=200):
    from g1_base.logging import LOG_FILE

    if not LOG_FILE.is_file():
        return []
    with open(LOG_FILE, "r", encoding="utf-8", errors="replace") as fp:
        content = fp.readlines()
    return [line.rstrip("\n") for line in content[-lines:]]


# ─────────────────────────────────────────────────────────────
# HTTP 层
# ─────────────────────────────────────────────────────────────

_STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def make_handler(node: BridgeNode):
    class Handler(BaseHTTPRequestHandler):
        server_version = "G1WebBridge/1.0"

        def log_message(self, fmt, *args):
            node.get_logger().debug("%s - %s" % (self.address_string(), fmt % args))

        def _send_json(self, payload, code=200):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, body, content_type, code=200, cache=None, etag=None, extra_headers=None):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Expose-Headers", "X-Map-Geometry")
            if cache:
                self.send_header("Cache-Control", cache)
            if etag:
                self.send_header("ETag", etag)
            for key, value in (extra_headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def _send_not_modified(self, etag, cache):
            self.send_response(304)
            self.send_header("ETag", etag)
            if cache:
                self.send_header("Cache-Control", cache)
            self.end_headers()

        def _read_json_body(self):
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length == 0:
                return {}
            raw = self.rfile.read(length)
            try:
                return json.loads(raw.decode("utf-8")) if raw else {}
            except Exception as exc:
                raise ApiError(f"请求体不是合法 JSON: {exc}", 400) from exc

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            self._dispatch("GET")

        def do_POST(self):
            self._dispatch("POST")

        def do_PUT(self):
            self._dispatch("PUT")

        def do_DELETE(self):
            self._dispatch("DELETE")

        def _dispatch(self, method):
            parsed = urlparse(self.path)
            # 前端对地图 id 做了 encodeURIComponent，而 hall:xxx / snapshot:xxx
            # 这类 id 含冒号，不解码就永远匹配不上真实地图。
            path = unquote(parsed.path)
            query = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            try:
                if path.startswith("/api/"):
                    self._route_api(method, path, query)
                else:
                    self._serve_static(path)
            except ApiError as exc:
                self._send_json({"error": exc.message}, exc.code)
            except Exception as exc:  # noqa: BLE001
                node.get_logger().error(f"HTTP handler error: {exc}", throttle_duration_sec=2.0)
                self._send_json({"error": str(exc)}, 500)

        # ── 静态网页 ──
        def _serve_static(self, path):
            if path == "/":
                path = "/index.html"
            rel = path.lstrip("/")
            # 仅做字符串层面的越权检查（拒绝 .. 穿越），不对最终路径调用
            # resolve()——colcon --symlink-install 会把 webapp/ 下的文件做成
            # 指向源码树的符号链接，resolve() 会跟踪到 WEBAPP_DIR 之外，
            # 导致误判为越权。
            if ".." in Path(rel).parts:
                self._send_json({"error": "forbidden"}, 403)
                return
            file_path = WEBAPP_DIR / rel
            if not file_path.is_file():
                self._send_json({"error": "not found"}, 404)
                return
            content_type = _STATIC_TYPES.get(file_path.suffix, "application/octet-stream")
            if rel.startswith("assets/"):
                # 模型网格有 2.6MB，URL 上带了 ?v=<版本> 之后内容一变 URL 就变，
                # 所以可以放心长缓存；再挂个 ETag 兜住不带版本号访问的情况。
                try:
                    st = file_path.stat()
                    etag = '"%d-%d"' % (int(st.st_mtime), st.st_size)
                except OSError:
                    etag = None
                cache = "public, max-age=604800"
                if etag and self.headers.get("If-None-Match") == etag:
                    return self._send_not_modified(etag, cache)
                return self._send_bytes(file_path.read_bytes(), content_type, cache=cache, etag=etag)

            # 页面/脚本/样式体积小、改动频繁，直接禁用缓存
            payload = file_path.read_bytes()
            if rel == "index.html":
                payload = _stamp_asset_versions(payload)
            self._send_bytes(payload, content_type, cache="no-store")

        # ── API 路由 ──
        def _route_api(self, method, path, query):
            body = self._read_json_body() if method in ("POST", "PUT") else {}

            # ---- 状态 ----
            if method == "GET" and path == "/api/status":
                return self._send_json(node.snapshot_status())

            # 部署自检：确认静态资源（含 3D 模型）是否真的在 install 目录里
            if method == "GET" and path == "/api/health":
                required = [
                    "index.html", "app.js", "style.css", "robot3d.js",
                    "assets/three.min.js", "assets/g1_model.json",
                    "assets/g1_meshes.bin", "assets/g1_motions.json",
                ]
                files = {}
                for rel in required:
                    p = WEBAPP_DIR / rel
                    files[rel] = p.stat().st_size if p.is_file() else None
                return self._send_json({
                    "webapp_dir": str(WEBAPP_DIR),
                    "webapp_dir_exists": WEBAPP_DIR.is_dir(),
                    "files": files,
                    "missing": [k for k, v in files.items() if v is None],
                })

            # ---- 移动/动作控制 ----
            if method == "POST" and path == "/api/control/move":
                req = MoveRobot.Request()
                req.direction = str(body.get("direction", ""))
                req.distance_m = float(body.get("distance_m", 0.0))
                req.speed_scale = float(body.get("speed_scale", 1.0))
                resp = node.call_service(node.cli_move, req, timeout=30.0, name="move_robot")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/control/rotate":
                req = RotateRobot.Request()
                req.angle_deg = float(body.get("angle_deg", 0.0))
                req.speed_scale = float(body.get("speed_scale", 1.0))
                resp = node.call_service(node.cli_rotate, req, timeout=30.0, name="rotate_robot")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/control/velocity":
                # 摇杆按住时前端约 10Hz 调这里；松手发一次 0
                return self._send_json(node.publish_teleop(
                    body.get("vx", 0.0), body.get("vy", 0.0), body.get("wz", 0.0)))
            if method == "GET" and path == "/api/control/velocity/limits":
                return self._send_json({
                    "max_vx": node.args.teleop_max_vx,
                    "max_vy": node.args.teleop_max_vy,
                    "max_wz": node.args.teleop_max_wz,
                    "topic": TELEOP_TOPIC,
                })

            if method == "POST" and path == "/api/control/stop":
                resp = node.call_service(node.cli_stop, StopRobot.Request(), name="stop_robot")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/control/squat":
                req = SquatRobot.Request()
                req.action = str(body.get("action", ""))
                resp = node.call_service(node.cli_squat, req, timeout=15.0, name="squat_robot")
                return self._send_json(node.response_to_dict(resp))

            if method == "GET" and path == "/api/control/fsm":
                resp = node.call_service(node.cli_get_fsm, GetFsmId.Request(), name="get_fsm_id")
                return self._send_json(node.response_to_dict(resp, extra_fields=["fsm_id"]))

            if method == "POST" and path == "/api/control/fsm":
                req = SetFsmId.Request()
                req.fsm_id = int(body.get("fsm_id", 0))
                resp = node.call_service(node.cli_set_fsm, req, timeout=15.0, name="set_fsm_id")
                return self._send_json(node.response_to_dict(resp, extra_fields=["return_code"]))

            if method == "POST" and path == "/api/control/arm_action":
                req = ExecuteArmAction.Request()
                req.action_id = int(body.get("action_id", 0))
                resp = node.call_service(node.cli_arm_action, req, timeout=20.0, name="execute_arm_action")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/control/named_action":
                req = PlayNamedAction.Request()
                req.action_name = str(body.get("action_name", ""))
                resp = node.call_service(node.cli_named_action, req, timeout=60.0, name="play_named_action")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/control/custom_action":
                req = ExecuteCustomAction.Request()
                req.action_name = str(body.get("action_name", ""))
                resp = node.call_service(node.cli_custom_action, req, timeout=30.0, name="execute_custom_action")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/control/movement_script":
                req = RunMovementScript.Request()
                req.script_path = str(body.get("script_path", ""))
                resp = node.call_service(node.cli_movement_script, req, timeout=120.0, name="run_movement_script")
                return self._send_json(node.response_to_dict(resp))

            if method == "GET" and path == "/api/actions/presets":
                return self._send_json(list_preset_actions())
            if method == "GET" and path == "/api/actions/named":
                return self._send_json(list_named_actions())
            if method == "GET" and path == "/api/actions/scripts":
                return self._send_json(list_movement_scripts())

            # ---- 导航 ----
            if method == "POST" and path == "/api/nav/navigate":
                result = node.start_navigate(
                    body.get("waypoint_name", "target"),
                    float(body["x"]), float(body["y"]),
                    math.radians(float(body.get("yaw_deg", 0.0))) if "yaw_deg" in body else float(body.get("yaw", 0.0)),
                    bool(body.get("align_final_yaw", True)),
                )
                return self._send_json(result)

            if method == "GET" and path == "/api/nav/path":
                return self._send_json(node.plan_snapshot())
            if method == "POST" and path == "/api/nav/cancel":
                return self._send_json(node.cancel_navigate())

            if method == "POST" and path == "/api/nav/relocalize":
                req = Relocalize.Request()
                req.x = float(body["x"])
                req.y = float(body["y"])
                req.yaw = math.radians(float(body.get("yaw_deg", 0.0))) if "yaw_deg" in body else float(body.get("yaw", 0.0))
                req.duration_sec = float(body.get("duration_sec", 0.0))
                req.rate_hz = float(body.get("rate_hz", 0.0))
                resp = node.call_service(node.cli_relocalize, req, timeout=60.0, name="relocalize")
                return self._send_json(node.response_to_dict(resp))

            if method == "POST" and path == "/api/nav/ensure_ready":
                resp = node.call_service(node.cli_ensure_ready, Trigger.Request(), timeout=60.0, name="ensure_ready")
                return self._send_json(node.response_to_dict(resp))
            if method == "POST" and path == "/api/nav/restart_all":
                resp = node.call_service(node.cli_restart_all, Trigger.Request(), timeout=90.0, name="restart_all")
                return self._send_json(node.response_to_dict(resp))
            if method == "POST" and path == "/api/nav/stop_all":
                resp = node.call_service(node.cli_stop_all, Trigger.Request(), timeout=30.0, name="stop_all")
                return self._send_json(node.response_to_dict(resp))

            # ---- 巡航路线 ----
            if method == "GET" and path == "/api/routes":
                return self._send_json(list_routes())
            m = re.match(r"^/api/routes/([^/]+)$", path)
            if m:
                name = m.group(1)
                if method == "GET":
                    return self._send_json(get_route(name))
                if method == "PUT":
                    save_route(name, body.get("route_name"), body.get("waypoints", []))
                    return self._send_json({"success": True})
                if method == "DELETE":
                    return self._send_json(delete_route(name))

            if method == "POST" and path == "/api/nav/patrol/start":
                name = str(body.get("route_name", ""))
                route = get_route(name)
                return self._send_json(node.start_patrol(name, route["waypoints"]))
            if method == "POST" and path == "/api/nav/patrol/stop":
                return self._send_json(node.stop_patrol())
            if method == "GET" and path == "/api/nav/patrol/status":
                return self._send_json(node.patrol_status())

            # ---- 建图 ----
            if method == "POST" and path == "/api/map/start_mapping":
                node.live_map.reset()   # 每次建图从空白开始，不叠上一轮的栅格
                resp = node.call_service(node.cli_start_mapping, Trigger.Request(), timeout=60.0, name="start_mapping")
                return self._send_json(node.response_to_dict(resp))
            if method == "POST" and path == "/api/map/stop_mapping":
                resp = node.call_service(node.cli_stop_mapping, Trigger.Request(), timeout=60.0, name="stop_mapping")
                return self._send_json(node.response_to_dict(resp))
            # ---- 建图实时预览 ----
            if method == "GET" and path == "/api/map/live":
                return self._send_json(node.live_map.info())
            if method == "GET" and path == "/api/map/live/cloud":
                # 3D 点云增量流：body 是裸 float32 小端 xyz 三元组，
                # 元数据走响应头，前端一次请求就能既拿点又拿游标。
                since = int(query.get("since", "0") or 0)
                max_points = max(1, min(200000, int(query.get("max", "60000") or 60000)))
                payload, start, end, total, zlo, zhi = node.live_map.cloud_since(since, max_points)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Cloud-Start", str(start))
                self.send_header("X-Cloud-End", str(end))
                self.send_header("X-Cloud-Total", str(total))
                self.send_header("X-Cloud-Voxel", str(node.live_map.voxel_size))
                if zlo is not None:
                    self.send_header("X-Cloud-Zmin", f"{zlo:.3f}")
                    self.send_header("X-Cloud-Zmax", f"{zhi:.3f}")
                ground = node.live_map.ground_level()
                if ground is not None:
                    self.send_header("X-Cloud-Ground", f"{ground:.3f}")
                self.end_headers()
                self.wfile.write(payload)
                return

            if method == "GET" and path == "/api/map/live/image":
                png, geometry = node.live_map.render()
                if png is None:
                    raise ApiError("暂无点云数据，确认已开始建图且 super-lio 在发点云", 404)
                # 几何信息塞进响应头，前端一次请求就能同时拿到图和坐标系
                extra = {"X-Map-Geometry": json.dumps(geometry, separators=(",", ":"))}
                return self._send_bytes(png, "image/png", cache="no-store", extra_headers=extra)
            if method == "POST" and path == "/api/map/live/reset":
                out = node.live_map.reset()
                # 「清空点云」不只是清屏：本次建图落盘的 pcd 也一并删掉，
                # 否则下次「保存地图」会把上一次的点云当成新图存下来。
                if body.get("purge_pcd"):
                    out["removed"] = clear_lio_pcd()
                return self._send_json(out)

            # ---- 相机 ----
            if method == "GET" and path == "/api/camera/info":
                return self._send_json(node.camera.info())
            if method == "GET" and path == "/api/camera/frame":
                frame, fmt = node.camera.frame()
                if not frame:
                    raise ApiError("暂无相机画面", 404)
                ctype = "image/png" if "png" in (fmt or "").lower() else "image/jpeg"
                return self._send_bytes(frame, ctype, cache="no-store")

            if method == "POST" and path == "/api/map/save":
                return self._send_json(save_live_map(body.get("name")))

            if method == "POST" and path == "/api/map/generate_2d":
                resp = node.call_service(node.cli_generate_2d_map, Trigger.Request(), timeout=60.0, name="generate_2d_map")
                return self._send_json(node.response_to_dict(resp))

            # ---- 地图管理 ----
            if method == "GET" and path == "/api/maps":
                return self._send_json(list_maps())
            m = re.match(r"^/api/maps/([^/]+)$", path)
            if m and method == "GET":
                return self._send_json(map_yaml_info(m.group(1)))
            if m and method == "DELETE":
                return self._send_json(delete_map(m.group(1)))
            m = re.match(r"^/api/maps/([^/]+)/image$", path)
            if m and method == "GET":
                png = render_map_png(m.group(1))
                return self._send_bytes(png, "image/png")
            m = re.match(r"^/api/maps/([^/]+)/activate$", path)
            if m and method == "POST":
                return self._send_json(activate_map(m.group(1)))

            # ---- 设置 ----
            if method == "GET" and path == "/api/settings/walking_mode":
                return self._send_json(get_walking_mode_settings())
            if method == "PUT" and path == "/api/settings/walking_mode":
                return self._send_json(set_walking_mode(body.get("mode"), body.get("linear_speed")))
            if method == "GET" and path == "/api/logs":
                lines = int(query.get("lines", "200"))
                return self._send_json({"lines": tail_log(lines)})

            self._send_json({"error": "not found"}, 404)

    return Handler


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="G1 iPad 上位机 HTTP 网关")
    parser.add_argument("--net-if", default="enP8p1s0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    # 建图实时预览
    parser.add_argument("--cloud-topic", default="/lio/cloud_world",
                        help="super-lio 世界坐标点云话题，用于建图页实时预览")
    parser.add_argument("--live-map-resolution", type=float, default=0.05)
    # 相机（CompressedImage）。留空则不订阅
    parser.add_argument("--camera-topic", default="/camera/color/image_raw/compressed",
                        help="sensor_msgs/CompressedImage 话题；留空关闭相机中继")
    # Nav2 规划路径，画在地图上
    parser.add_argument("--cloud-frame-override", default="",
                        help="强制认定点云所在坐标系（留空则用消息里的 frame_id）。"
                             "G1 上域桥转发的 /lio/cloud_world 其实是传感器系，"
                             "frame_id 若不可信可用它指定，例如 base_link")
    parser.add_argument("--live-cloud-voxel", type=float, default=0.08,
                        help="建图 3D 点云的体素边长（米），越小越细但点数涨得快")
    parser.add_argument("--live-cloud-z-min", type=float, default=-6.0,
                        help="建图 3D 点云的离群下界（米，相对建图原点）。"
                             "拦掉 LIO 未收敛时甩出的野点，否则高度配色会被撑爆")
    parser.add_argument("--live-cloud-z-max", type=float, default=6.0,
                        help="建图 3D 点云的离群上界（米，相对建图原点）")
    parser.add_argument("--lio-odom-topic", default="/lio/odom",
                        help="super-lio 的里程计，建图阶段位姿只能从这里拿。"
                             "注意别用 /lio/robo/odom —— 基础镜像的 DDS 域桥会把宇树 "
                             "MCU 的 /dog_odom 改名发到那个话题上，两个发布者原点不同，"
                             "订阅端会拿到交替混合的位姿，而且停止建图后图标也不消失")
    parser.add_argument("--plan-topic", default="/plan",
                        help="nav_msgs/Path 话题，用于在网页地图上画规划路径")
    # 摇杆速度上限
    parser.add_argument("--teleop-max-vx", type=float, default=0.45)
    parser.add_argument("--teleop-max-vy", type=float, default=0.25)
    parser.add_argument("--teleop-max-wz", type=float, default=0.70)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = BridgeNode(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    handler_cls = make_handler(node)
    httpd = ThreadingHTTPServer((args.host, args.port), handler_cls)
    node.get_logger().info(f"g1_web_bridge listening on http://{args.host}:{args.port}")
    node.get_logger().info(f"webapp dir: {WEBAPP_DIR} (exists={WEBAPP_DIR.is_dir()})")
    missing = [
        rel for rel in ("index.html", "app.js", "robot3d.js", "assets/three.min.js", "assets/g1_meshes.bin")
        if not (WEBAPP_DIR / rel).is_file()
    ]
    if missing:
        node.get_logger().warning(
            f"静态资源缺失 {missing} —— 新增文件需重新 colcon build --symlink-install 才会进 install 目录"
        )
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

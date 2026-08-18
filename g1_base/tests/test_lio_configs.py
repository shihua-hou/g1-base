"""Super-LIO 两份配置（建图 / 重定位）的一致性约束。

建图和重定位必须用同一套传感器几何：外参、盲区、量程任意一项不一致，
重定位算出来的位姿就和地图对不上，表现为机器人以为自己在别处 ——
而且不会有任何报错，非常难查。这里把它钉死。
"""

import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MAPPING_CFG = REPO_ROOT / "docker" / "lio" / "livox_360.yaml"
RELOC_CFG = REPO_ROOT / "docker" / "lio" / "relocation_360.yaml"

# 两边必须逐字相同的项：决定"点云长什么样"和"世界原点在哪"
GEOMETRY_KEYS = (
    "lio.extrinsic.odom_robo",
    "lio.extrinsic.lidar_imu",
    "lio.sensor.blind",
    "lio.sensor.maxrange",
    "lio.sensor.filter_rate",
    "lio.sensor.gravity_norm",
    "lio.sensor.lidar_type",
    "lio.sensor.imu_type",
    "lio.ros.lidar_topic",
    "lio.ros.imu_topic",
)


def _parse(path):
    """够用的极简解析：这两份文件只有 `key: value` 和行内/折行列表。

    不用 pyyaml——运行测试的环境不保证装了它。
    """
    text = path.read_text(encoding="utf-8")
    body = text.split("ros__parameters:", 1)[1]
    # 折行的列表拼回一行，再统一空白，方便逐字比对
    body = re.sub(
        r"\[\s*([^\]]*?)\s*\]",
        lambda m: "[" + " ".join(m.group(1).split()) + "]",
        body,
        flags=re.S,
    )
    out = {}
    for line in body.splitlines():
        line = line.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        key, value = line.split(":", 1)
        out[key.strip()] = value.strip()
    return out


def test_relocation_config_exists():
    """start_pc2_localization.sh 按这个文件名找配置，缺了整条定位链路起不来。"""
    assert RELOC_CFG.is_file()
    script = (REPO_ROOT / "g1_base" / "start_pc2_localization.sh").read_text(encoding="utf-8")
    assert "relocation_360.yaml" in script


def test_mapping_and_relocation_share_the_same_geometry():
    mapping = _parse(MAPPING_CFG)
    reloc = _parse(RELOC_CFG)
    for key in GEOMETRY_KEYS:
        assert key in mapping, f"建图配置缺少 {key}"
        assert key in reloc, f"重定位配置缺少 {key}"
        assert mapping[key] == reloc[key], (
            f"{key} 两边不一致：建图={mapping[key]} 重定位={reloc[key]}；"
            "改一处必须改两处，否则重定位位姿与地图对不上"
        )


def test_world_origin_sits_on_the_ground():
    """odom_robo 的平移量定义世界系原点（super_lio.cpp:155）。

    z 必须是 +1.28（雷达在机器人原点上方 1.28m），填成负值整张图会
    偏一个雷达高度 —— 这个坑之前踩过一次。
    """
    for cfg in (MAPPING_CFG, RELOC_CFG):
        value = _parse(cfg)["lio.extrinsic.odom_robo"]
        nums = [float(x) for x in value.strip("[]").split(",")]
        assert nums[2] == 1.28, f"{cfg.name}: odom_robo z 应为 +1.28，实际 {nums[2]}"


def test_relocation_never_overwrites_the_map():
    """save_map=true 的话，relocation_node 退出时会把当前帧写回地图文件。"""
    assert _parse(RELOC_CFG)["lio.map.save_map"] == "false"


def test_relocation_publishes_cloud_world():
    """lio.output.map 控制 /lio/cloud_world 发不发（super_lio_reloc.cpp:310）。

    navigation_manager 拿 cloud_world 当定位就绪的判据之一，
    关掉就永远等不到就绪，Nav2 一辈子起不来。
    """
    assert _parse(RELOC_CFG)["lio.output.map"] == "true"


def test_upstream_relocation_yaml_bugs_are_not_copied():
    """上游 relocation.yaml 里的三个坑，别照抄回来。"""
    reloc = _parse(RELOC_CFG)
    # 拼写错误：正确名是 imu_nbg（ROSWrapper.cpp:79），写成 nbg 会静默走默认值
    assert "lio.sensor.nbg" not in reloc
    assert "lio.sensor.imu_nbg" in reloc
    # ROSWrapper.cpp 里根本没声明这个参数
    assert "lio.hash_map.insert_resolution" not in reloc
    # filter_rate 声明为 int（ROSWrapper.cpp:55），写成 3.0 节点直接起不来
    assert reloc["lio.sensor.filter_rate"] == "3"

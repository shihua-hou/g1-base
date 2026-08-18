import subprocess
from pathlib import Path


G1_BASE_ROOT = Path(__file__).resolve().parents[1]
PC2_SCRIPTS = (
    "start_pc2_mapping.sh",
    "start_pc2_localization.sh",
)
ENTRYPOINT_SCRIPTS = (
    "start_navigation_manager.sh",
    "start-g1-base-manager.sh",
)


def _read_script(name: str) -> str:
    return (G1_BASE_ROOT / name).read_text(encoding="utf-8")


def test_pc2_scripts_do_not_use_ros2_cli_topic_readiness():
    for script_name in PC2_SCRIPTS:
        content = _read_script(script_name)

        assert "ros2 topic echo" not in content
        assert "wait_for_topic_message" not in content
        assert "topic not ready" not in content
        assert "navigation_manager will wait for LIO topics" in content


def test_pc2_scripts_manage_child_process_groups():
    for script_name in PC2_SCRIPTS:
        content = _read_script(script_name)

        assert "setsid bash -c" in content
        assert "signal_process_group" in content
        assert "stop_process_group" in content

    assert "sending SIGINT to mapping process" in _read_script("start_pc2_mapping.sh")


def test_pc2_scripts_start_driver_gate_then_super_lio():
    for script_name in PC2_SCRIPTS:
        content = _read_script(script_name)

        driver_pos = content.index("starting ${LIVOX_PACKAGE} ${LIVOX_LAUNCH}")
        gate_pos = content.index("ros2 run g1_base wait_imu_steady")
        super_lio_pos = content.index("starting ${SUPER_LIO_PACKAGE} ${SUPER_LIO_LAUNCH}")

        assert driver_pos < gate_pos < super_lio_pos


def test_pc2_scripts_preserve_imu_gate_exit_code_four():
    for script_name in PC2_SCRIPTS:
        content = _read_script(script_name)

        assert "imu_gate_rc=$?" in content
        assert "imu_gate_rc == 4" in content
        assert "exit 4" in content
        assert "IMU steady gate process exited with code ${imu_gate_rc}" in content


def test_pc2_scripts_allow_field_tuning_imu_gate_thresholds():
    for script_name in PC2_SCRIPTS:
        content = _read_script(script_name)

        assert 'IMU_STEADY_GYRO_MAX_RAD_S="${IMU_STEADY_GYRO_MAX_RAD_S:-0.05}"' in content
        assert '-p gyro_max_rad_s:="$IMU_STEADY_GYRO_MAX_RAD_S"' in content
        assert 'IMU_STEADY_ACCEL_MIN_G="${IMU_STEADY_ACCEL_MIN_G:-0.950}"' in content
        assert 'IMU_STEADY_ACCEL_MAX_G="${IMU_STEADY_ACCEL_MAX_G:-1.050}"' in content
        assert 'IMU_STEADY_GYRO_VAR_MAX="${IMU_STEADY_GYRO_VAR_MAX:-1e-4}"' in content
        assert 'IMU_STEADY_ACCEL_VAR_MAX="${IMU_STEADY_ACCEL_VAR_MAX:-8e-4}"' in content
        assert '-p accel_min_g:="$IMU_STEADY_ACCEL_MIN_G"' in content
        assert '-p accel_max_g:="$IMU_STEADY_ACCEL_MAX_G"' in content
        assert '-p gyro_var_max:="$IMU_STEADY_GYRO_VAR_MAX"' in content
        assert '-p accel_var_max:="$IMU_STEADY_ACCEL_VAR_MAX"' in content


def test_start_scripts_map_legacy_topic_env_to_navigation_manager_args():
    for script_name in ENTRYPOINT_SCRIPTS:
        content = _read_script(script_name)

        assert "READY_TIMEOUT" in content
        assert "--localization-timeout" in content
        assert "POINTCLOUD_TOPIC" in content
        assert "--pointcloud-topic" in content
        assert "RELOCATION_TOPIC" in content
        assert "--relocal-odom-topic" in content


def test_start_navigation_resolves_maps_dir_like_the_web_bridge():
    """Nav2 的 pgm 必须和重定位的 pcd 来自同一个目录。

    这里原本写死 $ROOT_DIR/config/maps —— 镜像里的出厂地图。而重定位读的是
    数据卷里现场建的图，于是 Super-LIO 在现场地图的坐标系里算位姿，Nav2 却
    拿另一个场馆的底图做代价地图，界面上位姿完全对不上，两边都不报错。
    """
    content = _read_script("start_navigation.sh")

    assert 'if [[ -n "${G1_MAPS_DIR:-}" ]]' in content
    assert 'echo "$G1_DATA_DIR/maps"' in content
    assert 'maps_dir="$(resolve_maps_dir)"' in content
    assert 'local default_map="$maps_dir/exhibit_2d_map.yaml"' in content
    assert "-name '*_exhibit_2d_map.yaml'" in content
    assert 'MAP_FILE="${MAP_FILE:-$(resolve_default_map_file)}"' in content
    # 数据卷空时兜底回包内出厂图，至少让 Nav2 起得来
    assert 'local packaged="$ROOT_DIR/config/maps/exhibit_2d_map.yaml"' in content


def test_navigation_and_localization_agree_on_the_maps_dir():
    """两个脚本的 resolve_maps_dir 必须逐字一致，任何一边漂移都会导致
    Nav2 和重定位用两张不同的地图。"""
    import re

    def _fn(name):
        text = _read_script(name)
        m = re.search(r"^resolve_maps_dir\(\) \{.*?^\}", text, re.S | re.M)
        assert m, f"{name} 里没有 resolve_maps_dir"
        return m.group(0)

    assert _fn("start_navigation.sh") == _fn("start_pc2_localization.sh")


def test_pc2_localization_resolves_maps_dir_like_the_web_bridge():
    """重定位的地图目录必须和网关一致，否则会加载到包内的出厂地图。

    网页上存的图落在数据卷（G1_DATA_DIR/maps）里，只看 ROOT_DIR/config/maps
    的话，机器人一上来就以为自己在别的场馆。
    """
    content = _read_script("start_pc2_localization.sh")

    assert 'if [[ -n "${G1_MAPS_DIR:-}" ]]' in content
    assert 'echo "$G1_DATA_DIR/maps"' in content
    assert 'echo "$ROOT_DIR/config/maps"' in content   # 兜底：包内出厂地图
    assert 'maps_dir="$(resolve_maps_dir)"' in content
    assert 'local default_map="$maps_dir/map.pcd"' in content
    assert "-name '*_map.pcd'" in content
    assert "using relocation PCD" in content
    assert "-p lio.map.save_map_dir:=\"$rel_pcd_dir\"" in content
    assert "-p lio.map.map_name:=\"$(basename \"$pcd_file\")\"" in content


def test_start_scripts_are_syntax_valid():
    for script_name in (*PC2_SCRIPTS, *ENTRYPOINT_SCRIPTS):
        subprocess.run(["bash", "-n", str(G1_BASE_ROOT / script_name)], check=True)


def _maps_dir_from_robot_env(env_assignments: str) -> str:
    """source 一遍 robot_env.sh，把它算出来的 G1_MAPS_DIR 打印出来。"""
    script = (
        f"set -e; {env_assignments} "
        f'source "{G1_BASE_ROOT.as_posix()}/config/robot_env.sh"; '
        "echo \"$G1_MAPS_DIR\""
    )
    out = subprocess.run(
        ["bash", "-c", script], check=True, capture_output=True, text=True
    )
    return out.stdout.strip().splitlines()[-1]


def test_container_maps_dir_lands_on_the_data_volume():
    """容器里 G1_MAPS_DIR 必须跟着数据卷走。

    写死 $HOME/g1_maps 的话，容器里就是 /root/g1_maps —— 那是可写层不是卷，
    每次 up -d 重建容器现场建的图全没。而且网关 / navigation_manager /
    start_pc2_localization.sh 三处都以 G1_MAPS_DIR 优先，一旦它指错，
    那三处各自的 G1_DATA_DIR 兜底分支就永远走不到。
    """
    assert _maps_dir_from_robot_env("G1_DATA_DIR=/data;") == "/data/maps"


def test_bare_metal_maps_dir_keeps_home_default():
    """裸机没有 G1_DATA_DIR，行为不能变。"""
    got = _maps_dir_from_robot_env("unset G1_DATA_DIR; HOME=/home/unitree; G1_USER_HOME=/home/unitree;")
    assert got == "/home/unitree/g1_maps"


def test_explicit_maps_dir_still_wins():
    """现场显式指定的仍然最高优先级。"""
    got = _maps_dir_from_robot_env("G1_DATA_DIR=/data; G1_MAPS_DIR=/mnt/usb/maps;")
    assert got == "/mnt/usb/maps"

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


def test_start_navigation_defaults_to_packaged_config_map():
    content = _read_script("start_navigation.sh")

    assert "current_map.json" not in content
    assert "G1_MAPS_DIR" not in content
    assert 'local maps_dir="$ROOT_DIR/config/maps"' in content
    assert 'local default_map="$maps_dir/exhibit_2d_map.yaml"' in content
    assert "-name '*_exhibit_2d_map.yaml'" in content
    assert 'MAP_FILE="${MAP_FILE:-$(resolve_default_map_file)}"' in content


def test_pc2_localization_reads_relocation_pcd_from_config_maps():
    content = _read_script("start_pc2_localization.sh")

    assert 'local maps_dir="$ROOT_DIR/config/maps"' in content
    assert 'local default_map="$maps_dir/map.pcd"' in content
    assert "-name '*_map.pcd'" in content
    assert "using relocation PCD" in content
    assert "-p lio.map.save_map_dir:=\"$rel_pcd_dir\"" in content
    assert "-p lio.map.map_name:=\"$(basename \"$pcd_file\")\"" in content


def test_start_scripts_are_syntax_valid():
    for script_name in (*PC2_SCRIPTS, *ENTRYPOINT_SCRIPTS):
        subprocess.run(["bash", "-n", str(G1_BASE_ROOT / script_name)], check=True)

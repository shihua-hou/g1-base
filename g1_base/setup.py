from glob import glob
import os

from setuptools import setup


package_name = "g1_base"


def package_files(directory):
    paths = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        install_dir = os.path.join("share", package_name, root)
        paths.append((install_dir, [os.path.join(root, name) for name in files]))
    return paths


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]

data_files.extend(package_files("config"))
data_files.extend(package_files("behavior_trees"))
data_files.extend(package_files("launch"))
data_files.extend(package_files("webapp"))
data_files.append((f"share/{package_name}", glob("start_*.sh")))


setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name, "teach"],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lemon",
    maintainer_email="lemon@example.com",
    description="ROS 2 navigation stack and mission tooling for the Unitree G1 robot.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_mock = g1_base.cmd_vel_mock:main",
            "collect_dynamic_obstacle_debug = g1_base.collect_dynamic_obstacle_debug:main",
            "diag_bt_navigator = g1_base.diag_bt_navigator:main",
            "diag_costmap_decay = g1_base.diag_costmap_decay:main",
            "diag_obstacle = g1_base.diag_obstacle:main",
            "g1_base_manager = g1_base.g1_base_manager:main",
            "g1_control_server = g1_base.g1_control_server:main",
            "g1_web_bridge = g1_base.g1_web_bridge:main",
            "gravity_health = g1_base.gravity_health:main",
            "local_plan_test = g1_base.local_plan_test:main",
            "nav_script = g1_base.nav_script:main",
            "navigation_manager = g1_base.navigation_manager:main",
            "nav_obstacle_cloud_filter = g1_base.nav_obstacle_cloud_filter:main",
            "odom_to_tf = g1_base.odom_to_tf:main",
            "pcd_to_2d_map = g1_base.pcd_to_2d_map:main",
            "publish_waypoints_to_rviz = g1_base.publish_waypoints_to_rviz:main",
            "show_robot_pose = g1_base.show_robot_pose:main",
            "wait_imu_steady = g1_base.wait_imu_steady:main",
        ],
    },
)

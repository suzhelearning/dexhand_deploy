from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _runtime_nodes(context):
    confirmed = LaunchConfiguration("confirm_real").perform(context)
    if confirmed.lower() not in {"1", "true", "yes"}:
        raise RuntimeError(
            "拒绝启动真机：必须设置 confirm_real:=true"
        )

    package_share = Path(
        get_package_share_directory("pico_body_tianji")
    )
    preview_config = str(package_share / "config" / "preview.yaml")
    real_config = str(package_share / "config" / "real.yaml")
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    velocity_ratio = int(
        LaunchConfiguration("velocity_ratio").perform(context)
    )
    acceleration_ratio = int(
        LaunchConfiguration("acceleration_ratio").perform(context)
    )

    hardware_overrides = {
        "velocity_ratio": velocity_ratio,
        "acceleration_ratio": acceleration_ratio,
    }
    if robot_ip:
        hardware_overrides["robot_ip"] = robot_ip

    return [
        Node(
            package="pico_body_tianji",
            executable="tianji_kinematic_sim",
            name="tianji_kinematic_sim",
            output="screen",
            parameters=[preview_config],
        ),
        Node(
            package="pico_body_tianji",
            executable="pico_controller_input",
            name="pico_controller_input",
            output="screen",
            parameters=[preview_config],
        ),
        Node(
            package="pico_body_tianji",
            executable="marvin_hardware_bridge",
            name="marvin_hardware_bridge",
            output="screen",
            arguments=["--confirm-real"],
            parameters=[real_config, hardware_overrides],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "confirm_real",
                default_value="false",
                description="必须显式设为 true 才允许连接真机",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="",
                description="Marvin 控制器地址；留空读取厂商配置",
            ),
            DeclareLaunchArgument(
                "velocity_ratio",
                default_value="10",
                description="Marvin 关节速度百分比",
            ),
            DeclareLaunchArgument(
                "acceleration_ratio",
                default_value="10",
                description="Marvin 关节加速度百分比",
            ),
            OpaqueFunction(function=_runtime_nodes),
        ]
    )

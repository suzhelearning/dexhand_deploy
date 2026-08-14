from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("pico_body_tianji")
    )
    parameters = str(
        package_share / "config" / "controller_only_ik.yaml"
    )
    ik_backend = LaunchConfiguration("ik_backend")
    backend_parameters = PathJoinSubstitution(
        [
            str(package_share),
            "config",
            "ik",
            ik_backend,
            "controller_only.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ik_backend",
                default_value="pinocchio_cpp",
                description="可选 pinocchio_cpp / pinocchio_qp / tianji_official",
            ),
            Node(
                package="pico_body_tianji",
                executable="tianji_kinematic_sim",
                name="tianji_kinematic_sim",
                output="screen",
                parameters=[
                    parameters,
                    backend_parameters,
                    {"ik_backend": ik_backend},
                ],
            ),
            Node(
                package="pico_body_tianji",
                executable="pico_controller_only_input",
                name="pico_controller_only_input",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )

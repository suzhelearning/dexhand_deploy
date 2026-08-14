from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("pico_body_tianji")
    )
    parameters = (
        package_share
        / "config"
        / "mode"
        / "controller_only"
        / "controller_only_ik.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="pico_body_tianji",
                executable="tianji_kinematic_sim",
                name="tianji_kinematic_sim",
                output="screen",
                parameters=[str(parameters)],
            ),
            Node(
                package="pico_body_tianji",
                executable="pico_controller_only_input",
                name="pico_controller_only_input",
                output="screen",
                parameters=[str(parameters)],
            ),
        ]
    )

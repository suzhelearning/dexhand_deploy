from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("pico_body_tianji"))
    parameters = (
        package_share
        / "config"
        / "mode"
        / "controller_only"
        / "controller_only_ik.yaml"
    )
    rviz_config = package_share / "rviz" / "preview.rviz"
    urdf_path = (
        package_share
        / "assets"
        / "marvin_m6_ccs"
        / "urdf"
        / "marvin_m6_s_ccs_696_v4.urdf"
    )
    with urdf_path.open(encoding="utf-8") as stream:
        robot_description = stream.read()

    trace_file = LaunchConfiguration("trace_file")
    replay_speed = LaunchConfiguration("replay_speed")
    with_rviz = LaunchConfiguration("with_rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("trace_file"),
            DeclareLaunchArgument("replay_speed", default_value="1.0"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="pico_body_sim",
                name="marvin_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("joint_states", "/pico_body_sim/model_joint_states")
                ],
            ),
            Node(
                package="pico_body_tianji",
                executable="tianji_kinematic_sim",
                name="tianji_kinematic_sim",
                output="screen",
                parameters=[str(parameters)],
            ),
            Node(
                package="pico_body_tianji",
                executable="controller_only_trace",
                name="controller_only_trace_replay_process",
                output="screen",
                arguments=["replay", trace_file, "--speed", replay_speed],
                on_exit=Shutdown(reason="controller-only replay exited"),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="pico_body_tianji_rviz",
                output="screen",
                arguments=["-d", str(rviz_config)],
                additional_env={"QT_X11_NO_MITSHM": "1"},
                on_exit=Shutdown(reason="RViz exited"),
                condition=IfCondition(with_rviz),
            ),
        ]
    )

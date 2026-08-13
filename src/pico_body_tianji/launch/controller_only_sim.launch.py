from __future__ import annotations

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("pico_body_tianji")
    )
    project_root = Path(__file__).resolve().parents[3]
    parameters = (
        project_root
        / "src"
        / "pico_body_tianji"
        / "config"
        / "controller_only_ik.yaml"
    )
    controller_only_input = (
        project_root
        / "src"
        / "pico_body_tianji"
        / "scripts"
        / "pico_controller_only_input"
    )
    rviz_config = package_share / "rviz" / "preview.rviz"
    urdf_path = (
        package_share
        / "assets"
        / "marvin_m6_ccs"
        / "urdf"
        / "marvin_m6_s_ccs_696_v4.urdf"
    )
    with urdf_path.open(encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    with_rviz = LaunchConfiguration("with_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_rviz",
                default_value="true",
                description="是否启动 RViz 纯运动学预览",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="pico_body_sim",
                name="marvin_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    (
                        "joint_states",
                        "/pico_body_sim/model_joint_states",
                    )
                ],
            ),
            Node(
                package="pico_body_tianji",
                executable="tianji_kinematic_sim",
                name="tianji_kinematic_sim",
                output="screen",
                parameters=[str(parameters)],
            ),
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    str(controller_only_input),
                    "--ros-args",
                    "--params-file",
                    str(parameters),
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="pico_body_tianji_rviz",
                output="screen",
                arguments=["-d", str(rviz_config)],
                additional_env={
                    "QT_X11_NO_MITSHM": "1",
                },
                on_exit=Shutdown(reason="RViz exited"),
                condition=IfCondition(with_rviz),
            ),
        ]
    )


if __name__ == "__main__":
    from ros2launch.api.api import launch_a_launch_file

    raise SystemExit(
        launch_a_launch_file(
            launch_file_path=__file__,
            launch_file_arguments=sys.argv[1:],
        )
    )

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    h5_file = LaunchConfiguration("h5_file")
    replay_speed = LaunchConfiguration("replay_speed")
    yaw_deg = LaunchConfiguration("yaw_deg")
    reference_frame = LaunchConfiguration("reference_frame")
    hold_arm = LaunchConfiguration("hold_arm")
    control = LaunchConfiguration("control")
    with_rviz = LaunchConfiguration("with_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("h5_file"),
            DeclareLaunchArgument("replay_speed", default_value="1.0"),
            DeclareLaunchArgument("yaw_deg", default_value="0.0"),
            DeclareLaunchArgument(
                "reference_frame", default_value="-1"
            ),
            DeclareLaunchArgument(
                "hold_arm", default_value="0.0",
                description="auto 模式下回放前保持 idle 的秒数",
            ),
            DeclareLaunchArgument(
                "control", default_value="keyboard",
                description="keyboard（按 s 开始/结束）或 auto（自动）",
            ),
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
            Node(
                package="pico_body_tianji",
                executable="mocap_h5_replay",
                name="mocap_h5_replay",
                output="screen",
                parameters=[str(parameters)],
                arguments=[
                    "--h5",
                    h5_file,
                    "--speed",
                    replay_speed,
                    "--yaw-deg",
                    yaw_deg,
                    "--reference-frame",
                    reference_frame,
                    "--control",
                    control,
                    "--hold-arm",
                    hold_arm,
                ],
                on_exit=Shutdown(reason="mocap h5 replay exited"),
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

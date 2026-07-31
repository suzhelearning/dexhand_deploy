from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("pico_body_tianji")
    parameters = os.path.join(package_share, "config", "preview.yaml")
    rviz_config = os.path.join(package_share, "rviz", "preview.rviz")
    urdf_path = os.path.join(
        package_share,
        "assets",
        "marvin_m6_ccs",
        "urdf",
        "marvin_m6_s_ccs_696_v4.urdf",
    )
    with open(urdf_path, encoding="utf-8") as urdf_file:
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
                parameters=[parameters],
            ),
            Node(
                package="pico_body_tianji",
                executable="pico_controller_input",
                name="pico_controller_input",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="pico_body_tianji_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                additional_env={
                    "QT_X11_NO_MITSHM": "1",
                },
                on_exit=Shutdown(reason="RViz exited"),
                condition=IfCondition(with_rviz),
            ),
        ]
    )

"""Launch file for the Raspberry Pi Zero 2W (Motor Controller)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="klevor_robot",
                executable="build_hat_node",
                name="build_hat_motors",
                output="screen",
                parameters=[{"cmd_vel_topic": "/wro_robot/cmd_vel"}],
            ),
        ]
    )

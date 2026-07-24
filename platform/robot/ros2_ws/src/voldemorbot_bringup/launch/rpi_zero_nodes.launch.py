"""Launch file for the Raspberry Pi Zero 2W — single-process merged node.

Runs all three Pi Zero responsibilities (motors, button, OLED display) as
separate LifecycleNodes inside a single Python process with a shared rclpy
init. This replaces the old three-process approach and saves ~60-100 MB of
RAM — critical on the Pi Zero 2W's 512 MB budget.

Individual entry points (ackermann_motor_node, button_node, oled_display_node)
are still available for development/testing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    pi_zero_node = Node(
        package="voldemorbot_drivers",
        executable="pi_zero_node",
        name="pi_zero_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    return [pi_zero_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
    ])

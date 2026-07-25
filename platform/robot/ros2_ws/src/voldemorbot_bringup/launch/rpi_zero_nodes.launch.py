"""Launch file for the Raspberry Pi Zero 2W — two processes.

ackermann_motor_node (steering + drive) runs on its own so it isn't sharing
executor threads/CPU with the lower-rate peripherals -- see
platform/robot/docs/sensor-verification.md's feedback-rate tuning section
for the measurements behind this split. button_node and oled_display_node
stay merged into pi_zero_peripherals_node (fewer DDS participants, less RAM
-- neither is latency-sensitive enough to need its own process).

Individual entry points (ackermann_motor_node, button_node, oled_display_node)
are still available standalone for development/testing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    ackermann_motor_node = Node(
        package="voldemorbot_drivers",
        executable="ackermann_motor_node",
        name="ackermann_motor_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    pi_zero_peripherals_node = Node(
        package="voldemorbot_drivers",
        executable="pi_zero_peripherals_node",
        name="pi_zero_peripherals_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    return [ackermann_motor_node, pi_zero_peripherals_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
    ])

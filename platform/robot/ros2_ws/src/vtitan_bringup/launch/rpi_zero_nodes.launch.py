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
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from src.config.launch_settings import MotorBackendLaunchSettings, RpiZeroLaunchDefaults

_rpi_zero_defaults = RpiZeroLaunchDefaults()
_motor_backend_settings = MotorBackendLaunchSettings()


def _launch_setup(_context, *_args, **_kwargs) -> list[Node]:
    ackermann_motor_node = Node(
        package="vtitan_drivers",
        executable="ackermann_motor_node",
        name="ackermann_motor_node",
        output="screen",
        parameters=[_motor_backend_settings.as_node_parameters()],
        respawn=True,
        respawn_delay=_rpi_zero_defaults.respawn_delay,
    )
    pi_zero_peripherals_node = Node(
        package="vtitan_drivers",
        executable="pi_zero_peripherals_node",
        # Deliberately unnamed. launch_ros turns ``name`` into
        # ``--ros-args -r __node:=...``, which is process-wide -- and this
        # process hosts three nodes. Naming it renamed button_node,
        # oled_display_node and challenge_mode_node all to the same string, so
        # the graph carried three identical node names and DDS discovery kept
        # only one of their subscriptions. Symptom: the OLED stayed on "Press
        # to START" through a real transition to RACING, because its
        # /robot_state subscription was the one that lost.
        output="screen",
        respawn=True,
        respawn_delay=_rpi_zero_defaults.respawn_delay,
    )
    return [ackermann_motor_node, pi_zero_peripherals_node]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_launch_setup),
        ],
    )

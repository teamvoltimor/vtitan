"""Launch file for bench-testing motors with a joystick (e.g. 8BitDo Ultimate 2 over Bluetooth).

Run on the Raspberry Pi 5, once paired to the controller over Bluetooth
(OS-level pairing, outside ROS2). Reaches ackermann_motor_node on the Pi
Zero over the network via /ackermann_cmd, same as scripts/hardware/test_motors.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
                output="screen",
            ),
            Node(
                package="vtitan_drivers",
                executable="joy_teleop_node",
                name="joy_teleop",
                output="screen",
            ),
        ],
    )

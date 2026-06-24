"""Launch file for the Raspberry Pi 5 nodes (State Machine, Vision, IMU, LiDAR bridge, Telemetry)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="voldemorbot_robot",
            executable="state_machine_node",
            name="state_machine",
            output="screen",
            respawn=True,
            respawn_delay=3.0,
        ),
        Node(
            package="voldemorbot_robot",
            executable="bno08x_uart_rvc_node",
            name="imu",
            output="screen",
            respawn=True,
            respawn_delay=3.0,
        ),
        Node(
            package="voldemorbot_robot",
            executable="vision_node",
            name="vision",
            output="screen",
            respawn=True,
            respawn_delay=3.0,
        ),
        Node(
            package="voldemorbot_robot",
            executable="telemetry_bridge_node",
            name="telemetry_bridge",
            output="screen",
            respawn=True,
            respawn_delay=5.0,
        ),
    ])

"""Launch file for the Raspberry Pi 5 nodes (Vision, State Machine, Display, Sensors)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Pi 5 nodes."""
    return LaunchDescription(
        [
            Node(package="voldemorbot_robot", executable="state_machine_node", name="state_machine", output="screen"),
            Node(package="voldemorbot_robot", executable="oled_display_node", name="oled_display", output="screen"),
            Node(package="voldemorbot_robot", executable="telemetry_bridge_node", name="telemetry_bridge", output="screen"),
            # Assuming the camera and IMU nodes will be wrapped as entry points
            # in the setup.py in the future.
        ],
    )

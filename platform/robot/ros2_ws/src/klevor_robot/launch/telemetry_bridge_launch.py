from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="klevor_robot",
                executable="telemetry_bridge_node",
                name="telemetry_bridge",
                output="screen",
            ),
        ]
    )

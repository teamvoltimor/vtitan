from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="voldemorbot_state_machine",
                executable="telemetry_bridge_node",
                name="telemetry_bridge",
                output="screen",
            ),
        ],
    )

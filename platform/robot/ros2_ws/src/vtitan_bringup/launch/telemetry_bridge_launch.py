from launch import LaunchDescription
from launch_ros.actions import Node

from src.config.launch_settings import TelemetryBridgeLaunchSettings

_telemetry_settings = TelemetryBridgeLaunchSettings()


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vtitan_state_machine",
                executable="telemetry_bridge_node",
                name="telemetry_bridge",
                output="screen",
                parameters=[_telemetry_settings.as_node_parameters()],
            ),
        ],
    )

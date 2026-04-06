"""Launch file for running the software stack against Gazebo Simulator."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="klevor_robot",
                executable="state_machine_node",
                name="state_machine_sim",
                output="screen",
                parameters=[{"is_simulation": True}],
            ),
            # In simulation, gazebo acts as the camera publisher and build_hat_node
            # is replaced by the gazebo diff_drive/ackermann plugin.
            Node(
                package="klevor_robot", executable="telemetry_bridge_node", name="telemetry_bridge_sim", output="screen"
            ),
        ]
    )

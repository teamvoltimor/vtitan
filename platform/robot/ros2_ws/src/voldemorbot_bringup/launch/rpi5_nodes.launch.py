"""Launch file for the Raspberry Pi 5 nodes (State Machine, Vision, IMU, LiDAR bridge, Telemetry)."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

# The vision node's own defaults are the simulation ones (ultralytics on a .pt
# checkpoint). On the Pi the detector is the compiled GMR HEF on the NPU, so the
# backend and model are named here rather than left to fall back.
HAILO_MODEL_PATH = os.environ.get("HAILO_MODEL_PATH", "/usr/local/hailo/models/gmr.hef")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="voldemorbot_state_machine",
                executable="state_machine_node",
                name="state_machine",
                output="screen",
                respawn=True,
                respawn_delay=3.0,
            ),
            Node(
                package="voldemorbot_drivers",
                executable="bno08x_uart_rvc_node",
                name="imu",
                output="screen",
                respawn=True,
                respawn_delay=3.0,
            ),
            Node(
                package="voldemorbot_vision",
                executable="vision_node",
                name="vision",
                output="screen",
                parameters=[{"backend": "hailo", "model_path": HAILO_MODEL_PATH}],
                respawn=True,
                respawn_delay=3.0,
            ),
            Node(
                package="voldemorbot_state_machine",
                executable="telemetry_bridge_node",
                name="telemetry_bridge",
                output="screen",
                respawn=True,
                respawn_delay=5.0,
            ),
        ],
    )

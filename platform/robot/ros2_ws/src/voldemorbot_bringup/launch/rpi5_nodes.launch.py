"""Launch file for the Raspberry Pi 5 nodes (State Machine, Vision, IMU, LiDAR bridge, Telemetry)."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

# The vision node's own defaults are the simulation ones (ultralytics on a .pt
# checkpoint). On the Pi the detector is the compiled GMR HEF on the NPU, so the
# backend and model are named here rather than left to fall back.
HAILO_MODEL_PATH = os.environ.get("HAILO_MODEL_PATH", "/usr/local/hailo/models/gmr.hef")

# The vision node opens the camera itself and feeds frames straight to the NPU,
# so a race puts detections on the wire and no imagery at all. Set
# VISION_DEBUG_VIDEO=1 (or launch with debug_video:=true) to also publish the
# annotated stream for testing -- roughly 1.2 MB per frame, so not for a run.
DEBUG_VIDEO = os.environ.get("VISION_DEBUG_VIDEO", "").lower() in {"1", "true", "yes"}


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
                parameters=[
                    {
                        "backend": "hailo",
                        "model_path": HAILO_MODEL_PATH,
                        "camera_source": "direct",
                        "publish_annotated": DEBUG_VIDEO,
                        "publish_raw": DEBUG_VIDEO,
                    },
                ],
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

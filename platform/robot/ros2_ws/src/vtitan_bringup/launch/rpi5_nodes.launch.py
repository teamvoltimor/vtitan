"""Launch file for the Raspberry Pi 5 nodes (State Machine, Vision, IMU, LiDAR bridge, Telemetry)."""

from launch import LaunchDescription
from launch_ros.actions import Node

from src.config.launch_settings import TelemetryBridgeLaunchSettings, VisionLaunchSettings

_vision_settings = VisionLaunchSettings()
_telemetry_settings = TelemetryBridgeLaunchSettings()

# Seconds to wait before restarting a crashed node.
_RESPAWN_DELAY_SEC = 3.0
# telemetry_bridge_node gets a longer respawn delay than the other Pi 5 nodes.
_TELEMETRY_RESPAWN_DELAY_SEC = 5.0


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="vtitan_state_machine",
                executable="state_machine_node",
                name="state_machine",
                output="screen",
                respawn=True,
                respawn_delay=_RESPAWN_DELAY_SEC,
            ),
            Node(
                package="vtitan_drivers",
                executable="bno08x_uart_rvc_node",
                name="imu",
                output="screen",
                respawn=True,
                respawn_delay=_RESPAWN_DELAY_SEC,
            ),
            Node(
                package="vtitan_vision",
                executable="vision_node",
                name="vision",
                output="screen",
                parameters=[
                    {
                        "backend": "hailo",
                        "model_path": _vision_settings.hailo_model_path,
                        "camera_source": "direct",
                        "publish_annotated": _vision_settings.vision_debug_video,
                        "publish_raw": _vision_settings.vision_debug_video,
                    },
                ],
                respawn=True,
                respawn_delay=_RESPAWN_DELAY_SEC,
            ),
            Node(
                package="vtitan_state_machine",
                executable="telemetry_bridge_node",
                name="telemetry_bridge",
                output="screen",
                parameters=[_telemetry_settings.as_node_parameters()],
                respawn=True,
                respawn_delay=_TELEMETRY_RESPAWN_DELAY_SEC,
            ),
        ],
    )

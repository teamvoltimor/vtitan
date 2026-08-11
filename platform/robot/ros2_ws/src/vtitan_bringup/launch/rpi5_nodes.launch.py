"""Launch file for the Raspberry Pi 5 nodes (State Machine, Vision, IMU, LiDAR bridge, Telemetry)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from src.config.launch_settings import TelemetryBridgeLaunchSettings, VisionLaunchSettings

_vision_settings = VisionLaunchSettings()
_telemetry_settings = TelemetryBridgeLaunchSettings()

# Single source of truth for the vision node's deployed name. It overrides
# VisionNode's own default ("vision_detector"), and telemetry_bridge_node
# needs the same string to reach its set_parameters service for
# SET_VISION_DEBUG -- so both are derived from here rather than each
# hardcoding a name that can silently drift apart.
_VISION_NODE_NAME = "vision"

# Seconds to wait before restarting a crashed node.
_RESPAWN_DELAY_SEC = 3.0
# telemetry_bridge_node gets a longer respawn delay than the other Pi 5 nodes.
_TELEMETRY_RESPAWN_DELAY_SEC = 5.0


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "is_simulation",
                default_value="false",
                description=(
                    "Bypass state_machine_node's IMU/LiDAR/Hailo/challenge-mode-jumper "
                    "readiness checks in BOOT_CHECK (see state_machine_node.py's "
                    "_check_system_status) -- for bench testing on hardware that isn't "
                    "fully assembled yet (e.g. camera+Hailo only, no LIDAR/IMU/Pi Zero). "
                    "Defaults to false; the boot-time systemd service always uses the "
                    "default and must never be started with this true. Since /challenge_mode/"
                    "active is only ever published from a real jumper resolution (see "
                    "_sample_challenge_mode), it never fires under is_simulation -- "
                    "challenge-scoped behavior needs real hardware or a manual "
                    "/challenge_mode/active publish to exercise, this flag alone only "
                    "gets you to RACING.",
                ),
            ),
            Node(
                package="vtitan_state_machine",
                executable="state_machine_node",
                name="state_machine",
                output="screen",
                parameters=[{"is_simulation": LaunchConfiguration("is_simulation")}],
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
                name=_VISION_NODE_NAME,
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
                parameters=[
                    _telemetry_settings.as_node_parameters()
                    | {"vision_node_name": _VISION_NODE_NAME},
                ],
                respawn=True,
                respawn_delay=_TELEMETRY_RESPAWN_DELAY_SEC,
            ),
        ],
    )

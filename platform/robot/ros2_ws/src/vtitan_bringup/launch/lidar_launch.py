"""Launch file for the Slamtec C1 LIDAR node.

Usage:
    ros2 launch vtitan_bringup lidar_launch.py
    ros2 launch vtitan_bringup lidar_launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from shared.config.constants import RobotSpecs

from src.config.launch_settings import LidarLaunchDefaults

_lidar_defaults = LidarLaunchDefaults()


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the LIDAR node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=_lidar_defaults.serial_port,
                description="Serial port the C1 LIDAR is connected to",
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "serial_baudrate": 460800,
                        # Matches static_tfs.launch.py's child_frame_id -- must stay in sync.
                        "frame_id": "lidar_link",
                        # Single source of truth: RobotSpecs.LIDAR_INVERTED (robot.toml's
                        # [lidar].inverted). Never hardcode this separately from the
                        # 180deg yaw rotation ros2_hardware_gateway.py and
                        # static_tfs.launch.py apply for the same upside-down-mount
                        # fact -- letting the two drift apart is exactly the bug found
                        # and fixed 2026-08-02.
                        "inverted": RobotSpecs.LIDAR_INVERTED,
                        "angle_compensate": True,
                        "scan_mode": "Standard",
                    },
                ],
                output="screen",
            ),
        ],
    )

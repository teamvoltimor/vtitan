"""Launch file for the Slamtec C1 LIDAR node.

Usage:
    ros2 launch vtitan_bringup lidar_launch.py
    ros2 launch vtitan_bringup lidar_launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
                        # Re-verified 2026-08-02 against a known object placed at the
                        # chassis's physical left/right -- see run-lidar's pixi.toml
                        # comment for the full story. inverted:=True (this file's
                        # setting until now) read the physically opposite side.
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": "Standard",
                    },
                ],
                output="screen",
            ),
        ],
    )

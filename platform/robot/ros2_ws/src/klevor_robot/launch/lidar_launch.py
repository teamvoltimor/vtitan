"""Launch file for the Slamtec C1 LIDAR node.

Usage:
    ros2 launch klevor_robot lidar_launch.py
    ros2 launch klevor_robot lidar_launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the LIDAR node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port the C1 LIDAR is connected to",
        ),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": 460800,
                "frame_id": "laser_frame",
                "angle_compensate": True,
                "scan_mode": "Standard",
            }],
            output="screen",
        ),
    ])

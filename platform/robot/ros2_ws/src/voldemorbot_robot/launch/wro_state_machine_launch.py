"""Launch file for WRO competition state machine system.

This launch file starts all required nodes for the WRO competition:
- State machine controller
- OLED display with live mirroring
- IMU sensor node
- LiDAR sensor node (from sllidar_ros2 package)
- Ackermann motor controller (runs on Pi Zero via network)
- (Optional) Hailo AI node - if available
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for WRO state machine system."""
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
    )

    # Get package directories
    voldemorbot_robot_dir = get_package_share_directory("voldemorbot_robot")

    # Include LiDAR launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(voldemorbot_robot_dir) / "launch" / "lidar_launch.py")),
    )

    # Static TF publishers for sensor frames
    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(voldemorbot_robot_dir) / "launch" / "static_tfs.launch.py")),
    )

    # State machine controller node
    state_machine_node = Node(
        package="voldemorbot_robot",
        executable="state_machine_node",
        name="state_machine",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )

    # OLED display node with live mirroring
    oled_display_node = Node(
        package="voldemorbot_robot",
        executable="oled_display_node",
        name="oled_display",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )

    # IMU node (BNO08x via UART RVC mode)
    imu_node = Node(
        package="voldemorbot_robot",
        executable="bno08x_uart_rvc_node",
        name="bno08x_imu",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )

    # Ackermann motor controller node (NOTE: Should run on Pi Zero, not Pi 5)
    # This node subscribes to /ackermann_cmd and controls the Build HAT motors
    ackermann_motor_node = Node(
        package="voldemorbot_robot",
        executable="ackermann_motor_node",
        name="ackermann_motors",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            lidar_launch,
            static_tfs,
            state_machine_node,
            oled_display_node,
            imu_node,
            ackermann_motor_node,
        ],
    )

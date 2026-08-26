"""Bench-test launch file: runs the state machine plus its peripherals.

Runs the state machine plus its Pi 5 and Pi Zero peripherals together on a
single machine (all hardware wired directly to one board), rather than split
across two networked boards.

For the real two-board competition topology, use ``rpi5_nodes.launch.py`` on
the Pi 5 and ``rpi_zero_nodes.launch.py`` on the Pi Zero instead -- do not run
both that pair and this file at the same time (duplicate nodes on the same
topics).

Starts:
- State machine controller
- OLED display with live mirroring
- IMU sensor node
- LiDAR sensor node (from sllidar_ros2 package)
- Ackermann motor controller
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from src.config.launch_settings import MotorBackendLaunchSettings, StateMachineLaunchDefaults

_state_machine_defaults = StateMachineLaunchDefaults()
_motor_backend_settings = MotorBackendLaunchSettings()


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for WRO state machine system."""
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=str(_state_machine_defaults.use_sim_time).lower(),
        description="Use simulation time if true",
    )

    # Get package directories
    vtitan_bringup_dir = get_package_share_directory("vtitan_bringup")

    # Include LiDAR launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(vtitan_bringup_dir) / "launch" / "lidar_launch.py")),
    )

    # Static TF publishers for sensor frames
    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(vtitan_bringup_dir) / "launch" / "static_tfs.launch.py")),
    )

    # State machine controller node
    state_machine_node = Node(
        package="vtitan_state_machine",
        executable="state_machine_node",
        name="state_machine",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=_state_machine_defaults.respawn_delay,
    )

    # OLED display node with live mirroring
    oled_display_node = Node(
        package="vtitan_drivers",
        executable="oled_display_node",
        name="oled_display",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=_state_machine_defaults.respawn_delay,
    )

    # IMU node (BNO08x via UART RVC mode)
    imu_node = Node(
        package="vtitan_drivers",
        executable="bno08x_uart_rvc_node",
        name="bno08x_imu",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=_state_machine_defaults.respawn_delay,
    )

    # Ackermann motor controller node. This node subscribes to /ackermann_cmd
    # and controls the drive/steering hardware.
    ackermann_motor_node = Node(
        package="vtitan_drivers",
        executable="ackermann_motor_node",
        name="ackermann_motors",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            _motor_backend_settings.as_node_parameters(),
        ],
        respawn=True,
        respawn_delay=_state_machine_defaults.respawn_delay,
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

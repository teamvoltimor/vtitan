"""Launch file for the Raspberry Pi Zero 2W (Motor Controller)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    ackermann_motor_node = Node(
        package="voldemorbot_robot",
        executable="ackermann_motor_node",
        name="ackermann_motors",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([use_sim_time_arg, ackermann_motor_node])

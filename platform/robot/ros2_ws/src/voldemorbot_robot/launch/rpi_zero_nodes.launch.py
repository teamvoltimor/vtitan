"""Launch file for the Raspberry Pi Zero 2W (Motors, Button, OLED)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch-arg name -> environment variable read by ackermann_motor_node.
# An empty launch arg is omitted so the node falls back to .env / systemd.
_BACKEND_ENV = {
    "steering_backend": "STEERING_BACKEND",
    "drive_backend": "DRIVE_BACKEND",
}


def _launch_setup(context, *_args, **_kwargs):
    additional_env = {}
    for arg, env_key in _BACKEND_ENV.items():
        value = LaunchConfiguration(arg).perform(context)
        if value:  # only override when explicitly provided
            additional_env[env_key] = value

    ackermann_motor_node = Node(
        package="voldemorbot_robot",
        executable="ackermann_motor_node",
        name="ackermann_motors",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        additional_env=additional_env,
        respawn=True,
        respawn_delay=2.0,
    )
    button_node = Node(
        package="voldemorbot_robot",
        executable="button_node",
        name="button_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )
    oled_display_node = Node(
        package="voldemorbot_robot",
        executable="oled_display_node",
        name="oled_display_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        respawn=True,
        respawn_delay=2.0,
    )
    return [ackermann_motor_node, button_node, oled_display_node]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    steering_backend_arg = DeclareLaunchArgument(
        "steering_backend",
        default_value="",
        description="Override STEERING_BACKEND (servo|build_hat); empty = use .env",
    )
    drive_backend_arg = DeclareLaunchArgument(
        "drive_backend",
        default_value="",
        description="Override DRIVE_BACKEND (dc_encoder|build_hat); empty = use .env",
    )

    return LaunchDescription([
        use_sim_time_arg,
        steering_backend_arg,
        drive_backend_arg,
        OpaqueFunction(function=_launch_setup),
    ])

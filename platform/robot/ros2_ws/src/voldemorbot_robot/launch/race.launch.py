"""Launch file for a competition run: starts the track navigator.

Run this on the Pi 5 in addition to ``rpi5_nodes.launch.py`` (already running
as a boot-time systemd service) once the scenario metadata for the round is
known — the navigator needs ``/scan``, ``/imu/data`` and ``/hailo/detections``,
all published by the nodes that file launches. It isn't bundled into that
boot-time launch file because, unlike those always-on nodes, it requires a
per-round metadata path that doesn't exist until the round is set up.

Usage:
    ros2 launch voldemorbot_robot race.launch.py metadata:=/path/to/scenario_metadata.json
    ros2 launch voldemorbot_robot race.launch.py metadata:=... laps:=3 params:=... tuning:=...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    arguments = ["--metadata", LaunchConfiguration("metadata").perform(context)]

    laps = LaunchConfiguration("laps").perform(context)
    if laps:
        arguments += ["--laps", laps]
    params = LaunchConfiguration("params").perform(context)
    if params:
        arguments += ["--params", params]
    tuning = LaunchConfiguration("tuning").perform(context)
    if tuning:
        arguments += ["--tuning", tuning]

    track_navigator_node = Node(
        package="voldemorbot_robot",
        executable="track_navigator_node",
        name="track_navigator",
        output="screen",
        arguments=arguments,
    )
    return [track_navigator_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "metadata",
            description="Path to the scenario metadata JSON for this round (required)",
        ),
        DeclareLaunchArgument(
            "laps",
            default_value="3",
            description="Laps to complete",
        ),
        DeclareLaunchArgument(
            "params",
            default_value="",
            description="Optional navigator_params.json for runtime overrides",
        ),
        DeclareLaunchArgument(
            "tuning",
            default_value="",
            description="Optional navigation tuning YAML",
        ),
        OpaqueFunction(function=_launch_setup),
    ])

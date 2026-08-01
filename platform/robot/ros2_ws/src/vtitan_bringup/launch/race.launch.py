"""Launch file for a competition run: starts the track navigator.

Run this on the Pi 5 in addition to ``rpi5_nodes.launch.py`` (already running
as a boot-time systemd service) once the scenario metadata for the round is
known — the navigator needs ``/scan``, ``/imu/data`` and ``/vision/detections``,
all published by the nodes that file launches. It isn't bundled into that
boot-time launch file because, unlike those always-on nodes, it requires a
per-round metadata path that doesn't exist until the round is set up.

By default this also records a rosbag of every topic relevant to a run
(sensor input, vision detections, drive commands, state machine, telemetry)
so a bad run can be replayed and inspected afterwards. Bags land in
``bag_dir`` (default ``~/vtitan_runs``) under a timestamped
``run_<metadata-stem>_<YYYYmmdd_HHMMSS>`` directory. Disable with
``record:=false``.

Usage:
    ros2 launch vtitan_bringup race.launch.py metadata:=/path/to/scenario_metadata.json
    ros2 launch vtitan_bringup race.launch.py metadata:=... laps:=3 params:=... tuning:=...
    ros2 launch vtitan_bringup race.launch.py metadata:=... record:=false
    ros2 launch vtitan_bringup race.launch.py metadata:=... bag_dir:=/path/to/runs
"""

import datetime
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from src.config.launch_settings import RaceLaunchDefaults

_race_defaults = RaceLaunchDefaults()

# Topics worth keeping for post-run analysis: sensor input, the vision and
# navigation decisions derived from it, the resulting drive command, and the
# state machine/telemetry view of what the robot thought was happening.
_BAG_TOPICS = [
    "/scan",
    "/imu/data",
    "/camera/image_raw",
    "/vision/detections",
    "/ackermann_cmd",
    "/robot_state",
    "/race_metrics",
    "/system_status",
    "/tf",
    "/tf_static",
]


def _launch_setup(context: LaunchContext, *_args, **_kwargs) -> list:
    metadata = LaunchConfiguration("metadata").perform(context)
    arguments = ["--metadata", metadata] if metadata else []
    direction = LaunchConfiguration("direction").perform(context)
    if direction:
        arguments += ["--direction", direction]

    laps = LaunchConfiguration("laps").perform(context)
    if laps:
        arguments += ["--laps", laps]
    params = LaunchConfiguration("params").perform(context)
    if params:
        arguments += ["--params", params]
    tuning = LaunchConfiguration("tuning").perform(context)
    if tuning:
        arguments += ["--tuning", tuning]
    blind = LaunchConfiguration("blind").perform(context).lower() not in ("false", "0", "")
    if blind:
        arguments += ["--blind"]

    track_navigator_node = Node(
        package="vtitan_navigation",
        executable="track_navigator_node",
        name="track_navigator",
        output="screen",
        arguments=arguments,
    )
    actions: list = [track_navigator_node]

    record = LaunchConfiguration("record").perform(context).lower() not in ("false", "0", "")
    if record:
        bag_dir = Path(LaunchConfiguration("bag_dir").perform(context)).expanduser()
        bag_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")  # noqa: DTZ005
        run_name = f"run_{Path(metadata).stem}_{stamp}" if metadata else f"run_{stamp}"
        bag_record_node = ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", str(bag_dir / run_name), *_BAG_TOPICS],
            output="screen",
        )
        actions.append(bag_record_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "metadata",
                default_value="",
                description=(
                    "Path to the scenario metadata JSON. Leave empty for competition: the "
                    "robot then runs with no scenario file at all, estimating the layout "
                    "from LIDAR. Supplying one is for reproducing a known layout in "
                    "testing, and implies the sighted navigator unless blind:=true."
                ),
            ),
            DeclareLaunchArgument(
                "direction",
                default_value=_race_defaults.direction,
                description=(
                    "Travel direction for the round (cw|ccw). The only start condition "
                    "that cannot be assumed: assuming the starting section merely rotates "
                    "the robot's own world frame, but the direction is a reflection and "
                    "getting it wrong puts the inner block on the wrong side."
                ),
            ),
            DeclareLaunchArgument(
                "blind",
                default_value=str(_race_defaults.blind).lower(),
                description=(
                    "Force layout estimation even when metadata is supplied. Implied "
                    "automatically when metadata is empty, since there is then nothing "
                    "to be told."
                ),
            ),
            DeclareLaunchArgument(
                "laps",
                default_value=str(_race_defaults.laps),
                description="Laps to complete",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=_race_defaults.params,
                description="Optional navigator_params.json for runtime overrides",
            ),
            DeclareLaunchArgument(
                "tuning",
                default_value=_race_defaults.tuning,
                description="Optional navigation tuning YAML",
            ),
            DeclareLaunchArgument(
                "record",
                default_value=str(_race_defaults.record).lower(),
                description="Record a rosbag of the run (sensors, decisions, drive commands)",
            ),
            DeclareLaunchArgument(
                "bag_dir",
                default_value=_race_defaults.bag_dir,
                description="Directory to write timestamped rosbag run folders into",
            ),
            OpaqueFunction(function=_launch_setup),
        ],
    )

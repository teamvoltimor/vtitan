"""Typed config for ROS2 launch files under ``ros2_ws/src/vtitan_bringup/launch/``.

Launch files run as plain Python (``PYTHONPATH=.`` set by the pixi ``launch-*``
tasks), so they can import from ``src/`` the same way node code does. This
replaces ad hoc ``os.environ.get(...)`` calls and hardcoded
``DeclareLaunchArgument(default_value=...)`` literals with pydantic-settings,
per the repo's env-config convention (see ``src/hardware/settings_base.py``).

Two distinct uses here:

- ``VisionLaunchSettings``/``TelemetryBridgeLaunchSettings`` feed ``Node``
  parameters directly (no ``ros2 launch key:=value`` layer exists for
  these today) -- field names map to env var names (case-insensitive, no
  prefix) so existing ``BACKEND_URL=...`` overrides keep working unchanged.
- ``LidarLaunchDefaults``/``RaceLaunchDefaults``/``StateMachineLaunchDefaults``
  only supply ``DeclareLaunchArgument(default_value=...)`` values.
  ``LaunchConfiguration``/``ros2 launch key:=value`` stays the actual
  override mechanism at runtime -- these just move the *default* out of a
  hardcoded string literal into a config file, so the ``ros2 launch ...
  key:=value`` CLI surface is unchanged.
"""

from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict
from shared.config.constants import CompetitionSpecs
from shared.config.ros_topics import RosTopicConfig

from src.hardware.settings_base import ROBOT_ROOT, HardwareBaseSettings


def _default_bag_topics() -> list[str]:
    """Return the topics worth keeping for post-run analysis.

    Sensor input, the vision and
    navigation decisions derived from it, the resulting drive command, the
    state machine/telemetry view of what the robot thought was happening,
    and the motor node's own measured speed/steering (the only ground truth
    for what the robot actually did, as opposed to what it was commanded).

    Sourced from ros_topics.toml (via RosTopicConfig) rather than restated as
    literals, so this list can't drift from what actually publishes. /tf and
    /tf_static are the two exceptions -- standard ROS2 TF topics with no
    RosTopicConfig entry of their own.

    /camera/image_raw is deliberately absent: it was measured at 63 MB/s,
    which dwarfs everything else here combined and is what turns a race bag
    into a full SD card. The detections it produces are recorded instead,
    which is what replaying a run's decisions actually needs.
    """
    topics = RosTopicConfig.load_default()
    return [
        topics.sensors.scan,
        topics.sensors.imu,
        topics.sensors.vision_detections,
        topics.commands.ackermann_cmd,
        topics.state_machine.state,
        topics.state_machine.race_metrics,
        topics.navigation.nav_debug,
        topics.state_machine.system_status,
        topics.actuators.drive_speed,
        topics.actuators.steering_position,
        # The bay-exit clearance guard's ONLY state input. It dead-reckons the
        # pocket pose from `get_wheel_odometry().distance_m`, which is this
        # topic's drive-wheel position -- not `drive_speed`, which is a smoothed
        # estimate with a different bias. Absent from the 2026-09-06 bags, an
        # offline replay of a 14.2 s deadlock could not be made faithful:
        # reconstructions that reproduced the stall destroyed the runs that
        # escaped, and no single one exceeded 90% agreement on all three.
        # Cheap to record -- a handful of floats, nowhere near /camera/image_raw.
        topics.actuators.joint_states,
        "/tf",
        "/tf_static",
    ]


# Separate from config/hardware/ (driver calibration) -- these are
# launch-time session/routing defaults, a different concern.
LAUNCH_CONFIG_DIR: Path = ROBOT_ROOT / "config" / "launch"


class VisionLaunchSettings(BaseSettings):
    """Overrides for the Hailo vision node, as launched on the Pi 5."""

    model_config = SettingsConfigDict(env_prefix="")

    hailo_model_path: str = "/usr/local/hailo/models/gmr.hef"
    """Compiled .hef the NPU loads. The vision node's own default is the
    simulation one (ultralytics .pt checkpoint), so this is always named
    explicitly on real hardware rather than left to fall back."""

    vision_debug_video: bool = False
    """Also publish the annotated/raw camera stream (~1.2 MB/frame) for
    testing. Never set for a real run -- the vision node opens the camera
    itself and feeds frames straight to the NPU, so enabling this puts
    imagery on the wire that a race doesn't need."""


class MotorBackendLaunchSettings(BaseSettings):
    """Steering/drive backend selection for ackermann_motor_node.

    Field names map to the STEERING_BACKEND/DRIVE_BACKEND env vars documented
    in ackermann_motor_node.py's own docstring and .env.example -- no prefix,
    same convention as VisionLaunchSettings. Fed straight into the node's
    ``steering_backend``/``drive_backend`` ROS2 params, since no
    ``ros2 launch key:=value`` layer exists for these today.
    """

    model_config = SettingsConfigDict(env_prefix="")

    steering_backend: str = "servo"
    drive_backend: str = "l298n"

    def as_node_parameters(self) -> dict[str, str]:
        """Dict of parameters to pass to ackermann_motor_node."""
        return {
            "steering_backend": self.steering_backend,
            "drive_backend": self.drive_backend,
        }


class TelemetryBridgeLaunchSettings(BaseSettings):
    """Overrides for telemetry_bridge_node, wherever it's launched from.

    The node's own parameter defaults point at localhost:8010/9010, which
    only works when the backend runs on the same host as the robot process.
    Override when the backend runs elsewhere (e.g. a dev machine on the
    same LAN).
    """

    model_config = SettingsConfigDict(env_prefix="")

    backend_url: str = "http://localhost:8010"
    command_channel_target: str = "localhost:9010"
    publish_rate_hz: float = 10.0
    """Rate of the HTTP POST to the backend, over WiFi/LAN."""

    ui_summary_rate_hz: float = 10.0
    """Rate of the JSON blob to the Pi Zero over the USB-gadget link.

    Deliberately independent of ``publish_rate_hz``: that one crosses the
    network, this one crosses a USB gadget with plenty of headroom at a few
    hundred bytes. Matched to the OLED's own 10 Hz redraw -- anything lower was
    just making every other redraw show stale numbers.
    """

    max_path_history: int = 120
    """Pose samples the bridge keeps for the path overlay."""

    telemetry_channel_target: str | None = None
    """Falls back to command_channel_target when unset -- most setups run
    both channels through the same backend host."""

    @property
    def resolved_telemetry_channel_target(self) -> str:
        """The telemetry channel to use, defaulting to the command channel."""
        return self.telemetry_channel_target or self.command_channel_target

    def as_node_parameters(self) -> dict[str, str]:
        """Dict of parameters to pass to telemetry_bridge_node."""
        return {
            "backend_url": self.backend_url,
            "command_channel_target": self.command_channel_target,
            "telemetry_channel_target": self.resolved_telemetry_channel_target,
        }


class LidarLaunchDefaults(HardwareBaseSettings):
    """Defaults for lidar_launch.py's LIDAR driver parameters.

    ``serial_port`` is a DeclareLaunchArgument (override at runtime with
    ``serial_port:=...``); the rest are fixed driver parameters passed straight
    to the sllidar_ros2 node.
    """

    model_config = SettingsConfigDict(env_prefix="lidar_launch_", toml_file=LAUNCH_CONFIG_DIR / "lidar.toml")

    serial_port: str = "/dev/ttyUSB0"
    serial_baudrate: int = 460800
    scan_mode: str = "Standard"
    angle_compensate: bool = True


class StateMachineLaunchDefaults(HardwareBaseSettings):
    """Default for wro_state_machine_launch.py's DeclareLaunchArguments."""

    model_config = SettingsConfigDict(
        env_prefix="state_machine_launch_",
        toml_file=LAUNCH_CONFIG_DIR / "wro_state_machine.toml",
    )

    use_sim_time: bool = False
    respawn_delay: float = 2.0


class Rpi5LaunchDefaults(HardwareBaseSettings):
    """Defaults for rpi5_nodes.launch.py: respawn delays and vision wiring."""

    model_config = SettingsConfigDict(env_prefix="rpi5_launch_", toml_file=LAUNCH_CONFIG_DIR / "rpi5.toml")

    respawn_delay: float = 3.0
    """Seconds to wait before restarting a crashed Pi 5 node."""

    telemetry_respawn_delay: float = 5.0
    """telemetry_bridge_node gets a longer respawn delay than the other Pi 5 nodes."""

    vision_backend: str = "hailo"
    """Vision backend the Pi 5 vision_node runs (``hailo`` vs the sim's ultralytics)."""

    camera_source: str = "direct"
    """Camera source the vision_node opens on real hardware."""


class RpiZeroLaunchDefaults(HardwareBaseSettings):
    """Default for rpi_zero_nodes.launch.py: respawn delay."""

    model_config = SettingsConfigDict(env_prefix="rpi_zero_launch_", toml_file=LAUNCH_CONFIG_DIR / "rpi_zero.toml")

    respawn_delay: float = 2.0
    """Seconds to wait before restarting a crashed Pi Zero node."""


class RaceLaunchDefaults(HardwareBaseSettings):
    """Defaults for race.launch.py's DeclareLaunchArguments.

    ``metadata`` deliberately has no useful default beyond "" (empty ->
    competition mode, LIDAR-only layout estimation) -- there's no single
    scenario file that makes sense as a checked-in default.
    """

    model_config = SettingsConfigDict(env_prefix="race_launch_", toml_file=LAUNCH_CONFIG_DIR / "race.toml")

    # "undetermined" rather than a real direction: a default that names a
    # direction is a claim nobody made, and the navigator cannot tell it apart
    # from an operator who meant it. Set direction:=cw|ccw per round to skip
    # blind inference entirely; leave it to infer as before.
    direction: str = "undetermined"
    blind: bool = False
    laps: int = CompetitionSpecs.OPEN_CHALLENGE_LAPS
    params: str = ""
    tuning: str = ""
    record: bool = True
    # Runs land in the repo-root data/live/runs tree (shared by the Python and
    # Go stacks as siblings, never the robot module's own dir -- see repo-root
    # .gitignore). Resolved relative to the repo root so a fresh clone or the Pi
    # both write to the same place without an absolute path.
    bag_dir: str = str(Path(__file__).resolve().parents[4] / "data" / "live" / "runs")
    # Retention caps for bag_recorder_node. Recording is race-gated, but a
    # competition day is many rounds and the card is finite, so old runs are
    # pruned oldest-first once either cap is exceeded. Sized for a diagnosis
    # session rather than a single day: a whole corpus of runs stays on the
    # card so bag scripts can compare across sessions instead of finding the
    # evidence already pruned. Check free space before deploying to a Pi -- 50
    # GB does not fit on a small card, and the count cap alone will not save it.
    # The size cap is the one that binds here: measured 2026-09-09 over 215 real
    # runs, mean 30.0 MB each, so 50 GB stops at ~1708 runs before the count
    # ever reaches 2000. race.toml pins both and carries the full distribution.
    bag_max_runs: int = 2000
    bag_max_total_gb: float = 50.0
    bag_topics: list[str] = Field(default_factory=_default_bag_topics)

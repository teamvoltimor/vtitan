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

from pydantic_settings import BaseSettings, SettingsConfigDict

from src.hardware.settings_base import ROBOT_ROOT, HardwareBaseSettings

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
    telemetry_channel_target: str | None = None
    """Falls back to command_channel_target when unset -- most setups run
    both channels through the same backend host."""

    @property
    def resolved_telemetry_channel_target(self) -> str:
        return self.telemetry_channel_target or self.command_channel_target

    def as_node_parameters(self) -> dict[str, str]:
        return {
            "backend_url": self.backend_url,
            "command_channel_target": self.command_channel_target,
            "telemetry_channel_target": self.resolved_telemetry_channel_target,
        }


class LidarLaunchDefaults(HardwareBaseSettings):
    """Default for lidar_launch.py's ``serial_port`` DeclareLaunchArgument."""

    model_config = SettingsConfigDict(env_prefix="lidar_launch_", toml_file=LAUNCH_CONFIG_DIR / "lidar.toml")

    serial_port: str = "/dev/ttyUSB0"


class StateMachineLaunchDefaults(HardwareBaseSettings):
    """Default for wro_state_machine_launch.py's ``use_sim_time`` DeclareLaunchArgument."""

    model_config = SettingsConfigDict(
        env_prefix="state_machine_launch_",
        toml_file=LAUNCH_CONFIG_DIR / "wro_state_machine.toml",
    )

    use_sim_time: bool = False


class RaceLaunchDefaults(HardwareBaseSettings):
    """Defaults for race.launch.py's DeclareLaunchArguments.

    ``metadata`` deliberately has no useful default beyond "" (empty ->
    competition mode, LIDAR-only layout estimation) -- there's no single
    scenario file that makes sense as a checked-in default.
    """

    model_config = SettingsConfigDict(env_prefix="race_launch_", toml_file=LAUNCH_CONFIG_DIR / "race.toml")

    direction: str = "cw"
    blind: bool = False
    laps: int = 3
    params: str = ""
    tuning: str = ""
    record: bool = True
    bag_dir: str = "~/vtitan_runs"

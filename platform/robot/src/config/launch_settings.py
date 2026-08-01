"""Typed env-var overrides for ROS2 launch files under ``ros2_ws/src/vtitan_bringup/launch/``.

Launch files run as plain Python (``PYTHONPATH=.`` set by the pixi ``launch-*``
tasks), so they can import from ``src/`` the same way node code does. This
replaces ad hoc ``os.environ.get(...)`` calls with pydantic-settings, per the
repo's env-config convention (see ``src/hardware/settings_base.py``).

Field names map directly to env var names (case-insensitive, no prefix) so
existing ``BACKEND_URL=...`` / ``HAILO_MODEL_PATH=...`` overrides (systemd
``EnvironmentFile=``, local ``.env``, shell exports) keep working unchanged.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict


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

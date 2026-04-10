"""Configuration and simulation constants.

This module centralizes all hardcoded values, environment variables,
and configuration parameters following Python-Architect standards.
"""

from __future__ import annotations

from pathlib import Path
from typing import Final

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class SimulationConstants:
    """Physics and generation parameters for the deterministic simulator.

    All values are documented with units and rationale to facilitate tuning.
    """

    RNG_SEED: Final[int] = 0
    ORBIT_PERIOD_FRAMES: Final[int] = 60
    TRACK_CENTER_X: Final[float] = 1.5
    TRACK_CENTER_Y: Final[float] = 1.5
    TRACK_HALF_WIDTH: Final[float] = 1.5
    ORBIT_RADIUS_BASE: Final[float] = 0.9
    ORBIT_RADIUS_MODULATION: Final[float] = 0.2
    ORBIT_MODULATION_FREQUENCY: Final[float] = 0.08
    STAGE_DURATION_FRAMES: Final[int] = 18
    STAGE_NAMES: Final[tuple[str, ...]] = (
        "start",
        "acceleration",
        "cornering",
        "straightaway",
        "finish",
    )
    LOG_ENTRIES_PER_SNAPSHOT: Final[int] = 3
    LOG_TEMPLATES: Final[tuple[str, ...]] = (
        "ROS bridge: synchronized · {frequency:.1f} Hz",
        "Navigator: {stage} segment · {percent:.0f}% complete",
        "LIDAR: captured {points} points · obstacle {obstacle:.2f} m ahead",
        "Telemetry node {health} · speed {speed:.2f} m/s",
        "Replay buffer: {entries} snapshots stored",
    )
    LIDAR_RESOLUTION: Final[int] = 360
    LIDAR_BASE_RANGE: Final[float] = 1.2
    LIDAR_SINE_TURBULENCE: Final[float] = 0.08
    LIDAR_RANDOM_TURBULENCE: Final[float] = 0.02
    LIDAR_PERIODIC_MODULATION: Final[float] = 0.2
    LIDAR_MODULATION_FREQUENCY: Final[int] = 4
    LIDAR_HEIGHT: Final[float] = 0.04


class RecorderConfig:
    """Persistence and replay configuration."""

    DEFAULT_MAX_SESSIONS: Final[int] = 20
    SESSION_FILENAME_PREFIX: Final[str] = "session_"
    SESSION_FILE_EXTENSION: Final[str] = ".jsonl"


class ServerConfig(BaseSettings):
    """Runtime server configuration loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_prefix="TELEMETRY_", env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )

    port: int = Field(default=8010, ge=1, le=65535)
    reload: bool = Field(default=False)
    sessions_dir: Path = Field(
        default_factory=lambda: Path(__file__).parent.parent.parent.parent / "telemetry_sessions"
    )
    max_sessions: int = Field(default=20, ge=1)
    poll_interval_ms: int = Field(default=2500, ge=10)

    @classmethod
    def from_env(cls) -> ServerConfig:
        """Load configuration from environment variables with validation.

        Returns:
            Validated ServerConfig instance.
        """
        return cls()

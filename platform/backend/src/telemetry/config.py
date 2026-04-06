"""Configuration and simulation constants.

This module centralizes all hardcoded values, environment variables,
and configuration parameters following Python-Architect standards.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Final


class SimulationConstants:
    """Physics and generation parameters for the deterministic simulator.

    All values are documented with units and rationale to facilitate tuning.
    """

    # Random Number Generation
    RNG_SEED: Final[int] = 0
    """Random seed for reproducible telemetry generation across runs."""

    # Orbital Motion Parameters
    ORBIT_PERIOD_FRAMES: Final[int] = 60
    """Number of frames required to complete one full orbit around the track."""

    # Track Geometry (3m x 3m square)
    TRACK_CENTER_X: Final[float] = 1.5
    """X-coordinate of track center in meters."""

    TRACK_CENTER_Y: Final[float] = 1.5
    """Y-coordinate of track center in meters."""

    TRACK_HALF_WIDTH: Final[float] = 1.5
    """Half-width of track (robot orbits within a 3m x 3m square)."""

    # Orbital Radius Modulation
    ORBIT_RADIUS_BASE: Final[float] = 0.9
    """Base orbital radius in meters."""

    ORBIT_RADIUS_MODULATION: Final[float] = 0.2
    """Amplitude of sine-wave radius modulation in meters."""

    ORBIT_MODULATION_FREQUENCY: Final[float] = 0.08
    """Frequency coefficient for orbital radius variation."""

    # Navigation Stages
    STAGE_DURATION_FRAMES: Final[int] = 18
    """Number of frames per navigation stage."""

    STAGE_NAMES: Final[tuple[str, ...]] = (
        "start",
        "acceleration",
        "cornering",
        "straightaway",
        "finish",
    )
    """Navigation mission stages."""

    # Logging
    LOG_ENTRIES_PER_SNAPSHOT: Final[int] = 3
    """Number of log messages to generate per telemetry snapshot."""

    LOG_TEMPLATES: Final[tuple[str, ...]] = (
        "ROS bridge: synchronized · {frequency:.1f} Hz",
        "Navigator: {stage} segment · {percent:.0f}% complete",
        "LIDAR: captured {points} points · obstacle {obstacle:.2f} m ahead",
        "Telemetry node {health} · speed {speed:.2f} m/s",
        "Replay buffer: {entries} snapshots stored",
    )
    """Template strings for log message generation."""

    # LiDAR Parameters
    LIDAR_RESOLUTION: Final[int] = 360
    """Number of points per LiDAR sweep."""

    LIDAR_BASE_RANGE: Final[float] = 1.2
    """Base detection range in meters."""

    LIDAR_SINE_TURBULENCE: Final[float] = 0.08
    """Amplitude of deterministic sine-wave turbulence."""

    LIDAR_RANDOM_TURBULENCE: Final[float] = 0.02
    """Amplitude of random turbulence (from seeded RNG)."""

    LIDAR_PERIODIC_MODULATION: Final[float] = 0.2
    """Amplitude of periodic modulation in radial distance."""

    LIDAR_MODULATION_FREQUENCY: Final[int] = 4
    """Frequency multiplier for periodic modulation (4 * angle)."""

    LIDAR_HEIGHT: Final[float] = 0.04
    """Z-height of LiDAR scanner above ground in meters."""


class RecorderConfig:
    """Persistence and replay configuration."""

    DEFAULT_MAX_SESSIONS: Final[int] = 20
    """Maximum number of recorded sessions to retain before eviction."""

    SESSION_FILENAME_PREFIX: Final[str] = "session_"
    """Prefix for recorded session files."""

    SESSION_FILE_EXTENSION: Final[str] = ".jsonl"
    """Newline-delimited JSON format for session files."""


@dataclass(slots=True)
class ServerConfig:
    """Runtime server configuration loaded from environment variables.

    All environment variables are validated and converted to appropriate types.
    Uses dataclass with slots for memory efficiency (Python-Architect Rule 2).
    """

    telemetry_port: int = field(default=8010)
    """Server listening port (1-65535)."""

    telemetry_reload: bool = field(default=False)
    """Enable auto-reload on code changes (development only)."""

    sessions_dir: Path = field(default_factory=lambda: Path("telemetry_sessions"))
    """Directory for storing recorded session files."""

    max_sessions: int = field(default=20)
    """Maximum number of recorded sessions to retain (must be >= 1)."""

    poll_interval_ms: int = field(default=2500)
    """Interval between simulation frame broadcasts in milliseconds."""

    def __post_init__(self) -> None:
        """Validate configuration values after initialization."""
        if not (1 <= self.telemetry_port <= 65535):
            raise ValueError(f"telemetry_port {self.telemetry_port} out of valid range (1-65535)")
        if self.max_sessions < 1:
            raise ValueError(f"max_sessions must be >= 1, got {self.max_sessions}")

    @classmethod
    def from_env(cls) -> ServerConfig:
        """Load configuration from environment variables with validation.

        Returns:
            Validated ServerConfig instance.

        Raises:
            ValueError: If any environment variable is invalid.
        """
        try:
            port = int(os.environ.get("TELEMETRY_PORT", "8010"))
            if not (1 <= port <= 65535):
                raise ValueError(f"Port {port} out of valid range (1-65535)")

            reload_flag = os.environ.get("TELEMETRY_RELOAD", "0") == "1"

            sessions_dir = Path(
                os.environ.get(
                    "TELEMETRY_SESSIONS_DIR",
                    str(Path(__file__).parent.parent.parent / "telemetry_sessions"),
                )
            )

            max_sessions = int(os.environ.get("TELEMETRY_MAX_SESSIONS", "20"))
            if max_sessions < 1:
                raise ValueError(f"max_sessions must be >= 1, got {max_sessions}")

            return cls(
                telemetry_port=port,
                telemetry_reload=reload_flag,
                sessions_dir=sessions_dir,
                max_sessions=max_sessions,
            )
        except (ValueError, TypeError) as exc:
            raise ValueError(f"Invalid configuration: {exc}") from exc

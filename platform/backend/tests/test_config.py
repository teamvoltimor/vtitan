"""Configuration loading and validation tests."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
from pydantic import ValidationError

from src.telemetry.config import ServerConfig, SimulationConstants


class TestSimulationConstants:
    """Verify simulation constants are properly defined."""

    def test_orbit_parameters_are_documented(self) -> None:
        """Orbit parameters have clear names."""
        assert SimulationConstants.ORBIT_PERIOD_FRAMES == 60
        assert SimulationConstants.TRACK_CENTER_X == 1.5
        assert SimulationConstants.TRACK_CENTER_Y == 1.5

    def test_lidar_parameters_are_realistic(self) -> None:
        """LiDAR parameters are within sensible ranges."""
        assert SimulationConstants.LIDAR_RESOLUTION == 360
        assert 0.1 < SimulationConstants.LIDAR_BASE_RANGE < 5.0
        assert SimulationConstants.LIDAR_HEIGHT > 0

    def test_stage_names_are_complete(self) -> None:
        """All navigation stages are defined."""
        assert len(SimulationConstants.STAGE_NAMES) == 5
        assert "start" in SimulationConstants.STAGE_NAMES


class TestServerConfigDefaults:
    """Test ServerConfig default values."""

    def test_default_port_is_8010(self) -> None:
        """Default port matches expected value."""
        config = ServerConfig()
        assert config.port == 8010

    def test_default_reload_is_false(self) -> None:
        """Default reload flag is disabled."""
        config = ServerConfig()
        assert config.reload is False

    def test_default_max_sessions_is_20(self) -> None:
        """Default max sessions is 20."""
        config = ServerConfig()
        assert config.max_sessions == 20

    def test_default_poll_interval_is_2500ms(self) -> None:
        """Default poll interval is 2500 milliseconds."""
        config = ServerConfig()
        assert config.poll_interval_ms == 2500


class TestServerConfigValidation:
    """Test ServerConfig validation."""

    def test_port_must_be_in_valid_range(self) -> None:
        """Port must be between 1 and 65535."""
        with pytest.raises(ValidationError):
            ServerConfig(port=0)

        with pytest.raises(ValidationError):
            ServerConfig(port=65536)

    def test_max_sessions_must_be_positive(self) -> None:
        """max_sessions must be >= 1."""
        with pytest.raises(ValidationError):
            ServerConfig(max_sessions=0)

        with pytest.raises(ValidationError):
            ServerConfig(max_sessions=-1)

    def test_valid_port_accepted(self) -> None:
        """Valid ports are accepted."""
        config = ServerConfig(port=9000)
        assert config.port == 9000


class TestServerConfigFromEnv:
    """Test ServerConfig loading from environment variables."""

    def test_from_env_uses_defaults(self, monkeypatch) -> None:
        """from_env applies sensible defaults when env vars not set."""
        # Clear environment variables
        monkeypatch.delenv("TELEMETRY_PORT", raising=False)
        monkeypatch.delenv("TELEMETRY_RELOAD", raising=False)
        monkeypatch.delenv("TELEMETRY_MAX_SESSIONS", raising=False)

        config = ServerConfig.from_env()
        assert config.port == 8010
        assert config.reload is False
        assert config.max_sessions == 20

    def test_from_env_reads_port(self, monkeypatch) -> None:
        """from_env reads TELEMETRY_PORT from environment."""
        monkeypatch.setenv("TELEMETRY_PORT", "9000")
        config = ServerConfig.from_env()
        assert config.port == 9000

    def test_from_env_reads_reload_flag(self, monkeypatch) -> None:
        """from_env reads TELEMETRY_RELOAD from environment."""
        monkeypatch.setenv("TELEMETRY_RELOAD", "1")
        config = ServerConfig.from_env()
        assert config.reload is True

    def test_from_env_reads_max_sessions(self, monkeypatch) -> None:
        """from_env reads TELEMETRY_MAX_SESSIONS from environment."""
        monkeypatch.setenv("TELEMETRY_MAX_SESSIONS", "50")
        config = ServerConfig.from_env()
        assert config.max_sessions == 50

    def test_from_env_handles_invalid_port(self, monkeypatch) -> None:
        """ServerConfig.from_env validates port from environment."""
        monkeypatch.setenv("TELEMETRY_PORT", "invalid")
        with pytest.raises(ValidationError):
            ServerConfig.from_env()

    def test_from_env_handles_invalid_max_sessions(self, monkeypatch) -> None:
        """ServerConfig.from_env validates max_sessions from environment."""
        monkeypatch.setenv("TELEMETRY_MAX_SESSIONS", "-5")
        with pytest.raises(ValidationError):
            ServerConfig.from_env()

"""Verify architectural standards compliance."""

from __future__ import annotations

from pathlib import Path

import pytest

from pydantic import ValidationError

from src.telemetry.config import ServerConfig
from src.telemetry.dependencies import create_app_state
from src.telemetry.exceptions import TelemetryError


class TestAppStateImmutability:
    """Verify TelemetryAppState is frozen (Python-Architect Rule 2)."""

    def test_app_state_cannot_be_modified(self, tmp_path: Path) -> None:
        """Frozen dataclass prevents attribute assignment."""
        config = ServerConfig(sessions_dir=tmp_path)
        state = create_app_state(config)

        # Attempt to modify frozen instance should raise AttributeError
        with pytest.raises(AttributeError):
            state.generator = None


class TestDependencyInjection:
    """Verify dependency injection works correctly (Python-Architect Rule 3)."""

    def test_app_state_factory_creates_all_dependencies(self, tmp_path: Path) -> None:
        """create_app_state returns fully initialized state."""
        config = ServerConfig(sessions_dir=tmp_path)
        state = create_app_state(config)

        assert state.generator is not None
        assert state.recorder is not None
        assert state.connection_manager is not None

    def test_dependencies_are_distinct_instances(self, tmp_path: Path) -> None:
        """Each app_state has independent dependency instances."""
        config = ServerConfig(sessions_dir=tmp_path)
        state1 = create_app_state(config)
        state2 = create_app_state(config)

        # Different instances (not singletons)
        assert state1.generator is not state2.generator
        assert state1.recorder is not state2.recorder


class TestConfigurationValidation:
    """Verify ServerConfig validates inputs (Error Handling: Configuration)."""

    def test_port_must_be_in_valid_range(self) -> None:
        """Port validation rejects out-of-range values."""
        with pytest.raises(ValidationError):
            ServerConfig(port=70000)

    def test_max_sessions_must_be_positive(self) -> None:
        """max_sessions validation rejects non-positive values."""
        with pytest.raises(ValidationError):
            ServerConfig(max_sessions=0)


class TestExceptionHierarchy:
    """Verify exception hierarchy is properly structured."""

    def test_all_telemetry_errors_inherit_from_base(self) -> None:
        """Custom exceptions inherit from TelemetryError."""
        from src.telemetry.exceptions import (
            BroadcastError,
            RecorderError,
            SessionNotFoundError,
        )

        assert issubclass(BroadcastError, TelemetryError)
        assert issubclass(RecorderError, TelemetryError)
        assert issubclass(SessionNotFoundError, RecorderError)

    def test_exception_can_chain_from_original(self) -> None:
        """Exceptions support error chaining."""
        original_error = OSError("Disk full")
        try:
            raise original_error
        except OSError as exc:
            from src.telemetry.exceptions import RecorderError

            with pytest.raises(RecorderError) as exc_info:
                raise RecorderError(f"Failed: {exc}") from exc

        assert exc_info.value.__cause__ is original_error

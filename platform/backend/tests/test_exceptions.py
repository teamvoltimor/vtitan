"""Exception hierarchy and error handling tests."""

from __future__ import annotations

import pytest

from src.telemetry.exceptions import (
    BroadcastError,
    ConfigurationError,
    CorruptedSessionError,
    RecorderError,
    SessionNotFoundError,
    SimulationError,
    TelemetryError,
)


class TestExceptionInheritance:
    """Verify exception hierarchy structure."""

    def test_broadcast_error_inherits_from_telemetry_error(self) -> None:
        """BroadcastError is a TelemetryError."""
        assert issubclass(BroadcastError, TelemetryError)

    def test_configuration_error_inherits_from_telemetry_error(self) -> None:
        """ConfigurationError is a TelemetryError."""
        assert issubclass(ConfigurationError, TelemetryError)

    def test_recorder_error_inherits_from_telemetry_error(self) -> None:
        """RecorderError is a TelemetryError."""
        assert issubclass(RecorderError, TelemetryError)

    def test_session_not_found_error_inherits_from_recorder_error(self) -> None:
        """SessionNotFoundError is a RecorderError."""
        assert issubclass(SessionNotFoundError, RecorderError)

    def test_corrupted_session_error_inherits_from_recorder_error(self) -> None:
        """CorruptedSessionError is a RecorderError."""
        assert issubclass(CorruptedSessionError, RecorderError)

    def test_simulation_error_inherits_from_telemetry_error(self) -> None:
        """SimulationError is a TelemetryError."""
        assert issubclass(SimulationError, TelemetryError)


class TestExceptionInstantiation:
    """Verify exceptions can be raised and caught."""

    def test_can_raise_and_catch_telemetry_error(self) -> None:
        """TelemetryError can be raised and caught."""
        with pytest.raises(TelemetryError):
            raise TelemetryError("Test error")

    def test_can_raise_and_catch_broadcast_error(self) -> None:
        """BroadcastError can be raised and caught."""
        with pytest.raises(BroadcastError):
            raise BroadcastError("Broadcast failed")

    def test_can_raise_and_catch_recorder_error(self) -> None:
        """RecorderError can be raised and caught."""
        with pytest.raises(RecorderError):
            raise RecorderError("Recording failed")

    def test_can_raise_and_catch_session_not_found_error(self) -> None:
        """SessionNotFoundError can be raised and caught."""
        with pytest.raises(SessionNotFoundError):
            raise SessionNotFoundError("Session not found")


class TestExceptionChaining:
    """Verify exception chaining works correctly."""

    def test_exception_preserves_cause(self) -> None:
        """Exception preserves original error with 'from' clause."""
        original = OSError("Disk full")
        try:
            raise original
        except OSError as exc:
            with pytest.raises(RecorderError) as exc_info:
                raise RecorderError(f"Failed: {exc}") from exc

        assert exc_info.value.__cause__ is original

    def test_exception_message_includes_detail(self) -> None:
        """Exception message preserves error details."""
        detail = "Permission denied on /data/sessions"
        with pytest.raises(RecorderError) as exc_info:
            raise RecorderError(detail)

        assert detail in str(exc_info.value)


class TestExceptionCatching:
    """Verify exceptions can be caught by base class."""

    def test_catch_recorder_error_as_telemetry_error(self) -> None:
        """RecorderError can be caught as TelemetryError."""
        with pytest.raises(TelemetryError):
            raise RecorderError("Test")

    def test_catch_broadcast_error_as_telemetry_error(self) -> None:
        """BroadcastError can be caught as TelemetryError."""
        with pytest.raises(TelemetryError):
            raise BroadcastError("Test")

    def test_catch_session_not_found_as_telemetry_error(self) -> None:
        """SessionNotFoundError can be caught as TelemetryError."""
        with pytest.raises(TelemetryError):
            raise SessionNotFoundError("Test")

"""Resilience and error recovery tests."""

from __future__ import annotations

from unittest import mock

import pytest

from src.telemetry.exceptions import RecorderError, SessionNotFoundError
from src.telemetry.models import RobotSnapshot
from src.telemetry.persistence.jsonl_reader import JSONLReader
from src.telemetry.recorder import TelemetryRecorder
from src.telemetry.ws.manager import ConnectionManager


class TestRecorderErrorHandling:
    """Test recorder resilience to I/O errors."""

    def test_recorder_handles_disk_full(self, tmp_path, sample_snapshot: RobotSnapshot) -> None:
        """Recorder handles OSError (disk full) gracefully."""
        recorder = TelemetryRecorder(base_dir=tmp_path)

        # First ensure the file is opened by attempting a successful record
        # to set up the file handle, then mock it for the error
        recorder.ensure_file_open()

        # Mock file.flush() to simulate disk full error
        with mock.patch.object(recorder.file, "flush") as mock_flush:
            mock_flush.side_effect = OSError(28, "No space left on device")

            with pytest.raises(RecorderError, match="Failed to persist"):
                recorder.record(sample_snapshot)

    def test_recorder_handles_permission_error(self, tmp_path, sample_snapshot: RobotSnapshot) -> None:
        """Recorder handles permission errors."""
        recorder = TelemetryRecorder(base_dir=tmp_path)

        # Ensure file is open before mocking
        recorder.ensure_file_open()

        with mock.patch.object(recorder.file, "flush") as mock_flush:
            mock_flush.side_effect = PermissionError("Permission denied")

            with pytest.raises(RecorderError):
                recorder.record(sample_snapshot)

    def test_load_session_not_found(self, tmp_path) -> None:
        """Recorder raises SessionNotFoundError for missing session."""
        recorder = TelemetryRecorder(base_dir=tmp_path)

        with pytest.raises(SessionNotFoundError):
            recorder.load_session("nonexistent_session")

    def test_load_session_with_valid_data(self, tmp_path, sample_snapshot: RobotSnapshot) -> None:
        """Recorder can load previously recorded session."""
        recorder = TelemetryRecorder(base_dir=tmp_path)
        recorder.record(sample_snapshot)
        recorder.close()

        # Load the recorded session
        loaded = recorder.load_session(recorder.session_id)
        assert len(loaded) == 1
        assert loaded[0].timestamp == sample_snapshot.timestamp


class TestBroadcastErrorHandling:
    """Test WebSocket broadcast resilience."""

    @pytest.mark.asyncio()
    async def test_broadcast_removes_disconnected_clients(self) -> None:
        """Broadcast removes clients that fail to send."""
        manager = ConnectionManager()

        # Mock WebSocket connections
        mock_conn1 = mock.AsyncMock()
        mock_conn2 = mock.AsyncMock()

        manager.active_connections = [mock_conn1, mock_conn2]

        # First connection succeeds, second fails
        mock_conn1.send_text.return_value = None
        mock_conn2.send_text.side_effect = RuntimeError("Connection closed")

        await manager.broadcast('{"test": "data"}')

        # After broadcast, disconnected client should be removed
        assert mock_conn1 in manager.active_connections
        assert mock_conn2 not in manager.active_connections

    @pytest.mark.asyncio()
    async def test_broadcast_timeout_tracking(self) -> None:
        """Broadcast tracks timeout metrics."""
        manager = ConnectionManager()

        mock_conn = mock.AsyncMock()
        manager.active_connections = [mock_conn]

        # Simulate timeout
        mock_conn.send_text.side_effect = TimeoutError()

        await manager.broadcast('{"test": "data"}')

        metrics = manager.get_metrics()
        assert metrics["broadcast_timeouts"] == 1

    def test_broadcast_metrics_tracking(self) -> None:
        """Broadcast metrics are correctly tracked."""
        manager = ConnectionManager()
        assert manager.get_metrics()["broadcast_failures"] == 0
        assert manager.get_metrics()["broadcast_timeouts"] == 0
        assert manager.get_metrics()["client_disconnects"] == 0


class TestJSONLParsingResilience:
    """Test JSONL reader resilience to corrupted files."""

    def test_jsonl_reader_skips_invalid_lines(self, tmp_path) -> None:
        """JSONLReader can skip corrupted lines."""
        # Create file with corrupted data structure
        jsonl_file = tmp_path / "test.jsonl"
        jsonl_file.write_text(
            '{"timestamp": 1.0, "missionName": "test", "metrics": {"timestamp": 1.0, "nodeHealth": "nominal"}}\n'
            "CORRUPTED_LINE\n"
            '{"timestamp": 2.0, "missionName": "test", "metrics": {"timestamp": 2.0, "nodeHealth": "nominal"}}\n',
        )

        # Read with skip_invalid=True
        snapshots, corrupted = JSONLReader.read_file(
            jsonl_file,
            RobotSnapshot,
            skip_invalid=True,
        )

        list(snapshots)
        assert len(corrupted) == 1
        assert corrupted[0][0] == 2  # Line 2 was corrupted

    def test_jsonl_reader_raises_on_invalid_without_skip(self, tmp_path) -> None:
        """JSONLReader raises when skip_invalid=False."""
        jsonl_file = tmp_path / "test.jsonl"
        jsonl_file.write_text("CORRUPTED_LINE\n")

        reader, _ = JSONLReader.read_file(
            jsonl_file,
            RobotSnapshot,
            skip_invalid=False,
        )
        with pytest.raises(ValueError, match="Line 1"):
            list(reader)


class TestConnectionManagerMetrics:
    """Test ConnectionManager metrics tracking."""

    def test_has_active_connections_returns_bool(self) -> None:
        """has_active_connections returns boolean."""
        manager = ConnectionManager()
        assert manager.has_active_connections() is False

    def test_get_metrics_returns_dict(self) -> None:
        """get_metrics returns a dictionary."""
        manager = ConnectionManager()
        metrics = manager.get_metrics()
        assert isinstance(metrics, dict)
        assert "broadcast_failures" in metrics
        assert "broadcast_timeouts" in metrics
        assert "client_disconnects" in metrics

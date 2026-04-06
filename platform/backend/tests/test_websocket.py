"""Integration tests for WebSocket endpoint."""

from __future__ import annotations

from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from src.telemetry.app import create_app
from src.telemetry.config import ServerConfig


@pytest.fixture
def client(tmp_path: Path):
    """Create TestClient with temporary session directory."""
    config = ServerConfig(sessions_dir=tmp_path)
    app = create_app(config)
    return TestClient(app)


class TestWebSocketEndpoint:
    """Test WebSocket endpoint behavior."""

    def test_websocket_connection_accepted(self, client: TestClient) -> None:
        """WebSocket connection is accepted without error."""
        with client.websocket_connect("/telemetry/ws") as websocket:
            # Connection should succeed
            assert websocket is not None

    def test_websocket_heartbeat_response(self, client: TestClient) -> None:
        """WebSocket responds to ping with pong."""
        with client.websocket_connect("/telemetry/ws") as websocket:
            websocket.send_text("ping")
            data = websocket.receive_text()
            assert data == "pong"

    def test_websocket_disconnect_cleanup(self, client: TestClient) -> None:
        """WebSocket cleanup happens on disconnect."""
        with client.websocket_connect("/telemetry/ws") as websocket:
            pass  # Connection closes on context manager exit
        # Should not raise any errors

    def test_websocket_multiple_connections(self, client: TestClient) -> None:
        """Multiple WebSocket connections can coexist."""
        with client.websocket_connect("/telemetry/ws") as ws1:
            with client.websocket_connect("/telemetry/ws") as ws2:
                ws1.send_text("ping")
                ws2.send_text("ping")
                assert ws1.receive_text() == "pong"
                assert ws2.receive_text() == "pong"

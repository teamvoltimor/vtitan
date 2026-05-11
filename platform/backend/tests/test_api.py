"""Integration tests for FastAPI routes."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
from fastapi.testclient import TestClient

from src.telemetry.app import create_app
from src.telemetry.config import ServerConfig

if TYPE_CHECKING:
    from pathlib import Path


@pytest.fixture()
def app_with_config(tmp_path: Path):
    """Create FastAPI app with temporary configuration."""
    config = ServerConfig(sessions_dir=tmp_path)
    return create_app(config)


@pytest.fixture()
def client(app_with_config):
    """Create TestClient for FastAPI app."""
    return TestClient(app_with_config)


class TestHealthEndpoint:
    """Test GET /telemetry/health endpoint."""

    def test_health_check_returns_ok(self, client: TestClient) -> None:
        """Health endpoint returns success status."""
        response = client.get("/telemetry/health")
        assert response.status_code == 200
        assert response.json()["status"] == "ok"

    def test_health_includes_version(self, client: TestClient) -> None:
        """Health endpoint includes version."""
        response = client.get("/telemetry/health")
        data = response.json()
        assert "version" in data
        assert data["version"] == "0.3.0"

    def test_health_includes_uptime(self, client: TestClient) -> None:
        """Health endpoint includes uptime_seconds."""
        response = client.get("/telemetry/health")
        data = response.json()
        assert "uptime_seconds" in data
        assert data["uptime_seconds"] >= 0


class TestConfigEndpoint:
    """Test GET /telemetry/config endpoint."""

    def test_config_endpoint_returns_config(self, client: TestClient) -> None:
        """Config endpoint returns current configuration."""
        response = client.get("/telemetry/config")
        assert response.status_code == 200
        data = response.json()
        assert "max_sessions" in data
        assert "sessions_dir" in data
        assert "poll_interval_ms" in data


class TestLatestSnapshotEndpoint:
    """Test GET /telemetry/latest endpoint."""

    def test_latest_returns_robot_snapshot(self, client: TestClient) -> None:
        """Latest endpoint returns valid RobotSnapshot."""
        response = client.get("/telemetry/latest")
        assert response.status_code == 200

        data = response.json()
        assert "timestamp" in data
        assert "missionName" in data
        assert "metrics" in data


class TestHistoryEndpoint:
    """Test GET /telemetry/history endpoint."""

    def test_history_returns_list(self, client: TestClient) -> None:
        """History endpoint returns list of snapshots."""
        response = client.get("/telemetry/history")
        assert response.status_code == 200
        assert isinstance(response.json(), list)

    def test_history_respects_limit(self, client: TestClient) -> None:
        """History endpoint respects limit parameter."""
        response = client.get("/telemetry/history?limit=30")
        assert response.status_code == 200
        assert len(response.json()) <= 30

    def test_history_rejects_invalid_limit(self, client: TestClient) -> None:
        """History endpoint rejects out-of-range limit."""
        response = client.get("/telemetry/history?limit=5")
        assert response.status_code == 422  # Validation error


class TestSessionsEndpoints:
    """Test session recording and replay endpoints."""

    def test_list_sessions_returns_empty_initially(self, client: TestClient) -> None:
        """Sessions list is empty before any recordings."""
        response = client.get("/telemetry/sessions")
        assert response.status_code == 200
        assert response.json() == []

    def test_load_missing_session_returns_404(self, client: TestClient) -> None:
        """Loading nonexistent session returns 404."""
        response = client.get("/telemetry/sessions/nonexistent")
        assert response.status_code == 404


class TestTopicsEndpoint:
    """Test GET /telemetry/topics endpoint."""

    def test_topics_endpoint_returns_snapshot(self, client: TestClient) -> None:
        """Topics endpoint returns TopicsSnapshot."""
        response = client.get("/telemetry/topics")
        assert response.status_code == 200
        data = response.json()
        assert "timestamp" in data
        assert "topics" in data

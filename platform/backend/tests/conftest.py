"""Shared pytest fixtures and configuration."""

from __future__ import annotations

from pathlib import Path

import pytest

from src.telemetry.config import ServerConfig
from src.telemetry.models import NodeHealth, RobotSnapshot, TelemetryMetrics


@pytest.fixture
def server_config(tmp_path: Path) -> ServerConfig:
    """Create a ServerConfig with temporary session directory."""
    return ServerConfig(sessions_dir=tmp_path)


@pytest.fixture
def sample_metrics() -> TelemetryMetrics:
    """Create a sample TelemetryMetrics object."""
    return TelemetryMetrics(
        timestamp=1.0,
        node_health=NodeHealth.NOMINAL,
        points_captured=360,
        range_min=0.1,
        range_max=3.0,
        range_mean=1.5,
        forward=1.0,
        left=0.8,
        right=0.8,
        back=1.2,
        speed=0.5,
        stage="straightaway",
    )


@pytest.fixture
def sample_snapshot(sample_metrics: TelemetryMetrics) -> RobotSnapshot:
    """Create a sample RobotSnapshot for testing."""
    return RobotSnapshot(
        timestamp=1.0,
        mission_name="test",
        robot_position=(1.5, 1.5, 0.0),
        robot_orientation=0.0,
        lidar_points=[],
        path_history=[],
        logs=[],
        metrics=sample_metrics,
    )

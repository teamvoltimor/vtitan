"""Unit tests for TelemetryGenerator."""

from __future__ import annotations

import pytest

from src.telemetry.generator import TelemetryGenerator
from src.telemetry.models import RobotSnapshot


def test_latest_snapshot_returns_robot_snapshot() -> None:
    gen = TelemetryGenerator()
    snap = gen.latest_snapshot()
    assert isinstance(snap, RobotSnapshot)


def test_robot_position_within_track_bounds() -> None:
    gen = TelemetryGenerator()
    for _ in range(30):
        snap = gen.latest_snapshot()
        x, y, z = snap.robot_position
        assert 0.0 <= x <= 3.0, f"x={x} out of bounds"
        assert 0.0 <= y <= 3.0, f"y={y} out of bounds"
        assert z == pytest.approx(0.0)


def test_history_grows_up_to_max() -> None:
    gen = TelemetryGenerator(history_length=5)
    for _ in range(10):
        gen.latest_snapshot()
    assert len(gen.history()) == 5


def test_history_limit_parameter() -> None:
    gen = TelemetryGenerator(history_length=20)
    for _ in range(15):
        gen.latest_snapshot()
    assert len(gen.history(limit=5)) == 5
    assert len(gen.history(limit=100)) == 15


def test_lidar_points_count_matches_resolution() -> None:
    gen = TelemetryGenerator(lidar_resolution=180)
    snap = gen.latest_snapshot()
    assert len(snap.lidar_points) == 180


def test_logs_are_non_empty() -> None:
    gen = TelemetryGenerator()
    snap = gen.latest_snapshot()
    assert len(snap.logs) > 0
    for log in snap.logs:
        assert isinstance(log, str)
        assert log


def test_mission_name_is_consistent() -> None:
    gen = TelemetryGenerator()
    names = {gen.latest_snapshot().mission_name for _ in range(5)}
    assert len(names) == 1


def test_deterministic_with_same_seed() -> None:
    gen1 = TelemetryGenerator()
    gen2 = TelemetryGenerator()
    # First snapshot from both generators should have identical positions
    s1 = gen1.latest_snapshot()
    s2 = gen2.latest_snapshot()
    assert s1.robot_position == s2.robot_position

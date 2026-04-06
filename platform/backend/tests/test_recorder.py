"""Unit tests for TelemetryRecorder."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.telemetry.models import NodeHealth, RobotSnapshot, TelemetryMetrics
from src.telemetry.recorder import TelemetryRecorder
from src.telemetry.exceptions import SessionNotFoundError


def _make_snapshot(timestamp: float = 1.0) -> RobotSnapshot:
    metrics = TelemetryMetrics(
        timestamp=timestamp,
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
    return RobotSnapshot(
        timestamp=timestamp,
        mission_name="test",
        robot_position=(1.5, 1.5, 0.0),
        robot_orientation=0.0,
        lidar_points=[],
        path_history=[],
        logs=[],
        metrics=metrics,
    )


def test_record_and_load(tmp_path: Path) -> None:
    with TelemetryRecorder(base_dir=tmp_path, session_id="session_001") as rec:
        snap = _make_snapshot()
        rec.record(snap)
        assert rec.entry_count == 1

    loaded = list(TelemetryRecorder(base_dir=tmp_path, session_id="session_002").load_session("session_001"))
    assert len(loaded) == 1
    assert loaded[0].mission_name == "test"
    assert loaded[0].robot_position == (1.5, 1.5, 0.0)


def test_multiple_records(tmp_path: Path) -> None:
    with TelemetryRecorder(base_dir=tmp_path, session_id="session_multi") as rec:
        for i in range(5):
            rec.record(_make_snapshot(timestamp=float(i)))
        assert rec.entry_count == 5

    loaded = list(TelemetryRecorder(base_dir=tmp_path, session_id="x").load_session("session_multi"))
    assert len(loaded) == 5
    assert [s.timestamp for s in loaded] == [0.0, 1.0, 2.0, 3.0, 4.0]


def test_load_missing_session_raises(tmp_path: Path) -> None:
    rec = TelemetryRecorder(base_dir=tmp_path, session_id="s")
    with pytest.raises(SessionNotFoundError):
        list(rec.load_session("nonexistent"))
    rec.close()


def test_eviction(tmp_path: Path) -> None:
    # Create 5 sessions with max_sessions=3 — oldest 2 should be evicted.
    for i in range(5):
        with TelemetryRecorder(base_dir=tmp_path, session_id=f"session_{i:04d}", max_sessions=3) as rec:
            rec.record(_make_snapshot())

    remaining = sorted(tmp_path.glob("session_*.jsonl"))
    assert len(remaining) == 3
    # The two oldest are gone; the three newest survive.
    names = [p.stem for p in remaining]
    assert "session_0000" not in names
    assert "session_0001" not in names


def test_list_sessions(tmp_path: Path) -> None:
    for i in range(3):
        with TelemetryRecorder(base_dir=tmp_path, session_id=f"session_{i:04d}") as rec:
            rec.record(_make_snapshot())

    sessions = list(TelemetryRecorder.enumerate_sessions(tmp_path))
    assert len(sessions) == 3
    # enumerate_sessions returns newest first
    ids = [s.session_id for s in sessions]
    assert ids == sorted(ids, reverse=True)


def test_context_manager_closes_file(tmp_path: Path) -> None:
    with TelemetryRecorder(base_dir=tmp_path, session_id="session_cm") as rec:
        rec.record(_make_snapshot())
    assert rec._file.closed


def test_jsonl_format(tmp_path: Path) -> None:
    with TelemetryRecorder(base_dir=tmp_path, session_id="session_fmt") as rec:
        rec.record(_make_snapshot(1.0))
        rec.record(_make_snapshot(2.0))

    lines = (tmp_path / "session_fmt.jsonl").read_text().splitlines()
    assert len(lines) == 2
    for line in lines:
        data = json.loads(line)
        assert "timestamp" in data

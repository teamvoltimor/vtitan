"""Unit tests for RaceTracker module.

Tests race metrics collection and lap tracking.
"""

import time

import pytest
from shared.domain.models import Waypoint

from src.logger.constants import DETAILS_KEY
from src.navigation.race_tracker import RaceMetrics, RaceTracker


class TestRaceMetrics:
    """Tests for RaceMetrics dataclass."""

    def test_race_metrics_initialization(self):
        metrics = RaceMetrics()

        assert metrics.elapsed_time == 0.0
        assert metrics.total_distance == 0.0
        assert metrics.current_lap == 1
        assert metrics.completed_laps == 0
        assert metrics.escape_maneuvers == 0
        assert metrics.stuck_detections == 0
        assert metrics.collision_warnings == 0
        assert metrics.lap_splits == []

    def test_race_metrics_with_values(self):
        metrics = RaceMetrics(
            elapsed_time=60.0,
            total_distance=50.0,
            current_lap=2,
            completed_laps=1,
            escape_maneuvers=3,
        )

        assert metrics.elapsed_time == 60.0
        assert metrics.total_distance == 50.0
        assert metrics.current_lap == 2
        assert metrics.completed_laps == 1
        assert metrics.escape_maneuvers == 3

    def test_race_metrics_to_dict(self):
        metrics = RaceMetrics(elapsed_time=30.0, current_lap=1, escape_maneuvers=2)
        d = metrics.to_dict()

        assert isinstance(d, dict)
        assert d["elapsed_time"] == 30.0
        assert d["current_lap"] == 1
        assert d["escape_maneuvers"] == 2
        assert "lap_splits" in d


class TestRaceTracker:
    """Tests for RaceTracker metrics collection."""

    @pytest.fixture()
    def tracker(self):
        return RaceTracker(num_laps=3)

    def test_tracker_initialization(self, tracker):
        assert tracker.num_laps == 3
        assert tracker._start_time is not None

    def test_tracker_initialization_with_start_time(self):
        tracker = RaceTracker(num_laps=2)
        assert tracker._start_time is not None
        assert isinstance(tracker._start_time, float)

    def test_get_metrics(self, tracker):
        metrics = tracker.get_race_metrics()

        assert isinstance(metrics, RaceMetrics)
        assert metrics.current_lap == 1
        assert metrics.elapsed_time >= 0.0

    def test_increment_lap(self, tracker):
        metrics = tracker.get_race_metrics()
        initial_lap = metrics.current_lap

        tracker.increment_lap()
        metrics = tracker.get_race_metrics()

        assert metrics.current_lap == initial_lap + 1

    def test_lap_splits_recorded(self, tracker):
        tracker.increment_lap()
        metrics = tracker.get_race_metrics()

        assert len(metrics.lap_splits) == 1
        assert metrics.lap_splits[0] >= 0.0

    def test_lap_splits_multiple(self, tracker):
        tracker.increment_lap()
        tracker.increment_lap()
        metrics = tracker.get_race_metrics()

        assert len(metrics.lap_splits) == 2
        assert metrics.lap_splits[1] >= metrics.lap_splits[0]

    def test_record_escape_maneuver(self, tracker):
        tracker.record_escape_maneuver()
        assert tracker.get_race_metrics().escape_maneuvers == 1

        tracker.record_escape_maneuver()
        assert tracker.get_race_metrics().escape_maneuvers == 2

    def test_record_stuck_detection(self, tracker):
        tracker.record_stuck_detection()
        assert tracker.get_race_metrics().stuck_detections == 1

    def test_record_collision_warning(self, tracker):
        tracker.record_collision_warning()
        assert tracker.get_race_metrics().collision_warnings == 1

    def test_multiple_incidents(self, tracker):
        tracker.record_escape_maneuver()
        tracker.record_stuck_detection()
        tracker.record_collision_warning()
        tracker.record_escape_maneuver()

        metrics = tracker.get_race_metrics()
        assert metrics.escape_maneuvers == 2
        assert metrics.stuck_detections == 1
        assert metrics.collision_warnings == 1

    def test_elapsed_time_increases(self, tracker):
        metrics1 = tracker.get_race_metrics()
        time.sleep(0.01)
        metrics2 = tracker.get_race_metrics()

        assert metrics2.elapsed_time >= metrics1.elapsed_time

    def test_metrics_are_fresh(self, tracker):
        tracker.record_escape_maneuver()
        count_after_one = tracker.get_race_metrics().escape_maneuvers

        tracker.record_escape_maneuver()
        count_after_two = tracker.get_race_metrics().escape_maneuvers

        assert count_after_two > count_after_one

    def test_tracker_persistent_state(self, tracker):
        tracker.increment_lap()
        tracker.record_escape_maneuver()
        tracker.increment_lap()
        tracker.record_escape_maneuver()

        metrics = tracker.get_race_metrics()
        assert metrics.escape_maneuvers == 2


class TestRaceSummary:
    """get_race_summary returns the typed rollup, not a dict[str, Any]."""

    @pytest.fixture()
    def tracker(self):
        return RaceTracker(num_laps=3)

    def test_before_the_first_lap_there_is_nothing_to_extrapolate(self, tracker):
        tracker.update_position(Waypoint(x=0.0, y=0.0), 0.2, waypoint_index=4)
        update_time = tracker.get_race_metrics().elapsed_time
        assert update_time > 0.0

        summary = tracker.get_race_summary()

        assert summary.laps_remaining == 3
        assert summary.est_finish_time == 0.0
        assert summary.metrics.waypoint_index == 4

    def test_laps_completed_extrapolate_the_finish(self, tracker):
        tracker.update_position(Waypoint(x=0.0, y=0.0), 0.2, waypoint_index=0)
        tracker.increment_lap()
        summary = tracker.get_race_summary()

        assert summary.laps_remaining == 2
        assert summary.est_finish_time == pytest.approx(summary.metrics.elapsed_time * 3.0)

    def test_laps_remaining_floors_at_zero(self, tracker):
        tracker.metrics.completed_laps = 5
        assert tracker.get_race_summary().laps_remaining == 0

    def test_to_dict_flattens_to_te_log_contract(self, tracker):
        """The payload log_summary emits unchanged: metrics by alias, then the
        two derived keys at top level."""
        tracker.update_position(Waypoint(x=1.0, y=0.0), 0.3, waypoint_index=2)
        tracker.increment_lap()
        tracker.metrics.extra_data = {"note": "x"}

        flat = tracker.get_race_summary().to_dict()

        assert flat["current_lap"] == 2
        assert flat["completed_laps"] == 1
        assert flat["extra"] == {"note": "x"}
        assert flat["laps_remaining"] == 2
        assert flat["est_finish_time"] == pytest.approx(tracker.get_race_summary().est_finish_time)

    def test_log_summary_carries_the_structured_payload(self, tracker, caplog):
        tracker.increment_lap()
        tracker.log_summary()

        record = next(r for r in caplog.records if r.message == "Race Summary")
        details = record.__dict__[DETAILS_KEY]
        assert isinstance(details, dict)
        assert details["laps_remaining"] == 2

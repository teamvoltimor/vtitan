"""Unit tests for RaceTracker module.

Tests race metrics collection and lap tracking.
"""

import pytest
from src.navigation.race_tracker import RaceTracker, RaceMetrics


class TestRaceMetrics:
    """Tests for RaceMetrics dataclass."""

    def test_race_metrics_initialization(self):
        """Test RaceMetrics initializes with correct state."""
        metrics = RaceMetrics()

        assert metrics.elapsed_time == 0.0
        assert metrics.total_distance == 0.0
        assert metrics.current_lap == 1
        assert metrics.completed_laps == 0
        assert metrics.escape_maneuvers == 0
        assert metrics.stuck_detections == 0
        assert metrics.collision_warnings == 0

    def test_race_metrics_with_values(self):
        """Test RaceMetrics with custom values."""
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
        """Test RaceMetrics serialization to dictionary."""
        metrics = RaceMetrics(
            elapsed_time=30.0,
            current_lap=1,
            escape_maneuvers=2,
        )

        d = metrics.to_dict()

        assert isinstance(d, dict)
        assert d["elapsed_time"] == 30.0
        assert d["current_lap"] == 1
        assert d["escape_maneuvers"] == 2


class TestRaceTracker:
    """Tests for RaceTracker metrics collection."""

    @pytest.fixture
    def tracker(self):
        """Create a RaceTracker instance for testing."""
        return RaceTracker(total_laps=3)

    def test_tracker_initialization(self, tracker):
        """Test tracker initializes with correct state."""
        assert tracker.total_laps == 3
        assert tracker.start_time is not None

    def test_tracker_initialization_with_start_time(self):
        """Test tracker initializes with start time."""
        tracker = RaceTracker(total_laps=2)
        assert tracker.start_time is not None
        assert isinstance(tracker.start_time, float)

    def test_get_metrics(self, tracker):
        """Test getting race metrics."""
        metrics = tracker.get_metrics()

        assert isinstance(metrics, RaceMetrics)
        assert metrics.current_lap == 0
        assert metrics.elapsed_time >= 0.0

    def test_increment_lap(self, tracker):
        """Test lap incrementing."""
        metrics = tracker.get_metrics()
        initial_lap = metrics.current_lap

        tracker.increment_lap()
        metrics = tracker.get_metrics()

        assert metrics.current_lap == initial_lap + 1

    def test_record_escape_maneuver(self, tracker):
        """Test recording escape maneuver."""
        tracker.record_escape_maneuver()
        metrics = tracker.get_metrics()

        assert metrics.escape_maneuvers == 1

        tracker.record_escape_maneuver()
        metrics = tracker.get_metrics()

        assert metrics.escape_maneuvers == 2

    def test_record_stuck_detection(self, tracker):
        """Test recording stuck robot detection."""
        tracker.record_stuck_detection()
        metrics = tracker.get_metrics()

        assert metrics.stuck_detections == 1

    def test_record_collision_warning(self, tracker):
        """Test recording collision warnings."""
        tracker.record_collision_warning()
        metrics = tracker.get_metrics()

        assert metrics.collision_warnings == 1

    def test_multiple_incidents(self, tracker):
        """Test recording multiple types of incidents."""
        tracker.record_escape_maneuver()
        tracker.record_stuck_detection()
        tracker.record_collision_warning()
        tracker.record_escape_maneuver()

        metrics = tracker.get_metrics()

        assert metrics.escape_maneuvers == 2
        assert metrics.stuck_detections == 1
        assert metrics.collision_warnings == 1

    def test_elapsed_time_increases(self, tracker):
        """Test that elapsed time increases over time."""
        import time

        metrics1 = tracker.get_metrics()
        time.sleep(0.01)  # Small delay
        metrics2 = tracker.get_metrics()

        assert metrics2.elapsed_time >= metrics1.elapsed_time

    def test_metrics_are_fresh(self, tracker):
        """Test that get_metrics returns current state."""
        tracker.record_escape_maneuver()
        metrics1 = tracker.get_metrics()

        tracker.record_escape_maneuver()
        metrics2 = tracker.get_metrics()

        assert metrics2.escape_maneuvers > metrics1.escape_maneuvers

    def test_tracker_persistent_state(self, tracker):
        """Test that tracker maintains state across multiple calls."""
        tracker.increment_lap()
        tracker.record_escape_maneuver()

        tracker.increment_lap()
        tracker.record_escape_maneuver()

        metrics = tracker.get_metrics()
        assert metrics.escape_maneuvers == 2

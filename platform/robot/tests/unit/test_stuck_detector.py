"""Unit tests for StuckDetector configuration guards and detection logic."""

from __future__ import annotations

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Waypoint

from src.navigation.control.controllers.stuck_detector import StuckDetector


@pytest.fixture()
def tuning():
    return NavigationTuning.load_default()


def test_history_size_smaller_than_timeout_frames_rejected(tuning):
    with pytest.raises(ValueError, match="history_size"):
        StuckDetector(timeout_frames=40, history_size=10, tuning=tuning)


def test_history_size_equal_to_timeout_frames_is_allowed(tuning):
    StuckDetector(timeout_frames=40, history_size=40, tuning=tuning)


def test_declares_stuck_after_timeout_without_movement(tuning):
    detector = StuckDetector(move_threshold=0.03, timeout_frames=5, history_size=10, tuning=tuning)
    stuck = False
    for _ in range(20):
        stuck = detector.update(Waypoint(0.0, 0.0))
    assert stuck is True


def test_not_stuck_when_moving(tuning):
    detector = StuckDetector(move_threshold=0.03, timeout_frames=5, history_size=10, tuning=tuning)
    stuck = False
    for i in range(20):
        stuck = detector.update(Waypoint(0.1 * i, 0.0))
    assert stuck is False

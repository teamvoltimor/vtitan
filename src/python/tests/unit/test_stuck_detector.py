"""Unit tests for StuckDetector configuration guards and detection logic."""

from __future__ import annotations

import math

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Waypoint

from src.navigation.control.controllers.stuck_detector import StuckDetector


def test_history_size_smaller_than_timeout_frames_rejected(tuning):
    with pytest.raises(ValueError, match="history_size"):
        StuckDetector(move_threshold=0.03, timeout_frames=40, history_size=10, confirmation_checks=3, tuning=tuning)


def test_history_size_equal_to_timeout_frames_is_allowed(tuning):
    StuckDetector(move_threshold=0.03, timeout_frames=40, history_size=40, confirmation_checks=3, tuning=tuning)


def test_declares_stuck_after_timeout_without_movement(tuning):
    detector = StuckDetector(move_threshold=0.03, timeout_frames=5, history_size=10, confirmation_checks=3, tuning=tuning)
    stuck = False
    for _ in range(20):
        stuck = detector.update(Waypoint(0.0, 0.0))
    assert stuck is True


def test_not_stuck_when_moving(tuning):
    detector = StuckDetector(move_threshold=0.03, timeout_frames=5, history_size=10, confirmation_checks=3, tuning=tuning)
    stuck = False
    for i in range(20):
        stuck = detector.update(Waypoint(0.1 * i, 0.0))
    assert stuck is False


def test_pendulum_with_no_net_progress_reads_as_stuck(tuning):
    """A robot swinging in place is stuck, however wide each swing is.

    Measured on ``run_20260913_191930``: 77.2 s held inside a 0.58 x 0.59 m box,
    11.079 m of ABSOLUTE wheel travel for 0.480 m of net pose displacement, with
    0.614 m of forward clearance and no sign committed -- and ``is_stuck`` read
    false on all 1543 ticks of it. The endpoint-distance test cannot see this by
    construction: the swing amplitude dwarfs ``move_threshold``, so the two ends
    of the window are nearly always far apart, ``stuck_count`` resets every tick
    and the latch never arms.
    """
    detector = StuckDetector(
        move_threshold=0.03, timeout_frames=40, history_size=40, confirmation_checks=3, tuning=tuning
    )
    ticks = 1544  # 77 s at 20 Hz
    stuck = False
    for i in range(ticks):
        # +-0.29 m swing on a 4 s period, carrying the measured 0.48 m of net drift.
        x = 0.29 * math.sin(2.0 * math.pi * i / 80.0) + 0.48 * i / ticks
        stuck = detector.update(Waypoint(x, 0.0))
        if stuck:
            break
    assert stuck is True


def test_rounding_a_corner_is_not_stuck(tuning):
    """The control: real cornering also has net < path, and must stay unstuck.

    Guards the progress-ratio test against firing on the very manoeuvre the
    robot is supposed to be making. A quarter turn of a 0.5 m radius at 0.3 m/s
    covers 0.79 m of path for 0.71 m of displacement.
    """
    detector = StuckDetector(
        move_threshold=0.03, timeout_frames=40, history_size=40, confirmation_checks=3, tuning=tuning
    )
    stuck = False
    for i in range(600):  # 30 s at 20 Hz, driving continuously around a 0.5 m circle
        angle = 0.3 / 0.5 * (i / 20.0)  # v/r * t
        stuck = detector.update(Waypoint(0.5 * math.cos(angle), 0.5 * math.sin(angle)))
        if stuck:
            break
    assert stuck is False


def test_driving_a_real_lap_is_not_stuck(tuning):
    """The control that matters: a lap of the 3x3 m track must never read stuck.

    The circle above is the tightest arc the chassis can hold; this is the shape
    it actually drives. If the no-progress window fires here it would abort a
    healthy run, which is worse than the wedge it exists to catch.
    """
    detector = StuckDetector(
        move_threshold=0.03, timeout_frames=40, history_size=40, confirmation_checks=3, tuning=tuning
    )
    # 0.30 m/s at 20 Hz = 0.015 m per tick, anticlockwise around the 1.0 m lane.
    corners = [(0.5, 0.5), (2.5, 0.5), (2.5, 2.5), (0.5, 2.5)]
    stuck = False
    pos = corners[0]
    for _lap in range(2):
        for leg in range(4):
            ax, ay = corners[leg]
            bx, by = corners[(leg + 1) % 4]
            steps = int(math.hypot(bx - ax, by - ay) / 0.015)
            for s in range(steps):
                f = s / steps
                pos = (ax + (bx - ax) * f, ay + (by - ay) * f)
                stuck = detector.update(Waypoint(pos[0], pos[1]))
                if stuck:
                    break
            if stuck:
                break
        if stuck:
            break
    assert stuck is False, f"read as stuck at {pos}"


def test_progress_window_of_one_frame_rejected(tuning):
    """maxlen would be 0 and the running-sum eviction indexes [0]."""
    with pytest.raises(ValueError, match="progress_window_frames"):
        StuckDetector(
            move_threshold=0.03,
            timeout_frames=5,
            history_size=10,
            confirmation_checks=3,
            tuning=tuning,
            progress_window_frames=1,
        )


def test_reset_clears_the_progress_window(tuning):
    """An escape resets the detector; the long window must not survive it.

    Without this the window stays full across the manoeuvre and re-declares
    no-progress on the very next tick, giving the escape no room to work.
    """
    detector = StuckDetector(
        move_threshold=0.03,
        timeout_frames=40,
        history_size=40,
        confirmation_checks=3,
        tuning=tuning,
        progress_window_frames=50,
    )
    for i in range(200):
        detector.update(Waypoint(0.29 * math.sin(2.0 * math.pi * i / 20.0), 0.0))
    assert detector.is_stuck is True
    detector.reset()
    assert detector.is_stuck is False
    assert detector.update(Waypoint(0.0, 0.0)) is False

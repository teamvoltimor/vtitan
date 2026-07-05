"""Unit tests for LapDetector (CR-03 — geometric lap detection).

Covers:
- Forward crossing counts a lap when waypoint also wrapped.
- Backward crossing (overshoot + return) does NOT double-count.
- Stuck-loop oscillating near the line does NOT double-count.
- Waypoint wrap alone (no geometric crossing) does NOT count.
- Geometric crossing alone (waypoint not wrapped) does NOT count.
- All four corridor sections × both directions work correctly.
- 3-lap simulation: exactly 3 laps counted for canonical trajectories.
"""

from __future__ import annotations

import math

import pytest
from shared.config.enums import Direction, Section

from src.navigation.race_tracker import LapDetector

# Helpers


def _make(
    section: Section = Section.SOUTH,
    direction: Direction = Direction.CLOCKWISE,
    start: tuple[float, float] = (1.5, 0.2),
) -> LapDetector:
    return LapDetector(start_pos=start, start_section=section, direction=direction)


def _feed(
    det: LapDetector,
    positions: list[tuple[float, float]],
    section: Section = Section.SOUTH,
    wrap_at: set[int] | None = None,
) -> int:
    """Feed positions to detector; wrap_at indices trigger notify_waypoint_wrapped.

    Returns the total lap count.
    """
    laps = 0
    for i, pos in enumerate(positions):
        if wrap_at and i in wrap_at:
            det.notify_waypoint_wrapped()
        if det.update(pos, section):
            laps += 1
    return laps


# Basic correctness


class TestBasicCounting:
    def test_forward_crossing_with_wrap_counts_lap(self):
        det = _make()
        det.notify_waypoint_wrapped()
        # Approach from x > 1.5 (dot < 0 for CW SOUTH normal = (-1,0))
        # dot = (rx - 1.5) * (-1): x=2.0 → dot=-0.5 (behind), x=1.2 → dot=0.3 (ahead)
        laps = _feed(det, [(2.0, 0.2), (1.2, 0.2)], section=Section.SOUTH)
        assert laps == 1

    def test_crossing_without_wrap_does_not_count(self):
        det = _make()
        # No notify_waypoint_wrapped() called
        laps = _feed(det, [(2.0, 0.2), (1.2, 0.2)], section=Section.SOUTH)
        assert laps == 0

    def test_wrap_without_crossing_does_not_count(self):
        det = _make()
        det.notify_waypoint_wrapped()
        # Robot stays on one side of the line
        laps = _feed(det, [(2.0, 0.2), (2.5, 0.2), (3.0, 0.2)], section=Section.SOUTH)
        assert laps == 0


# Overshoot / double-count prevention


class TestNoDoubleCounting:
    def test_overshoot_and_return_does_not_double_count(self):
        """Robot crosses line, overshoots a bit, comes back — should NOT count again."""
        det = _make()
        det.notify_waypoint_wrapped()
        positions = [
            (2.0, 0.2),  # behind (dot < 0)
            (1.2, 0.2),  # ahead  (dot > 0) → lap counted
            (0.8, 0.2),  # further ahead
            (1.6, 0.2),  # came back (behind again)
            (1.2, 0.2),  # crosses again — but waypoint_pending is False now
        ]
        laps = _feed(det, positions, section=Section.SOUTH)
        assert laps == 1

    def test_backward_crossing_not_counted(self):
        """Robot moving backward through the line (positive → negative) should not count."""
        det = _make()
        det.notify_waypoint_wrapped()
        positions = [
            (1.2, 0.2),  # start ahead (dot > 0)
            (2.0, 0.2),  # moved behind (backward) — dot goes negative
        ]
        laps = _feed(det, positions, section=Section.SOUTH)
        assert laps == 0

    def test_stuck_loop_at_finish_line_does_not_double_count(self):
        """Robot oscillating exactly at the finish line should count at most once."""
        det = _make()
        det.notify_waypoint_wrapped()
        positions = []
        # Oscillate: x alternates 1.6, 1.4, 1.6, 1.4 ... (crossing each time)
        for _ in range(10):
            positions.extend([(2.0, 0.2), (1.2, 0.2)])
        laps = _feed(det, positions, section=Section.SOUTH)
        assert laps == 1

    def test_two_wraps_two_crossings_gives_two_laps(self):
        """Second lap: after first lap, both pending + crossing must fire again."""
        det = _make()
        laps = 0

        # Lap 1
        det.notify_waypoint_wrapped()
        # Robot goes behind and then crosses
        for pos, sec in [((2.0, 0.2), Section.SOUTH), ((1.2, 0.2), Section.SOUTH)]:
            if det.update(pos, sec):
                laps += 1
        assert laps == 1

        # Lap 2: reset cycle
        det.notify_waypoint_wrapped()
        # Must first go to dot < 0 side again before crossing counts
        for pos, sec in [
            ((2.0, 0.2), Section.SOUTH),  # behind
            ((1.2, 0.2), Section.SOUTH),  # cross
        ]:
            if det.update(pos, sec):
                laps += 1
        assert laps == 2


# Wrong section guard


class TestSectionGuard:
    def test_crossing_in_wrong_section_not_counted(self):
        """Geometric crossing in a different corridor than start should be ignored."""
        det = _make(section=Section.SOUTH)  # start section = SOUTH
        det.notify_waypoint_wrapped()
        positions = [(2.0, 0.2), (1.2, 0.2)]
        # Feed with wrong section
        laps = _feed(det, positions, section=Section.NORTH)
        assert laps == 0

    def test_crossing_in_correct_section_counted(self):
        det = _make(section=Section.SOUTH)
        det.notify_waypoint_wrapped()
        positions = [(2.0, 0.2), (1.2, 0.2)]
        laps = _feed(det, positions, section=Section.SOUTH)
        assert laps == 1


# All four sections × both directions


@pytest.mark.parametrize(
    "section,direction,start,before,after",
    [
        # CW — normal is travel direction
        (Section.SOUTH, Direction.CLOCKWISE, (1.5, 0.2), (2.0, 0.2), (1.0, 0.2)),
        (Section.NORTH, Direction.CLOCKWISE, (1.5, 2.8), (1.0, 2.8), (2.0, 2.8)),
        (Section.EAST, Direction.CLOCKWISE, (2.8, 1.5), (2.8, 2.0), (2.8, 1.0)),
        (Section.WEST, Direction.CLOCKWISE, (0.2, 1.5), (0.2, 1.0), (0.2, 2.0)),
        # CCW — normal reversed
        (Section.SOUTH, Direction.COUNTERCLOCKWISE, (1.5, 0.2), (1.0, 0.2), (2.0, 0.2)),
        (Section.NORTH, Direction.COUNTERCLOCKWISE, (1.5, 2.8), (2.0, 2.8), (1.0, 2.8)),
        (Section.EAST, Direction.COUNTERCLOCKWISE, (2.8, 1.5), (2.8, 1.0), (2.8, 2.0)),
        (Section.WEST, Direction.COUNTERCLOCKWISE, (0.2, 1.5), (0.2, 2.0), (0.2, 1.0)),
    ],
    ids=[
        "south_cw",
        "north_cw",
        "east_cw",
        "west_cw",
        "south_ccw",
        "north_ccw",
        "east_ccw",
        "west_ccw",
    ],
)
def test_all_sections_and_directions(section, direction, start, before, after):
    """Forward crossing counts a lap for every section × direction combination."""
    det = LapDetector(start_pos=start, start_section=section, direction=direction)
    det.notify_waypoint_wrapped()
    laps = _feed(det, [before, after], section=section)
    assert laps == 1


# 3-lap simulation


def _simulate_laps(n_laps: int, det: LapDetector, section: Section) -> int:
    """Simulate n_laps passes across the finish line. Returns laps counted."""
    counted = 0
    for _ in range(n_laps):
        det.notify_waypoint_wrapped()
        det.update((2.0, 0.2), section)  # behind
        if det.update((1.2, 0.2), section):  # cross
            counted += 1
    return counted


def test_three_lap_simulation_counts_exactly_three():
    det = _make()
    laps = _simulate_laps(3, det, Section.SOUTH)
    assert laps == 3


def test_overshoot_on_every_lap_still_counts_exactly_three():
    """Even if robot overshoots slightly on each lap, exactly 3 laps are counted."""
    det = _make()
    counted = 0
    for _ in range(3):
        det.notify_waypoint_wrapped()
        # behind → cross → overshoot → come back (backward cross, should not double count)
        steps = [
            (2.0, 0.2),  # behind
            (1.2, 0.2),  # cross → count
            (0.9, 0.2),  # overshoot
            (1.6, 0.2),  # return backward through line (should not count)
        ]
        for pos in steps:
            if det.update(pos, Section.SOUTH):
                counted += 1
    assert counted == 3

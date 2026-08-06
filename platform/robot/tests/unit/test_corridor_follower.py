"""Blind corridor following, and the corner turn it must not commit to early.

The behaviour pinned here is what run_20260806_162008 lost: a 305 s round in
which the corner branch held hard-over steering for 53% of the ticks, so the
chassis never came square to a corridor, never inferred its travel direction,
never planned a path, and scored zero laps.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import RobotSpecs

from src.navigation.corridor_follower import (
    _MAX_CENTERING_STEER,
    TURN_CLEARANCE_M,
    follow_corridor,
)

CREEP_SPEED = 0.15
BEAMS = 720
JUST_INSIDE_TURN_M = TURN_CLEARANCE_M - 0.1


def _angles() -> list[float]:
    return [-math.pi + i * 2 * math.pi / BEAMS for i in range(BEAMS)]


def _scan(bearing_to_range) -> tuple[list[float], list[float]]:
    """Build a scan from a function of robot-frame bearing."""
    angles = _angles()
    return [bearing_to_range(a) for a in angles], angles


def _corridor(left_m: float, right_m: float, ahead_m: float) -> tuple[list[float], list[float]]:
    """A square corridor: walls either side, and the way ahead open to ``ahead_m``."""

    def rng(a: float) -> float:
        if abs(a) < math.pi / 4:
            return ahead_m / max(math.cos(a), 1e-3)
        if a > 0:
            return left_m / max(math.sin(a), 1e-3)
        return right_m / max(math.sin(-a), 1e-3)

    return _scan(rng)


class TestCornerTurn:
    def test_wall_spanning_the_track_commits_to_the_turn(self) -> None:
        """A corridor that has genuinely ended must still turn, hard over."""
        ranges, angles = _corridor(left_m=0.5, right_m=0.5, ahead_m=JUST_INSIDE_TURN_M)
        cmd = follow_corridor(ranges, angles, CREEP_SPEED)
        assert abs(cmd.steering_norm) == pytest.approx(_MAX_CENTERING_STEER)
        assert cmd.speed_mps > 0.0

    def test_turns_toward_the_side_with_more_room(self) -> None:
        """The open side is where the track continues; +1 is full left."""
        ranges, angles = _corridor(left_m=0.9, right_m=0.3, ahead_m=JUST_INSIDE_TURN_M)
        assert follow_corridor(ranges, angles, CREEP_SPEED).steering_norm > 0

        ranges, angles = _corridor(left_m=0.3, right_m=0.9, ahead_m=JUST_INSIDE_TURN_M)
        assert follow_corridor(ranges, angles, CREEP_SPEED).steering_norm < 0

    def test_oblique_chassis_in_an_open_corridor_does_not_turn(self) -> None:
        """The regression that cost run_20260806_162008 its round.

        A chassis 0.24 m off the left wall at 30 deg puts the +/-8 deg forward
        cone on that wall at 0.24/sin(30) = 0.48 m -- under TURN_CLEARANCE_M --
        while the corridor's own axis, 30 deg off the nose, is wide open. The
        old test saw only the cone and committed to a corner turn mid-corridor.
        """
        oblique = math.radians(30)

        def rng(a: float) -> float:
            # Corridor axis sits at +30 deg in the robot frame and runs 3 m.
            if abs(a - oblique) < math.radians(20):
                return 3.0
            if abs(a) < math.pi / 3:
                return 0.24 / max(math.sin(oblique), 1e-3)
            return 0.9

        ranges, angles = _scan(rng)
        cmd = follow_corridor(ranges, angles, CREEP_SPEED)
        assert abs(cmd.steering_norm) < _MAX_CENTERING_STEER, (
            "committed to a hard-over corner turn in an open corridor"
        )
        assert cmd.speed_mps == pytest.approx(CREEP_SPEED), "slowed to corner speed mid-corridor"

    def test_a_single_dropped_beam_cannot_veto_a_real_corner(self) -> None:
        """The gateway substitutes max range for a no-return; 23-26% of beams
        were max range in both 2026-08-06 bags. Counted as open track, one such
        beam in the arc would refuse every corner turn of the round."""
        ranges, angles = _corridor(left_m=0.5, right_m=0.5, ahead_m=JUST_INSIDE_TURN_M)
        ranges[BEAMS // 2] = RobotSpecs.LIDAR_MAX_RANGE
        cmd = follow_corridor(ranges, angles, CREEP_SPEED)
        assert abs(cmd.steering_norm) == pytest.approx(_MAX_CENTERING_STEER)


class TestSafety:
    def test_backs_off_when_boxed_in_even_though_the_way_ahead_is_open(self) -> None:
        """The emergency back-off is a fact about room, not about layout.

        It used to sit inside the corner branch and so depended on that branch
        firing for every close wall. The corner branch no longer does.
        """
        close = RobotSpecs.LENGTH - 0.05

        def rng(a: float) -> float:
            if abs(a) < math.radians(10):
                return close
            if abs(a - math.radians(30)) < math.radians(15):
                return 3.0  # open corridor off to one side
            if abs(_wrap_pi(a - math.pi)) < math.radians(20):
                return 2.0  # clear behind
            return 0.9

        ranges, angles = _scan(rng)
        cmd = follow_corridor(ranges, angles, CREEP_SPEED)
        assert cmd.speed_mps < 0.0, "did not back off from a wall inside chassis length"

    def test_holds_still_when_boxed_at_both_ends(self) -> None:
        ranges, angles = _scan(lambda _: RobotSpecs.LENGTH - 0.05)
        cmd = follow_corridor(ranges, angles, CREEP_SPEED)
        assert cmd.speed_mps == pytest.approx(0.0)


class TestCentring:
    def test_steers_toward_the_roomier_side(self) -> None:
        ranges, angles = _corridor(left_m=0.8, right_m=0.2, ahead_m=2.5)
        assert follow_corridor(ranges, angles, CREEP_SPEED).steering_norm > 0

    def test_centred_chassis_drives_straight(self) -> None:
        ranges, angles = _corridor(left_m=0.5, right_m=0.5, ahead_m=2.5)
        cmd = follow_corridor(ranges, angles, CREEP_SPEED)
        assert cmd.steering_norm == pytest.approx(0.0, abs=1e-6)
        assert cmd.speed_mps == pytest.approx(CREEP_SPEED)


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

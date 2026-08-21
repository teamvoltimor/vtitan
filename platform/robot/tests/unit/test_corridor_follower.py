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
from shared.config.navigation_tuning import NavigationTuning

from src.navigation.corridor_follower import TurnSide, follow_corridor
from tests.fixtures import LidarScanBuilder
from tests.test_constants import CREEP_SPEED_MPS, TURN_ENTRY_MARGIN_M


@pytest.fixture()
def tuning():
    return NavigationTuning.load_default()


def _just_inside_turn_m(tuning: NavigationTuning) -> float:
    return tuning.corridor_follower.TURN_CLEARANCE_M - TURN_ENTRY_MARGIN_M


def _max_centering_norm(tuning: NavigationTuning) -> float:
    """The centring cap as the normalised command the actuator receives.

    The cap is configured as a physical road-wheel angle, so the normalised
    value it produces depends on the servo's reach -- which is the entire
    point of expressing it that way. Asserting against the converted value
    keeps these tests true on any steering geometry, where comparing to a bare
    0.25 only held on the 55 degree chassis.
    """
    return math.radians(tuning.corridor_follower.MAX_CENTERING_STEER_DEG) / RobotSpecs.MAX_STEERING_ANGLE


class TestCornerTurn:
    def test_wall_spanning_the_track_commits_to_the_turn(self, tuning) -> None:
        """A corridor that has genuinely ended must still turn, hard over."""
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=_just_inside_turn_m(tuning)).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert abs(cmd.steering_norm) == pytest.approx(_max_centering_norm(tuning))
        assert cmd.speed_mps > 0.0

    def test_turns_toward_the_side_with_more_room(self, tuning) -> None:
        """The open side is where the track continues; +1 is full left."""
        scan = LidarScanBuilder().corridor(left_m=0.9, right_m=0.3, ahead_m=_just_inside_turn_m(tuning)).build()
        assert follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning).steering_norm > 0

        scan = LidarScanBuilder().corridor(left_m=0.3, right_m=0.9, ahead_m=_just_inside_turn_m(tuning)).build()
        assert follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning).steering_norm < 0

    def test_oblique_chassis_in_an_open_corridor_does_not_turn(self, tuning) -> None:
        """The regression that cost run_20260806_162008 its round.

        A chassis 0.24 m off the left wall at 30 deg puts the +/-8 deg forward
        cone on that wall at 0.24/sin(30) = 0.48 m -- under TURN_CLEARANCE_M --
        while the corridor's own axis, 30 deg off the nose, is wide open. The
        old test saw only the cone and committed to a corner turn mid-corridor.
        """
        oblique = math.radians(30)
        scan = (
            LidarScanBuilder()
            .oblique_corridor(
                corridor_bearing_rad=oblique,
                corridor_distance_m=3.0,
                offset_from_wall_m=0.24,
            )
            .build()
        )
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert abs(cmd.steering_norm) < _max_centering_norm(tuning), (
            "committed to a hard-over corner turn in an open corridor"
        )
        assert cmd.speed_mps == pytest.approx(CREEP_SPEED_MPS), "slowed to corner speed mid-corridor"

    def test_a_single_dropped_beam_cannot_veto_a_real_corner(self, tuning) -> None:
        """The gateway substitutes max range for a no-return; 23-26% of beams
        were max range in both 2026-08-06 bags. Counted as open track, one such
        beam in the arc would refuse every corner turn of the round."""
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=_just_inside_turn_m(tuning)).build()
        ranges_with_dropout = list(scan.ranges)
        ranges_with_dropout[len(ranges_with_dropout) // 2] = RobotSpecs.LIDAR_MAX_RANGE
        cmd = follow_corridor(ranges_with_dropout, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert abs(cmd.steering_norm) == pytest.approx(_max_centering_norm(tuning))


class TestSafety:
    def test_backs_off_when_boxed_in_even_though_the_way_ahead_is_open(self, tuning) -> None:
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

        angles = _bearings()
        cmd = follow_corridor([rng(a) for a in angles], angles, CREEP_SPEED_MPS, tuning=tuning)
        assert cmd.speed_mps < 0.0, "did not back off from a wall inside chassis length"

    def test_holds_still_when_boxed_at_both_ends(self, tuning) -> None:
        angles = _bearings()
        boxed = [RobotSpecs.LENGTH - 0.05] * len(angles)
        cmd = follow_corridor(boxed, angles, CREEP_SPEED_MPS, tuning=tuning)
        assert cmd.speed_mps == pytest.approx(0.0)


class TestForcedTurnSide:
    """A sign's WRO pass-side rule must be able to override the plain
    clearance heuristic in both branches that pick a turn side -- see
    [[sign_router_blind_creep_gap_2026_08_13]]. Without ``forced_turn_side``
    both branches turn toward whichever side has more LIDAR room, which is
    the generic-obstacle behaviour a sign must not get.
    """

    def test_corner_branch_honours_the_forced_side_over_clearance(self, tuning) -> None:
        # More room on the right (0.9 m) than the left (0.3 m): unforced, the
        # corner branch would turn right (negative steering).
        scan = LidarScanBuilder().corridor(left_m=0.3, right_m=0.9, ahead_m=_just_inside_turn_m(tuning)).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning, forced_turn_side=TurnSide.LEFT)
        assert cmd.steering_norm > 0

    def test_back_off_branch_honours_the_forced_side_over_clearance(self, tuning) -> None:
        # More room on the right (0.9 m) than the left (0.3 m), close enough
        # ahead to trigger the reversing back-off branch. Unforced, and
        # forcing "right" (matching clearance), both back off with the same
        # (mirrored, since reversing) steering sign; forcing "left" against
        # clearance must flip it.
        close = RobotSpecs.LENGTH - 0.05
        scan = LidarScanBuilder().corridor(left_m=0.3, right_m=0.9, ahead_m=close).build()

        unforced = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert unforced.speed_mps < 0.0
        matching = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning, forced_turn_side=TurnSide.RIGHT)
        assert matching.steering_norm == pytest.approx(unforced.steering_norm)

        forced = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning, forced_turn_side=TurnSide.LEFT)
        assert forced.speed_mps < 0.0, "forcing the side must not disable the back-off itself"
        assert forced.steering_norm == pytest.approx(-unforced.steering_norm)


class TestCentring:
    def test_steers_toward_the_roomier_side(self, tuning) -> None:
        scan = LidarScanBuilder().corridor(left_m=0.8, right_m=0.2, ahead_m=2.5).build()
        assert follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning).steering_norm > 0

    def test_centred_chassis_drives_straight(self, tuning) -> None:
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=2.5).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert cmd.steering_norm == pytest.approx(0.0, abs=1e-6)
        assert cmd.speed_mps == pytest.approx(CREEP_SPEED_MPS)


class TestHeadingDamping:
    """The term that stops centring oscillating -- see corridor_follower._HEADING_GAIN.

    Centring on lateral offset alone is proportional control on position, and in
    a steered chassis position and heading are 90 degrees out of phase, so it can
    only overshoot and come back. Measured on hardware 2026-08-07 as a 3.2 s limit
    cycle with the heading 30 degrees off axis at the median, which starves the
    direction gate: it needs the chassis square to a corridor at the moment one
    side opens, and on run_141814 those coincided on 0 of 1763 scans.
    """

    def test_centred_but_oblique_chassis_steers_back_to_the_axis(self, tuning) -> None:
        """The case offset-only centring cannot see: dead centre, pointing wrong.

        Lateral offset is zero here, so the undamped law commands exactly zero
        and the chassis keeps drifting off axis. Yaw is a quarter of the way to
        the next axis multiple, well outside the direction gate's tolerance.
        """
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=2.5).build()
        oblique = math.radians(20.0)

        undamped = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert undamped.steering_norm == pytest.approx(0.0, abs=1e-6)

        damped = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, oblique, tuning=tuning)
        assert damped.steering_norm < 0.0, "nose left of the axis must steer right"

    def test_the_correction_is_signed_by_which_way_the_nose_points(self, tuning) -> None:
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=2.5).build()
        left_of_axis = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, math.radians(20.0), tuning=tuning)
        right_of_axis = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, math.radians(-20.0), tuning=tuning)
        assert left_of_axis.steering_norm == pytest.approx(-right_of_axis.steering_norm)

    def test_square_to_any_of_the_four_axes_needs_no_correction(self, tuning) -> None:
        """The track is a Manhattan world, so every 90 degrees is "square"."""
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=2.5).build()
        for quarter in range(4):
            cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, quarter * math.pi / 2, tuning=tuning)
            assert cmd.steering_norm == pytest.approx(0.0, abs=1e-6), f"axis {quarter} asked for steering"

    def test_the_cap_still_binds_with_both_terms(self, tuning) -> None:
        """Damping adds to the demand; it must not widen the steering envelope."""
        scan = LidarScanBuilder().corridor(left_m=0.9, right_m=0.1, ahead_m=2.5).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, math.radians(-40.0), tuning=tuning)
        assert abs(cmd.steering_norm) <= _max_centering_norm(tuning) + 1e-9


def _bearings(num_rays: int = 360) -> list[float]:
    """Evenly spaced robot-frame bearings over the full sweep, 0 = forward.

    The two safety cases describe their scans as a function of bearing rather
    than as a corridor, so they need the raw angle grid LidarScanBuilder builds
    internally. They previously called a module-level ``_scan`` helper that no
    longer exists -- the tests had been raising NameError, unnoticed, since it
    was removed.
    """
    return [-math.pi + 2.0 * math.pi * i / num_rays for i in range(num_rays)]


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

"""Blind corridor following, and the corner turn it must not commit to early.

The behaviour pinned here is what a lost hardware round cost: the corner branch
held hard-over steering for most of the ticks, so the chassis never came square
to a corridor, never inferred its travel direction, never planned a path, and
scored zero laps. See adr:0057-blind-corridor-follower-and-width.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import pytest
from shared.config.constants import RobotSpecs

from src.navigation.corridor_follower import TurnSide, follow_corridor
from tests.fixtures import LidarScanBuilder
from tests.test_constants import CREEP_SPEED_MPS, TURN_ENTRY_MARGIN_M

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning


@pytest.fixture()
def rear_visible(tuning, override_tuning):
    """Tuning with the mount's historical ~25 deg rear slot restored.

    The shipped occlusion wedges meet at 180 deg -- the build lost the slot that
    was its only rear vision -- so nothing behind can be measured and every
    sensed-clearance reverse is refused. These are the earlier, slotted bounds,
    and the reversing branches cannot be exercised without them. Restoring a rear
    sensor on the real chassis is this same edit in the config, which is why the
    branches are kept rather than deleted. See
    adr:0056-raw-and-masked-scan.
    """
    return override_tuning(
        tuning,
        lidar_sectors={"blind_wedge_left_min_deg": -160.0, "blind_wedge_right_max_deg": 175.0},
    )


def _just_inside_turn_m(tuning: NavigationTuning) -> float:
    return tuning.corridor_follower.turn_clearance_m - TURN_ENTRY_MARGIN_M


def _max_centering_norm(tuning: NavigationTuning) -> float:
    """The centring cap as the normalised command the actuator receives.

    The cap is configured as a physical road-wheel angle, so the normalised
    value it produces depends on the servo's reach -- which is the entire
    point of expressing it that way. Asserting against the converted value
    keeps these tests true on any steering geometry, where comparing to a bare
    0.25 only held on the 55 degree chassis.
    """
    return math.radians(tuning.corridor_follower.max_centering_steer_deg) / RobotSpecs.MAX_STEERING_ANGLE


def _max_corner_norm(tuning: NavigationTuning) -> float:
    """The angle the corner and back-off branches steer AT, normalised.

    A different constant from the centring cap: that one is sized by the
    measured limit cycle, this one by the turn arc having to fit inside
    TURN_CLEARANCE_M. Converted for the same reason as its sibling above, so
    these stay true on any steering geometry. See
    adr:0057-blind-corridor-follower-and-width.
    """
    return math.radians(tuning.corridor_follower.max_corner_steer_deg) / RobotSpecs.MAX_STEERING_ANGLE


class TestCornerTurn:
    def test_wall_spanning_the_track_commits_to_the_turn(self, tuning) -> None:
        """A corridor that has genuinely ended must still turn, hard over."""
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=_just_inside_turn_m(tuning)).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert abs(cmd.steering_norm) == pytest.approx(_max_corner_norm(tuning))
        assert cmd.speed_mps > 0.0

    def test_turns_toward_the_side_with_more_room(self, tuning) -> None:
        """The open side is where the track continues; +1 is full left."""
        scan = LidarScanBuilder().corridor(left_m=0.9, right_m=0.3, ahead_m=_just_inside_turn_m(tuning)).build()
        assert follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning).steering_norm > 0

        scan = LidarScanBuilder().corridor(left_m=0.3, right_m=0.9, ahead_m=_just_inside_turn_m(tuning)).build()
        assert follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning).steering_norm < 0

    def test_oblique_chassis_in_an_open_corridor_does_not_turn(self, tuning) -> None:
        """The regression that cost a hardware round.

        A chassis offset from the left wall at an angle puts the +/-8 deg
        forward cone on that wall inside TURN_CLEARANCE_M while the corridor's
        own axis, off the nose, is wide open. The old test saw only the cone and
        committed to a corner turn mid-corridor. See
        adr:0057-blind-corridor-follower-and-width.
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
        """The gateway substitutes max range for a no-return, and a large
        fraction of beams were max range in the hardware bags. Counted as open
        track, one such beam in the arc would refuse every corner turn of the
        round. See adr:0056-raw-and-masked-scan."""
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=_just_inside_turn_m(tuning)).build()
        ranges_with_dropout = list(scan.ranges)
        ranges_with_dropout[len(ranges_with_dropout) // 2] = RobotSpecs.LIDAR_MAX_RANGE
        cmd = follow_corridor(ranges_with_dropout, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert abs(cmd.steering_norm) == pytest.approx(_max_corner_norm(tuning))


class TestSafety:
    def test_backs_off_when_boxed_in_even_though_the_way_ahead_is_open(self, rear_visible) -> None:
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
        cmd = follow_corridor([rng(a) for a in angles], angles, CREEP_SPEED_MPS, tuning=rear_visible)
        assert cmd.speed_mps < 0.0, "did not back off from a wall inside chassis length"

    def test_refuses_to_back_off_when_the_rear_cannot_be_measured(self, tuning) -> None:
        """Unreadable behind is not permission to reverse.

        The masked bearings say 2.0 m of clear road behind, and that is not a
        measurement -- the gateway substitutes max range for a no-return, and
        the old single-ray gate read it as open track. Refusing costs a back-off
        the robot might have got away with; accepting costs a reverse into
        whatever is actually there.

        The rear is made unreadable the way the mount actually makes it
        unreadable, rather than by assuming a particular wedge width: an
        occluded bearing returns a short self-detection distance off the mount's
        own structure, so the readable part of the sector is given a
        self-detection return and only the masked part gets the deceptive 2.0 m.
        Before that measurement this test set 2.0 m across the whole sector and
        passed only because the shipped wedges masked all of it; when the wedges
        narrowed it started authorising a reverse, which is the premise changing
        rather than the safety property. See
        adr:0056-raw-and-masked-scan.
        """
        close = RobotSpecs.LENGTH - 0.05
        sectors = tuning.lidar_sectors
        occluded = sectors.self_detection_threshold_m / 2.0

        def masked(deg: float) -> bool:
            return (
                sectors.blind_wedge_left_min_deg <= deg <= sectors.blind_wedge_left_max_deg
                or sectors.blind_wedge_right_min_deg <= deg <= sectors.blind_wedge_right_max_deg
            )

        def rng(a: float) -> float:
            if abs(a) < math.radians(10):
                return close
            if abs(a - math.radians(30)) < math.radians(15):
                return 3.0
            # The WHOLE rear sector, read from tuning rather than a hardcoded
            # arc: a ray just outside it still counts as rear evidence, and
            # covering only part of the sector leaves valid rays behind that
            # authorise the reverse this test exists to forbid.
            if abs(_wrap_pi(a - math.pi)) <= math.radians(sectors.threat_half_fov_deg):
                # Masked bearings carry the substituted max-range lie; the rest
                # of the sector is genuinely blocked by the mount.
                return 2.0 if masked(math.degrees(_wrap_pi(a))) else occluded
            return 0.9

        angles = _bearings()
        cmd = follow_corridor([rng(a) for a in angles], angles, CREEP_SPEED_MPS, tuning=tuning)
        # Not "holds still": a rear-blind robot with an open side pivots toward
        # it under lock rather than stopping, because on this mount the rear is
        # ALWAYS unreadable and stopping here is a permanent deadlock (a legal
        # in-bay start sat at 0.00 m across the scenarios). What this test
        # protects is unchanged and is the thing the docstring names -- an
        # unmeasurable rear must never authorise REVERSE. See
        # adr:0055-escape-maneuver-selection.
        assert cmd.speed_mps >= 0.0, "reversed into a rear it could not measure"
        assert cmd.steering_norm != pytest.approx(0.0), "still steers toward the open side"

    def test_holds_still_when_boxed_at_both_ends(self, rear_visible) -> None:
        # Rear slot restored on purpose: this is the DISTANCE branch (something
        # is measurably too close behind), distinct from the unmeasurable case
        # above, which reaches the same stop for a different reason.
        angles = _bearings()
        boxed = [RobotSpecs.LENGTH - 0.05] * len(angles)
        cmd = follow_corridor(boxed, angles, CREEP_SPEED_MPS, tuning=rear_visible)
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

    def test_back_off_branch_honours_the_forced_side_over_clearance(self, rear_visible) -> None:
        # More room on the right (0.9 m) than the left (0.3 m), close enough
        # ahead to trigger the reversing back-off branch. Unforced, and
        # forcing "right" (matching clearance), both back off with the same
        # (mirrored, since reversing) steering sign; forcing "left" against
        # clearance must flip it.
        close = RobotSpecs.LENGTH - 0.05
        scan = LidarScanBuilder().corridor(left_m=0.3, right_m=0.9, ahead_m=close).build()

        unforced = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=rear_visible)
        assert unforced.speed_mps < 0.0
        matching = follow_corridor(
            scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=rear_visible, forced_turn_side=TurnSide.RIGHT
        )
        assert matching.steering_norm == pytest.approx(unforced.steering_norm)

        forced = follow_corridor(
            scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=rear_visible, forced_turn_side=TurnSide.LEFT
        )
        assert forced.speed_mps < 0.0, "forcing the side must not disable the back-off itself"
        assert forced.steering_norm == pytest.approx(-unforced.steering_norm)


class TestCentring:
    """The centring branch, which SHIPS DISABLED (CENTERING_GAIN_DEG_PER_M = 0).

    Both halves are asserted deliberately: that the shipped creep holds its lane,
    and that the branch still works when a tuning re-enables it. A zeroed gain
    with no second test would let the code path rot unnoticed, which is the same
    inert-configuration trap that has already cost this project three wrong
    conclusions.
    """

    def test_shipped_creep_holds_its_lane_instead_of_centring(self, tuning) -> None:
        # Hard against one wall but square to the corridor. Chasing this offset
        # is what swung the heading past the direction estimator's alignment
        # gate, costing a share of wide outer-band starts their direction
        # entirely, so the shipped creep must not steer off-axis for it. See
        # adr:0057-blind-corridor-follower-and-width.
        scan = LidarScanBuilder().corridor(left_m=0.8, right_m=0.2, ahead_m=2.5).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert cmd.steering_norm == pytest.approx(0.0, abs=1e-6)
        assert cmd.speed_mps == pytest.approx(CREEP_SPEED_MPS)

    def test_steers_toward_the_roomier_side_when_the_gain_is_restored(self, tuning, override_tuning) -> None:
        # Dormant, not dead: 44.0 is the value shipped before the gain was
        # zeroed (see adr:0057-blind-corridor-follower-and-width).
        centring = override_tuning(tuning, corridor_follower={"centering_gain_deg_per_m": 44.0})
        scan = LidarScanBuilder().corridor(left_m=0.8, right_m=0.2, ahead_m=2.5).build()
        assert follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=centring).steering_norm > 0

    def test_centred_chassis_drives_straight(self, tuning) -> None:
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=2.5).build()
        cmd = follow_corridor(scan.ranges, scan.angles, CREEP_SPEED_MPS, tuning=tuning)
        assert cmd.steering_norm == pytest.approx(0.0, abs=1e-6)
        assert cmd.speed_mps == pytest.approx(CREEP_SPEED_MPS)


class TestHeadingDamping:
    """The term that stops centring oscillating -- see corridor_follower._HEADING_GAIN.

    Centring on lateral offset alone is proportional control on position, and in
    a steered chassis position and heading are 90 degrees out of phase, so it can
    only overshoot and come back. Measured on hardware as a limit cycle with the
    heading well off axis at the median, which starves the direction gate: it
    needs the chassis square to a corridor at the moment one side opens, and on
    a failing run those never coincided. See
    adr:0057-blind-corridor-follower-and-width.
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

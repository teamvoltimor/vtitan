"""The two sense guards: the search gate and the sign-deform guard.

Both answer the same question -- is this aim point approached along the path's
own direction of travel -- at two different places. See
``PurePursuitParams.TARGET_SENSE_GATE`` and
``SignRouterParams.SIGN_DEFORM_SENSE_GUARD``.

**The deform guard is INERT on the shipped tree** and its predicate is tested
here on its own terms only. ``SIGN_LANE_SUPPRESS_DEFORM`` ships true, so
``navigator.step`` never reassigns ``steer_target`` to the deformed point and
the guard's identity check short-circuits: it evaluated on 0 ticks of four
sighted scenarios with the flag forced on. These tests pin the geometry so the
deform cannot be re-enabled without it; they do not claim the guard does
anything today.

Each guard needs its OFF state pinned as well as its ON state. Both ship False,
so a test that only exercises the new behaviour would let a regression in the
default path through unnoticed.
"""

from __future__ import annotations

import math

from shared.domain.models import Waypoint

from src.navigation.control.controllers.waypoint_controller import (
    WaypointController,
    _agrees_with_path_sense,
)
from src.navigation.core_navigator.navigator import _bearing_agrees_with_path


def _controller(*, gate: bool) -> WaypointController:
    """A controller with only the fields ``select_target_point`` reads."""
    return WaypointController(
        max_steering_angle=0.5,
        lookahead_short=0.16,
        lookahead_long=0.32,
        lookahead_transition=0.10,
        steer_kp=1.2,
        max_steering_rate=1.2,
        waypoint_reached_distance_m=0.10,
        corner_turn_threshold_rad=0.30,
        target_search_span_m=1.0,
        target_sense_gate=gate,
    )


def _ring(n: int = 24, radius: float = 1.0) -> list[tuple[float, float]]:
    """A closed counterclockwise loop, so both tangent senses exist."""
    return [
        (radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


class TestAgreesWithPathSense:
    """The predicate itself, independent of the search that calls it."""

    def test_approaching_along_the_path_agrees(self) -> None:
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        # Robot at the origin, candidate one metre ahead: the path at that
        # candidate also points +x, so the approach agrees.
        assert _agrees_with_path_sense(1.0, 0.0, 1.0, path, 1)

    def test_approaching_against_the_path_disagrees(self) -> None:
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        # Candidate one metre BEHIND the robot while the path there still runs
        # +x: reaching it means travelling -x, against the path.
        assert not _agrees_with_path_sense(-1.0, 0.0, 1.0, path, 1)

    def test_perpendicular_approach_disagrees(self) -> None:
        # Exactly 90 deg projects to zero, and the comparison is strict, so a
        # perpendicular approach is not admitted. Pinned because the boundary
        # is a choice, not an accident.
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert not _agrees_with_path_sense(0.0, 1.0, 1.0, path, 1)

    def test_duplicated_waypoint_has_no_direction_and_cannot_disagree(self) -> None:
        # A path can legitimately repeat a point; the guard must not reject
        # everything when the outgoing direction is undefined.
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.0)]
        assert _agrees_with_path_sense(-1.0, 0.0, 1.0, path, 1)

    def test_zero_distance_cannot_disagree(self) -> None:
        path = [(0.0, 0.0), (1.0, 0.0)]
        assert _agrees_with_path_sense(0.0, 0.0, 0.0, path, 0)

    def test_index_wraps_past_the_end(self) -> None:
        # The search passes waypoint_index + offset unbounded, so the predicate
        # owns the modulo. Index 3 on a 3-point path is index 0.
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert _agrees_with_path_sense(1.0, 0.0, 1.0, path, 3)


class TestSearchGate:
    """``select_target_point`` with the gate off and on."""

    def test_off_is_the_shipped_path_and_the_two_agree_when_aligned(self) -> None:
        ring = _ring()
        # Sitting just inside waypoint 0, facing along the loop: nothing here
        # is wrong-sense, so the gate must not change the answer.
        pos = (ring[0][0] * 0.95, ring[0][1] * 0.95)
        yaw = math.atan2(ring[1][1] - ring[0][1], ring[1][0] - ring[0][0])
        off = _controller(gate=False).select_target_point(pos, yaw, ring, 0, 0.20)
        on = _controller(gate=True).select_target_point(pos, yaw, ring, 0, 0.20)
        assert off == on

    def test_a_wrong_sense_candidate_is_skipped_for_a_later_right_sense_one(self) -> None:
        # The mechanism, minimally: index 1's outgoing segment doubles back, so
        # a candidate reached by travelling +x runs AGAINST the path there,
        # while index 2's runs with it. Ungated, the search returns the first
        # thing merely in front of the chassis.
        path = [(0.0, 0.0), (1.0, 0.0), (0.5, 0.0), (2.0, 0.0), (3.0, 0.0)]
        pos, yaw = (0.0, 0.0), 0.0
        assert _controller(gate=False).select_target_point(pos, yaw, path, 1, 0.20) == (1.0, 0.0)
        assert _controller(gate=True).select_target_point(pos, yaw, path, 1, 0.20) == (0.5, 0.0)

    def test_a_fully_rotated_chassis_is_not_rescued_by_the_gate(self) -> None:
        # THE GATE'S LIMIT, pinned deliberately rather than left to be
        # discovered on track. Once the chassis is turned right round, EVERY
        # candidate in the span is wrong-sense, so the filter empties the scan
        # and the existing fallback tiers hand back the same point the ungated
        # search would have returned.
        #
        # This is not a defect in the gate, it is the reason the gate is
        # described as PREVENTION. Recovery needs something that reorients the
        # chassis, and no manoeuvre in this tree latches on heading at all --
        # every latch keys on forward-lane LIDAR or on 3 cm of odometry over
        # 2 s. See SIGN_DEFORM_SENSE_GUARD and the reversal memory.
        ring = _ring()
        pos = (ring[0][0] * 0.95, ring[0][1] * 0.95)
        yaw = math.atan2(ring[1][1] - ring[0][1], ring[1][0] - ring[0][0]) + math.pi
        off = _controller(gate=False).select_target_point(pos, yaw, ring, 0, 0.20)
        on = _controller(gate=True).select_target_point(pos, yaw, ring, 0, 0.20)
        assert on == off

    def test_gate_never_returns_nothing(self) -> None:
        # When everything in the span is wrong-sense the existing fallback
        # tiers must still answer -- the gate is a filter, not a hard reject,
        # and a None here would be a new silent failure mode.
        ring = _ring()
        pos = (ring[0][0] * 0.95, ring[0][1] * 0.95)
        yaw = math.atan2(ring[1][1] - ring[0][1], ring[1][0] - ring[0][0]) + math.pi
        target = _controller(gate=True).select_target_point(pos, yaw, ring, 0, 0.20)
        assert target is not None
        assert len(target) == 2


class TestDeformSenseGuard:
    """The navigator-side predicate, which judges the FINAL aim point."""

    def test_a_deform_that_crosses_the_path_direction_is_detected(self) -> None:
        path = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0), Waypoint(2.0, 0.0)]
        # Undeformed target 0.28 m ahead: agrees.
        assert _bearing_agrees_with_path((0.28, 0.0), 0.0, 0.0, path, 0)
        # The same point shoved 0.554 m sideways AND backwards, which is what a
        # shove nearly twice the target range does to the bearing.
        assert not _bearing_agrees_with_path((-0.10, 0.554), 0.0, 0.0, path, 0)

    def test_a_large_but_harmless_shove_still_agrees(self) -> None:
        # The measured refutation of a ratio clamp: a shove larger than the
        # range is routine on a healthy round (12% of the clean control's
        # right-sense ticks) and must NOT be flagged while the bearing still
        # points along the path.
        path = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)]
        assert _bearing_agrees_with_path((0.28, 0.40), 0.0, 0.0, path, 0)

    def test_empty_path_cannot_disagree(self) -> None:
        assert _bearing_agrees_with_path((1.0, 0.0), 0.0, 0.0, [], 0)

    def test_target_on_the_pose_cannot_disagree(self) -> None:
        path = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)]
        assert _bearing_agrees_with_path((0.0, 0.0), 0.0, 0.0, path, 0)

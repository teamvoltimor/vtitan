"""``project_onto_path`` must measure in the path's frame, not a global one.

The reason this primitive exists: the sign-avoidance tracking figures were
taken as the difference of two GLOBAL coordinates, and two-thirds of legal sign
positions sit on a corner. On a corner that difference charges the robot for
the turn it was supposed to be making, so a chassis perfectly on its arc reads
as steadily diverging from the line it was commanded to hold. See
``scripts/sim/diag_failure_split.py`` and the 46% "tracking failure" bucket it
had to withdraw.

The two load-bearing properties, in order:

* on a straight, the path frame and the global-axis frame agree EXACTLY -- so
  swapping frames cannot silently move the numbers that were already sound;
* on a corner they must not, and the axis frame is the one that is wrong.
"""

from __future__ import annotations

import math

import pytest
from shared.domain.models import Waypoint

from src.navigation.track_geometry import cross_track_error, project_onto_path

_ARC_RADIUS_M = 0.5
_DEFORM_M = 0.18
"""A representative sign deformation, i.e. the lateral gap the frames must agree on."""


def _straight_east(n: int = 21, spacing: float = 0.05) -> list[Waypoint]:
    """Along +x at y=0, so the global y coordinate IS the lateral offset."""
    return [Waypoint(i * spacing, 0.0) for i in range(n)]


def _quarter_arc(step_deg: float = 2.0) -> list[Waypoint]:
    """A left turn about the origin, from due north-of-east round to due north.

    Travelled counter-clockwise, so the outward radial direction is to the
    chassis's RIGHT and reads as a negative offset.
    """
    steps = int(90.0 / step_deg)
    return [
        Waypoint(_ARC_RADIUS_M * math.cos(math.radians(i * step_deg)), _ARC_RADIUS_M * math.sin(math.radians(i * step_deg)))
        for i in range(steps + 1)
    ]


def _on_arc(angle_deg: float, radial_offset_m: float = 0.0) -> tuple[float, float]:
    r = _ARC_RADIUS_M + radial_offset_m
    return (r * math.cos(math.radians(angle_deg)), r * math.sin(math.radians(angle_deg)))


class TestTheFrameItself:
    def test_a_point_on_the_path_has_no_offset(self):
        assert project_onto_path(_straight_east(), 0.5, 0.0).signed_offset_m == pytest.approx(0.0, abs=1e-12)

    def test_left_of_travel_is_positive(self):
        assert project_onto_path(_straight_east(), 0.5, 0.12).signed_offset_m == pytest.approx(0.12)

    def test_right_of_travel_is_negative(self):
        assert project_onto_path(_straight_east(), 0.5, -0.12).signed_offset_m == pytest.approx(-0.12)

    def test_the_tangent_is_the_direction_of_travel(self):
        assert project_onto_path(_straight_east(), 0.5, 0.12).tangent_rad == pytest.approx(0.0)

    def test_the_tangent_follows_the_arc(self):
        # Tangent to a CCW circle at 40 deg points 90 deg further round.
        at_40 = project_onto_path(_quarter_arc(), *_on_arc(40.0))
        assert at_40.tangent_rad == pytest.approx(math.radians(130.0), abs=math.radians(1.0))

    def test_the_offset_and_the_distance_agree_beside_a_segment(self):
        at = project_onto_path(_straight_east(), 0.5, 0.12)
        assert at.distance_m == pytest.approx(abs(at.signed_offset_m))

    def test_they_part_company_outside_a_convex_vertex(self):
        # The artifact this frame exists to avoid: measured to the VERTEX, a
        # target sweeping past a corner picks up an along-track term and shows
        # a bulge in its offset that no controller produced.
        corner = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0), Waypoint(1.0, 1.0)]
        outside = project_onto_path(corner, 1.20, -0.05)

        assert outside.distance_m == pytest.approx(math.hypot(0.20, 0.05))
        assert outside.signed_offset_m == pytest.approx(-0.05)

    def test_it_projects_onto_the_segment_not_the_nearest_waypoint(self):
        # Mid-segment, where nearest-waypoint would overstate the offset by half
        # the spacing -- enough to matter against a +-6.7 cm sign-pass budget.
        coarse = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)]
        assert project_onto_path(coarse, 0.5, 0.02).signed_offset_m == pytest.approx(0.02)


class TestCrossTrackErrorIsUnchanged:
    """The existing consumer must not shift under the refactor."""

    def test_it_is_the_unsigned_offset(self):
        path = _straight_east()
        assert cross_track_error(path, 0.5, -0.12) == pytest.approx(0.12)

    def test_a_point_beyond_the_end_clamps_to_the_endpoint(self):
        path = _straight_east()
        assert cross_track_error(path, 5.0, 0.0) == pytest.approx(5.0 - path[-1].x)

    def test_a_degenerate_path_still_reports_a_distance(self):
        # One waypoint offers no tangent, but reporting a confident zero would
        # be worse than reporting the distance to it.
        assert cross_track_error([Waypoint(0.0, 0.0)], 0.3, 0.4) == pytest.approx(0.5)

    def test_an_empty_path_is_infinitely_far(self):
        assert cross_track_error([], 0.3, 0.4) == math.inf


class TestTheFramesAgreeOnAStraight:
    """The sabotage check: if these diverge, the path frame is miswired."""

    def test_the_lateral_separation_matches_the_global_axis(self):
        path = _straight_east()
        robot, target = (0.30, 0.05), (0.70, 0.05 + _DEFORM_M)

        in_path = project_onto_path(path, *target).signed_offset_m - project_onto_path(path, *robot).signed_offset_m
        on_axis = target[1] - robot[1]

        assert in_path == pytest.approx(on_axis)
        assert in_path == pytest.approx(_DEFORM_M)


class TestTheFramesDisagreeOnACorner:
    """And the axis frame is the one that is wrong."""

    def test_a_chassis_on_its_arc_reads_as_on_the_path(self):
        assert project_onto_path(_quarter_arc(), *_on_arc(20.0)).signed_offset_m == pytest.approx(0.0, abs=2e-4)

    def test_the_path_frame_recovers_the_true_lateral_gap(self):
        # Robot on the arc at 20 deg; commanded line deformed outward at 50 deg.
        # The gap that decides whether the pass clears is _DEFORM_M, wherever
        # along the arc the two happen to sit.
        path = _quarter_arc()
        robot = project_onto_path(path, *_on_arc(20.0)).signed_offset_m
        target = project_onto_path(path, *_on_arc(50.0, _DEFORM_M)).signed_offset_m

        assert abs(target - robot) == pytest.approx(_DEFORM_M, abs=2e-3)

    def test_the_global_axis_nearly_doubles_it(self):
        # The confound, sized. Same two points, differenced on the global y
        # axis as the published figures were: the arc contributes most of it.
        _, robot_y = _on_arc(20.0)
        _, target_y = _on_arc(50.0, _DEFORM_M)

        on_axis = abs(target_y - robot_y)

        assert on_axis > 1.9 * _DEFORM_M

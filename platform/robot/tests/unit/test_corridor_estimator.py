"""Unit tests for estimating the track layout from LIDAR instead of metadata.

The closed-loop coverage lives in ``scripts/sim/diag_localization.py blind`` (27/28
Open Challenge fixtures, layout learned in 28/28). These pin the pieces that
closed-loop runs can only exercise indirectly — in particular the outlier
resistance, which is where a working estimator regressed once already.
"""

from __future__ import annotations

import math

import pytest
from shared.config.enums import Direction, Section

from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    classify_width,
    measure_corridor_width,
    section_from_heading,
)

_NARROW = 0.6
_WIDE = 1.0


def _scan(left_m: float, right_m: float, rays: int = 360) -> tuple[list[float], list[float]]:
    """A sweep whose only meaningful returns are the two perpendicular rays.

    The bearings are placed as the simulator places them, which does not
    guarantee a ray exactly at +-pi/2, so the two values are written to
    whichever bearings are nearest — the same ones the estimator will read.
    """
    angles = [-math.pi + 2 * math.pi * i / (rays - 1) for i in range(rays)]
    ranges = [3.0] * rays

    def nearest(target: float) -> int:
        return min(
            range(rays), key=lambda i: abs(math.atan2(math.sin(angles[i] - target), math.cos(angles[i] - target)))
        )

    ranges[nearest(math.pi / 2)] = left_m
    ranges[nearest(-math.pi / 2)] = right_m
    return ranges, angles


class TestMeasureCorridorWidth:
    """Left range + right range spans the corridor through the chassis centre."""

    def test_sums_the_two_perpendicular_rays(self) -> None:
        ranges, angles = _scan(0.35, 0.25)
        result = measure_corridor_width(ranges, angles, yaw=0.0)
        assert result is not None
        assert result.width_m == pytest.approx(0.60, abs=0.02)

    def test_independent_of_where_in_the_corridor_the_robot_sits(self) -> None:
        centred, angles = _scan(0.30, 0.30)
        offset, _ = _scan(0.45, 0.15)
        c = measure_corridor_width(centred, angles, yaw=0.0)
        o = measure_corridor_width(offset, angles, yaw=0.0)
        assert c is not None and o is not None
        assert c.width_m == pytest.approx(o.width_m)

    def test_works_on_every_track_axis(self) -> None:
        ranges, angles = _scan(0.5, 0.5)
        for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            result = measure_corridor_width(ranges, angles, yaw=yaw)
            assert result is not None
            assert result.width_m == pytest.approx(1.0, abs=0.02)

    def test_rejects_a_badly_misaligned_chassis(self) -> None:
        """Off-axis the side rays cut a diagonal, which is not the width."""
        ranges, angles = _scan(0.3, 0.3)
        assert measure_corridor_width(ranges, angles, yaw=math.radians(45)) is None

    def test_rejects_a_ray_that_escaped_past_the_inner_block(self) -> None:
        """At a corner the inward ray runs off down the next corridor."""
        ranges, angles = _scan(0.3, 2.5)
        assert measure_corridor_width(ranges, angles, yaw=0.0) is None


class TestClassifyWidth:
    """Only two widths are legal, so this is a two-class decision."""

    @pytest.mark.parametrize(
        ("measured", "expected"),
        [(0.55, _NARROW), (0.65, _NARROW), (0.79, _NARROW), (0.81, _WIDE), (0.95, _WIDE), (1.10, _WIDE)],
    )
    def test_snaps_to_the_nearer_legal_width(self, measured: float, expected: float) -> None:
        assert classify_width(measured) == pytest.approx(expected)


class TestSectionFromHeading:
    """Heading identifies the corridor without touching the position estimate."""

    @pytest.mark.parametrize(
        ("direction", "heading_deg", "expected"),
        [
            (Direction.CLOCKWISE, 180, Section.SOUTH),
            (Direction.CLOCKWISE, 0, Section.NORTH),
            (Direction.CLOCKWISE, -90, Section.EAST),
            (Direction.CLOCKWISE, 90, Section.WEST),
            (Direction.COUNTERCLOCKWISE, 0, Section.SOUTH),
            (Direction.COUNTERCLOCKWISE, 180, Section.NORTH),
            (Direction.COUNTERCLOCKWISE, 90, Section.EAST),
            (Direction.COUNTERCLOCKWISE, -90, Section.WEST),
        ],
    )
    def test_every_travel_vector_is_distinguishable(
        self,
        direction: Direction,
        heading_deg: float,
        expected: Section,
    ) -> None:
        assert section_from_heading(math.radians(heading_deg), direction) is expected

    def test_tolerates_heading_error_short_of_the_next_corridor(self) -> None:
        for error_deg in (-30, -10, 10, 30):
            assert section_from_heading(math.radians(error_deg), Direction.COUNTERCLOCKWISE) is Section.SOUTH


class TestCorridorWidthEstimator:
    """Votes, rather than streaks — corner leakage is clustered, not random."""

    def _feed(self, estimator: CorridorWidthEstimator, section: Section, width: float, times: int) -> None:
        ranges, angles = _scan(width / 2, width / 2)
        for _ in range(times):
            estimator.observe(section, ranges, angles, yaw=0.0)

    def test_assumes_narrow_before_seeing_anything(self) -> None:
        """The safe prior: planning wide-as-narrow stays inside the corridor."""
        estimator = CorridorWidthEstimator()
        assert all(w == pytest.approx(_NARROW) for w in estimator.widths.values())
        assert not estimator.observed_sections

    def test_prior_can_be_seeded_for_a_round_whose_width_is_a_rule(self) -> None:
        """The Obstacles Challenge fixes every corridor at 1.0 m.

        There the narrow prior is not conservative, it is known-wrong for every
        corridor, and a blind run pays for it: the robot turns into a corridor
        still holding the default and meets a traffic sign before it has taken
        enough readings to correct it. Seeding the prior removed the entire
        blind penalty over the 16 obstacles fixtures (16/16 collisions and 0
        three-lap finishes, to 14/16 and 2 — exactly matching the sighted run).
        """
        estimator = CorridorWidthEstimator(assumed_width=_WIDE)
        assert all(w == pytest.approx(_WIDE) for w in estimator.widths.values())
        # Seeded, not decided: nothing has been measured yet, so the estimator
        # must not report these as observed or it would suppress its own
        # replanning when a real reading disagrees.
        assert not estimator.observed_sections

    def test_a_seeded_prior_is_still_overridden_by_measurement(self) -> None:
        """Seeding changes where the estimator starts, not whether it measures."""
        estimator = CorridorWidthEstimator(assumed_width=_WIDE)
        self._feed(estimator, Section.EAST, _NARROW, times=20)
        assert estimator.widths[Section.EAST] == pytest.approx(_NARROW)
        assert Section.EAST in estimator.observed_sections

    def test_learns_a_wide_corridor(self) -> None:
        estimator = CorridorWidthEstimator()
        self._feed(estimator, Section.EAST, _WIDE, times=20)
        assert estimator.widths[Section.EAST] == pytest.approx(_WIDE)
        assert Section.EAST in estimator.observed_sections

    def test_reports_a_change_so_the_caller_can_replan(self) -> None:
        estimator = CorridorWidthEstimator(min_samples=4)
        ranges, angles = _scan(0.5, 0.5)
        changes = [estimator.observe(Section.NORTH, ranges, angles, yaw=0.0) for _ in range(8)]
        assert changes.count(True) == 1, "should announce the change once, not every tick"

    def test_a_burst_of_corner_leakage_does_not_flip_a_settled_corridor(self) -> None:
        """The regression this class was rewritten for.

        Corner leakage reads as a wide corridor and arrives in runs, so a
        consecutive-agreement rule flips a corridor that had already settled
        correctly. Measured on ``go_open_0002``: a truly 0.6 m corridor
        averaging 0.635 m still peaked at 1.229 m.
        """
        estimator = CorridorWidthEstimator()
        self._feed(estimator, Section.NORTH, _NARROW, times=40)
        assert estimator.widths[Section.NORTH] == pytest.approx(_NARROW)

        self._feed(estimator, Section.NORTH, 1.2, times=8)
        assert estimator.widths[Section.NORTH] == pytest.approx(_NARROW), (
            "a minority of leaked readings must not overturn the majority"
        )

    def test_is_complete_only_once_every_corridor_was_measured(self) -> None:
        estimator = CorridorWidthEstimator()
        for section in (Section.NORTH, Section.SOUTH, Section.EAST):
            self._feed(estimator, section, _WIDE, times=20)
        assert not estimator.is_complete
        self._feed(estimator, Section.WEST, _NARROW, times=20)
        assert estimator.is_complete

"""Open Challenge navigation tests — 3-lap completion via mock runner.

Validates that the ``OpenLapPlanner`` can solve every generated Open Challenge
scenario (complete 3 laps) without hitting walls, for:
  • 10 deterministic seeded scenarios (from conftest fixture)
  • Every possible corridor width combination (2^4 = 16 combos, exhaustive)
  • All 8 starting configurations (4 sections × 2 directions, exhaustive)

No Gazebo, no ROS2.  Pure Python geometry.

Run with:
    pixi run test -v -k navigation
    pixi run test -v                    # included in full suite
"""

from __future__ import annotations

import logging
import math
from itertools import product

import pytest

from shared.config.constants import CompetitionSpecs, TrackDimensions
from shared.domain.enums import Direction, Section
from src.navigation.open_lap_planner import (
    MockLapRunner,
    OpenLapPlanner,
    TrackCorners,
)
from src.scenario.models import CorridorWidth, Position, ScenarioMetadata, StartingConditions

logger = logging.getLogger(__name__)

_PLANNER = OpenLapPlanner()
_CORNER_TOL = 1e-9

# Allowed corridor widths for Open challenge
_NARROW_M = 0.6
_WIDE_M = 1.0
_WIDTH_CHOICES = [_NARROW_M, _WIDE_M]

_ALL_SECTIONS = list(Section)
_ALL_DIRECTIONS = list(Direction)


def _make_scenario(
    south_m: float,
    north_m: float,
    east_m: float,
    west_m: float,
    section: Section = Section.SOUTH,
    direction: Direction = Direction.CLOCKWISE,
) -> ScenarioMetadata:
    """Build a minimal ScenarioMetadata for unit testing."""

    def cw(w: float) -> CorridorWidth:
        return CorridorWidth(
            type="narrow" if w == _NARROW_M else "wide",
            width_mm=int(w * 1000),
        )

    return ScenarioMetadata(
        scenario_id=0,
        challenge_type="open",  # type: ignore[arg-type]
        seed=None,
        num_signs=0,
        has_parking_lot=False,
        corridor_widths={
            Section.SOUTH: cw(south_m),
            Section.NORTH: cw(north_m),
            Section.EAST: cw(east_m),
            Section.WEST: cw(west_m),
        },
        starting_conditions=StartingConditions(
            section=section,
            direction=direction,
            position=Position(x=1.5, y=south_m / 2),
            yaw=math.pi,
        ),
    )


class TestCornerComputation:
    """Unit tests for TrackCorners geometry."""

    def test_symmetric_wide_track_corners(self) -> None:
        s = _make_scenario(1.0, 1.0, 1.0, 1.0)
        c = _PLANNER.corners(s)
        assert math.isclose(c.sw[0], 0.5, abs_tol=_CORNER_TOL)
        assert math.isclose(c.sw[1], 0.5, abs_tol=_CORNER_TOL)
        assert math.isclose(c.se[0], 2.5, abs_tol=_CORNER_TOL)
        assert math.isclose(c.ne[1], 2.5, abs_tol=_CORNER_TOL)
        assert math.isclose(c.nw[0], 0.5, abs_tol=_CORNER_TOL)

    def test_symmetric_narrow_track_corners(self) -> None:
        s = _make_scenario(0.6, 0.6, 0.6, 0.6)
        c = _PLANNER.corners(s)
        assert math.isclose(c.sw[0], 0.3, abs_tol=_CORNER_TOL)  # west_x = 0.6/2
        assert math.isclose(c.sw[1], 0.3, abs_tol=_CORNER_TOL)  # south_y = 0.6/2
        assert math.isclose(c.ne[0], 2.7, abs_tol=_CORNER_TOL)  # east_x = 3 - 0.6/2
        assert math.isclose(c.ne[1], 2.7, abs_tol=_CORNER_TOL)  # north_y = 3 - 0.6/2

    def test_asymmetric_corners(self) -> None:
        s = _make_scenario(south_m=0.6, north_m=1.0, east_m=0.6, west_m=1.0)
        c = _PLANNER.corners(s)
        assert math.isclose(c.sw[1], 0.3, abs_tol=_CORNER_TOL)   # south_y = 0.6/2
        assert math.isclose(c.ne[1], 2.5, abs_tol=_CORNER_TOL)   # north_y = 3 - 1.0/2
        assert math.isclose(c.se[0], 2.7, abs_tol=_CORNER_TOL)   # east_x = 3 - 0.6/2
        assert math.isclose(c.sw[0], 0.5, abs_tol=_CORNER_TOL)   # west_x = 1.0/2

    def test_all_corners_within_track_bounds(self) -> None:
        for sw, sn, se_w, ww in product(_WIDTH_CHOICES, repeat=4):
            s = _make_scenario(sw, sn, se_w, ww)
            c = _PLANNER.corners(s)
            for pt in (c.sw, c.se, c.ne, c.nw):
                assert TrackDimensions.MIN_COORD <= pt[0] <= TrackDimensions.MAX_COORD, f"x={pt[0]} out of bounds"
                assert TrackDimensions.MIN_COORD <= pt[1] <= TrackDimensions.MAX_COORD, f"y={pt[1]} out of bounds"

    def test_corner_order_cw_south(self) -> None:
        s = _make_scenario(1.0, 1.0, 1.0, 1.0)
        c = _PLANNER.corners(s)
        ordered = c.ordered_for(Section.SOUTH, Direction.CLOCKWISE)
        assert ordered[0] == c.sw, "CW from South: first corner should be SW"
        assert ordered[1] == c.nw
        assert ordered[2] == c.ne
        assert ordered[3] == c.se, "CW from South: last corner should be SE"

    def test_corner_order_ccw_south(self) -> None:
        s = _make_scenario(1.0, 1.0, 1.0, 1.0)
        c = _PLANNER.corners(s)
        ordered = c.ordered_for(Section.SOUTH, Direction.COUNTERCLOCKWISE)
        assert ordered[0] == c.se, "CCW from South: first corner should be SE"
        assert ordered[1] == c.ne
        assert ordered[2] == c.nw
        assert ordered[3] == c.sw

    def test_corner_order_all_sections_cw(self) -> None:
        s = _make_scenario(1.0, 1.0, 1.0, 1.0)
        c = _PLANNER.corners(s)
        cw_first_corner = {
            Section.SOUTH: c.sw,
            Section.WEST:  c.nw,
            Section.NORTH: c.ne,
            Section.EAST:  c.se,
        }
        for section, expected_first in cw_first_corner.items():
            ordered = c.ordered_for(section, Direction.CLOCKWISE)
            assert ordered[0] == expected_first, (
                f"CW from {section}: first corner should be {expected_first}, got {ordered[0]}"
            )

    def test_all_four_corners_visited_each_lap(self) -> None:
        for section in _ALL_SECTIONS:
            for direction in _ALL_DIRECTIONS:
                s = _make_scenario(0.6, 1.0, 0.6, 1.0, section, direction)
                c = _PLANNER.corners(s)
                ordered = c.ordered_for(section, direction)
                assert len(ordered) == 4
                assert set(ordered) == {c.sw, c.se, c.ne, c.nw}, (
                    f"{section}/{direction}: not all 4 corners visited"
                )


class TestMockLapRunner:
    """Unit tests for the mock runner mechanics."""

    def test_runner_reaches_all_waypoints(self) -> None:
        s = _make_scenario(1.0, 1.0, 1.0, 1.0)
        runner = _PLANNER.plan(s, n_laps=1)
        assert len(runner.laps) == 1
        assert len(runner.laps[0].waypoints) == 4

    def test_runner_records_distance(self) -> None:
        s = _make_scenario(1.0, 1.0, 1.0, 1.0)
        runner = _PLANNER.plan(s, n_laps=1)
        assert runner.laps[0].distance_m > 0.0

    def test_runner_all_waypoints_in_bounds(self) -> None:
        s = _make_scenario(0.6, 0.6, 0.6, 0.6)
        runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
        for lap in runner.laps:
            assert lap.all_in_bounds, f"lap {lap.lap}: waypoints out of bounds"

    def test_distance_consistent_across_laps(self) -> None:
        # Lap 1 starts from the robot's initial position (may not be at a corner),
        # so its distance can differ from subsequent laps.  Laps 2+ all start from
        # the last-visited corner and traverse the same 4 waypoints — they must match.
        s = _make_scenario(0.6, 1.0, 1.0, 0.6)
        runner = _PLANNER.plan(s, n_laps=3)
        distances = [lap.distance_m for lap in runner.laps]
        assert math.isclose(distances[1], distances[2], rel_tol=1e-9)


class TestOpenChallengeSolvability:
    """Core claim: every Open Challenge scenario is solvable in 3 laps."""

    def test_all_16_corridor_combos_3_laps(self) -> None:
        """Exhaustive check: all 2^4 = 16 corridor width combinations × default start."""
        failed = []
        for sw, sn, se_w, ww in product(_WIDTH_CHOICES, repeat=4):
            s = _make_scenario(sw, sn, se_w, ww)
            runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
            if runner.completed_laps != CompetitionSpecs.OPEN_CHALLENGE_LAPS:
                failed.append((sw, sn, se_w, ww))
        assert not failed, f"Could not complete {CompetitionSpecs.OPEN_CHALLENGE_LAPS} laps for combos: {failed}"

    def test_all_8_starting_configs_3_laps(self) -> None:
        """All 4 sections × 2 directions complete 3 laps on a symmetric track."""
        failed = []
        for section in _ALL_SECTIONS:
            for direction in _ALL_DIRECTIONS:
                s = _make_scenario(0.8, 0.8, 0.8, 0.8, section, direction)
                runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
                if runner.completed_laps != CompetitionSpecs.OPEN_CHALLENGE_LAPS:
                    failed.append((section, direction))
        assert not failed, f"Could not complete 3 laps for configs: {failed}"

    def test_all_16_combos_all_8_starts_3_laps(self) -> None:
        """128-case matrix: every corridor combo × every start config."""
        failed = []
        for sw, sn, se_w, ww in product(_WIDTH_CHOICES, repeat=4):
            for section in _ALL_SECTIONS:
                for direction in _ALL_DIRECTIONS:
                    s = _make_scenario(sw, sn, se_w, ww, section, direction)
                    runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
                    if runner.completed_laps != CompetitionSpecs.OPEN_CHALLENGE_LAPS:
                        failed.append((sw, sn, se_w, ww, section, direction))
        assert not failed, (
            f"{len(failed)}/128 combinations failed to complete 3 laps: {failed[:5]}..."
        )

    def test_generated_scenarios_3_laps(self, open_scenarios: list[ScenarioMetadata]) -> None:
        """The 10 seeded generated scenarios are all solvable in 3 laps."""
        failed = []
        for s in open_scenarios:
            runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
            if runner.completed_laps != CompetitionSpecs.OPEN_CHALLENGE_LAPS:
                failed.append(s.scenario_id)
        assert not failed, f"Scenarios {failed} did not complete {CompetitionSpecs.OPEN_CHALLENGE_LAPS} laps"


class TestNavigationSummaryLog:
    """Human-readable per-scenario navigation logs for review."""

    def test_log_3_lap_summary_seeded(
        self, open_scenarios: list[ScenarioMetadata]
    ) -> None:
        sep = "─" * 72
        logger.info(sep)
        logger.info(
            "OPEN CHALLENGE — 3-LAP NAVIGATION SUMMARY  (seed=42, n=%d)",
            len(open_scenarios),
        )
        logger.info(sep)
        logger.info(
            "  %-4s  %-5s  %-16s  %-7s  %-8s  %-s",
            "#ID", "DIR", "SECTION", "LAPS", "DIST(m)", "CORRIDORS S/N/E/W mm",
        )
        logger.info(sep)

        for s in open_scenarios:
            runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
            sc = s.starting_conditions
            widths = {k: v.width_mm for k, v in s.corridor_widths.items()}
            solved = "✓" if runner.completed_laps == CompetitionSpecs.OPEN_CHALLENGE_LAPS else "✗"
            logger.info(
                "  %02d    %s  %-16s  %s %d/%d   %7.2f   %d/%d/%d/%d",
                s.scenario_id,
                "CW " if sc.direction == Direction.CLOCKWISE else "CCW",
                sc.section.capitalize(),
                solved,
                runner.completed_laps,
                CompetitionSpecs.OPEN_CHALLENGE_LAPS,
                runner.total_distance_m,
                widths[Section.SOUTH],
                widths[Section.NORTH],
                widths[Section.EAST],
                widths[Section.WEST],
            )

        logger.info(sep)
        print(f"\n\n{sep}")
        print(f"OPEN CHALLENGE — 3-LAP NAVIGATION SUMMARY  (seed=42, n={len(open_scenarios)})")
        print(sep)
        print(f"  {'#ID':<4}  {'DIR':<5}  {'SECTION':<16}  {'LAPS':<7}  {'DIST':>8}  CORRIDORS S/N/E/W mm")
        print(sep)
        for s in open_scenarios:
            runner = _PLANNER.plan(s, n_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
            sc = s.starting_conditions
            widths = {k: v.width_mm for k, v in s.corridor_widths.items()}
            solved = "✓" if runner.completed_laps == CompetitionSpecs.OPEN_CHALLENGE_LAPS else "✗"
            print(
                f"  {s.scenario_id:02d}    "
                f"{'CW ' if sc.direction == Direction.CLOCKWISE else 'CCW'}  "
                f"{sc.section.capitalize():<16}  "
                f"{solved} {runner.completed_laps}/{CompetitionSpecs.OPEN_CHALLENGE_LAPS}   "
                f"{runner.total_distance_m:7.2f}m  "
                f"{widths[Section.SOUTH]}/{widths[Section.NORTH]}/{widths[Section.EAST]}/{widths[Section.WEST]}"
            )
        print(sep)

    def test_log_corridor_combo_lap_distances(self) -> None:
        """Show lap distance for all 16 corridor combos — useful for waypoint tuning."""
        sep = "─" * 60
        logger.info(sep)
        logger.info("OPEN CHALLENGE — LAP DISTANCES FOR ALL 16 CORRIDOR COMBOS")
        logger.info(sep)
        logger.info("  S mm  N mm  E mm  W mm  |  lap dist (m)")
        logger.info(sep)
        for sw, sn, se_w, ww in product(_WIDTH_CHOICES, repeat=4):
            s = _make_scenario(sw, sn, se_w, ww)
            runner = _PLANNER.plan(s, n_laps=1)
            dist = runner.laps[0].distance_m
            logger.info(
                "  %4d  %4d  %4d  %4d  |  %.4f",
                int(sw * 1000), int(sn * 1000), int(se_w * 1000), int(ww * 1000), dist,
            )
        logger.info(sep)

"""Realistic closed-loop Open Challenge simulation tests.

These drive the *real* ``CoreNavigator`` (pure-pursuit + collision + stuck +
LapDetector) through a simulated ``HardwareGateway`` backed by an Ackermann
bicycle model and a raycast LIDAR (``src/simulation``). Unlike a teleport mock,
the car must actually steer itself around the corridor — with finite turning
radius, servo slew, drive acceleration limits and noisy LIDAR — and not clip a
wall, exactly as the physical robot would.

Run with:
    PYTHONPATH=. pytest tests/unit/test_open_challenge_sim.py -v
    PYTHONPATH=. pytest tests/unit/test_open_challenge_sim.py -v -s   # + summaries
"""

from __future__ import annotations

import logging
import math
from itertools import product
from typing import Any

import numpy as np
import pytest
from shared.config.constants import CompetitionSpecs, CorridorDimensions, RobotSpecs, TrackDimensions
from shared.config.enums import Direction, Section

from src.navigation.planning.waypoints import _OUTER_WALL_BIAS
from src.navigation.race_tracker import _TRAVEL_DIRS
from src.simulation import ScenarioSimulator, TrackModel

logger = logging.getLogger(__name__)

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
_TRACK_MAX = TrackDimensions.MAX_COORD
_OUTER_BIAS = _OUTER_WALL_BIAS

# Travel-direction unit vectors, imported directly from race_tracker so this
# test can never silently drift from the real finish-line normals it mirrors.
_TRAVEL = _TRAVEL_DIRS

_ALL_SECTIONS = list(Section)
_ALL_DIRECTIONS = list(Direction)


def _start_pose(
    section: Section, direction: Direction, widths_m: dict[str, float],
) -> tuple[float, float, float]:
    """Spawn pose on the biased corridor centerline, aligned with travel."""
    south_cy = widths_m["south"] / 2 - _OUTER_BIAS
    north_cy = _TRACK_MAX - widths_m["north"] / 2 + _OUTER_BIAS
    east_cx = _TRACK_MAX - widths_m["east"] / 2 + _OUTER_BIAS
    west_cx = widths_m["west"] / 2 - _OUTER_BIAS
    center = {
        Section.SOUTH: (1.5, south_cy),
        Section.NORTH: (1.5, north_cy),
        Section.EAST: (east_cx, 1.5),
        Section.WEST: (west_cx, 1.5),
    }[section]
    nx, ny = _TRAVEL[(section, direction)]
    return center[0], center[1], math.atan2(ny, nx)


def build_open_metadata(
    widths_mm: dict[str, int],
    section: Section,
    direction: Direction,
    scenario_id: int = 0,
) -> dict[str, Any]:
    """Construct a valid Open Challenge metadata dict (same schema as simgen)."""
    widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
    sx, sy, yaw = _start_pose(section, direction, widths_m)
    return {
        "scenario_id": scenario_id,
        "challenge_type": "open",
        "seed": None,
        "num_signs": 0,
        "has_parking_lot": False,
        "parking_lot": None,
        "sign_positions": [],
        "corridor_widths": {
            side: {
                "type": "narrow" if widths_mm[side] == _NARROW_MM else "wide",
                "width_mm": widths_mm[side],
            }
            for side in ("north", "south", "east", "west")
        },
        "starting_conditions": {
            "direction": str(direction),
            "section": section.capitalized,
            "position": {"x": sx, "y": sy},
            "yaw": yaw,
        },
    }


def _uniform_widths(mm: int) -> dict[str, int]:
    return dict.fromkeys(("north", "south", "east", "west"), mm)


# Track model unit checks


class TestTrackModelGeometry:
    """The raycast LIDAR and collision model must match the wall geometry."""

    def test_forward_ray_hits_inner_block(self) -> None:
        # Wide symmetric track: inner block spans [1.0, 2.0]^2.
        track = TrackModel(dict.fromkeys(Section, 1.0))
        # Stand in the south corridor centerline, facing north (+y).
        ranges = track.raycast_scan(
            x=1.5, y=0.5, yaw=math.pi / 2, angles_robot=np.array([0.0]),
        )
        # Distance to inner block south face at y=1.0 -> 0.5 m.
        assert math.isclose(ranges[0], 0.5, abs_tol=1e-6)

    def test_backward_ray_hits_outer_wall(self) -> None:
        track = TrackModel(dict.fromkeys(Section, 1.0))
        # Facing north, the rear ray (pi) points south to the outer wall at y=0.
        ranges = track.raycast_scan(
            x=1.5, y=0.5, yaw=math.pi / 2, angles_robot=np.array([math.pi]),
        )
        assert math.isclose(ranges[0], 0.5, abs_tol=1e-6)

    def test_centerline_does_not_collide(self) -> None:
        track = TrackModel(dict.fromkeys(Section, 1.0))
        assert not track.footprint_collides(1.5, 0.5, 0.0)

    def test_into_outer_wall_collides(self) -> None:
        track = TrackModel(dict.fromkeys(Section, 1.0))
        # Chassis centre 0.05 m from the south wall -> half the 0.15 m width
        # (0.075) crosses the collision face at y=0.04.
        assert track.footprint_collides(1.5, 0.05, 0.0)

    def test_into_inner_block_collides(self) -> None:
        track = TrackModel(dict.fromkeys(Section, 1.0))
        # Just inside the inner block (centre at 1.5, 1.05) is solid.
        assert track.footprint_collides(1.5, 1.05, 0.0)


# Planner sanity: the canonical path must sit inside the corridor


class TestPlannedWaypointsClearCorridor:
    """Every planned waypoint must lie in free space with chassis clearance."""

    @pytest.mark.parametrize(
        ("south", "north", "east", "west"),
        [
            (_WIDE_MM, _WIDE_MM, _WIDE_MM, _WIDE_MM),
            (_NARROW_MM, _NARROW_MM, _NARROW_MM, _NARROW_MM),
            (_NARROW_MM, _WIDE_MM, _NARROW_MM, _WIDE_MM),
            (_WIDE_MM, _NARROW_MM, _WIDE_MM, _NARROW_MM),
        ],
    )
    def test_waypoints_in_free_space(
        self, south: int, north: int, east: int, west: int,
    ) -> None:
        meta = build_open_metadata(
            {"south": south, "north": north, "east": east, "west": west},
            Section.SOUTH,
            Direction.CLOCKWISE,
        )
        sim = ScenarioSimulator(meta, num_laps=_N_LAPS)
        # Chassis half-width clearance to the nearest visual wall.
        clearance = RobotSpecs.WIDTH / 2
        offenders = [
            wp
            for wp in sim.waypoints
            if not sim.track.point_in_free_space(wp[0], wp[1], clearance)
        ]
        assert not offenders, f"{len(offenders)} waypoints too close to a wall: {offenders[:3]}"


# Closed-loop 3-lap solvability with the real navigator


def _log_result(label: str, result: Any) -> None:
    status = "OK " if result.success else "FAIL"
    logger.info(
        "%s | %s laps=%d/%d collided=%s timeout=%s | dist=%.2fm t=%.1fs "
        "vmax=%.2f vavg=%.2f minLIDAR=%.2fm",
        status,
        label,
        result.laps_completed,
        result.target_laps,
        result.collided,
        result.timed_out,
        result.distance_m,
        result.sim_time_s,
        result.max_speed_mps,
        result.avg_speed_mps,
        result.min_lidar_range_m,
    )


def _within_round_limit(result: Any) -> bool:
    """Solved AND finished inside the official WRO round time limit.

    ``SimResult.success`` only checks laps-completed/collision; a scenario that
    finishes 3 laps at, say, 195s "passes" a lap-count-only check but scores
    zero in competition. The sim's own step budget (200s) is looser than the
    180s round limit, so this must be checked explicitly.
    """
    return result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S


class TestThreeLapSolvability:
    """The real car must complete 3 laps, inside the round time limit, on
    every Open Challenge layout.
    """

    def test_symmetric_wide_all_starts(self) -> None:
        failures = []
        for section, direction in product(_ALL_SECTIONS, _ALL_DIRECTIONS):
            meta = build_open_metadata(_uniform_widths(_WIDE_MM), section, direction)
            result = ScenarioSimulator(meta, num_laps=_N_LAPS).run()
            _log_result(f"WIDE  {section.capitalized:<5} {direction}", result)
            if not _within_round_limit(result):
                failures.append((section, direction, result))
        assert not failures, _describe(failures)

    def test_symmetric_narrow_all_starts(self) -> None:
        failures = []
        for section, direction in product(_ALL_SECTIONS, _ALL_DIRECTIONS):
            meta = build_open_metadata(_uniform_widths(_NARROW_MM), section, direction)
            result = ScenarioSimulator(meta, num_laps=_N_LAPS).run()
            _log_result(f"NARROW {section.capitalized:<5} {direction}", result)
            if not _within_round_limit(result):
                failures.append((section, direction, result))
        assert not failures, _describe(failures)

    @pytest.mark.parametrize(
        ("south", "north", "east", "west"),
        [
            (_NARROW_MM, _WIDE_MM, _NARROW_MM, _WIDE_MM),
            (_WIDE_MM, _NARROW_MM, _WIDE_MM, _NARROW_MM),
            (_NARROW_MM, _NARROW_MM, _WIDE_MM, _WIDE_MM),
            (_WIDE_MM, _WIDE_MM, _NARROW_MM, _NARROW_MM),
        ],
    )
    def test_mixed_width_combos(
        self, south: int, north: int, east: int, west: int,
    ) -> None:
        meta = build_open_metadata(
            {"south": south, "north": north, "east": east, "west": west},
            Section.SOUTH,
            Direction.CLOCKWISE,
        )
        result = ScenarioSimulator(meta, num_laps=_N_LAPS).run()
        _log_result(f"MIX S{south} N{north} E{east} W{west}", result)
        assert _within_round_limit(result), (
            f"laps={result.laps_completed}/{_N_LAPS} collided={result.collided} "
            f"timeout={result.timed_out} t={result.sim_time_s:.1f}s "
            f"(limit {CompetitionSpecs.ROUND_TIME_LIMIT_S:.0f}s) "
            f"at {result.collision_xy or result.final_pose}"
        )

    def test_random_scenarios(self) -> None:
        rng = np.random.default_rng(2026)
        failures = []
        for i in range(8):
            widths = {
                s: int(rng.choice([_NARROW_MM, _WIDE_MM]))
                for s in ("north", "south", "east", "west")
            }
            section = _ALL_SECTIONS[int(rng.integers(len(_ALL_SECTIONS)))]
            direction = _ALL_DIRECTIONS[int(rng.integers(len(_ALL_DIRECTIONS)))]
            meta = build_open_metadata(widths, section, direction, scenario_id=i)
            result = ScenarioSimulator(meta, num_laps=_N_LAPS, seed=i).run()
            _log_result(
                f"RAND#{i} {section.capitalized:<5} {direction} "
                f"S{widths['south']} N{widths['north']} E{widths['east']} W{widths['west']}",
                result,
            )
            if not _within_round_limit(result):
                failures.append((section, direction, result))
        assert not failures, _describe(failures)


def _describe(failures: list[tuple[Section, Direction, Any]]) -> str:
    lines = [
        f"  {sec.capitalized}/{dir_} -> laps={r.laps_completed}/{r.target_laps} "
        f"collided={r.collided} timeout={r.timed_out} t={r.sim_time_s:.1f}s "
        f"(limit {CompetitionSpecs.ROUND_TIME_LIMIT_S:.0f}s) "
        f"@={r.collision_xy or (round(r.final_pose[0], 2), round(r.final_pose[1], 2))}"
        for sec, dir_, r in failures
    ]
    return f"{len(failures)} scenario(s) failed to complete 3 laps:\n" + "\n".join(lines)

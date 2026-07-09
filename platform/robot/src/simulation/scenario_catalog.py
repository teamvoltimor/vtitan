"""Enumerate the exact scenarios ``TestThreeLapSolvability`` runs.

Mirrors ``tests/unit/test_open_challenge_sim.py`` scenario-by-scenario
(same widths, sections, directions, and RNG seeds) so visualizing "test N" in
RViz means watching the literal input pytest already validated — not an
approximation of it. The waypoint-clearance tests aren't included: they check
static planner output, never call ``ScenarioSimulator.run()``, and so have
nothing to animate.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from typing import Any

import numpy as np
from shared.config.constants import ColorNames, CompetitionSpecs, CorridorDimensions, TrackDimensions
from shared.config.enums import Direction, Section

from src.simulation.scenario_builder import (
    build_obstacles_metadata,
    build_open_metadata,
    parking_lot_dict,
    sign_world_pos,
    uniform_widths,
)

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
_ALL_SECTIONS = list(Section)
_ALL_DIRECTIONS = list(Direction)

# corridor_for_position() only unambiguously classifies a point into the
# corridor it visually sits in while its depth coordinate falls inside the
# inner block's own span [CORNER_MIN, CORNER_MAX] — outside that, both axes
# are simultaneously "outside the square" and it falls back to a nearest-face
# corner tie-break, which can silently reassign the sign to an ADJACENT
# corridor SignRouter never actually drives close to. The official WRO grid
# (test_sign_router.py's depth in {1.0, 1.5, 2.0}) always stays inside this
# same safe zone — demo placement must too, or "avoidance" silently no-ops.
_DEPTH_FRAC_MIN = TrackDimensions.CORNER_MIN / TrackDimensions.MAX_COORD
_DEPTH_FRAC_MAX = TrackDimensions.CORNER_MAX / TrackDimensions.MAX_COORD

_COLOR_COIN_FLIP_PROB = 0.5
"""Probability threshold for a fair red/green split in demo sign colours."""

_MIXED_COMBOS = [
    (_NARROW_MM, _WIDE_MM, _NARROW_MM, _WIDE_MM),
    (_WIDE_MM, _NARROW_MM, _WIDE_MM, _NARROW_MM),
    (_NARROW_MM, _NARROW_MM, _WIDE_MM, _WIDE_MM),
    (_WIDE_MM, _WIDE_MM, _NARROW_MM, _NARROW_MM),
]


@dataclass(frozen=True, slots=True)
class NamedScenario:
    """One (metadata, laps, seed) input, labeled after the test it mirrors."""

    label: str
    metadata: dict[str, Any]
    laps: int
    seed: int


def all_test_scenarios() -> list[NamedScenario]:
    """The 28 closed-loop scenarios the three ``TestThreeLapSolvability`` tests run."""
    scenarios: list[NamedScenario] = []

    for section, direction in product(_ALL_SECTIONS, _ALL_DIRECTIONS):
        meta = build_open_metadata(uniform_widths(_WIDE_MM), section, direction)
        scenarios.append(NamedScenario(
            f"symmetric_wide[{section.capitalized}/{direction}]", meta, _N_LAPS, seed=0,
        ))

    for section, direction in product(_ALL_SECTIONS, _ALL_DIRECTIONS):
        meta = build_open_metadata(uniform_widths(_NARROW_MM), section, direction)
        scenarios.append(NamedScenario(
            f"symmetric_narrow[{section.capitalized}/{direction}]", meta, _N_LAPS, seed=0,
        ))

    for south, north, east, west in _MIXED_COMBOS:
        meta = build_open_metadata(
            {"south": south, "north": north, "east": east, "west": west},
            Section.SOUTH, Direction.CLOCKWISE,
        )
        scenarios.append(NamedScenario(
            f"mixed[S{south}-N{north}-E{east}-W{west}]", meta, _N_LAPS, seed=0,
        ))

    rng = np.random.default_rng(2026)
    for i in range(8):
        widths = {s: int(rng.choice([_NARROW_MM, _WIDE_MM])) for s in ("north", "south", "east", "west")}
        section = _ALL_SECTIONS[int(rng.integers(len(_ALL_SECTIONS)))]
        direction = _ALL_DIRECTIONS[int(rng.integers(len(_ALL_DIRECTIONS)))]
        meta = build_open_metadata(widths, section, direction, scenario_id=i)
        scenarios.append(NamedScenario(
            f"random#{i}[{section.capitalized}/{direction} "
            f"S{widths['south']}-N{widths['north']}-E{widths['east']}-W{widths['west']}]",
            meta, _N_LAPS, seed=i,
        ))

    return scenarios


def all_obstacles_demo_scenarios() -> list[NamedScenario]:
    """Representative Obstacles Challenge scenarios (sign routing + parking).

    Unlike :func:`all_test_scenarios`, there is no closed-loop obstacles test
    battery to mirror 1:1 (``test_sign_router.py``/``test_parking.py`` only
    unit-test ``SignRouter``/``ParkController`` in isolation) — these are
    seeded demo layouts for visualization, not pytest-validated inputs.
    """
    scenarios: list[NamedScenario] = []
    rng = np.random.default_rng(2026)

    for section, direction in product(_ALL_SECTIONS, _ALL_DIRECTIONS):
        widths_mm = uniform_widths(_WIDE_MM)
        widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
        signs = []
        for _ in range(2):
            sign_section = _ALL_SECTIONS[int(rng.integers(len(_ALL_SECTIONS)))]
            depth_frac = float(rng.uniform(_DEPTH_FRAC_MIN, _DEPTH_FRAC_MAX))
            lane_frac = float(rng.uniform(0.3, 0.7))
            color = ColorNames.RED if rng.random() < _COLOR_COIN_FLIP_PROB else ColorNames.GREEN
            x, y = sign_world_pos(sign_section, depth_frac, lane_frac, widths_m)
            signs.append({"x": x, "y": y, "color": color})
        meta = build_obstacles_metadata(widths_mm, section, direction, signs)
        scenarios.append(NamedScenario(
            f"obstacles_demo[{section.capitalized}/{direction}]", meta, _N_LAPS, seed=0,
        ))

    for section in _ALL_SECTIONS:
        widths_mm = uniform_widths(_WIDE_MM)
        widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
        x, y = sign_world_pos(section, depth_frac=0.5, lane_frac=0.5, widths_m=widths_m)
        signs = [{"x": x, "y": y, "color": ColorNames.GREEN}]
        # Parking lot must sit in the same corridor the scenario starts in — the robot
        # only engages parking once it's driving through ``ParkController.section``,
        # which it's guaranteed to revisit there after the final lap.
        meta = build_obstacles_metadata(
            widths_mm, section, Direction.CLOCKWISE, signs, parking_lot=parking_lot_dict(section),
        )
        scenarios.append(NamedScenario(
            f"obstacles_demo_parking[{section.capitalized}]", meta, _N_LAPS, seed=0,
        ))

    return scenarios


def find_scenario(selector: str, scenarios: list[NamedScenario]) -> NamedScenario:
    """Resolve a CLI selector to one scenario: an index, or a label substring."""
    if selector.isdigit():
        idx = int(selector)
        if 0 <= idx < len(scenarios):
            return scenarios[idx]
        msg = f"Scenario index {idx} out of range (0-{len(scenarios) - 1})."
        raise ValueError(msg)
    matches = [s for s in scenarios if selector.lower() in s.label.lower()]
    if not matches:
        msg = f"No scenario label contains {selector!r}."
        raise ValueError(msg)
    if len(matches) > 1:
        labels = ", ".join(m.label for m in matches)
        msg = f"{selector!r} matches multiple scenarios: {labels}"
        raise ValueError(msg)
    return matches[0]

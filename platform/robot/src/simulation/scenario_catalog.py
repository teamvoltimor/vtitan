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
from shared.config.constants import CompetitionSpecs, CorridorDimensions
from shared.config.enums import Direction, Section

from src.simulation.scenario_builder import build_open_metadata, uniform_widths

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
_ALL_SECTIONS = list(Section)
_ALL_DIRECTIONS = list(Direction)

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


def find_scenario(selector: str, scenarios: list[NamedScenario]) -> NamedScenario:
    """Resolve a CLI selector to one scenario: an index, or a label substring."""
    if selector.isdigit():
        idx = int(selector)
        if 0 <= idx < len(scenarios):
            return scenarios[idx]
        raise ValueError(f"Scenario index {idx} out of range (0-{len(scenarios) - 1}).")
    matches = [s for s in scenarios if selector.lower() in s.label.lower()]
    if not matches:
        raise ValueError(f"No scenario label contains {selector!r}.")
    if len(matches) > 1:
        labels = ", ".join(m.label for m in matches)
        raise ValueError(f"{selector!r} matches multiple scenarios: {labels}")
    return matches[0]

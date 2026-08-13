"""Find the max pose disturbance the navigator can recover from — a reference number.

Unlike ``tests/unit/test_deviation_recovery.py`` (a pass/fail regression guard
pinned well below what the car can actually take), this binary-searches the
largest lateral offset / heading error the real ``CoreNavigator`` still steers
back onto the path from, per representative corridor layout. Not a pytest test
— it's slow-ish (binary search x several scenarios x both axes) and its output
is a report to read, not a gate to enforce.

Run with:
    pixi run -e dev deviation-envelope
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions
from shared.domain.enums import Direction, Section

from src.navigation.track_geometry import cross_track_error
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_constants import N_LAPS, NARROW_MM, WIDE_MM
from src.simulation.scenario_result import PoseDisturbance
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from src.simulation.kinematics import AckermannState

logger = logging.getLogger(__name__)

_DISTURB_AT_STEP = 150
_RECOVERY_WINDOW_STEPS = 150
_RECOVERY_THRESHOLD_M = 0.05
_BISECTION_ITERATIONS = 10
"""10 halvings of the search range -> ~1mm (lateral) / ~0.05 deg (heading) resolution."""

_LATERAL_SEARCH_HI_M = min(CorridorDimensions.NARROW, CorridorDimensions.WIDE) / 2
"""Never search past half the narrowest corridor — a bigger kick starts inside a wall."""

_HEADING_SEARCH_HI_RAD = math.pi / 2


@dataclass(frozen=True, slots=True)
class _Scenario:
    label: str
    widths_mm: dict[str, int]
    section: Section
    direction: Direction


_SCENARIOS = [
    _Scenario("wide/cw", uniform_widths(WIDE_MM), Section.SOUTH, Direction.CLOCKWISE),
    _Scenario("wide/ccw", uniform_widths(WIDE_MM), Section.SOUTH, Direction.COUNTERCLOCKWISE),
    _Scenario("narrow/cw", uniform_widths(NARROW_MM), Section.SOUTH, Direction.CLOCKWISE),
    _Scenario("narrow/ccw", uniform_widths(NARROW_MM), Section.SOUTH, Direction.COUNTERCLOCKWISE),
]


def _recovers(scenario: _Scenario, disturbance: PoseDisturbance) -> bool:
    meta = build_open_metadata(scenario.widths_mm, scenario.section, scenario.direction)
    sim = ScenarioSimulator(meta, num_laps=N_LAPS)
    trace: list[float] = []

    def on_step(state: AckermannState, _scan: object) -> None:
        trace.append(cross_track_error(sim.waypoints, state.x, state.y))

    result = sim.run(on_step=on_step, disturb_at_step=_DISTURB_AT_STEP, disturbance=disturbance)
    if result.collided or not result.success:
        return False
    window = trace[_DISTURB_AT_STEP - 1 : _DISTURB_AT_STEP - 1 + _RECOVERY_WINDOW_STEPS]
    return any(e <= _RECOVERY_THRESHOLD_M for e in window)


def _max_recoverable(scenario: _Scenario, axis: str, search_hi: float) -> float:
    """Binary-search the max magnitude on ``axis`` ("lateral"/"heading") that still recovers."""

    def disturbance_at(magnitude: float) -> PoseDisturbance:
        if axis == "lateral":
            return PoseDisturbance(lateral_m=magnitude)
        return PoseDisturbance(lateral_m=0.0, heading_rad=magnitude)

    if not _recovers(scenario, disturbance_at(0.0)):
        return 0.0  # Even a near-zero kick doesn't recover — shouldn't happen, but don't lie.

    lo, hi = 0.0, search_hi
    for _ in range(_BISECTION_ITERATIONS):
        mid = (lo + hi) / 2
        if _recovers(scenario, disturbance_at(mid)):
            lo = mid
        else:
            hi = mid
    return lo


def main() -> None:
    """Entry point for `python -m src.simulation.find_recovery_envelope`."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    logger.info(
        "Recovery envelope: disturb at step %d, must recover within %d steps (cross-track error <= %.2fm).",
        _DISTURB_AT_STEP,
        _RECOVERY_WINDOW_STEPS,
        _RECOVERY_THRESHOLD_M,
    )
    logger.info("%-12s %-10s %s", "scenario", "axis", "max recoverable")
    for scenario in _SCENARIOS:
        lateral_max = _max_recoverable(scenario, "lateral", _LATERAL_SEARCH_HI_M)
        logger.info("%-12s %-10s %.3f m", scenario.label, "lateral", lateral_max)

        heading_max = _max_recoverable(scenario, "heading", _HEADING_SEARCH_HI_RAD)
        logger.info("%-12s %-10s %.1f deg", scenario.label, "heading", math.degrees(heading_max))


if __name__ == "__main__":
    main()

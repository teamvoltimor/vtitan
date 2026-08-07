"""Show the scans that vote a travel direction into place.

``diag_open_direction.py`` reports whether a fixture settles on the right
direction; this shows the evidence behind it. ``infer_direction`` decides on
the left/right span, so each accepted vote is printed with the pose and the two
side ranges that produced it.

Repeated identical side ranges across consecutive steps are the tell that a
single cached scan is being voted more than once -- scans arrive at half the
control rate, so ``min_votes`` can be satisfied by one reading.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/diag_open_direction_votes.py go_open_0012
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.navigation import direction_estimator as de
from src.navigation.utils import _nearest_ray
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.enums import Direction

_MAX_VOTES_SHOWN = 12
_DEFAULT_MAX_STEPS = 600
_DEFAULT_SCENARIOS = ("go_open_0010", "go_open_0012", "go_open_0013")


class _VoteTracer:
    """Records every accepted direction vote, with the scan behind it."""

    def __init__(self) -> None:
        self.votes: list[str] = []
        self.pos: tuple[float, float] = (0.0, 0.0)

    def patch(self) -> None:
        """Wrap ``infer_direction`` where the estimator resolves it."""
        self._real = de.infer_direction
        tracer = self

        def traced(
            ranges_m: Sequence[float],
            angles_rad: Sequence[float],
            yaw: float,
        ) -> Direction | None:
            result = tracer._real(ranges_m, angles_rad, yaw)
            if result is not None:
                left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
                right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
                tracer.votes.append(
                    f"  vote={result.value:<17} pos=({tracer.pos[0]:.2f},{tracer.pos[1]:.2f}) "
                    f"yaw={yaw:+.2f} left={left:.2f} right={right:.2f} span={left + right:.2f}"
                )
            return result

        de.infer_direction = traced
        # DirectionEstimator.observe resolved the name at import time.
        de.DirectionEstimator.observe.__globals__["infer_direction"] = traced

    def unpatch(self) -> None:
        """Restore the real ``infer_direction``."""
        de.infer_direction = self._real
        de.DirectionEstimator.observe.__globals__["infer_direction"] = self._real


def _report(scenario: Any, max_steps: int) -> None:
    """Run one fixture blind and print the votes it accepted."""
    tracer = _VoteTracer()
    tracer.patch()
    try:
        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )

        def on_step(state: Any, _scan: Any) -> None:
            tracer.pos = (state.x, state.y)

        sim.run(max_steps=max_steps, on_step=on_step)
    finally:
        tracer.unpatch()

    truth = scenario.metadata["starting_conditions"]["direction"]
    print(f"\n{scenario.label} truth={truth}")
    for line in tracer.votes[:_MAX_VOTES_SHOWN]:
        print(line)


def main() -> None:
    """Print every accepted direction vote for the fixtures named on argv."""
    wanted = sys.argv[1:] or list(_DEFAULT_SCENARIOS)
    for scenario in all_test_scenarios():
        if any(w in scenario.label for w in wanted):
            _report(scenario, max_steps=_DEFAULT_MAX_STEPS)


if __name__ == "__main__":
    main()

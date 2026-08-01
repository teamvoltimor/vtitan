"""Trace the blind startup creep on fixtures that collide during it.

A blind run that hits a wall before direction inference settles never reaches
the navigator -- ``follow_corridor`` is steering. This reports whether the
estimator settled at all, and the pose/steer trace over the last second before
contact, so the creep's own behaviour can be read directly.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/diag_open_creep_crash.py [label ...]
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_TRACE_TICKS = 20


class _CreepTracer:
    """Records the pose/steer trace and when the direction settled."""

    def __init__(self, simulator: ScenarioSimulator) -> None:
        self._simulator = simulator
        self.trace: list[str] = []
        self.settled_at: int | None = None
        self.step = 0

    def on_step(self, state: Any, scan: Any) -> None:
        """Record one tick of the run."""
        self.step += 1
        estimator = self._simulator.direction_estimator
        if estimator is not None and estimator.is_settled and self.settled_at is None:
            self.settled_at = self.step
        nearest = min((r for r in scan.ranges_m if r > 0.0), default=0.0)
        self.trace.append(
            f"  step{self.step:>4} pos=({state.x:.2f},{state.y:.2f}) yaw={state.yaw:+.2f} "
            f"v={state.v:+.2f} steer={state.steer:+.2f} minLIDAR={nearest:.2f}"
        )


def _report(scenario: Any) -> None:
    """Run one fixture blind and print the trace leading into its collision."""
    sim = ScenarioSimulator(
        scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
    )
    tracer = _CreepTracer(sim)
    result = sim.run(on_step=tracer.on_step)

    widths = " ".join(
        f"{s[0].upper()}{scenario.metadata['corridor_widths'][s]['width_mm']}"
        for s in ("south", "north", "east", "west")
    )
    believed = {s.value: round(w, 3) for s, w in (sim.believed_widths or {}).items()}
    print(
        f"\n{scenario.label} {widths} collided={result.collided} "
        f"at={result.collision_xy} steps={result.steps} "
        f"settled_at={tracer.settled_at or 'NEVER'} believed={believed}"
    )
    for line in tracer.trace[-_TRACE_TICKS:]:
        print(line)


def main() -> None:
    """Print the creep trace leading into each named fixture's collision."""
    wanted = sys.argv[1:] or ["go_open_0012", "go_open_0020"]
    for scenario in all_test_scenarios():
        if any(w in scenario.label for w in wanted):
            _report(scenario)


if __name__ == "__main__":
    main()

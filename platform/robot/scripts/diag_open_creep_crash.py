"""Trace the blind startup creep on the fixtures that collide during it.

Both surviving blind failures hit a wall before direction inference settles,
so the navigator never steers -- ``follow_corridor`` does. This reports whether
the estimator ever settled, and the pose/steer trace over the final second
before contact, so the creep's own steering can be read directly.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/diag_open_creep_crash.py [label ...]
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator


def main() -> None:
    """Print the creep trace leading into each fixture's collision."""
    wanted = sys.argv[1:] or ["go_open_0012", "go_open_0020"]

    for scenario in all_test_scenarios():
        if not any(w in scenario.label for w in wanted):
            continue

        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )
        trace: list[str] = []
        settled_at: list[int] = []
        step = {"n": 0}
        estimator = sim._direction_estimator

        def on_step(state, scan) -> None:
            step["n"] += 1
            if estimator is not None and estimator.is_settled and not settled_at:
                settled_at.append(step["n"])
            trace.append(
                f"  step{step['n']:>4} pos=({state.x:.2f},{state.y:.2f}) yaw={state.yaw:+.2f} "
                f"v={state.v:+.2f} steer={state.steer:+.2f} minLIDAR={min(r for r in scan.ranges_m if r > 0.0):.2f}"
            )

        result = sim.run(on_step=on_step)

        widths = {
            s[0].upper(): scenario.metadata["corridor_widths"][s]["width_mm"]
            for s in ("south", "north", "east", "west")
        }
        print(
            f"\n{scenario.label} {' '.join(f'{k}{v}' for k, v in widths.items())} "
            f"collided={result.collided} at={result.collision_xy} "
            f"steps={result.steps} settled_at={settled_at[0] if settled_at else 'NEVER'} "
            f"believed={ {s.value: round(w, 3) for s, w in (sim.believed_widths or {}).items()} }"
        )
        for line in trace[-20:]:
            print(line)


if __name__ == "__main__":
    main()

"""One table of the 16 Obstacles fixtures across the switches that change them.

The suite reports a scenario as failed without saying which of its three
independent variables did it: parking (a manoeuvre that runs only after the
laps are already won), the vision path (colour comes from the emulated camera
rather than the metadata), and the sign routing itself. A run that collides
with park on and completes with it off is not a routing failure, and reading
one number for all three hides that.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/sim/diag_obstacles_matrix.py [--laps 3]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.formats import TIME_FORMAT as _TIME_FORMAT
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

MAX_STEPS = 6000
_LABEL_WIDTH = 12
_SCENARIO_WIDTH = 40
_OUTCOME_WIDTH = 9

COMBOS: tuple[tuple[str, bool, bool], ...] = (
    ("laps-only", False, False),
    ("laps+vision", False, True),
    ("laps+park", True, False),
)
"""(label, park, emit_vision_detections) -- laps alone first, then one switch at a time."""


def main() -> None:
    """Run every fixture under each combo and print one line per run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=None, help="override each fixture's lap target")
    args = parser.parse_args()

    scenarios = all_obstacles_demo_scenarios()
    for label, park, vision in COMBOS:
        outcomes = []
        for scenario in scenarios:
            laps = args.laps if args.laps is not None else scenario.laps
            result = ScenarioSimulator(
                scenario.metadata,
                num_laps=laps,
                seed=scenario.seed,
                park=park,
                emit_vision_detections=vision,
            ).run(max_steps=MAX_STEPS)
            ok = not result.collided and result.laps_completed >= laps
            outcomes.append(ok)
            why = "ok" if ok else ("collided" if result.collided else "short")
            print(
                f"{label:<{_LABEL_WIDTH}} {scenario.label:<{_SCENARIO_WIDTH}} {why:<{_OUTCOME_WIDTH}} "
                f"laps={result.laps_completed}/{laps} t={result.sim_time_s:{_TIME_FORMAT}}s "
                f"surface={result.terminal_surface.value}",
                flush=True,
            )
        print(f"{label:<{_LABEL_WIDTH}} TOTAL {sum(outcomes)}/{len(outcomes)}", flush=True)


if __name__ == "__main__":
    main()

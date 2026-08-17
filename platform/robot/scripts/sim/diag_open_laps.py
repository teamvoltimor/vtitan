"""Audit blind-mode lap credits against the distance actually driven.

``diag_open_fixtures.py`` reports ``laps=3/3`` without showing *when* each lap
was credited. A lap credited a few steps after the previous one is not a lap
the robot drove -- it is a bookkeeping artifact. This prints the step index and
the path distance between consecutive lap credits, so a spurious credit shows
up as a near-zero inter-lap distance.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/sim/diag_open_laps.py [--blind] [--only go_open_0013]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.formats import DISTANCE_FORMAT as _DISTANCE_FORMAT
from scripts.common.formats import TIME_FORMAT as _TIME_FORMAT
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator


def _report(scenario: object, blind: bool) -> None:
    """Run one fixture and print its per-lap step and distance splits."""
    # Sample the path at every tick so inter-lap distance can be integrated
    # between the recorded lap step indices.
    track: list[tuple[float, float]] = []
    sim = ScenarioSimulator(
        scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=blind
    )
    result = sim.run(on_step=lambda state, _scan: track.append((state.x, state.y)))

    cumulative = [0.0]
    for i in range(1, len(track)):
        cumulative.append(
            cumulative[-1] + math.hypot(track[i][0] - track[i - 1][0], track[i][1] - track[i - 1][1])
        )

    splits = []
    prev_step = 0
    for lap_step in result.lap_step_indices:
        # on_step samples lag the loop's step counter by one on creep ticks.
        a = min(prev_step, len(cumulative) - 1)
        b = min(lap_step, len(cumulative) - 1)
        splits.append(f"lap@step{lap_step}(+{cumulative[b] - cumulative[a]:{_DISTANCE_FORMAT}}m)")
        prev_step = lap_step

    print(
        f"{'OK  ' if result.success else 'FAIL'} {scenario.label} "
        f"laps={result.laps_completed}/{result.target_laps} "
        f"collided={result.collided} dist={result.distance_m:{_DISTANCE_FORMAT}}m "
        f"t={result.sim_time_s:{_TIME_FORMAT}}s{' OVER-TIME' if result.over_time else ''} | "
        + " ".join(splits)
    )


def main() -> None:
    """Print per-lap step and distance splits for each Open Challenge fixture."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blind", action="store_true")
    parser.add_argument("--only", default=None, help="Run a single fixture label.")
    args = parser.parse_args()

    for scenario in all_test_scenarios():
        if args.only and args.only not in scenario.label:
            continue
        _report(scenario, args.blind)


if __name__ == "__main__":
    main()

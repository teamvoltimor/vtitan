"""Run the closed-loop Open Challenge sim over every Go-generated fixture.

``tests/unit/test_open_challenge_sim.py`` builds its own metadata via
``build_open_metadata``, so it never exercises the 28 ``simgen`` fixtures in
``tests/fixtures/scenarios/open/``. Those are only checked for their count by
``test_scenario_catalog.py``. This walks all of them through the real
``CoreNavigator`` and reports laps, collisions and speed per scenario.

``--blind`` withholds the layout and the travel direction, which is the
condition the real round runs under: the inner walls are randomised and the
direction is drawn on the day, so a sighted run measures a robot with
information no robot has.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/sim/diag_open_fixtures.py [--blind]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.formats import DISTANCE_FORMAT as _DISTANCE_FORMAT
from scripts.common.formats import TIME_FORMAT as _TIME_FORMAT
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_SPEED_FORMAT = ".2f"


def main() -> None:
    """Report the outcome of a full run for each Open Challenge fixture."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--blind",
        action="store_true",
        help="Estimate corridor widths and travel direction from LIDAR instead of metadata.",
    )
    args = parser.parse_args()

    failures = 0
    for scenario in all_test_scenarios():
        widths = {
            s[0].upper(): scenario.metadata["corridor_widths"][s]["width_mm"]
            for s in ("south", "north", "east", "west")
        }
        result = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=args.blind
        ).run()
        ok = (
            result.laps_completed >= result.target_laps
            and not result.collided
            and not result.timed_out
        )
        failures += not ok
        widths_str = " ".join(f"{k}{v}" for k, v in widths.items())
        print(
            f"{'OK ' if ok else 'FAIL'} | {scenario.label} {widths_str} "
            f"laps={result.laps_completed}/{result.target_laps} "
            f"collided={result.collided} timeout={result.timed_out} | "
            f"dist={result.distance_m:{_DISTANCE_FORMAT}}m t={result.sim_time_s:{_TIME_FORMAT}}s "
            f"vmax={result.max_speed_mps:{_SPEED_FORMAT}} vavg={result.avg_speed_mps:{_SPEED_FORMAT}} "
            f"minLIDAR={result.min_lidar_range_m:{_DISTANCE_FORMAT}}m contacts={result.contact_count}"
        )

    total = len(all_test_scenarios())
    print(f"\n{total - failures}/{total} fixtures completed cleanly")


if __name__ == "__main__":
    main()

r"""Score the IN-BAY start the way the hardware runs it: does the car get out and lap?

`obstacles_start_in_bay` places the simulated chassis inside the parking lot, as
the rules allow and as the real robot always starts on Obstacles. It ships off
because every fixture dies there, and that is the gate for measuring anything
about the lot: with the car starting mid-corridor the simulator never comes
within 0.42 m of the fins, while the hardware runs 0.28-0.34 m from them.

Measured cause, 2026-09-17: the bay IS recognised (``direction_from_parking_bay``
answers on the first scan) and the exit manoeuvre DOES run. Then the clearance
guard refuses BOTH legs and returns speed 0.0 on every tick, flipping the
steering back and forth, while the chassis coasts the last 6.4 cm into a fin.
Touching a fin ends the round (rule 9.24.7), so the round is over in 34 steps
having travelled 6 cm. The guard's own frame is the suspect: ``_dr_out`` ends at
0.4 mm while the IMU says the chassis turned 65-73 degrees.

This prints one line per fixture and a summary line the sweep greps, so an arm
is judged on LAPS OUT OF THE BAY rather than on the corpus suite, which cannot
see this configuration at all.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e dev python scripts/sim/diag_bay_exit_start.py [--limit 8]
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.navigation_tuning import NavigationTuning

from scripts.common.diag_base import resolve_jobs, run_pool
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

logging.disable(logging.CRITICAL)


def run_case(payload: tuple[int, int]) -> tuple[str, int, bool, bool, bool, int, float]:
    """Run one fixture from inside the bay. Module-level for ProcessPoolExecutor."""
    index, max_steps = payload
    scenario = all_obstacles_demo_scenarios()[index]
    base = NavigationTuning.load_default()
    tuning = replace(base, simulation=base.simulation.model_copy(update={"obstacles_start_in_bay": True}))
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, tuning=tuning)
    start = (sim._gateway.state.x, sim._gateway.state.y)  # noqa: SLF001  (diagnostic)
    result = sim.run(max_steps=max_steps)
    end = (sim._gateway.state.x, sim._gateway.state.y)  # noqa: SLF001  (diagnostic)
    travelled = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
    return (
        scenario.label,
        result.laps_completed,
        result.collided,
        result.timed_out,
        result.stuck,
        result.steps,
        travelled,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--limit", type=int, default=0, help="Fixtures to run; 0 means all.")
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    count = len(all_obstacles_demo_scenarios())
    indices = list(range(count if not args.limit else min(args.limit, count)))
    rows = run_pool(run_case, [(i, args.max_steps) for i in indices], resolve_jobs(args.jobs))

    escaped = 0
    for label, laps, collided, timed_out, stuck, steps, travelled in sorted(rows):
        # "Escaped" is deliberately weaker than "scored": 0.5 m of travel means
        # the chassis is out of a 0.45 m pocket, which is the thing being fixed.
        # A fixture can escape and still lose the round later, and that is
        # progress worth seeing separately from laps.
        out = travelled > 0.5
        escaped += out
        print(
            f"{label:46s} laps={laps} collided={str(collided):5s} timeout={str(timed_out):5s} "
            f"stuck={str(stuck):5s} steps={steps:5d} travelled={travelled:.2f} m escaped={out}"
        )
    laps3 = sum(1 for _l, laps, *_r in rows if laps >= 3)
    clean = sum(1 for _l, _laps, collided, *_r in rows if not collided)
    print(f"SUMMARY escaped={escaped}/{len(rows)} no_collision={clean}/{len(rows)} laps3={laps3}/{len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

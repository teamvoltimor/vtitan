"""Score the blind direction estimator against ground truth on every fixture.

In blind mode the provisional direction is seeded from the fixture metadata
(``scenario_simulator`` line 314), so the estimator only *replaces* it when it
disagrees -- meaning every observed "flip" is a misinference, not a correction.
This reports settled-vs-truth per fixture so the error rate is measured rather
than inferred from the handful of fixtures that happened to misbehave.

Only the creep matters here, so the run is cut short well before a full race.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/sim/diag_open_direction.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_DEFAULT_CREEP_STEPS = 1400
"""Long enough for the slowest fixture to settle.

Not a tuning knob -- a fixture that has not settled by here has not settled.
Measured before the dropout filter, go_open_0021 needed past step 400, so a
shorter budget reported it as "never settles" when it merely settles late.
"""
_VERDICT_WIDTH = 9


def main() -> None:
    """Report inferred vs true travel direction for every Open fixture."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--creep-steps", type=int, default=_DEFAULT_CREEP_STEPS)
    args = parser.parse_args()

    wrong = 0
    unsettled = 0

    for scenario in all_test_scenarios():
        truth = scenario.metadata["starting_conditions"]["direction"]
        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )
        sim.run(max_steps=args.creep_steps)
        estimator = sim.direction_estimator
        settled = estimator is not None and estimator.is_settled
        inferred = estimator.direction.value if (settled and estimator.direction) else None

        if not settled:
            verdict, unsettled = "UNSETTLED", unsettled + 1
        elif inferred != truth:
            verdict, wrong = "WRONG", wrong + 1
        else:
            verdict = "ok"

        print(f"{verdict:>{_VERDICT_WIDTH}} | {scenario.label} truth={truth} inferred={inferred}")

    total = len(all_test_scenarios())
    print(f"\n{total - wrong - unsettled}/{total} correct, {wrong} wrong, {unsettled} unsettled")


if __name__ == "__main__":
    main()

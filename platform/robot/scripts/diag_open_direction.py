"""Score the blind direction estimator against ground truth on every fixture.

In blind mode the provisional direction is seeded from the fixture metadata
(``scenario_simulator`` line 314), so the estimator only *replaces* it when it
disagrees -- meaning every observed "flip" is a misinference, not a correction.
This reports settled-vs-truth per fixture so the error rate is measured rather
than inferred from the handful of fixtures that happened to misbehave.

Only the creep matters here, so the run is cut short well before a full race.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/diag_open_direction.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_CREEP_STEPS = 400


def main() -> None:
    """Report inferred vs true travel direction for every Open fixture."""
    wrong = 0
    unsettled = 0

    for scenario in all_test_scenarios():
        truth = scenario.metadata["starting_conditions"]["direction"]
        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )
        sim.run(max_steps=_CREEP_STEPS)
        estimator = sim._direction_estimator
        settled = estimator is not None and estimator.is_settled
        inferred = estimator.direction.value if (settled and estimator.direction) else None

        if not settled:
            verdict, unsettled = "UNSETTLED", unsettled + 1
        elif inferred != truth:
            verdict, wrong = "WRONG", wrong + 1
        else:
            verdict = "ok"

        print(f"{verdict:>9} | {scenario.label} truth={truth} inferred={inferred}")

    total = len(all_test_scenarios())
    print(f"\n{total - wrong - unsettled}/{total} correct, {wrong} wrong, {unsettled} unsettled")


if __name__ == "__main__":
    main()

r"""How many ticks does the side-correction refusal actually decide?

``obstacles_side_correction_follows_committed_sign`` costs the obstacles corpus
12 -> 15, and all three formulations tried so far lose ``go_obstacles_0004``.
Before that is read as "the mechanism fails", it is worth knowing how much
evidence the number rests on, because ``escape.toml`` already records that this
simulator exercises ``side_correction`` on **1.09% of ticks against hardware's
19-25%**. A verdict resting on a handful of ticks in a regime the simulator
renders 20x less often is weak evidence either way.

So this counts, per scenario and per arm:

* SIDE_CORRECTION manoeuvres issued at all,
* of those, how many the refusal actually changed (the router's committed side
  and the threat on the SAME flank),
* where the run ended, and whether it collided.

Read the REFUSALS column first. If a scenario flips from pass to fail on a
single-digit refusal count, the corpus is not adjudicating the mechanism; it is
adjudicating one manoeuvre, and a hardware round is the only instrument left.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/sim/diag_side_correction_refusal_weight.py \
        --only 0004 0006
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from collections.abc import Callable
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import ManeuverType

from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning
from src.navigation.control.controllers.collision_avoidance.controller import (
    CollisionAvoidanceController,
)
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator


def _instrumented(counts: dict[str, int]) -> tuple[Callable, Callable]:
    """Wrap the controller so every manoeuvre and every refusal is counted.

    Patching the class rather than threading a counter through the simulator
    keeps this script out of production code: the flag being measured is read
    inside ``_side_correction_steer_sign``, so that is the only honest place to
    observe whether it fired.
    """
    original_sign = CollisionAvoidanceController._side_correction_steer_sign  # noqa: SLF001
    original_escape = CollisionAvoidanceController.compute_escape_maneuver

    def counting_sign(
        self: CollisionAvoidanceController,
        away_sign: float,
        threat_is_left: bool,
        preferred_sign: float | None,
    ) -> tuple[float, bool]:
        steer, refused = original_sign(self, away_sign, threat_is_left, preferred_sign)
        if refused:
            counts["refusals"] += 1
        return steer, refused

    def counting_escape(self: CollisionAvoidanceController, *args: object, **kwargs: object) -> object:
        maneuver = original_escape(self, *args, **kwargs)
        if maneuver is not None and maneuver.maneuver_type is ManeuverType.SIDE_CORRECTION:
            counts["side_corrections"] += 1
        return maneuver

    CollisionAvoidanceController._side_correction_steer_sign = counting_sign  # noqa: SLF001
    CollisionAvoidanceController.compute_escape_maneuver = counting_escape
    return original_sign, original_escape


def _restore(original_sign: Callable, original_escape: Callable) -> None:
    CollisionAvoidanceController._side_correction_steer_sign = original_sign  # noqa: SLF001
    CollisionAvoidanceController.compute_escape_maneuver = original_escape


def main() -> int:
    """Count refusals per scenario, with the flag off and on."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", nargs="*", default=None, help="fixture id fragments, e.g. 0004")
    parser.add_argument("--vision", action="store_true", help="emit vision detections (the arm that failed)")
    args = parser.parse_args()

    scenarios = [
        s for s in all_obstacles_demo_scenarios() if args.only is None or any(frag in s.label for frag in args.only)
    ]
    if not scenarios:
        print("No fixture matched --only. Nothing to conclude.")
        return 2

    rows = []
    for follows in (False, True):
        for scenario in scenarios:
            tuning = get_tuning(None)
            escape = tuning.escape.model_copy(
                update={
                    "side_correction_follows_committed_sign": follows,
                    "obstacles_side_correction_follows_committed_sign": follows,
                }
            )
            counts = {"side_corrections": 0, "refusals": 0}
            originals = _instrumented(counts)
            try:
                result = ScenarioSimulator(
                    scenario.metadata,
                    num_laps=scenario.laps,
                    seed=scenario.seed,
                    park=False,
                    emit_vision_detections=args.vision,
                    tuning=dataclasses.replace(tuning, escape=escape),
                ).run(max_steps=OBSTACLES_MAX_STEPS)
            finally:
                _restore(*originals)

            rows.append(
                [
                    scenario.label,
                    "on" if follows else "off",
                    counts["side_corrections"],
                    counts["refusals"],
                    "COLLIDED" if result.collided else "ok",
                    f"{result.laps_completed}/{scenario.laps}",
                    f"{result.sim_time_s:.1f}s",
                ]
            )

    print_table(rows, ["scenario", "flag", "side corr", "REFUSALS", "outcome", "laps", "time"])
    print()
    print(
        "Read REFUSALS first. A scenario that flips on a single-digit count is not "
        "evidence about the mechanism; escape.toml records this simulator running "
        "side_correction on 1.09% of ticks against hardware's 19-25%."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

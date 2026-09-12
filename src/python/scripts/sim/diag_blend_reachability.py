"""Is ``SIDE_CORRECTION_BLENDS`` even REACHABLE in the sim corpus?

A 256-run A/B of the flag came back flat (in_time 98 vs 98, laps>=3 99 vs 99,
collisions 16 vs 15) with ``pass_side`` violations at 0 in BOTH arms. A flat
result has two readings and only one of them is a decision:

  * the flag genuinely does nothing, or
  * the code path it gates is never taken in the corpus, so the sweep compared
    two identical worlds and measured its own noise.

This repo has been burned by the second before: a whole steering axis was swept
and the arms came back byte-identical because the wiring was dead. So this
counts, per tick, how often the gate could fire at all:

  * SIDE_CORRECTION ticks -- the manoeuvre type the flag is about.
  * BLENDABLE ticks       -- those of them that also satisfy the rest of the
                             gate (forward speed), i.e. ticks where flipping
                             the flag actually changes which branch runs.

If BLENDABLE is ~0 the sweep is void and the question has to move to the bags,
where side_correction holds the wheel for 19-25% of ticks. The control for that
claim is the hardware number, which is why it is quoted here rather than left
implicit.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_blend_reachability.py --scenarios 6 --seeds 2
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from shared.domain.enums import ManeuverType
from shared.domain.models import ScenarioMetadata

from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.config.tuning_helpers import tuning_with_overrides
from src.simulation.scenario_simulator import ScenarioSimulator

logging.disable(logging.CRITICAL)

_FIXTURES = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"

HARDWARE_SIDE_CORRECTION_SHARE = "19-25% of ALL ticks (3 rounds, 2026-09-12)"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenarios", type=int, default=6)
    parser.add_argument("--seeds", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    paths = sorted(_FIXTURES.glob("*_metadata.json"))[: args.scenarios]
    tuning = tuning_with_overrides({"SIDE_CORRECTION_BLENDS": True}, group="escape")

    totals = Counter()
    for path in paths:
        for seed in range(args.seeds):
            metadata = ScenarioMetadata.model_validate(json.loads(path.read_text()))
            sim = ScenarioSimulator(
                metadata,
                num_laps=3,
                seed=seed,
                blind=False,
                park=False,
                tuning=tuning,
                emit_vision_detections=True,
            )
            nav = sim.navigator

            def on_step(_state, _scan, nav=nav) -> None:
                totals["ticks"] += 1
                man = nav._active_maneuver  # noqa: SLF001  (no public accessor)
                if man is None:
                    return
                totals["maneuver"] += 1
                totals[f"type:{man.maneuver_type.name}"] += 1
                if man.maneuver_type is ManeuverType.SIDE_CORRECTION:
                    totals["side_correction"] += 1
                    if man.speed >= 0.0:
                        totals["blendable"] += 1

            sim.run(max_steps=args.max_steps, on_step=on_step)

    n = totals["ticks"]
    print(f"\n{len(paths)} scenarios x {args.seeds} seeds, SIGHTED, flag ON")
    print(f"  ticks                 {n}")
    if not n:
        print("  no ticks -- nothing to conclude")
        return
    for key in ("maneuver", "side_correction", "blendable"):
        print(f"  {key:<20}  {totals[key]:6d}  ({100 * totals[key] / n:5.2f}% of ticks)")
    print("\n  manoeuvre mix:")
    for key, c in sorted(totals.items()):
        if key.startswith("type:"):
            print(f"    {key[5:]:<20} {c:6d}")
    print(f"\n  CONTROL, hardware side_correction share: {HARDWARE_SIDE_CORRECTION_SHARE}")
    print(
        "\n  VERDICT: "
        + (
            "REACHABLE -- the flat A/B is a real null"
            if totals["blendable"] > 0.01 * n
            else "NOT REACHABLE -- the A/B compared two identical worlds and is VOID"
        )
    )


if __name__ == "__main__":
    main()

r"""What did the 2026-09-07 changes buy, all three together?

Each was measured on its own as it landed, but never as a set against the state
the day started in. All three are tuning parameters, so both arms run in ONE
process from overrides -- no checkout, and no risk of mixing old and new code
across a warm worker pool, which is how a previous measurement fooled itself.

    arm "before"   MIN_TURN_RADIUS_M 0.0   VISION_RANGE_MODEL off  COMMIT_HYSTERESIS off
    arm "after"    MIN_TURN_RADIUS_M 0.29  VISION_RANGE_MODEL on   COMMIT_HYSTERESIS on
                   MAX_ESCAPE_S 1.0 / 1.8 with the K-turn bounds scaled to match

MEASURED 2026-09-07, 16 fixtures x 6 seeds x 2 arms, blind, park off:

    before   in_time 56   laps3 56   collided 10   stuck 7   timed 20
    after    in_time 59   laps3 70   collided 19   stuck 0   timed  0

Stuck and timeouts go to ZERO -- 27 runs that previously ended without finishing
now complete, and laps>=3 goes 56 -> 70. That is the escape work. Collisions
nearly double, which is why in-time gains only 3, and the increase is part real
cost and part the model no longer flattering itself: the turn-radius floor ALONE
was measured at collisions 5 -> 8 on a smaller set, so roughly half of it is the
chassis losing a dodge it could never physically perform.

Read the "before" arm as a HISTORICAL curiosity, not as a baseline to beat. It
runs a chassis that pivots in 1.5 cm against a measured 0.29 m and a camera that
resolves signs to twice the range the real one manages, so its score is inflated
by defects, not earned. The honest summary of the day is that the simulator got
harder and more truthful; a flat or lower score across this pair is the expected
result, not a regression.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_today_stack_ab.py --seeds 6 --jobs 14
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from shared.domain.models import ScenarioMetadata

from scripts.common.diag_base import print_pool_progress, resolve_jobs, run_pool
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.config.tuning_helpers import tuning_with_overrides
from src.simulation.scenario_simulator import ScenarioSimulator

# Module level, NOT inside main(): a spawned worker re-imports this module but
# never runs main(), so disabling there silences the parent only and every
# worker's navigator logs still flood the output.
logging.disable(logging.CRITICAL)

ROUND_TIME_LIMIT_S = 180.0

_FIXTURES = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"


def _tuning(after: bool):  # noqa: ANN202  (NavigationTuning, avoided at import time)
    """Build the whole arm's tuning: simulation, escape and sign-router together."""
    escape_s = 1.8 if after else 1.0
    tuning = tuning_with_overrides(
        {
            "MIN_TURN_RADIUS_M": 0.29 if after else 0.0,
            "VISION_RANGE_MODEL": after,
        },
        group="simulation",
    )
    tuning = tuning_with_overrides(
        {
            "MAX_ESCAPE_S": escape_s,
            "K_TURN_MIN_S": round(escape_s * 0.3, 2),
            "K_TURN_MAX_S": round(escape_s * 0.6, 2),
        },
        group="escape",
        base=tuning,
    )
    return tuning_with_overrides({"COMMIT_HYSTERESIS": after}, group="sign_router", base=tuning)


def run_case(payload: tuple[str, int, bool, int]) -> tuple[bool, bool, bool, bool, bool, bool]:
    """Run one (scenario, seed, arm) case. Module-level for ProcessPoolExecutor."""
    path, seed, after, max_steps = payload
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path).read_text()))
    result = ScenarioSimulator(
        metadata, num_laps=3, seed=seed, blind=True, park=False, tuning=_tuning(after)
    ).run(max_steps=max_steps)
    in_time = (
        result.laps_completed >= 3
        and not result.collided
        and not result.pass_side_violation
        and result.sim_time_s <= ROUND_TIME_LIMIT_S
    )
    return (
        after,
        in_time,
        result.laps_completed >= 3,
        result.collided,
        result.stuck,
        result.timed_out,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seeds", type=int, default=6)
    parser.add_argument("--limit", type=int, default=0, help="Scenarios to use; 0 means all.")
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    paths = sorted(_FIXTURES.glob("*_metadata.json"))
    if args.limit:
        paths = paths[: args.limit]
    arms = (False, True)
    payloads = [
        (str(p), seed, after, args.max_steps) for after in arms for p in paths for seed in range(args.seeds)
    ]

    jobs = resolve_jobs(args.jobs)
    print(f"{len(payloads)} runs over {jobs} workers, {len(paths)} scenarios x {args.seeds} seeds, blind")
    results = run_pool(run_case, payloads, jobs, on_result=print_pool_progress("today-stack"))

    print()
    print(f"{'arm':>8} {'n':>4} {'in_time':>8} {'laps3':>6} {'collided':>9} {'stuck':>6} {'timed':>6}")
    for after in arms:
        rows = [r for r in results if r[0] == after]
        if not rows:
            continue
        print(
            f"{'after' if after else 'before':>8} {len(rows):>4} {sum(r[1] for r in rows):>8} "
            f"{sum(r[2] for r in rows):>6} {sum(r[3] for r in rows):>9} {sum(r[4] for r in rows):>6} "
            f"{sum(r[5] for r in rows):>6}"
        )


if __name__ == "__main__":
    main()

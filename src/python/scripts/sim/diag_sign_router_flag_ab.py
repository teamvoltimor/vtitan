r"""A/B any boolean ``SignRouterParams`` flag over the Obstacles fixtures.

Written for ``COMMIT_HYSTERESIS``, but the flag is an argument because the same
question keeps recurring: a sign-router switch is measured on the corpus, reads
flat or marginally worse, and ships off -- and the corpus may or may not be able
to see what it does.

For ``COMMIT_HYSTERESIS`` specifically it demonstrably cannot see the whole
effect. The switch stops the router re-racing its commitment every tick, and
what it saves is the aim point jumping between two tracks of the SAME pillar.
Duplicate tracks exist in both worlds at the same rate (sim 2.0x, hardware 2.2x)
-- but their SEPARATION does not:

    duplicate nearest-neighbour, p50:   sim 0.012 m    hardware 0.21 m

So switching between duplicates moves the commanded line 1.2 cm in the corpus
and 21 cm on the mat, a factor of 17. The corpus verdict (flat sighted,
marginally worse blind) was taken where the defect is almost absent. Replayed
over recorded detections, the flag cuts aim-point jumps 38 -> 21 with the
committed-tick count unchanged.

This script therefore prices the COST side only. The benefit lives in
``scripts/bag/diag_bag_sign_target_churn.py``, on the bags, which is the only
instrument that can see it.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_sign_router_flag_ab.py \\
            --field COMMIT_HYSTERESIS --seeds 6 --jobs 14
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


def run_case(payload: tuple[str, int, str, bool, int, str, bool]) -> tuple[bool, bool, bool, bool, bool, bool]:
    """Run one (scenario, seed, flag value) case. Module-level for ProcessPoolExecutor."""
    path, seed, field, value, max_steps, group, sighted = payload
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path).read_text()))
    tuning = tuning_with_overrides({field: value}, group=group)
    result = ScenarioSimulator(
        metadata,
        num_laps=3,
        seed=seed,
        blind=not sighted,
        park=False,
        tuning=tuning,
        emit_vision_detections=sighted,
    ).run(max_steps=max_steps)
    in_time = (
        result.laps_completed >= 3
        and not result.collided
        and not result.pass_side_violation
        and result.sim_time_s <= ROUND_TIME_LIMIT_S
    )
    return (
        value,
        in_time,
        result.laps_completed >= 3,
        result.collided,
        result.pass_side_violation,
        result.timed_out,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--field", default="COMMIT_HYSTERESIS", help="boolean tuning field to A/B")
    parser.add_argument(
        "--group",
        default="sign_router",
        help="tuning group the field lives in, e.g. escape for SIDE_CORRECTION_BLENDS",
    )
    parser.add_argument(
        "--sighted",
        action="store_true",
        help="run the emulated camera. Pass-side violations and sign collisions only "
        "mean anything sighted; the blind default measures the laps, not the routing.",
    )
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
        (str(p), seed, args.field, value, args.max_steps, args.group, args.sighted)
        for value in arms
        for p in paths
        for seed in range(args.seeds)
    ]

    jobs = resolve_jobs(args.jobs)
    print(
        f"{len(payloads)} runs over {jobs} workers, {args.group}.{args.field}, "
        f"{len(paths)} scenarios x {args.seeds} seeds, {'SIGHTED' if args.sighted else 'blind'}"
    )
    results = run_pool(run_case, payloads, jobs, on_result=print_pool_progress(args.field.lower()))

    print()
    print(f"{args.field:>20} {'n':>4} {'in_time':>8} {'laps3':>6} {'collided':>9} {'pass_side':>10} {'timed':>6}")
    for value in arms:
        rows = [r for r in results if r[0] == value]
        if not rows:
            continue
        print(
            f"{str(value):>20} {len(rows):>4} {sum(r[1] for r in rows):>8} {sum(r[2] for r in rows):>6} "
            f"{sum(r[3] for r in rows):>9} {sum(r[4] for r in rows):>10} {sum(r[5] for r in rows):>6}"
        )


if __name__ == "__main__":
    main()

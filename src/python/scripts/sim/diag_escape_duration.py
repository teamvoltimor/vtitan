r"""How long must an escape run to actually rotate the chassis out of a corner?

An escape is terminated by ELAPSED TIME, and the shipped durations cannot deliver
the rotation the manoeuvre exists to produce. `MAX_ESCAPE_S` 1.0 at `REV_SPEED`
0.2 m/s buys 0.20 m of path, which at the chassis's MEASURED 0.29 m minimum turn
radius is 40 deg -- against the 90 deg+ that clearing a corner needs. Hardware
agrees: an escape episode achieves a median 27.4 deg, 39% under 20 deg, and
re-triggers up to 50 times because the same corner is still there.

**This sweep is only meaningful with ``MIN_TURN_RADIUS_M`` ON.** It ships at 0.0,
where the model gives full lock a 1.5 cm radius, so every simulated escape
pivots freely and works -- an arm run that way can refute a duration change but
can never confirm one. The floor is forced on here for exactly that reason.

Known result at the endpoints (16 scenarios x 3 seeds, floor 0.29):

    MAX_ESCAPE_S 1.0   in-time 26   collided  8   stuck 1   timed 9
    MAX_ESCAPE_S 2.3   in-time 31   collided 11   stuck 0   timed 0

Timeouts go to ZERO, at a cost of 3 collisions. This sweep looks for a knee in
between. Terminating on achieved YAW instead was implemented, measured and
REFUTED -- a 60 deg target is byte-identical to off, because the escape never
gets that far before the time cap binds.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_escape_duration.py --jobs 14
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
TURN_RADIUS_FLOOR_M = 0.29
"""The MEASURED chassis floor. Forced on -- see the module docstring."""

_FIXTURES = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"


def run_case(payload: tuple[str, int, float, int]) -> tuple[float, bool, bool, bool, bool, bool]:
    """Run one (scenario, seed, duration) case. Module-level for ProcessPoolExecutor.

    Takes primitives only and rebuilds the tuning inside the worker, because the
    pool pickles this argument and a ``NavigationTuning`` tree is both large and
    awkward to send.
    """
    path, seed, max_escape_s, max_steps = payload
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path).read_text()))
    tuning = tuning_with_overrides({"MIN_TURN_RADIUS_M": TURN_RADIUS_FLOOR_M}, group="simulation")
    tuning = tuning_with_overrides(
        {
            # The K-turn bounds are scaled with the cap rather than pinned, so a
            # longer budget is actually reachable by the manoeuvre that uses it.
            "K_TURN_MIN_S": round(max_escape_s * 0.3, 2),
            "K_TURN_MAX_S": round(max_escape_s * 0.6, 2),
            "MAX_ESCAPE_S": max_escape_s,
        },
        group="escape",
        base=tuning,
    )
    result = ScenarioSimulator(
        metadata, num_laps=3, seed=seed, blind=True, park=False, tuning=tuning
    ).run(max_steps=max_steps)
    in_time = (
        result.laps_completed >= 3
        and not result.collided
        and not result.pass_side_violation
        and result.sim_time_s <= ROUND_TIME_LIMIT_S
    )
    return (
        max_escape_s,
        in_time,
        result.laps_completed >= 3,
        result.collided,
        result.stuck,
        result.timed_out,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--durations", type=float, nargs="+", default=[1.0, 1.4, 1.8, 2.3, 3.0])
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--limit", type=int, default=0, help="Scenarios to use; 0 means all.")
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    paths = sorted(_FIXTURES.glob("*_metadata.json"))
    if args.limit:
        paths = paths[: args.limit]
    payloads = [
        (str(p), seed, duration, args.max_steps)
        for duration in args.durations
        for p in paths
        for seed in range(args.seeds)
    ]

    jobs = resolve_jobs(args.jobs)
    print(f"{len(payloads)} runs over {jobs} workers, MIN_TURN_RADIUS_M={TURN_RADIUS_FLOOR_M}")
    results = run_pool(run_case, payloads, jobs, on_result=print_pool_progress("escape-duration"))

    print()
    print(f"{'MAX_ESCAPE_S':>13} {'n':>4} {'in_time':>8} {'laps3':>6} {'collided':>9} {'stuck':>6} {'timed':>6}")
    for duration in args.durations:
        rows = [r for r in results if r[0] == duration]
        if not rows:
            continue
        print(
            f"{duration:>13.1f} {len(rows):>4} {sum(r[1] for r in rows):>8} {sum(r[2] for r in rows):>6} "
            f"{sum(r[3] for r in rows):>9} {sum(r[4] for r in rows):>6} {sum(r[5] for r in rows):>6}"
        )


if __name__ == "__main__":
    main()

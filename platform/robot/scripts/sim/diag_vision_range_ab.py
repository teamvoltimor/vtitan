r"""What does Obstacles score once the simulated camera goes blind with distance?

``VISION_RANGE_MODEL`` ships OFF, and off the emulated camera detects a sign out
to ``CAMERA_FAR_CLIP`` -- **10 m** -- with no misses and fixed confidence. The
real detector's measured distribution over the 09-07 runs is p50 **0.70 m**, p90
1.06-1.31 m. The simulated camera therefore sees roughly TEN TIMES further than
the one bolted to the robot, and every Obstacles baseline in the repo was scored
with that advantage.

This sweep prices honesty. Turning the model on is expected to COST score: the
robot loses sign vision it was scored with and never had on the mat. The number
that matters is how much, because the flag is a precondition for any sign
experiment meaning anything -- ``SIGN_LIDAR_PROPOSE`` above all, whose whole
value is the LIDAR spotting a pillar at a median 1.31 m. Against a camera that
already saw it at 10 m, such a feature can only read as cost, which is exactly
what its A/B did read.

MEASURED 2026-09-07, 16 fixtures x 6 seeds x 2 arms, blind, park off:

    VISION_RANGE_MODEL False   in_time 59   laps3 69   collided 21   pass_side 1
    VISION_RANGE_MODEL True    in_time 59   laps3 71   collided 18   pass_side 0

The cost this sweep was written to price IS NOT THERE: in-time is identical and
collisions move the favourable way by the same margin that other arms of this
corpus move under reseeding, so read it as FREE rather than as a gain. The arms
are not identical, so the flag is live rather than inert.

The likely reason it is free: at 10 m the emulator was handing the router
detections from OTHER corridors, and sign tracks are keyed on the robot's own
corridor -- the known ~2.7x track duplication. Blinding the camera to ~1 m
removes cross-corridor phantoms about as fast as it removes real early
sightings.

Run BLIND, which is what makes this measurable at all: on Obstacles a blind run
forces ``emit_vision_detections`` on and the router discovers signs through the
emulator rather than being handed the layout. With detections off the flag is
inert.

``MIN_TURN_RADIUS_M`` is left at its shipped 0.29 -- since ``72e7172b`` the
measured floor IS the default, so this arm pair is a true before/after of the
one flag.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_vision_range_ab.py --seeds 6 --jobs 14
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


def run_case(payload: tuple[str, int, bool, int]) -> tuple[bool, bool, bool, bool, bool, bool]:
    """Run one (scenario, seed, range-model) case. Module-level for ProcessPoolExecutor.

    Takes primitives only and rebuilds the tuning inside the worker, because the
    pool pickles this argument and a ``NavigationTuning`` tree is both large and
    awkward to send.
    """
    path, seed, range_model, max_steps = payload
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path).read_text()))
    tuning = tuning_with_overrides({"VISION_RANGE_MODEL": range_model}, group="simulation")
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
        range_model,
        in_time,
        result.laps_completed >= 3,
        result.collided,
        result.pass_side_violation,
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
        (str(p), seed, range_model, args.max_steps)
        for range_model in arms
        for p in paths
        for seed in range(args.seeds)
    ]

    jobs = resolve_jobs(args.jobs)
    print(f"{len(payloads)} runs over {jobs} workers, {len(paths)} scenarios x {args.seeds} seeds, blind")
    results = run_pool(run_case, payloads, jobs, on_result=print_pool_progress("vision-range"))

    print()
    print(f"{'RANGE_MODEL':>12} {'n':>4} {'in_time':>8} {'laps3':>6} {'collided':>9} {'pass_side':>10} {'timed':>6}")
    for range_model in arms:
        rows = [r for r in results if r[0] == range_model]
        if not rows:
            continue
        print(
            f"{str(range_model):>12} {len(rows):>4} {sum(r[1] for r in rows):>8} {sum(r[2] for r in rows):>6} "
            f"{sum(r[3] for r in rows):>9} {sum(r[4] for r in rows):>10} {sum(r[5] for r in rows):>6}"
        )


if __name__ == "__main__":
    main()

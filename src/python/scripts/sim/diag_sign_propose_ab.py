r"""Does the LIDAR sign proposer buy anything once the camera is honestly blind?

``SIGN_LIDAR_PROPOSE`` lets a pillar-shaped LIDAR cluster settle a sign's
POSITION for the camera to colour later. Its entire value is that the LIDAR
spots a pillar earlier than the deployed detector resolves one.

It was A/B'd and found to have NO measurable benefit -- but that sweep ran
against an emulated camera that detected every sign out to a long range. A
feature whose value is seeing a pillar EARLY cannot show a benefit against a
camera that already saw it far up the corridor; the sweep could only have priced
its cost. That result is therefore VOID as evidence about benefit, and this
script re-runs it now that ``VISION_RANGE_MODEL`` defaults on. See
``adr:0088-refuted-config-knobs``.

This is the general trap worth remembering: an A/B is only evidence if the
instrument can resolve the effect. Both arms were measured correctly and the
conclusion still did not follow.

The re-run does NOT rescue the feature: it turns a void result into a negative
one, with laps and collisions both moving against it by more than the corpus
moves under reseeding. The off arm reproduces the ``VISION_RANGE_MODEL``
sweep's on arm run for run, so the two are directly comparable.

There is a mechanism that predicts this, and it is worth ruling out before the
feature is abandoned: a proposal carries NO COLOUR, and while sign routing is
colour-keyed end to end, the DEFORMATION step falls through to ``else
green_mult``. Every colourless proposal is therefore deformed around as though
it were green, and many of them are red. That is wrong-side deformation on a
real pillar, which is exactly the shape of a collision increase.

Run BLIND, which is what puts the camera in the loop at all: on Obstacles a
blind run forces ``emit_vision_detections`` on and the router discovers signs
through the emulator rather than being handed the layout.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_sign_propose_ab.py --seeds 6 --jobs 14
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from shared.config.constants import CompetitionSpecs

from scripts.common.diag_base import print_pool_progress, resolve_jobs, run_pool
from scripts.common.scenarios import load_scenario, scenario_paths
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.config.tuning_helpers import tuning_with_overrides
from src.simulation.scenario_simulator import ScenarioSimulator

# Module level, NOT inside main(): a spawned worker re-imports this module but
# never runs main(), so disabling there silences the parent only and every
# worker's navigator logs still flood the output.
logging.disable(logging.CRITICAL)

_FIXTURES = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"


def run_case(payload: tuple[str, int, bool, int]) -> tuple[bool, bool, bool, bool, bool, bool]:
    """Run one (scenario, seed, proposer) case. Module-level for ProcessPoolExecutor.

    Takes primitives only and rebuilds the tuning inside the worker, because the
    pool pickles this argument and a ``NavigationTuning`` tree is both large and
    awkward to send. ``VISION_RANGE_MODEL`` is NOT set here: it now ships on, and
    forcing it would hide a later change to that default from this sweep.
    """
    path, seed, propose, max_steps = payload
    metadata = load_scenario(Path(path))
    tuning = tuning_with_overrides({"sign_lidar_propose": propose}, group="sign_router")
    result = ScenarioSimulator(
        metadata, num_laps=3, seed=seed, blind=True, park=False, tuning=tuning
    ).run(max_steps=max_steps)
    in_time = (
        result.laps_completed >= 3
        and not result.collided
        and not result.pass_side_violation
        and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    )
    return (
        propose,
        in_time,
        result.laps_completed >= 3,
        result.collided,
        result.pass_side_violation,
        result.timed_out,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seeds", type=int, default=6)
    parser.add_argument("--limit", type=int, default=0, help="Scenarios to use; 0 means all.")
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    paths = scenario_paths(_FIXTURES)
    if args.limit:
        paths = paths[: args.limit]
    arms = (False, True)
    payloads = [
        (str(p), seed, propose, args.max_steps)
        for propose in arms
        for p in paths
        for seed in range(args.seeds)
    ]

    jobs = resolve_jobs(args.jobs)
    print(f"{len(payloads)} runs over {jobs} workers, {len(paths)} scenarios x {args.seeds} seeds, blind")
    results = run_pool(run_case, payloads, jobs, on_result=print_pool_progress("sign-propose"))

    print()
    print(f"{'PROPOSE':>12} {'n':>4} {'in_time':>8} {'laps3':>6} {'collided':>9} {'pass_side':>10} {'timed':>6}")
    for propose in arms:
        rows = [r for r in results if r[0] == propose]
        if not rows:
            continue
        print(
            f"{str(propose):>12} {len(rows):>4} {sum(r[1] for r in rows):>8} {sum(r[2] for r in rows):>6} "
            f"{sum(r[3] for r in rows):>9} {sum(r[4] for r in rows):>10} {sum(r[5] for r in rows):>6}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

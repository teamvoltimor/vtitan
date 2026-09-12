"""Price the declared RISK of ``SIDE_CORRECTION_BLENDS``, not its benefit.

The flag lets a forward side correction BIAS the planned steering instead of
replacing it. Its docstring says it ships off because "adding two steering
signals can saturate the wheel or produce a curvature neither layer asked for
-- exactly the kind of thing a corpus has to rule out". The outcome sweep
(``diag_sign_router_flag_ab.py --field SIDE_CORRECTION_BLENDS --group escape``)
counts laps and collisions; neither can see a saturated wheel, because a run
that saturates and still finishes scores as a win.

So this measures the wheel itself, per tick, from ``AckermannState.steer``:

  * SATURATED   -- |steer| within ``SAT_FRAC`` of ``MAX_STEERING_ANGLE``. Two
                   layers pushing the same way pins the wheel at full lock and
                   the chassis stops tracking either request.
  * CHATTER     -- mean |d steer| per tick, and the count of SIGN REVERSALS.
                   "A curvature neither layer asked for" shows up as the wheel
                   crossing zero repeatedly, which is the bay pendulum's
                   signature on the open track.

Both are reported for the flag OFF and ON over the same scenarios and seeds, so
the arms differ in one bit. A rise in either is the cost the outcome table
cannot show.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
        pixi run -e dev python scripts/sim/diag_blend_saturation.py --seeds 4 --jobs 14
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from shared.config.constants import RobotSpecs
from shared.domain.models import ScenarioMetadata

from scripts.common.diag_base import print_pool_progress, resolve_jobs, run_pool
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.config.tuning_helpers import tuning_with_overrides
from src.simulation.scenario_simulator import ScenarioSimulator

logging.disable(logging.CRITICAL)

SAT_FRAC = 0.98
"""Within this fraction of full lock counts as saturated. Not 1.0: the servo
command is quantised, so exact equality under-counts."""

_FIXTURES = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"


def run_case(payload: tuple[str, int, bool, int]) -> tuple[bool, int, int, float, int, int]:
    """Run one (scenario, seed, flag) case and return wheel statistics."""
    path, seed, value, max_steps = payload
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path).read_text()))
    tuning = tuning_with_overrides({"SIDE_CORRECTION_BLENDS": value}, group="escape")

    limit = RobotSpecs.MAX_STEERING_ANGLE * SAT_FRAC
    steers: list[float] = []

    def on_step(state, _scan) -> None:  # noqa: ANN001
        steers.append(state.steer)

    result = ScenarioSimulator(
        metadata,
        num_laps=3,
        seed=seed,
        blind=False,
        park=False,
        tuning=tuning,
        emit_vision_detections=True,
    ).run(max_steps=max_steps, on_step=on_step)

    saturated = sum(1 for s in steers if abs(s) >= limit)
    deltas = [abs(b - a) for a, b in zip(steers, steers[1:], strict=False)]
    reversals = sum(1 for a, b in zip(steers, steers[1:], strict=False) if a * b < 0)
    return (
        value,
        len(steers),
        saturated,
        sum(deltas) / len(deltas) if deltas else 0.0,
        reversals,
        int(result.laps_completed >= 3 and not result.collided),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seeds", type=int, default=4)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--jobs", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    paths = sorted(_FIXTURES.glob("*_metadata.json"))
    if args.limit:
        paths = paths[: args.limit]
    arms = (False, True)
    payloads = [
        (str(p), seed, value, args.max_steps) for value in arms for p in paths for seed in range(args.seeds)
    ]

    jobs = resolve_jobs(args.jobs)
    print(
        f"{len(payloads)} runs over {jobs} workers, SIDE_CORRECTION_BLENDS, "
        f"{len(paths)} scenarios x {args.seeds} seeds, SIGHTED"
    )
    print(f"full lock = {RobotSpecs.MAX_STEERING_ANGLE:.3f} rad; saturated at >= {SAT_FRAC:.2f} of it")
    results = run_pool(run_case, payloads, jobs, on_result=print_pool_progress("blend"))

    print()
    print(f"{'BLENDS':>8} {'runs':>5} {'ticks':>8} {'saturated':>18} {'mean |d steer|':>16} {'reversals/1k':>13} {'clean':>6}")
    for value in arms:
        rows = [r for r in results if r[0] == value]
        if not rows:
            continue
        ticks = sum(r[1] for r in rows)
        sat = sum(r[2] for r in rows)
        rev = sum(r[4] for r in rows)
        mean_d = sum(r[3] * r[1] for r in rows) / ticks if ticks else 0.0
        print(
            f"{str(value):>8} {len(rows):>5} {ticks:>8} {sat:>8} ({100 * sat / ticks:5.2f}%)"
            f" {mean_d:>16.5f} {1000 * rev / ticks:>13.2f} {sum(r[5] for r in rows):>6}"
        )


if __name__ == "__main__":
    main()

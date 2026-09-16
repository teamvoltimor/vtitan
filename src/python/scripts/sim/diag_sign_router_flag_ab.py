r"""A/B any boolean ``SignRouterParams`` flag over the Obstacles fixtures.

Written for ``COMMIT_HYSTERESIS``, but the flag is an argument because the same
question keeps recurring: a sign-router switch is measured on the corpus, reads
flat or marginally worse, and ships off -- and the corpus may or may not be able
to see what it does.

For ``COMMIT_HYSTERESIS`` specifically it demonstrably cannot see the whole
effect. The switch stops the router re-racing its commitment every tick, and
what it saves is the aim point jumping between two tracks of the SAME pillar.
Duplicate tracks exist in both worlds at a similar rate, but their SEPARATION
does not: switching between duplicates moves the commanded line centimetres in
the corpus and tens of centimetres on the mat, an order of magnitude apart. The
corpus verdict (flat sighted, marginally worse blind) was taken where the defect
is almost absent. Replayed over recorded detections, the flag cuts aim-point
jumps sharply with the committed-tick count unchanged. See
``adr:0051-sign-lane-planner``.

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


def run_case(payload: tuple[str, int, str, bool, int, str, bool]) -> tuple[bool, bool, bool, bool, bool, bool]:
    """Run one (scenario, seed, flag value) case. Module-level for ProcessPoolExecutor."""
    path, seed, field, value, max_steps, group, sighted = payload
    metadata = load_scenario(Path(path))
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
        and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    )
    return (
        value,
        in_time,
        result.laps_completed >= 3,
        result.collided,
        result.pass_side_violation,
        result.timed_out,
        # Case identity, so the two arms can be PAIRED. Both arms run the same
        # (scenario, seed) list, and an unpaired test throws that away: the
        # between-scenario variance dwarfs the effect, and a flag worth a few
        # points reads flat. See the McNemar block in main().
        (Path(path).name, seed),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--field", default="COMMIT_HYSTERESIS", help="boolean tuning field to A/B")
    parser.add_argument(
        "--group",
        default="sign_router",
        help="tuning group the field lives in, e.g. escape for side_correction_blends",
    )
    parser.add_argument(
        "--sighted",
        action="store_true",
        help="run the emulated camera. Pass-side violations and sign collisions only "
        "mean anything sighted; the blind default measures the laps, not the routing.",
    )
    parser.add_argument("--seeds", type=int, default=6)
    parser.add_argument("--limit", type=int, default=0, help="Scenarios to use; 0 means all.")
    parser.add_argument(
        "--scenarios",
        default="",
        help="Directory of scenarios; defaults to the 32 Obstacles fixtures. Point it at "
        ".corpus/obstacles/scenarios for the generated corpus, which carries the scenario "
        "diversity extra seeds on the same fixtures cannot buy.",
    )
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    paths = scenario_paths(Path(args.scenarios) if args.scenarios else _FIXTURES)
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

    # PAIRED view. Both arms ran the same (scenario, seed) cases, so the only
    # cases carrying information about the flag are the ones where the two arms
    # DISAGREE -- that is McNemar's test. An unpaired chi-square on the same
    # data spends its power on between-scenario variance that the pairing
    # already cancels, which is how a real few-point effect reads flat.
    by_case: dict[tuple[str, int], dict[bool, tuple]] = {}
    for r in results:
        by_case.setdefault(r[6], {})[r[0]] = r
    both = {k: v for k, v in by_case.items() if len(v) == 2}
    print()
    print(f"PAIRED over {len(both)} cases run in both arms")
    print(f"{'metric':>10} {'only ON':>8} {'only OFF':>9} {'both':>6} {'neither':>8} {'p (McNemar)':>12}")
    for idx, name, good in ((1, "in_time", True), (2, "laps3", True), (3, "collided", False), (4, "pass_side", False)):
        on_only = off_only = both_n = neither = 0
        for v in both.values():
            a, b = bool(v[True][idx]), bool(v[False][idx])
            if not good:
                a, b = not a, not b
            if a and b:
                both_n += 1
            elif a:
                on_only += 1
            elif b:
                off_only += 1
            else:
                neither += 1
        n = on_only + off_only
        # Exact binomial on the discordant pairs; scipy is not a hard dep here.
        if n == 0:
            p = float("nan")
        else:
            from math import comb

            k = min(on_only, off_only)
            p = min(1.0, 2.0 * sum(comb(n, i) for i in range(k + 1)) / (2.0**n))
        print(f"{name:>10} {on_only:>8} {off_only:>9} {both_n:>6} {neither:>8} {p:>12.4f}")
    print("  'only ON' = the flag turned this case good; 'only OFF' = it broke it.")
    print("  Only those two columns carry information; both/neither are ties.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

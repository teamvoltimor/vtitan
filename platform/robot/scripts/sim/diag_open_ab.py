"""A/B one tuning change across the Open Challenge sweep, in memory.

``diag_open_parallel.py`` answers "how does the committed tuning do". This
answers the question that actually decides whether to keep a change: "does this
field change anything, holding everything else fixed".

Both arms run the same seeded sample, and each case is seeded by its index, so
a case that behaves differently between arms did so *because of the override* --
there is no run-to-run noise to explain it away. That is what makes a per-case
sim-time delta readable rather than decorative.

Overrides are applied to an in-memory ``NavigationTuning`` and handed to
``ScenarioSimulator``. Nothing on disk is touched, so this is safe to run while
the checked-in TOML tree is being edited elsewhere.

A verdict is the headline, but on a sweep that already passes 100% the verdicts
cannot move and sim time is the only signal left. Read the mean delta, not the
per-case spread: the arms differ by trajectory, so individual cases swing in
both directions even when the change is neutral overall.

Usage (from ``platform/robot``)::

    pixi run -e dev python scripts/sim/diag_open_ab.py waypoints.NARROW_CENTER_BIAS_SIDE=outer
    pixi run -e dev python scripts/sim/diag_open_ab.py waypoints.ARC_RADIUS=0.35 --sample 12
    pixi run -e dev python scripts/sim/diag_open_ab.py waypoints.ARC_RADIUS=0.35 --tuning custom.yaml
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

from shared.config.constants import CompetitionSpecs

from scripts.common.diag_base import (
    add_sweep_args,
    add_tuning_arg,
    draw_sample,
    load_tuning,
    print_pool_progress,
    resolve_jobs,
    run_pool,
)
from scripts.common.open_cases import SIDES, WIDE_MM, balanced_128_cases, case_space
from scripts.common.tables import print_table
from scripts.sim.diag_open_exhaustive import _verdict
from src.simulation.scenario_builder import build_open_metadata
from src.simulation.scenario_simulator import ScenarioSimulator

_DEFAULT_SAMPLE_SIZE = 24
_DEFAULT_SEED = 0


def _apply_overrides(tuning: NavigationTuning, overrides: dict[str, str]) -> NavigationTuning:
    """Return a copy of ``tuning`` with ``group.FIELD=value`` overrides applied.

    Values are parsed by the group's own pydantic model rather than by this
    function, so a field keeps whatever coercion and validation it declares --
    an enum field accepts its string spelling, a float rejects a typo.
    """
    grouped: dict[str, dict[str, Any]] = {}
    for path, raw in overrides.items():
        group, _, field = path.partition(".")
        if not field:
            message = f"override must be group.FIELD=value, got {path!r}"
            raise ValueError(message)
        if not hasattr(tuning, group):
            message = f"no such tuning group: {group!r}"
            raise ValueError(message)
        grouped.setdefault(group, {})[field] = raw

    updated: dict[str, Any] = {}
    for group, fields in grouped.items():
        current = getattr(tuning, group)
        for field in fields:
            if not hasattr(current, field):
                message = f"no such field: {group}.{field}"
                raise ValueError(message)
        updated[group] = current.model_validate({**current.model_dump(), **fields})
    # NavigationTuning is a frozen dataclass of pydantic groups, so the groups
    # revalidate themselves and the aggregate is rebuilt rather than mutated.
    return replace(tuning, **updated)


@dataclass(frozen=True, slots=True)
class _ArmResult:
    """One scenario's outcome under one A/B arm."""

    index: int
    verdict: str
    laps: int
    sim_time_s: float
    label: str
    # Scoring rule 1.3, worth 3 points: "stopped in the finish section after
    # three laps". Scored from the TRUE final pose against the section the round
    # started in, not from anything the navigator believes about where it came
    # to rest -- the same separation the pass-side scorer needed.
    #
    # None when the run never completed its laps: a run that timed out has no
    # finish to be in the wrong section of, and folding those in as failures
    # would report the lap rate a second time under a different name.
    finished_in_section: bool | None


def _run_case(
    payload: tuple[int, tuple[int, ...], str, str, int, int, str | None, dict[str, str] | None],
) -> _ArmResult:
    """Run one scenario under one arm. Primitive-valued so it pickles."""
    from shared.domain.enums import Direction, Section

    index, widths, section_value, direction_value, cell, laps, tuning_path, overrides = payload
    section = Section(section_value)
    direction = Direction(direction_value)
    widths_mm = dict(zip(SIDES, widths, strict=True))
    meta = build_open_metadata(widths_mm, section, direction, scenario_id=index, start_cell=cell)

    tuning = load_tuning(tuning_path)
    if overrides:
        tuning = _apply_overrides(tuning, overrides)

    result = ScenarioSimulator(meta, num_laps=laps, tuning=tuning, seed=index, blind=True).run()
    finished_in_section: bool | None = None
    if result.laps_completed >= laps:
        from src.navigation.planning.waypoints import corridor_for_position

        final_x, final_y, _final_yaw = result.final_pose
        finished_in_section = corridor_for_position(final_x, final_y) is section
    return _ArmResult(
        index=index,
        verdict=_verdict(result),
        laps=result.laps_completed,
        sim_time_s=result.sim_time_s,
        finished_in_section=finished_in_section,
        label=f"{'-'.join(str(w) for w in widths)} {section.value}/{direction.value} c{cell}",
    )


def _run_arm(name: str, payloads: list[tuple[Any, ...]], jobs: int) -> dict[int, _ArmResult]:
    """Run every case for one arm, returning results keyed by case index."""
    started = time.perf_counter()
    results = run_pool(_run_case, payloads, jobs, on_result=print_pool_progress(name))
    print(f"  {name}: {len(results)}/{len(payloads)} in {time.perf_counter() - started:.0f}s", flush=True)
    return {row.index: row for row in results}


def _report_by_width(cases: list, base: dict[int, _ArmResult], variant: dict[int, _ArmResult]) -> None:
    """Split both arms' pass rate by how many corridors are WIDE.

    A layout-shaped failure and a speed-shaped one look identical in an
    aggregate pass count, and they call for opposite fixes. If failures track
    the number of wide corridors in a scenario, the problem is the layout the
    robot is driving; if they are flat across wide-count and only the arms
    differ, the change under test is what moved them.

    Wide-count is read from the case's own widths tuple rather than the label,
    so it cannot drift from what was actually simulated.
    """
    buckets: dict[int, dict[str, int]] = {}
    for i, (widths, _section, _direction, _cell) in enumerate(cases):
        wide = sum(1 for w in widths if w == WIDE_MM)
        b = buckets.setdefault(wide, {"n": 0, "base_ok": 0, "variant_ok": 0})
        b["n"] += 1
        b["base_ok"] += base[i].verdict == "ok"
        b["variant_ok"] += variant[i].verdict == "ok"
        for arm, results in (("base", base), ("var", variant)):
            verdict = results[i].verdict
            if verdict != "ok":
                b[f"{arm}_{verdict}"] = b.get(f"{arm}_{verdict}", 0) + 1

    def failures(b: dict[str, int], arm: str) -> str:
        """Failure kinds for one arm, most common first.

        Printed because a pass count cannot separate the two ways a speed
        change hurts: a COLLISION means the robot ran out of room, an
        over-time/incomplete means it ran out of clock. They point at opposite
        remedies, and an aggregate that mixes them reads as one problem.
        """
        kinds = sorted(
            ((k.split("_", 1)[1], v) for k, v in b.items() if k.startswith(f"{arm}_")),
            key=lambda kv: -kv[1],
        )
        return ", ".join(f"{kind} {count}" for kind, count in kinds) or "-"

    print("\npass rate by number of WIDE corridors:", flush=True)
    print_table(
        [
            [
                wide,
                b["n"],
                f"{b['base_ok']}/{b['n']} ({b['base_ok'] / b['n']:.0%})",
                failures(b, "base"),
                f"{b['variant_ok']}/{b['n']} ({b['variant_ok'] / b['n']:.0%})",
                failures(b, "var"),
            ]
            for wide, b in sorted(buckets.items())
        ],
        ["wide", "n", "baseline ok", "baseline fails", "variant ok", "variant fails"],
    )


def _report_failures(cases: list, variant: dict[int, _ArmResult]) -> None:
    """List and characterise the VARIANT arm's remaining failures.

    A pass count says how many are left, not what they have in common. Once a
    config is near the ceiling the residue is what decides whether the next fix
    is a broad one or a special case -- and the two look identical in the
    headline number.

    Attributes are read from the case tuple, so the start corridor's width is
    the one the robot actually begins in rather than a guess from the label.
    """
    failures = [(i, cases[i], variant[i].verdict) for i in sorted(variant) if variant[i].verdict != "ok"]
    if not failures:
        print("\nno variant failures", flush=True)
        return

    print(f"\nvariant failures ({len(failures)}):", flush=True)
    rows = []
    for i, (widths, section, direction, cell) in [(i, c) for i, c, _ in failures]:
        verdict = next(v for j, _, v in failures if j == i)
        start_w = dict(zip(SIDES, widths, strict=True))[section.value.lower()]
        rows.append([
            i,
            "-".join(str(w) for w in widths),
            section.value,
            "cw" if direction.value == "clockwise" else "ccw",
            f"c{cell}",
            "WIDE" if start_w == WIDE_MM else "narrow",
            sum(1 for w in widths if w == WIDE_MM),
            verdict,
        ])
    print_table(rows, ["#", "widths S-N-E-W", "start", "dir", "cell", "start corridor", "n wide", "verdict"])

    def tally(label: str, key) -> None:  # noqa: ANN001 - local formatting helper
        counts: dict[str, int] = {}
        for _i, case, _v in failures:
            counts[str(key(case))] = counts.get(str(key(case)), 0) + 1
        ordered = ", ".join(f"{k}={v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1]))
        print(f"  {label:<22} {ordered}", flush=True)

    print("\n  failure composition:", flush=True)
    tally("start section", lambda c: c[1].value)
    tally("direction", lambda c: "cw" if c[2].value == "clockwise" else "ccw")
    tally("start cell", lambda c: f"c{c[3]}")
    tally(
        "start corridor",
        lambda c: "WIDE" if dict(zip(SIDES, c[0], strict=True))[c[1].value.lower()] == WIDE_MM else "narrow",
    )
    tally("wide corridors", lambda c: sum(1 for w in c[0] if w == WIDE_MM))
    counts: dict[str, int] = {}
    for _i, _c, v in failures:
        counts[v] = counts.get(v, 0) + 1
    print(f"  {'verdict':<22} " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])))


def _report_run_header(cases: list, population: list, seed: int, jobs: int, corpus: str, changed: str) -> None:
    print(f"{len(cases)} of {len(population)} scenarios, seed={seed}, {jobs} workers", flush=True)
    print(f"corpus: {corpus}", flush=True)
    print(f"variant: {changed}\n", flush=True)


def _report_verdict_summary(base: dict[int, _ArmResult], variant: dict[int, _ArmResult]) -> None:
    base_ok = sum(1 for r in base.values() if r.verdict == "ok")
    variant_ok = sum(1 for r in variant.values() if r.verdict == "ok")
    print(f"\nverdicts: baseline {base_ok}/{len(base)} ok, variant {variant_ok}/{len(variant)} ok", flush=True)

    # Rule 1.3 (3 pts). Denominator is runs that actually finished their laps,
    # printed alongside, so a change in the lap rate cannot be misread as a
    # change in where the robot stops.
    for name, arm in (("baseline", base), ("variant", variant)):
        scored = [r.finished_in_section for r in arm.values() if r.finished_in_section is not None]
        in_section = sum(1 for v in scored if v)
        if scored:
            print(
                f"  rule 1.3 stopped in finish section, {name}: "
                f"{in_section}/{len(scored)} of runs that completed their laps",
                flush=True,
            )
        else:
            print(f"  rule 1.3 stopped in finish section, {name}: no run completed its laps", flush=True)


def _report_flipped_verdicts(base: dict[int, _ArmResult], variant: dict[int, _ArmResult]) -> None:
    flipped = [
        (i, base[i].verdict, variant[i].verdict) for i in sorted(base) if base[i].verdict != variant[i].verdict
    ]
    if flipped:
        print("\nverdict changes:", flush=True)
        print_table(
            [[i, base[i].label, was, now] for i, was, now in flipped],
            ["#", "scenario", "baseline", "variant"],
        )
    else:
        print("no verdict changed", flush=True)


def _report_sim_time_delta(base: dict[int, _ArmResult], variant: dict[int, _ArmResult], both_ok: list[int]) -> None:
    # Sim time only compares where both arms finished; a timed-out run's clock
    # measures the time limit, not the lap.
    deltas = [(i, variant[i].sim_time_s - base[i].sim_time_s) for i in both_ok]
    total = sum(d for _, d in deltas)
    faster = sum(1 for _, d in deltas if d < 0)
    slower = sum(1 for _, d in deltas if d > 0)
    print(
        f"\nsim time over {len(deltas)} cases both arms finished: "
        f"mean {total / len(deltas):+.2f}s, total {total:+.1f}s "
        f"({faster} faster, {slower} slower, {len(deltas) - faster - slower} identical)",
        flush=True,
    )
    ranked = sorted(deltas, key=lambda d: d[1])
    # The three biggest moves each way, without repeating a case when the sample
    # is smaller than six.
    extremes = dict(ranked[:3] + ranked[-3:])
    print_table(
        [
            [i, base[i].label, f"{base[i].sim_time_s:.1f}s", f"{variant[i].sim_time_s:.1f}s", f"{d:+.1f}s"]
            for i, d in sorted(extremes.items(), key=lambda kv: kv[1])
        ],
        ["#", "scenario", "baseline", "variant", "delta"],
    )


def main() -> None:
    """Run both arms over the same sample and report what the override changed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("overrides", nargs="+", help="group.FIELD=value, applied to the variant arm.")
    add_sweep_args(
        parser,
        default_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        default_sample=_DEFAULT_SAMPLE_SIZE,
        default_seed=_DEFAULT_SEED,
        jobs=True,
    )
    add_tuning_arg(parser)
    args = parser.parse_args()

    overrides: dict[str, str] = {}
    for item in args.overrides:
        key, sep, value = item.partition("=")
        if not sep:
            parser.error(f"override must be group.FIELD=value, got {item!r}")
        overrides[key] = value

    # Fail on a bad --tuning or a bad override here, before spending a sweep on it.
    base_tuning = load_tuning(args.tuning)
    _apply_overrides(base_tuning, overrides)

    # DEFAULT is the balanced 128: every layout/section/direction exactly once,
    # with the start cell varied. The alternative -- sampling 128 uniformly from
    # the 640-case space -- leaves layout and section coverage to chance, so two
    # seeds are not comparable scenario-for-scenario. `--all` still means the
    # exhaustive 640, and an explicit `--sample N` still draws uniformly from it
    # for anyone who wants the old behaviour at a different size.
    explicit_sample = args.sample != _DEFAULT_SAMPLE_SIZE
    if args.all or explicit_sample:
        population = case_space()
        cases = draw_sample(population, sample=args.sample, seed=args.seed, all_=args.all)
        corpus = "uniform sample of the 640-case space"
    else:
        population = case_space()
        cases = balanced_128_cases(seed=args.seed)
        corpus = "balanced 128 (every layout/section/direction, start cell varied)"

    jobs = resolve_jobs(args.jobs)
    changed = ", ".join(f"{k}={v}" for k, v in overrides.items())
    _report_run_header(cases, population, args.seed, jobs, corpus, changed)

    base_payloads = [
        (i, widths, section.value, direction.value, cell, args.laps, args.tuning, None)
        for i, (widths, section, direction, cell) in enumerate(cases)
    ]
    variant_payloads = [(*p[:-1], overrides) for p in base_payloads]

    base = _run_arm("baseline", base_payloads, jobs)
    variant = _run_arm("variant ", variant_payloads, jobs)

    _report_verdict_summary(base, variant)

    _report_by_width(cases, base, variant)
    _report_failures(cases, variant)

    _report_flipped_verdicts(base, variant)

    both_ok = [i for i in sorted(base) if base[i].verdict == "ok" and variant[i].verdict == "ok"]
    if not both_ok:
        return
    _report_sim_time_delta(base, variant, both_ok)


if __name__ == "__main__":
    main()

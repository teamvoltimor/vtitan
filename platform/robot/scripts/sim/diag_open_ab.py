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

    pixi run -e dev python scripts/sim/diag_open_ab.py waypoints.CENTER_BIAS_SIDE=outer
    pixi run -e dev python scripts/sim/diag_open_ab.py waypoints.ARC_RADIUS=0.35 --sample 12
"""

from __future__ import annotations

import argparse
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.tables import print_table
from scripts.sim.diag_open_exhaustive import _SIDES, _all_cases, _verdict
from shared.config.navigation_tuning import NavigationTuning
from src.simulation.scenario_builder import build_open_metadata
from src.simulation.scenario_simulator import ScenarioSimulator

_DEFAULT_LAPS = 3
_DEFAULT_SAMPLE_SIZE = 24
_DEFAULT_SEED = 0
_SPARE_CORES = 2


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


def _run_case(payload: tuple[int, tuple[int, ...], str, str, int, int, dict[str, str] | None]) -> dict[str, object]:
    """Run one scenario under one arm. Primitive-valued so it pickles."""
    from shared.config.enums import Direction, Section

    index, widths, section_value, direction_value, cell, laps, overrides = payload
    section = Section(section_value)
    direction = Direction(direction_value)
    widths_mm = dict(zip(_SIDES, widths, strict=True))
    meta = build_open_metadata(widths_mm, section, direction, scenario_id=index, start_cell=cell)

    tuning = NavigationTuning.load_default()
    if overrides:
        tuning = _apply_overrides(tuning, overrides)

    result = ScenarioSimulator(meta, num_laps=laps, tuning=tuning, seed=index, blind=True).run()
    return {
        "index": index,
        "verdict": _verdict(result),
        "laps": result.laps_completed,
        "sim_time_s": result.sim_time_s,
        "label": f"{'-'.join(str(w) for w in widths)} {section.value}/{direction.value} c{cell}",
    }


def _run_arm(name: str, payloads: list[tuple[Any, ...]], jobs: int) -> dict[int, dict[str, object]]:
    """Run every case for one arm, returning results keyed by case index."""
    started = time.perf_counter()
    rows: dict[int, dict[str, object]] = {}
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(_run_case, p) for p in payloads]
        for done in as_completed(futures):
            row = done.result()
            rows[int(row["index"])] = row
            print(f"  {name}: {len(rows)}/{len(payloads)}", end="\r", flush=True)
    print(f"  {name}: {len(rows)}/{len(payloads)} in {time.perf_counter() - started:.0f}s", flush=True)
    return rows


def main() -> None:
    """Run both arms over the same sample and report what the override changed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("overrides", nargs="+", help="group.FIELD=value, applied to the variant arm.")
    parser.add_argument("--laps", type=int, default=_DEFAULT_LAPS)
    parser.add_argument("--sample", type=int, default=_DEFAULT_SAMPLE_SIZE)
    parser.add_argument("--seed", type=int, default=_DEFAULT_SEED)
    parser.add_argument("--all", action="store_true", help="Run all 640 instead of a sample.")
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    args = parser.parse_args()

    overrides: dict[str, str] = {}
    for item in args.overrides:
        key, sep, value = item.partition("=")
        if not sep:
            parser.error(f"override must be group.FIELD=value, got {item!r}")
        overrides[key] = value

    # Fail on a bad override here, before spending a sweep on it.
    _apply_overrides(NavigationTuning.load_default(), overrides)

    population = _all_cases()
    if args.all or args.sample >= len(population):
        cases = population
    else:
        # Same seeded draw as diag_open_parallel, so the two are comparable.
        cases = random.Random(args.seed).sample(population, args.sample)  # noqa: S311

    jobs = args.jobs or max(1, (os.cpu_count() or 4) - _SPARE_CORES)
    changed = ", ".join(f"{k}={v}" for k, v in overrides.items())
    print(f"{len(cases)} of {len(population)} scenarios, seed={args.seed}, {jobs} workers", flush=True)
    print(f"variant: {changed}\n", flush=True)

    base_payloads = [
        (i, widths, section.value, direction.value, cell, args.laps, None)
        for i, (widths, section, direction, cell) in enumerate(cases)
    ]
    variant_payloads = [(*p[:-1], overrides) for p in base_payloads]

    base = _run_arm("baseline", base_payloads, jobs)
    variant = _run_arm("variant ", variant_payloads, jobs)

    base_ok = sum(1 for r in base.values() if r["verdict"] == "ok")
    variant_ok = sum(1 for r in variant.values() if r["verdict"] == "ok")
    print(f"\nverdicts: baseline {base_ok}/{len(base)} ok, variant {variant_ok}/{len(variant)} ok", flush=True)

    flipped = [
        (i, str(base[i]["verdict"]), str(variant[i]["verdict"]))
        for i in sorted(base)
        if base[i]["verdict"] != variant[i]["verdict"]
    ]
    if flipped:
        print("\nverdict changes:", flush=True)
        print_table(
            [[i, str(base[i]["label"]), was, now] for i, was, now in flipped],
            ["#", "scenario", "baseline", "variant"],
        )
    else:
        print("no verdict changed", flush=True)

    # Sim time only compares where both arms finished; a timed-out run's clock
    # measures the time limit, not the lap.
    both_ok = [i for i in sorted(base) if base[i]["verdict"] == "ok" and variant[i]["verdict"] == "ok"]
    if not both_ok:
        return
    deltas = [(i, float(variant[i]["sim_time_s"]) - float(base[i]["sim_time_s"])) for i in both_ok]
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
    extremes = {i: d for i, d in ranked[:3] + ranked[-3:]}
    print_table(
        [
            [i, str(base[i]["label"]), f"{base[i]['sim_time_s']:.1f}s", f"{variant[i]['sim_time_s']:.1f}s", f"{d:+.1f}s"]
            for i, d in sorted(extremes.items(), key=lambda kv: kv[1])
        ],
        ["#", "scenario", "baseline", "variant", "delta"],
    )


if __name__ == "__main__":
    main()

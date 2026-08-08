"""Same sweep as diag_open_exhaustive.py, across every core instead of one.

The headless simulator is pure computation -- the only ``time.sleep`` in
``src/simulation`` is in the RViz visualiser -- so a case already runs about 4.6x
faster than the race it simulates. It just runs one at a time: measured 2026-08-07,
~39 s of wall clock per case on a 16-core machine with 15 cores idle.

Cases are independent draws and each is seeded by its own index, so running them
concurrently changes nothing about what any single case does. Same seed, same
sample, same per-case seed, same verdicts -- only sooner. That matters because a
control change cannot be judged by the fast unit suite, and a validation loop
measured in tens of minutes is one that gets skipped.

Usage (from ``platform/robot``)::

    pixi run -e dev python scripts/sim/diag_open_parallel.py [--sample 24] [--jobs 14]
"""

from __future__ import annotations

import argparse
import os
import random
import sys
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.tables import print_table
from scripts.sim.diag_open_exhaustive import _SIDES, _all_cases, _summarise, _verdict
from src.simulation.scenario_builder import build_open_metadata
from src.simulation.scenario_simulator import ScenarioSimulator

_DEFAULT_LAPS = 3
_DEFAULT_SAMPLE_SIZE = 24
_DEFAULT_SEED = 0

_SPARE_CORES = 2
"""Cores left for the rest of the machine, so a sweep does not make it unusable."""


def _run_case(payload: tuple[int, tuple[int, ...], str, str, int, int]) -> dict[str, object]:
    """Run one scenario. Module-level and primitive-valued, so it can be pickled.

    Mirrors diag_open_exhaustive's loop body exactly, including seeding the
    simulator by case index -- that is what makes the parallel result identical
    to the serial one rather than merely similar.
    """
    from shared.config.enums import Direction, Section

    index, widths, section_value, direction_value, cell, laps = payload
    section = Section(section_value)
    direction = Direction(direction_value)
    widths_mm = dict(zip(_SIDES, widths, strict=True))
    meta = build_open_metadata(widths_mm, section, direction, scenario_id=index, start_cell=cell)
    result = ScenarioSimulator(meta, num_laps=laps, seed=index, blind=True).run()
    return {
        "index": index,
        "verdict": _verdict(result),
        "section": section.value,
        "direction": direction.value,
        "cell": cell,
        "start_width_mm": widths_mm[section.value.lower()],
        "laps": result.laps_completed,
        "target": result.target_laps,
        "sim_time_s": result.sim_time_s,
        "label": f"{'-'.join(str(w) for w in widths)} {section.value}/{direction.value} c{cell}",
    }


def main() -> None:
    """Run a seeded sample concurrently and summarise by dimension."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=_DEFAULT_LAPS)
    parser.add_argument("--sample", type=int, default=_DEFAULT_SAMPLE_SIZE)
    parser.add_argument("--seed", type=int, default=_DEFAULT_SEED)
    parser.add_argument("--all", action="store_true", help="Run all 640 instead of a sample.")
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    args = parser.parse_args()

    population = _all_cases()
    if args.all or args.sample >= len(population):
        cases = population
    else:
        # Same suppression and same reasoning as the serial script: the draw has
        # to be reproducible from a seed, which a cryptographic generator will
        # not do. Identical call, so identical sample.
        cases = random.Random(args.seed).sample(population, args.sample)  # noqa: S311

    jobs = args.jobs or max(1, (os.cpu_count() or 4) - _SPARE_CORES)
    print(f"{len(cases)} of {len(population)} scenarios, seed={args.seed}, {jobs} workers\n", flush=True)

    payloads = [
        (i, widths, section.value, direction.value, cell, args.laps)
        for i, (widths, section, direction, cell) in enumerate(cases)
    ]

    started = time.perf_counter()
    results: list[dict[str, object]] = []
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(_run_case, p): p[0] for p in payloads}
        for done in as_completed(futures):
            row = done.result()
            results.append(row)
            mark = "OK  " if row["verdict"] == "ok" else "FAIL"
            print(
                f"{mark} [{row['index']:>3}] {row['label']:<46} {row['verdict']:<10} "
                f"laps={row['laps']}/{row['target']} t={row['sim_time_s']:.1f}s "
                f"({len(results)}/{len(payloads)})",
                flush=True,
            )
    elapsed = time.perf_counter() - started

    # Sorted so the report reads the same regardless of completion order.
    results.sort(key=lambda r: r["index"])
    by_verdict: Counter[str] = Counter(str(r["verdict"]) for r in results)
    dims = {
        "by start section:": Counter((str(r["section"]), str(r["verdict"])) for r in results),
        "by direction:": Counter((str(r["direction"]), str(r["verdict"])) for r in results),
        "by start cell:": Counter((f"cell {r['cell']}", str(r["verdict"])) for r in results),
        "by start corridor width:": Counter((f"{r['start_width_mm']}mm", str(r["verdict"])) for r in results),
    }

    ok = by_verdict.get("ok", 0)
    print(f"\n{ok}/{len(results)} ok ({ok / len(results):.0%}) in {elapsed:.0f}s wall", flush=True)
    for verdict, count in by_verdict.most_common():
        if verdict != "ok":
            print(f"  {verdict:<12} {count}", flush=True)
    for title, counts in dims.items():
        _summarise(title, counts)

    failed = [r for r in results if r["verdict"] != "ok"]
    if failed:
        print("\nfailures:", flush=True)
        print_table(
            [
                [r["index"], r["label"], r["verdict"], f"{r['laps']}/{r['target']}", f"{r['sim_time_s']:.1f}s"]
                for r in failed
            ],
            ["#", "scenario", "verdict", "laps", "sim time"],
        )


if __name__ == "__main__":
    main()

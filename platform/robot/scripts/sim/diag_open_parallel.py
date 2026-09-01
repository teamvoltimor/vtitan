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

    pixi run -e dev python scripts/sim/diag_open_parallel.py [--sample 24] [--jobs 14] [--tuning custom.yaml]
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs

from scripts.common.diag_base import add_sweep_args, add_tuning_arg, load_tuning, resolve_jobs, run_pool, select_cases
from scripts.common.open_cases import SIDES, case_space
from scripts.common.tables import print_table
from scripts.sim.diag_open_exhaustive import _summarise, _verdict
from src.simulation.scenario_builder import build_open_metadata
from src.simulation.scenario_simulator import ScenarioSimulator

_DEFAULT_SAMPLE_SIZE = 24
_DEFAULT_SEED = 0


def _run_case(payload: tuple[int, tuple[int, ...], str, str, int, int, str | None]) -> dict[str, object]:
    """Run one scenario. Module-level and primitive-valued, so it can be pickled.

    Mirrors diag_open_exhaustive's loop body exactly, including seeding the
    simulator by case index -- that is what makes the parallel result identical
    to the serial one rather than merely similar. The tuning is passed as a
    path (or ``None``), not a loaded object, for the same picklability reason
    the enums are passed as their string values.
    """
    from shared.domain.enums import Direction, Section

    index, widths, section_value, direction_value, cell, laps, tuning_path = payload
    section = Section(section_value)
    direction = Direction(direction_value)
    widths_mm = dict(zip(SIDES, widths, strict=True))
    meta = build_open_metadata(widths_mm, section, direction, scenario_id=index, start_cell=cell)
    tuning = load_tuning(tuning_path)
    result = ScenarioSimulator(meta, num_laps=laps, tuning=tuning, seed=index, blind=True).run()
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
    add_sweep_args(
        parser,
        default_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        default_sample=_DEFAULT_SAMPLE_SIZE,
        default_seed=_DEFAULT_SEED,
        jobs=True,
    )
    add_tuning_arg(parser)
    args = parser.parse_args()

    # Fail on a bad --tuning here, before spending a sweep on it.
    load_tuning(args.tuning)

    population = case_space()
    cases = select_cases(population, sample=args.sample, seed=args.seed, all_=args.all, case=args.case)

    jobs = resolve_jobs(args.jobs)
    print(f"{len(cases)} of {len(population)} scenarios, seed={args.seed}, {jobs} workers\n", flush=True)

    payloads = [
        (i, widths, section.value, direction.value, cell, args.laps, args.tuning)
        for i, (widths, section, direction, cell) in cases
    ]

    def _print_progress(row: dict[str, object], done: int, total: int) -> None:
        mark = "OK  " if row["verdict"] == "ok" else "FAIL"
        print(
            f"{mark} [{row['index']:>3}] {row['label']:<46} {row['verdict']:<10} "
            f"laps={row['laps']}/{row['target']} t={row['sim_time_s']:.1f}s "
            f"({done}/{total})",
            flush=True,
        )

    started = time.perf_counter()
    results = run_pool(_run_case, payloads, jobs, on_result=_print_progress)
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

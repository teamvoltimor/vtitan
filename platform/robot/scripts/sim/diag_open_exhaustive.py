"""Sample the blind Open Challenge across the whole scenario space.

The 28 ``simgen`` fixtures are not a sample of it. They cover 24 of the 128
layout/section/direction combinations, only 13 of the 16 width layouts, and
exactly one starting in South -- and every one of them spawns on the corridor
centreline, which is not a legal starting position at all. The mat marks six
cells per side (bands of 40/20/40 cm out from the outer wall, each split into
two cells of 50 cm along), of which the innermost two fall inside the centre
square when that corridor is narrow, leaving four.

The full space is therefore 16 layouts x 4 sections x 2 directions x the 4 or 6
cells legal for that side: 640 scenarios.

Cases are drawn uniformly at random from all 640, seeded, so a run is a real
sample and is reproducible. Enumerating systematically instead correlates the
axes -- stepping the cell index alongside section and direction makes "South"
and "the middle band" the same cases, and a failure in one reads as a failure
in the other.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/sim/diag_open_exhaustive.py [--sample 128] [--seed 0] [--all]
"""

from __future__ import annotations

import argparse
import itertools
import random
import sys
from collections import Counter
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction, Section

from src.simulation.scenario_builder import build_open_metadata, start_cells
from src.simulation.scenario_simulator import ScenarioSimulator

_SIDES = ("south", "north", "east", "west")
_WIDTHS_MM = (int(CorridorDimensions.NARROW * 1000), int(CorridorDimensions.WIDE * 1000))

_DEFAULT_LAPS = 3
_DEFAULT_SAMPLE_SIZE = 128
_DEFAULT_SEED = 0


def _all_cases() -> list[tuple[tuple[int, ...], Section, Direction, int]]:
    """Every (layout, section, direction, start cell) the track can present."""
    cases = []
    for widths in itertools.product(_WIDTHS_MM, repeat=len(_SIDES)):
        widths_m = {k: v / 1000.0 for k, v in zip(_SIDES, widths, strict=True)}
        for section in Section:
            n_cells = len(start_cells(section, widths_m))
            for direction in Direction:
                cases.extend((widths, section, direction, cell) for cell in range(n_cells))
    return cases


def _verdict(result: Any) -> str:
    """Why this run failed, or ``ok``. One label, most severe first."""
    if result.collided:
        return "collision"
    if result.laps_completed < result.target_laps:
        return "incomplete"
    if result.over_time:
        return "over-time"
    return "ok"


def _summarise(title: str, counts: Counter[tuple[str, str]]) -> None:
    """Print an ok/total breakdown for one dimension."""
    print(f"\n{title}", flush=True)
    for key in sorted({k for k, _ in counts}):
        row = {v: c for (k, v), c in counts.items() if k == key}
        ok = row.pop("ok", 0)
        print(f"  {key:<18} {ok:>3}/{ok + sum(row.values()):<3} ok  {row or ''}", flush=True)


def main() -> None:
    """Run a seeded sample of the scenario space and summarise by dimension."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=_DEFAULT_LAPS)
    parser.add_argument("--sample", type=int, default=_DEFAULT_SAMPLE_SIZE, help="How many scenarios to draw.")
    parser.add_argument("--seed", type=int, default=_DEFAULT_SEED, help="Draw seed; same seed, same sample.")
    parser.add_argument("--all", action="store_true", help="Run all 640 instead of a sample.")
    args = parser.parse_args()

    population = _all_cases()
    if args.all or args.sample >= len(population):
        cases = population
    else:
        # Suppression is justified here: the draw must be reproducible from a
        # seed, which is the one thing a cryptographic generator will not do.
        cases = random.Random(args.seed).sample(population, args.sample)  # noqa: S311

    print(f"{len(cases)} de {len(population)} escenarios ({len(cases) / len(population):.1%}), seed={args.seed}\n", flush=True)

    by_verdict: Counter[str] = Counter()
    dims: dict[str, Counter[tuple[str, str]]] = {
        "por seccion de salida:": Counter(),
        "por direccion:": Counter(),
        "por celda de salida:": Counter(),
        "por ancho del pasillo de salida:": Counter(),
    }
    failures: list[str] = []

    for i, (widths, section, direction, cell) in enumerate(cases):
        widths_mm = dict(zip(_SIDES, widths, strict=True))
        meta = build_open_metadata(widths_mm, section, direction, scenario_id=i, start_cell=cell)
        # Seed by case index so a rerun of the same sample behaves identically.
        result = ScenarioSimulator(meta, num_laps=args.laps, seed=i, blind=True).run()

        verdict = _verdict(result)
        by_verdict[verdict] += 1
        dims["por seccion de salida:"][(section.value, verdict)] += 1
        dims["por direccion:"][(direction.value, verdict)] += 1
        dims["por celda de salida:"][(f"cell {cell}", verdict)] += 1
        dims["por ancho del pasillo de salida:"][(f"{widths_mm[section.value.lower()]}mm", verdict)] += 1

        label = f"{'-'.join(str(w) for w in widths)} {section.value}/{direction.value} c{cell}"
        line = (
            f"{'OK  ' if verdict == 'ok' else 'FAIL'} [{i:>3}] {label:<46} "
            f"{verdict:<10} laps={result.laps_completed}/{result.target_laps} "
            f"t={result.sim_time_s:.1f}s"
        )
        if verdict != "ok":
            failures.append(line)
        print(line, flush=True)

    print(f"\n{by_verdict['ok']}/{len(cases)} limpios -- {dict(by_verdict)}", flush=True)
    for title, counts in dims.items():
        _summarise(title, counts)

    if failures:
        print(f"\n{len(failures)} fallos:", flush=True)
        for line in failures:
            print(f"  {line}", flush=True)


if __name__ == "__main__":
    main()

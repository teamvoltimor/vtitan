"""Where did a run STALL in place, and what was it doing there?

Reported from the track on 2026-09-12: a round was stopped by hand because the
operator saw the car "in a loop". A loop and a slow lap look identical in the
inventory row (both are ticks without laps), so this locates the loop
geometrically instead: bucket every posed tick into a grid cell and rank cells
by dwell time.

A high-dwell cell is only meaningful next to a control, because the START cell
always dwells (the car sits there before the run begins) and so does the
finish hold. So the same pass reports, per hot cell, the phase mix and the
manoeuvre mix -- a genuine loop shows escape/manoeuvre churn, a legitimate stop
shows finished_hold or bay_exit -- and the run's own median cell dwell as the
floor to compare against.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_dwell_loop.py BAG [BAG ...]
"""

from __future__ import annotations

import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows, posed_rows  # noqa: E402

CELL_M = 0.25
"""Grid pitch. The chassis is 0.30 x 0.194 m, so a cell this size is entered
and left by ordinary driving; anything that dwells has stopped making
progress rather than merely passing through slowly."""

TOP_N = 6


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        posed = posed_rows(rows)
        if not posed:
            print(f"\n### {bag.name}: NO POSED ROWS")
            continue

        dwell: dict[tuple[int, int], float] = defaultdict(float)
        phases: dict[tuple[int, int], Counter[str]] = defaultdict(Counter)
        manoeuvres: dict[tuple[int, int], Counter[str]] = defaultdict(Counter)
        first_t: dict[tuple[int, int], float] = {}
        last_t: dict[tuple[int, int], float] = {}
        visits: dict[tuple[int, int], int] = defaultdict(int)
        prev_cell: tuple[int, int] | None = None

        for i, (t, s) in enumerate(posed):
            cell = (int(s.pose_x // CELL_M), int(s.pose_y // CELL_M))
            dt = (posed[i + 1][0] - t) if i + 1 < len(posed) else 0.0
            # A gap longer than a second is a recording hole, not dwell.
            dwell[cell] += min(dt, 1.0)
            phases[cell][str(s.phase)] += 1
            if s.active_maneuver_type is not None:
                manoeuvres[cell][str(s.active_maneuver_type)] += 1
            first_t.setdefault(cell, t)
            last_t[cell] = t
            if cell != prev_cell:
                visits[cell] += 1
                prev_cell = cell

        # Control for the per-cell manoeuvre mix: a manoeuvre can only be
        # blamed for a hot cell if the cell is hotter in it than the run is.
        run_man: Counter[str] = Counter()
        for _, s in posed:
            if s.active_maneuver_type is not None:
                run_man[str(s.active_maneuver_type)] += 1

        total = sum(dwell.values())
        median_cell = statistics.median(dwell.values())
        print(f"\n### {bag.name}")
        print(f"  posed ticks {len(posed)}  span {posed[-1][0] - posed[0][0]:.1f}s  cells {len(dwell)}")
        print(f"  median cell dwell {median_cell:.2f}s   <- the floor a hot cell must beat")
        man_total = sum(run_man.values())
        mix = ", ".join(f"{m}:{c} ({100 * c / man_total:.0f}%)" for m, c in run_man.most_common())
        print(f"  RUN-WIDE manoeuvre ticks {man_total} of {len(posed)} ({100 * man_total / len(posed):.0f}%): {mix}")
        print(f"  {'cell (x,y) m':>16} {'dwell s':>8} {'%run':>6} {'visits':>7}  window        phases / manoeuvres")

        for cell, d in sorted(dwell.items(), key=lambda kv: -kv[1])[:TOP_N]:
            cx, cy = cell[0] * CELL_M, cell[1] * CELL_M
            ph = ", ".join(f"{p}:{c}" for p, c in phases[cell].most_common(3))
            mn = ", ".join(f"{m}:{c}" for m, c in manoeuvres[cell].most_common(3)) or "-"
            print(
                f"  ({cx:5.2f},{cy:5.2f})   {d:8.1f} {100 * d / total:5.1f}% {visits[cell]:7d}"
                f"  {first_t[cell]:6.1f}-{last_t[cell]:6.1f}s  {ph}  |  {mn}",
            )


if __name__ == "__main__":
    main()

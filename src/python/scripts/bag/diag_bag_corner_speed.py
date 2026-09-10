r"""Why does the robot slow down through an Open corner, and for how long?

The operator's report is a dip of "about half a second" at each corner. Three
different mechanisms produce that shape and they want different fixes, so the
question is which one BINDS:

* the CORNER PREVIEW cap -- ``speed = min(speed, corner_mps())`` applied when
  ``FIRST_LAP_CORNER_CAUTION`` sees ``path_turn_ahead_rad``, on every lap once
  ``CORNER_CAUTION_ALL_LAPS`` is set. Deliberate, predictive, and cheap to
  change.
* the CLEARANCE ladder -- ``clearance_speed_mps``, reactive to forward range.
  Slowing because a wall got close is the controller working.
* the HEADING term -- ``heading_speed_mps``, reactive to angle error. Slowing
  because the robot is not pointed where it is going.

The snapshot publishes the last two but NOT the corner cap, so the cap is
identified by RESIDUAL: a tick whose commanded speed sits below both published
terms was cut by something else, and through a corner that something is the
preview cap. Ticks are attributed to the LOWEST term, and the residual bucket
is reported rather than hidden, because a large residual means the attribution
is wrong and not that the cap is guilty.

TRAP: ``current_corridor`` FLAPS -- 37-39 transitions in a 3-lap run that has
12 real corners -- so the transition count is reported but is NOT usable to
split corner ticks from straight ones, and no such split is attempted here.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_corner_speed.py \
        data/live/runs/run_20260908_003520 data/live/runs/run_20260908_004023
"""

from __future__ import annotations

import statistics
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader

WINDOW_S = 3.0
"""Seconds either side of a corridor transition to report."""

CRAWL_MARGIN = 1e-6
"""A tick is CRAWLING when the commanded speed matches the heading term's floor.

Defining the slowdown relative to the run's own MEDIAN speed measured nothing:
the crawl is the MAJORITY state (44-63% of a run), so the median IS the crawled
value and the detector found zero dips in two runs that crawl for 96 s and 45 s.
The floor is read from the data instead -- the heading term is binary, taking
either ``creep_mps()`` or ``fast_mps()`` and nothing between."""


def rows_for(bag_dir: Path):
    reader = open_reader(bag_dir)
    t0 = None
    out = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic != Topics.NAV_DEBUG:
            continue
        snap = decode_nav_debug(data)
        if str(getattr(snap.phase, "value", snap.phase)) != "normal_drive":
            continue
        if snap.commanded_speed_mps is None:
            continue
        out.append((elapsed_seconds(t, t0), snap))
    return out


def analyse(bag_dir: Path) -> dict | None:
    """Crawl episodes and what bound the speed on them."""
    rows = rows_for(bag_dir)
    if len(rows) < 20:
        return None

    corners: list[float] = []
    prev = None
    for ts, snap in rows:
        c = snap.current_corridor
        if c is None:
            continue
        c = str(getattr(c, "value", c))
        if prev is not None and c != prev:
            corners.append(ts)
        prev = c

    headings = [s.heading_speed_mps for _, s in rows if s.heading_speed_mps is not None]
    if not headings:
        return None
    floor = min(headings)

    episodes: list[tuple[float, float]] = []
    start = None
    for ts, s in rows:
        crawling = s.heading_speed_mps is not None and s.heading_speed_mps <= floor + CRAWL_MARGIN
        if crawling and start is None:
            start = ts
        elif not crawling and start is not None:
            episodes.append((start, ts - start))
            start = None
    if start is not None:
        episodes.append((start, rows[-1][0] - start))

    attrib: Counter[str] = Counter()
    for _, s in rows:
        cmd = s.commanded_speed_mps
        live = {k: v for k, v in (("clearance", s.clearance_speed_mps), ("heading", s.heading_speed_mps)) if v is not None}
        if not live:
            attrib["unpublished"] += 1
        elif cmd < min(live.values()) - 1e-6:
            attrib["residual (corner cap / envelope)"] += 1
        else:
            attrib[min(live, key=lambda k: live[k])] += 1

    durs = [d for _, d in episodes]
    span = rows[-1][0] - rows[0][0]
    return {
        "run": bag_dir.name[-6:],
        "ticks": len(rows),
        "corners": len(corners),
        "floor": floor,
        "episodes": len(durs),
        "med_s": statistics.median(durs) if durs else 0.0,
        "max_s": max(durs) if durs else 0.0,
        "share": 100.0 * sum(durs) / span if span else 0.0,
        "attrib": attrib,
    }


def main() -> None:
    results = [r for r in (analyse(Path(a)) for a in sys.argv[1:] if Path(a).is_dir()) if r]
    if not results:
        print("no bag had enough normal_drive ticks")
        return

    hdr = f"{'run':8}{'ticks':>7}{'transit':>8}{'floor':>7}{'crawls':>8}{'medS':>7}{'maxS':>7}{'share%':>8}"
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        print(f"{r['run']:8}{r['ticks']:7}{r['corners']:8}{r['floor']:7.3f}"
              f"{r['episodes']:8}{r['med_s']:7.2f}{r['max_s']:7.2f}{r['share']:8.0f}")

    total: Counter[str] = Counter()
    for r in results:
        total.update(r["attrib"])
    n = sum(total.values())
    print(f"\nwhat bound the speed across all {n} normal_drive ticks:")
    for k, v in total.most_common():
        print(f"  {k:24} {v:6}  {100.0 * v / n:5.1f}%")


if __name__ == "__main__":
    main()

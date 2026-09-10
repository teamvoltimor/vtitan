r"""What state does an escape leave the robot in, and why does it re-trigger?

Rotation and duration transfer from the simulator almost exactly (sim 22.2 deg /
1.05 s against hardware 19.0 deg / 0.97 s), yet the OUTCOME does not: raising
``MAX_ESCAPE_S`` 1.0 -> 1.8 makes the sim need FEWER escapes at flat total cost,
and makes the robot need MORE, doubling time spent reversing to 31% of the round.
So the difference is not the manoeuvre -- it is the state the manoeuvre leaves
behind. Two causes want opposite fixes:

* **Released too early** -- the escape ends while the robot is still wedged and
  the next tick re-triggers. Tell: a short gap to the next escape and low
  forward clearance at release. Wants a CLEARANCE-based termination instead of
  a time-based one.
* **Cleared, then drove back in** -- the escape worked and the path steered
  straight back into the same corner. Tell: a longer gap, healthy clearance at
  release, and the re-trigger happening near the same pose. Wants a fix to what
  the robot does AFTER recovery, not to the escape.

Reported per episode: forward clearance at release, the gap to the next escape,
and how far the robot travelled in between. Re-triggers within ``IMMEDIATE_S``
are counted separately -- those are the ones that never really recovered.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_escape_aftermath.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows  # noqa: E402
from scripts.common.episodes import reversing_spans
from scripts.common.tables import print_table  # noqa: E402

IMMEDIATE_S = 1.0
"""A re-trigger sooner than this never recovered; it resumed."""

WEDGED_M = 0.20
"""Forward clearance at release below this means the escape released into a wall."""


@dataclass
class Aftermath:
    """One escape episode, described by what followed it."""

    run: str
    release_clearance_m: float | None
    gap_s: float | None
    travelled_m: float
    rotation_deg: float


def _episodes(run: str, rows: list) -> list[Aftermath]:
    """Reduce each reversing episode to the state it left behind."""
    spans = reversing_spans(rows)
    out: list[Aftermath] = []
    for i, span in enumerate(spans):
        nxt = spans[i + 1] if i + 1 < len(spans) else None
        gap = nxt.t_end - span.t_end if nxt else None
        moved = math.hypot(nxt.end_x - span.end_x, nxt.end_y - span.end_y) if nxt else 0.0
        out.append(Aftermath(run, span.end_row.forward_clearance_m, gap, moved, span.turned_deg))
    return out


def main() -> None:
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()

    episodes: list[Aftermath] = []
    for bag in args.bag_dirs:
        rows, _ = load_nav_debug_rows(str(bag))
        episodes.extend(_episodes(Path(bag).name.replace("run_", ""), rows))

    if not episodes:
        print("No escape episodes in these bags.")
        return

    table = []
    for run in sorted({e.run for e in episodes}):
        group = [e for e in episodes if e.run == run]
        gaps = [e.gap_s for e in group if e.gap_s is not None]
        clears = [e.release_clearance_m for e in group if e.release_clearance_m is not None]
        table.append(
            [
                run,
                len(group),
                round(statistics.median(gaps), 2) if gaps else None,
                sum(1 for g in gaps if g < IMMEDIATE_S),
                round(statistics.median(clears), 3) if clears else None,
                round(statistics.median(e.travelled_m for e in group), 3),
            ]
        )
    print(f"== ESCAPE AFTERMATH  ({len(episodes)} episodes)")
    print_table(
        table,
        ["run", "episodes", "med gap s", f"re-fire <{IMMEDIATE_S}s", "med release clr m", "med moved m"],
    )

    gaps = [e.gap_s for e in episodes if e.gap_s is not None]
    clears = [e.release_clearance_m for e in episodes if e.release_clearance_m is not None]
    immediate = [e for e in episodes if e.gap_s is not None and e.gap_s < IMMEDIATE_S]
    print()
    if gaps:
        gs = sorted(gaps)
        print(f"  gap to next escape s: p10 {gs[len(gs) // 10]:.2f} / median {statistics.median(gs):.2f} / p90 {gs[int(len(gs) * 0.9)]:.2f}")
        print(f"  re-fired within {IMMEDIATE_S}s: {len(immediate)}/{len(gaps)} ({100 * len(immediate) / len(gaps):.0f}%)")
    if clears:
        cs = sorted(clears)
        print(f"  forward clearance at release m: p10 {cs[len(cs) // 10]:.3f} / median {statistics.median(cs):.3f}")
        print(f"  released with under {WEDGED_M} m ahead: {sum(1 for c in cs if c < WEDGED_M)}/{len(cs)}")
    if immediate:
        ic = [e.release_clearance_m for e in immediate if e.release_clearance_m is not None]
        if ic:
            print(f"  ...of those that re-fired fast, median release clearance: {statistics.median(ic):.3f} m")
        print(f"  ...median distance travelled before re-firing: {statistics.median(e.travelled_m for e in immediate):.3f} m")


if __name__ == "__main__":
    main()

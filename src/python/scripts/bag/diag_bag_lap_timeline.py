r"""What did each LAP cost, and which one lost the round?

"Laps completed" is the wrong success metric and it hid the state of the car for
a whole session. Obstacles rounds reported three laps often enough to look
healthy while almost none were scoreable inside the time limit: a cruise lap
costs well under the round limit, so three fit, and every round that ran over did
so because of a SINGLE long wedge inside one lap. Averaged over the round that
wedge disappears into a slightly slow run. Split by lap, it is the whole finding,
and it names the lap to go and look at. See
``adr:0055-escape-maneuver-selection`` for the measured verdict.

This prints one block per bag: a row per lap segment, bounded by the lap-credit
ticks, carrying the things that separate a lap that was merely slow from a lap
that was stuck:

* **esc** -- escape/manoeuvre EPISODES in the segment, segmented on the
  manoeuvre being active. NOT ``escape_count``, which resets on the escape's own
  reverse and reads 1 on 96 percent of triggers.
* **manTicks** -- ticks per manoeuvre type. A lap dominated by ``K_TURN`` and one
  dominated by ``SIDE_CORRECTION`` are different failures, and the k_turn is the
  bulk of manoeuvre time and much of Obstacles overtime.
* **stuckTicks** -- whether the stuck detector agreed with the escape count.
* **near** and **subfloor** -- proximity ticks split at ``min_valid_range_m``.
  They are printed apart because ``min_lidar_range_m`` sits BELOW the filter
  floor on effectively every tick, so a single "under 8 cm" column reads full
  every lap and means nothing. ``near`` is the credible half.
* **phases** -- the top few navigator phases, which is where a lap that spent
  its time in ``BAY_EXIT`` rather than driving shows itself.

Then a terminal block over the last 30 s: the bounding box the chassis stayed
inside, and the corridor and waypoint index it was holding. A pose box under
about 0.3 m across is a car that ended the round pinned in one place, and the
waypoint index printed beside it says whether the planner had also given up
(a frozen index) or was still asking for a point the car could not reach.

TRAP: the final segment after the last lap credit is usually a fraction of a
second of ``FINISHED_HOLD``, not a lap. It is labelled and left in rather than
dropped, because a round that ends with a LONG unfinished segment is the
interesting case and silently trimming the tail hides it.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_lap_timeline.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
from collections import Counter
from typing import TYPE_CHECKING

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows
from scripts.common.fidelity_axes import contiguous_episodes

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.models import NavigatorDebugSnapshot

_NEAR_CONTACT_M = 0.08
"""A LIDAR minimum under this is the car touching or about to touch something.

Not the escape trigger's threshold and not meant to be: this is a descriptive
counter for reading a lap, and the contact instruments that adjudicate live in
``diag_bag_contact_*.py``.
"""

_SUB_FLOOR_M = 0.044
"""``min_valid_range_m``. Readings below it are counted SEPARATELY, not as contact.

``min_lidar_range_m`` sits below this floor on effectively every tick. A naive
"ticks under 8 cm" counter therefore reads full on every lap of every round and
carries no information at all -- it is measuring the sub-floor artefact
(``diag_bag_subfloor_ranges.py``), not proximity. Splitting the two is the
difference between a column that says something and a column that always says
everything.
"""

_TERMINAL_WINDOW_S = 30.0
_PINNED_SPAN_M = 0.30


def _episodes(rows: Sequence[tuple[float, NavigatorDebugSnapshot]]) -> int:
    """Manoeuvre episodes in a slice of ticks."""
    return len(contiguous_episodes([snap.active_maneuver_type is not None for _t, snap in rows]))


def _name(value: object) -> str:
    return getattr(value, "name", "-") if value is not None else "-"


def _segment_line(label: str, start: float, end: float, rows: Sequence[tuple[float, NavigatorDebugSnapshot]]) -> str:
    manoeuvres = Counter(_name(s.active_maneuver_type) for _t, s in rows if s.active_maneuver_type)
    phases = Counter(_name(s.phase) for _t, s in rows if s.phase)
    stuck = sum(1 for _t, s in rows if s.is_stuck)
    minima = [s.min_lidar_range_m for _t, s in rows if s.min_lidar_range_m is not None]
    near = sum(1 for m in minima if _SUB_FLOOR_M <= m < _NEAR_CONTACT_M)
    sub = sum(1 for m in minima if m < _SUB_FLOOR_M)
    return (
        f"  {label:16} {start:6.1f}-{end:6.1f}s ({end - start:5.1f}s) esc={_episodes(rows):3d} "
        f"man={dict(manoeuvres)} stuckTicks={stuck} "
        f"near={near}/{len(minima)} subfloor={sub}/{len(minima)} "
        f"phases={dict(phases.most_common(3))}"
    )


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--window",
        type=float,
        default=_TERMINAL_WINDOW_S,
        help=f"Terminal window to characterise, seconds (default {_TERMINAL_WINDOW_S:.0f})",
    )
    args = parser.parse_args()

    for bag_dir in args.bag_dirs:
        rows, _ = load_nav_debug_rows(bag_dir)
        if not rows:
            print(f"\n=== {bag_dir.name}: no /nav_debug rows")
            continue

        # The navigator publishes one post-run RESET snapshot after the round:
        # phase NOT_YET_STEPPED, laps 0, num_laps 0. Reading the summary off
        # rows[-1] therefore reports "final_phase=NOT_YET_STEPPED laps=3/0" for a
        # round that actually finished three laps and held. Trim the trailing
        # reset; do NOT trim on num_laps == 0, which also covers the opening
        # BAY_EXIT ticks and those are part of the round.
        trimmed = list(rows)
        while trimmed and _name(trimmed[-1][1].phase) == "NOT_YET_STEPPED":
            trimmed.pop()
        if not trimmed:
            print(f"\n=== {bag_dir.name}: every row is NOT_YET_STEPPED -- the navigator never ran")
            continue

        t0 = trimmed[0][0]
        timeline = [(t - t0, snap) for t, snap in trimmed]
        duration = timeline[-1][0]
        target_laps = max((s.num_laps for _t, s in timeline if s.num_laps), default=0)

        lap_credits: list[float] = []
        seen_laps = 0
        for t, snap in timeline:
            if snap.laps_completed is not None and snap.laps_completed > seen_laps:
                lap_credits.append(t)
                seen_laps = snap.laps_completed

        last = timeline[-1][1]
        dropped = len(rows) - len(trimmed)
        reset = f" (+{dropped} post-run reset tick{'s' if dropped > 1 else ''} trimmed)" if dropped else ""
        print(
            f"\n=== {bag_dir.name}  dur={duration:.1f}s ticks={len(timeline)}{reset} "
            f"laps={seen_laps}/{target_laps} final_phase={_name(last.phase)}"
        )
        directions = Counter(_name(s.direction) for _t, s in timeline)
        print(f"  direction ticks: {dict(directions.most_common(3))}")

        bounds = [0.0, *lap_credits, duration]
        for i in range(len(bounds) - 1):
            start, end = bounds[i], bounds[i + 1]
            segment = [(t, s) for t, s in timeline if start <= t < end]
            if not segment:
                continue
            credited = i < len(lap_credits)
            label = f"L{i}" if credited else f"L{i} UNFINISHED"
            print(_segment_line(label, start, end, segment))

        tail = [(t, s) for t, s in timeline if t >= duration - args.window]
        xs = [s.pose_x for _t, s in tail if s.pose_x is not None]
        ys = [s.pose_y for _t, s in tail if s.pose_y is not None]
        if xs and ys:
            span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            corridors = Counter(_name(s.current_corridor) for _t, s in tail)
            waypoints = Counter(s.waypoint_index for _t, s in tail)
            verdict = "  <-- PINNED" if span < _PINNED_SPAN_M else ""
            print(
                f"  last{args.window:.0f}s: pose span={span:.2f}m centre=({sum(xs) / len(xs):.2f},"
                f"{sum(ys) / len(ys):.2f}) esc={_episodes(tail)} "
                f"corridors={dict(corridors.most_common(2))} wp={dict(waypoints.most_common(2))}{verdict}"
            )

    print(
        f"\nA lap that merely ran slow and a lap that WEDGED look the same in a lap count. Read the\n"
        f"per-lap seconds against the ~41-53 s a cruise lap costs: one segment far above that, with\n"
        f"escapes concentrated in it, is a wedge. A terminal pose span under {_PINNED_SPAN_M} m means the\n"
        f"round ended pinned in one place -- diag_bag_wedge_trace.py separates the three causes."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

r"""Did the car ever drive the loop BACKWARDS, and for how far?

The mechanism that lost all three 2026-09-11 evening rounds: a burst of k_turns
accumulates a 180-300 deg heading change, the lookahead search re-acquires the
path happily in the OTHER direction, and the car drives up to 0.63 of a lap the
wrong way at POSITIVE commanded speed. See
``kturn_flips_heading_and_waypoint_index_freezes_2026_09_11``.

TRAP, and it produced a wrong "no reversal" verdict once already: instantaneous
wrong-way arcs read ZERO in every affected run. Per-tick angular velocity is
dominated by noise and by the car's own weaving, and the reversal is a slow
accumulation. It only appears as **drawdown from the running maximum** of the
unwrapped angle about the mat centre, signed so that progress is positive.

So that is the only metric here. ``waypoint_index`` is reported alongside, but
only as corroboration: it FREEZES rather than stepping backward, which is why
every backward-jump guard in the tree reads clean through a reversal.

A positive control belongs in the same invocation. Pass a run that is known to
have finished 3 clean laps -- it should report drawdown at noise level (~18 deg)
against 86-227 deg on an affected run. Without it a low number cannot be told
apart from a broken reader.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_lap_drawdown.py \
        data/live/runs/run_20260912_06* data/live/runs/run_20260911_152318
"""

from __future__ import annotations

import math
import sys
from itertools import pairwise
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from shared.domain.enums import Direction

from scripts.common.bag_io import (
    create_bags_parser,
    load_nav_debug_rows,
    posed_rows,
    settled_direction,
)
from scripts.common.tables import print_table

if TYPE_CHECKING:
    from collections.abc import Sequence

BAY_SETTLE_S = 25.0
"""Ticks before this are the in-bay pocket, whose walls are not in the corridor
model and whose angle about the mat centre swings wildly for millimetres of
travel. Every run including the clean controls looks bad there."""

ON_LOOP_DEG = 45.0
"""A wall-clock floor is not enough, and the control proved it: at 25 s the
clean 3-lap run of 2026-09-11 still scored 106 deg of drawdown in a 25.1-31.4 s
window, against the 18 deg it is known to deserve. The bay exit does not finish
on a schedule. So the scan starts only once the car has actually made this much
angular progress about the mat centre, which is direction-agnostic and needs no
clock. Values under a quarter-corner would still admit the pocket."""

MIN_TICKS = 50
"""Fewer post-bay ticks than this is a run that never left the pocket."""

MANEUVER_KINDS = ("k_turn", "side_correction")
"""The two latched types worth splitting: a k_turn is a deliberate
re-orientation, side_correction is the reactive layer taking the wheel."""

NOISE_DEG = 25.0
"""Drawdown at or below this is weaving, not a reversal: the clean 3-lap control
of 2026-09-11 measured 18 deg over 1080 deg of progress."""


def _progress_deg(rows: Sequence[tuple[float, object]], centre: tuple[float, float], sign: float) -> list[tuple[float, float]]:
    """Unwrapped angle about `centre`, signed so forward progress increases."""
    out: list[tuple[float, float]] = []
    prev: float | None = None
    total = 0.0
    for t, s in rows:
        a = math.atan2(s.pose_y - centre[1], s.pose_x - centre[0])
        if prev is not None:
            d = a - prev
            # Unwrap: a genuine step between 20 Hz ticks is far under pi, so any
            # jump past pi is the atan2 branch cut, not motion.
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            total += d
        prev = a
        out.append((t, math.degrees(total) * sign))
    return out


def _maneuver_rate(
    rows: Sequence[tuple[float, object]], lo: float, hi: float, kind: str
) -> tuple[int, int]:
    """Latched-manoeuvre ticks of `kind` in `[lo, hi]`, and the ticks in range."""
    sel = [s for t, s in rows if lo <= t <= hi]
    hit = sum(
        1 for s in sel
        if s.active_maneuver_type is not None and kind in str(s.active_maneuver_type)
    )
    return hit, len(sel)


def _worst_drawdown(series: list[tuple[float, float]]):  # noqa: ANN202
    """Largest regression from the running maximum, and the window it spans."""
    peak = -math.inf
    peak_t = 0.0
    worst = 0.0
    window = (0.0, 0.0, 0.0)
    for t, v in series:
        if v > peak:
            peak, peak_t = v, t
        if peak - v > worst:
            worst = peak - v
            window = (peak_t, t, peak)
    return worst, window


def main() -> None:
    """Report backward travel per bag, with a positive control alongside."""
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()

    rows_out = []
    for bag in args.bag_dirs:
        run = Path(bag).name.replace("run_", "")
        try:
            rows = posed_rows(load_nav_debug_rows(Path(bag))[0])
        except (RuntimeError, OSError, ValueError) as exc:
            rows_out.append([run, "-", type(exc).__name__, "-", "-", "-", "-", "UNREADABLE"])
            continue
        rows = [(t, s) for t, s in rows if t >= BAY_SETTLE_S]
        if len(rows) < MIN_TICKS:
            rows_out.append([run, "-", "too few post-bay ticks", "-", "-", "-", "-", "SKIP"])
            continue
        direction = settled_direction(rows)
        # The mat centre is the midpoint of the travelled extent, not a constant:
        # the track origin is not in the repo for a blind-mode run, and a loop's
        # own bounding-box centre is within centimetres of the mat centre.
        xs = [s.pose_x for _, s in rows]
        ys = [s.pose_y for _, s in rows]
        centre = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
        # Clockwise travel decreases the angle, so flip it to make progress
        # positive in both directions and keep one drawdown convention.
        sign = -1.0 if direction is Direction.CLOCKWISE else 1.0
        series = _progress_deg(rows, centre, sign)
        # Drop everything before the car is demonstrably on the loop, then
        # re-zero so the reported peak is progress since that moment.
        on_loop = next((i for i, (_, v) in enumerate(series) if v >= ON_LOOP_DEG), None)
        if on_loop is None:
            rows_out.append([run, str(direction), round(series[-1][1], 0),
                             "-", "-", "-", "never on loop", "-", "SKIP"])
            continue
        base = series[on_loop][1]
        series = [(t, v - base) for t, v in series[on_loop:]]
        worst, (t_peak, t_end, peak_v) = _worst_drawdown(series)
        total = series[-1][1] - series[0][1]
        # Corroboration only: the index freezes through a reversal rather than
        # stepping back, so a frozen span is the expected signature, not proof.
        idx = [s.waypoint_index for _, s in rows if isinstance(s.waypoint_index, int)]
        frozen = 0
        run_len = 0
        for a, b in pairwise(idx):
            run_len = run_len + 1 if b == a else 0
            frozen = max(frozen, run_len)
        # The causal link the metric alone cannot make: if the reversal is the
        # k_turn mechanism, manoeuvre ticks are ENRICHED inside the drawdown
        # window against the same run's own rate outside it. Same run is the
        # control, so a run-to-run difference in escape rate cannot fake it.
        inside = {k: _maneuver_rate(rows, t_peak, t_end, k) for k in MANEUVER_KINDS}
        out_pre = {k: _maneuver_rate(rows, 0.0, t_peak, k) for k in MANEUVER_KINDS}
        enrich = []
        for k in MANEUVER_KINDS:
            hi, hn = inside[k]
            lo, ln = out_pre[k]
            r_in = hi / hn if hn else 0.0
            r_out = lo / ln if ln else 0.0
            enrich.append(f"{k.split('_')[0]} {100 * r_in:.0f}%/{100 * r_out:.0f}%")
        rows_out.append([
            run,
            str(direction),
            round(total, 0),
            round(peak_v, 0),
            round(worst, 0),
            round(worst / 360.0, 2),
            f"{t_peak:.1f}-{t_end:.1f}s",
            " ".join(enrich),
            frozen,
            "REVERSAL" if worst > NOISE_DEG else "clean",
        ])

    print("== BACKWARD TRAVEL as drawdown from the running max of loop angle")
    print_table(
        rows_out,
        ["run", "direction", "net deg", "peak deg", "drawdown deg", "laps back",
         "window", "in-window/before rate", "max frozen idx", "verdict"],
    )
    print()
    print(f"  a drawdown over {NOISE_DEG:.0f} deg is a reversal; the 2026-09-11 clean 3-lap control read 18")
    print("  affected 2026-09-11 runs read 86 / 220 / 227 deg -- if today reads clean, check the control read clean too")


if __name__ == "__main__":
    main()

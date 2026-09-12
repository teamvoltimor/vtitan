r"""Does the steer target point the WRONG WAY ROUND THE LOOP?

The reversal that lost all three 2026-09-11 evening rounds and still cost one
of three on 2026-09-12 ([[lap_reversal_survives_the_search_span_fix_2026_09_12]])
is the car driving a quarter to two thirds of a lap backwards at POSITIVE
commanded speed. ``target_search_span_cured_the_reversal_2026_09_11`` bounded
the lookahead search to 1.0 m of path and it did not close.

**Why a span bound cannot close it, which is the hypothesis under test.** The
question already answered is "is the target BEHIND the chassis", and it reads
0.0% of ticks -- so the car is not aiming backwards in its own frame. But a
closed loop has two tangent directions at every point, and a target one metre
along the path in the WRONG one is still one metre away and still dead ahead of
a chassis that has already rotated. The bound constrains distance. Nothing in
it constrains SENSE.

So this measures sense, not distance. At each tick, the direction of correct
travel is the tangent about the mat centre signed by the adopted ``direction``
-- no track model needed, and it is exactly the convention
``diag_bag_lap_drawdown.py`` scores progress in. Three bearings are then
compared against it:

* **target sense** -- pose to ``steer_target``. Negative projection means the
  point the controller is steering at lies the wrong way round the loop. If
  this is high inside a reversal window, target SELECTION is the defect and the
  fix belongs in the search, not in the span.
* **heading sense** -- ``pose_yaw``. How far the chassis itself has turned
  round. Expected to follow the target, not to lead it.
* **motion sense** -- the pose delta. The outcome, and the only one of the
  three that cannot be an artefact of a debug field.

``crosstrack_error_m`` is reported alongside because the tracker believing it
is on-path throughout is what makes this invisible to every existing guard.

A POSITIVE CONTROL IS REQUIRED in the same invocation. A run known to have
finished 3 clean laps must read near zero on all three; without it a low number
cannot be told apart from a sign convention that came out backwards.

WHAT IT MEASURED, 2026-09-12, and the hypothesis above came out INVERTED. Inside
each known reversal window, against the clean 3-lap control of 2026-09-11:

| run | target wrong | target behind | heading wrong |
|---|---|---|---|
| `152318` control, whole span | 0.7% | 0.0% | 0.0% |
| `171915` pre-fix | 90.9% | **0.0%** | 98.5% |
| `172543` pre-fix | 56.2% | **0.0%** | 99.6% |
| `172334` pre-fix | 71.1% | **0.0%** | 100.0% |
| `064539` POST-fix | **2.8%** | **35.4%** | 98.6% |

Three exact zeros reproduce the recorded "the target is essentially never behind
the chassis", which is what validates this reader against an independent result.

**The span fix worked at what it aimed at and CONVERTED the failure.** Wrong-sense
targets inside the window collapsed from 56-91% to 2.8%. The target now asks
correctly for the way back -- and to ask it, it sits BEHIND the chassis on 35.4%
of ticks, where before it was 0.0% exactly. The car still does not recover, so
the defect moved DOWNSTREAM of selection: nothing turns a chassis round to chase
a target behind it, and ``crosstrack_error_m`` stays at 0.142 m p50 throughout,
so no guard fires.

Selection is still the PRECURSOR, not the exonerated party. Splitting `064539`
at its own window boundary: before it, target-wrong 23.0% with heading-wrong
only 6.6% and behind 3.0%. The wrong-sense target still fires while the chassis
is correctly oriented, and that is what rotates it.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_target_loop_sense.py \
        data/live/runs/run_20260912_064539 data/live/runs/run_20260911_152318
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
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
ON_LOOP_DEG = 45.0
"""Both carried from ``diag_bag_lap_drawdown.py`` for the same reason: the bay
pocket swings the angle about the mat centre wildly for millimetres of travel,
and the bay exit does not finish on a schedule, so the scan starts on angular
progress rather than on the clock."""

MIN_TICKS = 50
MOTION_EPS_M = 0.002
"""Pose deltas under this are localizer jitter, not travel, and their bearing
is meaningless. At 20 Hz and 0.25 m/s a real step is ~12 mm."""

TARGET_WRONG_THRESHOLD = 0.05
"""Calibrated on the control, not chosen: the clean 3-lap run of 2026-09-11
reads 0.7% target-wrong, and every reversed run reads 16-36%. Anything in
between is unobserved, so the threshold sits an order of magnitude above the
control and well below every positive.

The MOTION column gets no threshold, because the control REFUTED one. That run
reads 14.2% motion-wrong while reading 0.0% heading-wrong: a per-tick pose
delta projected on the tangent is dominated by weaving, and on a 0.25 m/s
chassis at 20 Hz the lateral component of a weave routinely exceeds the
tangential one. Motion-wrong is kept as context and must not be scored."""


def _tangent(px: float, py: float, centre: tuple[float, float], sign: float) -> tuple[float, float]:
    """Unit vector of CORRECT travel at `(px, py)`: the tangent about `centre`.

    `sign` is +1 counterclockwise, -1 clockwise, matching the drawdown script.
    """
    rx, ry = px - centre[0], py - centre[1]
    n = math.hypot(rx, ry)
    if n == 0.0:
        return (0.0, 0.0)
    # Rotate the outward radial by +90 deg for ccw, -90 for cw.
    return (-ry / n * sign, rx / n * sign)


def _frac_wrong(pairs: Sequence[tuple[float, float]]) -> tuple[float, int]:
    """Fraction of `(projection, _)` pairs pointing against correct travel."""
    if not pairs:
        return (0.0, 0)
    wrong = sum(1 for p, _ in pairs if p < 0.0)
    return (wrong / len(pairs), len(pairs))


def _progress_deg(rows: Sequence[tuple[float, object]], centre: tuple[float, float], sign: float) -> list[float]:
    """Unwrapped angle about `centre`, signed so forward progress increases."""
    out: list[float] = []
    prev: float | None = None
    total = 0.0
    for _t, s in rows:
        a = math.atan2(s.pose_y - centre[1], s.pose_x - centre[0])
        if prev is not None:
            d = a - prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            total += d
        prev = a
        out.append(math.degrees(total) * sign)
    return out


UNIT_EPS = 1e-9
"""Below this a difference of two poses has no defined bearing."""


def _unit(dx: float, dy: float, floor: float = UNIT_EPS) -> tuple[float, float] | None:
    """`(dx, dy)` normalised, or None when it is too short to have a bearing."""
    n = math.hypot(dx, dy)
    return None if n <= floor else (dx / n, dy / n)


@dataclass
class Senses:
    """Per-tick projections, each `(projection, elapsed_s)`.

    A negative projection is the thing pointing against its reference.
    """

    target: list[tuple[float, float]] = field(default_factory=list)
    """Pose to steer target, against the loop tangent."""
    heading: list[tuple[float, float]] = field(default_factory=list)
    """Chassis yaw, against the loop tangent."""
    motion: list[tuple[float, float]] = field(default_factory=list)
    """Pose delta, against the loop tangent. Context only -- see MOTION note."""
    behind: list[tuple[float, float]] = field(default_factory=list)
    """Pose to steer target, against the HEADING. This is the disambiguation
    the two tangent columns cannot make on their own: a target that is
    correct-sense while the heading is wrong-sense has to be BEHIND the
    chassis, and a prior measurement put target-behind at 0.0% of ticks. Both
    cannot hold, so measure it in the same pass rather than reasoning about
    which one gave way."""
    crosstrack: list[float] = field(default_factory=list)


def _analyse(rows, centre, sign) -> Senses:  # noqa: ANN001
    """Per-tick sense projections for target, heading and actual motion."""
    out = Senses()
    for i, (t, s) in enumerate(rows):
        tangent = _tangent(s.pose_x, s.pose_y, centre, sign)
        if tangent == (0.0, 0.0):
            continue
        tx, ty = tangent
        to_target = None
        if isinstance(s.steer_target_x, (int, float)) and isinstance(s.steer_target_y, (int, float)):
            to_target = _unit(s.steer_target_x - s.pose_x, s.steer_target_y - s.pose_y)
        if to_target is not None:
            out.target.append((to_target[0] * tx + to_target[1] * ty, t))
        if isinstance(s.pose_yaw, (int, float)):
            hx, hy = math.cos(s.pose_yaw), math.sin(s.pose_yaw)
            out.heading.append((hx * tx + hy * ty, t))
            if to_target is not None:
                out.behind.append((to_target[0] * hx + to_target[1] * hy, t))
        if i + 1 < len(rows):
            nxt = rows[i + 1][1]
            step = _unit(nxt.pose_x - s.pose_x, nxt.pose_y - s.pose_y, MOTION_EPS_M)
            if step is not None:
                out.motion.append((step[0] * tx + step[1] * ty, t))
        if isinstance(s.crosstrack_error_m, (int, float)):
            out.crosstrack.append(abs(s.crosstrack_error_m))
    return out


def main() -> None:
    """Report wrong-sense target, heading and motion shares per bag."""
    parser = create_bags_parser(__doc__)
    parser.add_argument(
        "--window",
        default=None,
        help="Restrict to a 'lo-hi' seconds window, e.g. 134.7-196.5 for a known reversal.",
    )
    args = parser.parse_args()
    lo_hi = None
    if args.window:
        a, b = args.window.split("-")
        lo_hi = (float(a), float(b))

    rows_out = []
    for bag in args.bag_dirs:
        run = Path(bag).name.replace("run_", "")
        try:
            rows = posed_rows(load_nav_debug_rows(Path(bag))[0])
        except (RuntimeError, OSError, ValueError) as exc:
            rows_out.append([run, type(exc).__name__, "-", "-", "-", "-", "-", "-", "UNREADABLE"])
            continue
        rows = [(t, s) for t, s in rows if t >= BAY_SETTLE_S]
        if len(rows) < MIN_TICKS:
            rows_out.append([run, "too few ticks", "-", "-", "-", "-", "-", "-", "SKIP"])
            continue
        direction = settled_direction(rows)
        xs = [s.pose_x for _, s in rows]
        ys = [s.pose_y for _, s in rows]
        centre = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
        sign = -1.0 if direction is Direction.CLOCKWISE else 1.0
        prog = _progress_deg(rows, centre, sign)
        on_loop = next((i for i, v in enumerate(prog) if v >= ON_LOOP_DEG), None)
        if on_loop is None:
            rows_out.append([run, str(direction), "never on loop", "-", "-", "-", "-", "-", "SKIP"])
            continue
        sel = rows[on_loop:]
        if lo_hi:
            sel = [(t, s) for t, s in sel if lo_hi[0] <= t <= lo_hi[1]]
        if len(sel) < MIN_TICKS:
            rows_out.append([run, str(direction), "window too short", "-", "-", "-", "-", "-", "SKIP"])
            continue
        senses = _analyse(sel, centre, sign)
        f_t, n_t = _frac_wrong(senses.target)
        f_h, _ = _frac_wrong(senses.heading)
        f_m, _ = _frac_wrong(senses.motion)
        f_b, _ = _frac_wrong(senses.behind)
        xt = sorted(senses.crosstrack)
        p50 = xt[len(xt) // 2] if xt else float("nan")
        rows_out.append([
            run,
            str(direction),
            n_t,
            f"{100 * f_t:.1f}%",
            f"{100 * f_h:.1f}%",
            f"{100 * f_m:.1f}%",
            f"{100 * f_b:.1f}%",
            f"{p50:.3f}",
            "WRONG SENSE" if f_t > TARGET_WRONG_THRESHOLD else "clean",
        ])

    scope = f" inside {args.window}s" if args.window else " over the whole on-loop span"
    print(f"== TARGET / HEADING / MOTION pointing the WRONG WAY round the loop{scope}")
    print_table(
        rows_out,
        ["run", "direction", "ticks", "target wrong", "heading wrong", "motion wrong",
         "target behind", "crosstrack p50 m", "verdict"],
    )
    print()
    print("  target wrong  = the point the controller steers at lies the wrong way round the loop")
    print("                  -- if this is high, SELECTION is the defect and a span bound cannot fix it")
    print("  target behind = pose->target against the HEADING; disambiguates a correct-sense target")
    print("                  under a wrong-sense heading, which can only mean the target is behind")
    print("  heading wrong = how far the chassis itself has turned round; follows the target, does not lead it")
    print("  motion wrong  = CONTEXT ONLY, never scored: the clean control reads 14.2% here against 0.0%")
    print("                  heading-wrong, because a per-tick pose delta on a weaving chassis is mostly lateral")
    print(f"  verdict fires on target-wrong over {100 * TARGET_WRONG_THRESHOLD:.0f}%; the clean 3-lap control reads 0.7%")


if __name__ == "__main__":
    main()

r"""Does the escape help the plan, or undo it?

Operator report 2026-09-10: *"sometimes the escapes are not the right ones, or
they leave the car in a difficult position to pass on the correct side, instead
of helping."*

That is falsifiable without any model of what the escape SHOULD do. Just before
an escape latches, the navigator is steering somewhere on purpose -- the sign
lane, or the pursuit, has a side it wants. If the escape then steers the OTHER
way, it is not merely a backstop that costs time; it actively spends the
positioning the plan had already bought, and the robot re-approaches the pillar
worse placed than before.

So: for each escape episode, compare the sign of ``commanded_steering_norm`` in
the ticks BEFORE the latch against the sign of ``maneuver_steering`` during it.
AGREES means the escape pushed the way the plan was already going. OPPOSES
means it undid it.

This is deliberately a low-assumption test. It does not need the sign map, the
routing table or the pass-side rule -- only the robot's own intent one tick
earlier, which is the thing the escape interrupts.

CONTROLS:

* ``episodes`` and ``usable`` -- an episode is dropped when either side has no
  steering sign to compare (the pre-window sits at dead centre, or the manoeuvre
  never commands a steer), and the count is printed so a thin sample is visible.
* AGREES is reported alongside OPPOSES rather than netted. If the split is near
  50/50 the escape is indifferent to the plan, which is a different finding from
  either "helps" or "fights" and should not be readable as one.
* ``pre-window steer magnitude`` -- if the plan was barely steering, its "side"
  is noise and OPPOSES means little. Episodes under ``--min-intent`` are dropped
  and counted separately.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_escape_intent.py \
        ../../data/live/runs/run_20260910_2148*
"""

from __future__ import annotations

import statistics
from collections import Counter
from pathlib import Path

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, elapsed_seconds, open_reader


def _sign(v: float, dead: float = 1e-6) -> int:
    return 1 if v > dead else (-1 if v < -dead else 0)


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--pre-ticks", type=int, default=6, help="ticks before the latch that define the plan's intent")
    parser.add_argument("--min-intent", type=float, default=0.05, help="mean |steer| below which the pre-window has no side")
    args = parser.parse_args()

    agrees = opposes = 0
    dropped_no_side = dropped_flat = 0
    episodes = 0
    by_kind: Counter[str] = Counter()
    opposed_by_kind: Counter[str] = Counter()

    for bag_dir in args.bag_dirs:
        reader = open_reader(Path(bag_dir))
        t0 = None
        hist: list[float] = []          # commanded steering while NOT in a manoeuvre
        cur: dict | None = None

        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic != Topics.NAV_DEBUG:
                continue
            if t0 is None:
                t0 = stamp
            _ = elapsed_seconds(stamp, t0)
            snap = decode_nav_debug(data)
            if snap is None:
                continue
            kind = getattr(snap.active_maneuver_type, "value", snap.active_maneuver_type)
            latched = kind is not None

            if not latched:
                if cur is not None:
                    episodes += 1
                    pre = cur["pre"]
                    dur = cur["steer"]
                    if not pre or not dur:
                        dropped_no_side += 1
                    else:
                        intent = statistics.mean(pre)
                        if abs(intent) < args.min_intent:
                            dropped_flat += 1
                        else:
                            during = statistics.mean(dur)
                            k = str(cur["kind"])
                            by_kind[k] += 1
                            if _sign(intent) == _sign(during) or _sign(during) == 0:
                                agrees += 1
                            else:
                                opposes += 1
                                opposed_by_kind[k] += 1
                    cur = None
                if snap.commanded_steering_norm is not None:
                    hist.append(snap.commanded_steering_norm)
                    del hist[: -args.pre_ticks]
                continue

            if cur is None:
                cur = {"kind": kind, "pre": list(hist), "steer": []}
            if snap.maneuver_steering is not None:
                cur["steer"].append(snap.maneuver_steering)

        if cur is not None:
            episodes += 1

    usable = agrees + opposes
    print(f"escape episodes {episodes}, usable {usable}"
          f"   (dropped: no side {dropped_no_side}, plan barely steering {dropped_flat})")
    print("   <- if usable is near zero, nothing below means anything")
    if not usable:
        return
    print()
    print(f"  the escape steers the SAME way the plan was going : {agrees}  ({agrees / usable:.0%})")
    print(f"  the escape steers the OPPOSITE way                : {opposes}  ({opposes / usable:.0%})")
    print("   <- near 50/50 means the escape is INDIFFERENT to the plan, which is")
    print("      neither 'helps' nor 'fights' and should not be read as either")
    if opposed_by_kind:
        print()
        print("  opposed, by manoeuvre kind:")
        for k, n in by_kind.most_common():
            print(f"    {k:18} {opposed_by_kind.get(k, 0):3} of {n:3}")


if __name__ == "__main__":
    main()

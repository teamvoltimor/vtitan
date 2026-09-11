"""Why does the bay ratchet rotate 800 deg and keep 7?

Four hardware configurations on 2026-09-10 -- the shipped 0.10, then 0.15, then
0.15 with the guard's coast budgeted from measured speed, then that plus 15 mm
of tolerated overlap -- moved the legs (324 -> 111), their length (0.06 ->
0.20 s) and the veto rate, and left ONE thing untouched: the chassis turns
240-870 deg in total and keeps 2-7 of it, with zero net travel. Three correct
diagnoses, three wrong terms. That invariant is the question.

A ratchet accumulates only if the steering REVERSES with the direction: drive
forward turning one way, back up turning the other, and both legs rotate the
same way. Hold the steering across a reversal and the reverse undoes exactly
what the forward leg did -- which is what 0.8% efficiency looks like.

So this segments `bay_exit` into legs by the SIGN of `commanded_speed_mps` and
reports, per leg, the commanded steering sign and the yaw it actually turned.
Then it pairs consecutive legs and asks whether the yaw ADDS or CANCELS.

Reads `/nav_debug` alone -- `commanded_speed_mps`, `commanded_steering_norm`
and `pose_yaw` are all on it -- so there is no cross-topic alignment to get
wrong.

The controls, because a null here would otherwise be unreadable:

* `legs with a yaw reading` against the leg count -- if pose_yaw is absent the
  cancellation figure is vacuous rather than zero;
* `steer sign flips across reversals` -- if the steering never reverses the
  answer is mechanical, and if it always does then cancellation has to come
  from somewhere else;
* the same-sign and opposite-sign pair counts are printed separately rather
  than netted, so a wash of both is not mistaken for neither happening.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_bay_ratchet.py \
        ../../data/live/runs/run_20260910_213746
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, elapsed_seconds, open_reader


def _sign(v: float, dead: float = 1e-6) -> int:
    if v > dead:
        return 1
    if v < -dead:
        return -1
    return 0


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--min-ticks", type=int, default=2, help="ignore legs shorter than this many ticks")
    args = parser.parse_args()

    for bag_dir in args.bag_dirs:
        name = Path(bag_dir).name
        reader = open_reader(Path(bag_dir))
        t0 = None
        prev_yaw: float | None = None
        legs: list[dict] = []
        cur: dict | None = None
        ticks_in_bay = 0
        ticks_with_yaw = 0

        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic != Topics.NAV_DEBUG:
                continue
            if t0 is None:
                t0 = stamp
            t = elapsed_seconds(stamp, t0)
            snap = decode_nav_debug(data)
            if snap is None:
                continue
            if str(getattr(snap.phase, "value", snap.phase)) != "bay_exit":
                continue
            ticks_in_bay += 1

            dyaw = 0.0
            if snap.pose_yaw is not None:
                ticks_with_yaw += 1
                if prev_yaw is not None:
                    dyaw = math.atan2(
                        math.sin(snap.pose_yaw - prev_yaw), math.cos(snap.pose_yaw - prev_yaw)
                    )
                prev_yaw = snap.pose_yaw

            spd = snap.commanded_speed_mps
            steer = snap.commanded_steering_norm
            if spd is None:
                continue
            s = _sign(spd)
            if cur is None or s != cur["dir"]:
                if cur is not None:
                    legs.append(cur)
                cur = {"dir": s, "ticks": 0, "yaw": 0.0, "steer": [], "t0": t}
            cur["ticks"] += 1
            cur["yaw"] += dyaw
            if steer is not None:
                cur["steer"].append(steer)
        if cur is not None:
            legs.append(cur)

        moving = [lg for lg in legs if lg["dir"] != 0 and lg["ticks"] >= args.min_ticks]
        print(f"== {name}")
        print(f"  bay ticks {ticks_in_bay}, with pose_yaw {ticks_with_yaw}"
              f"   <- if these differ a lot, the yaw figures below are thin")
        print(f"  legs total {len(legs)}, moving legs of >= {args.min_ticks} ticks: {len(moving)}")
        if not moving:
            print("  no usable legs -- nothing below means anything")
            continue

        fwd = [lg for lg in moving if lg["dir"] > 0]
        rev = [lg for lg in moving if lg["dir"] < 0]
        for label, group in (("forward", fwd), ("reverse", rev)):
            if not group:
                continue
            st = [statistics.mean(lg["steer"]) for lg in group if lg["steer"]]
            yaw = [math.degrees(lg["yaw"]) for lg in group]
            print(f"  {label:8} n={len(group):3}  steer mean {statistics.mean(st):+.2f}"
                  f"  yaw/leg mean {statistics.mean(yaw):+.2f} deg"
                  f"  |yaw| sum {sum(abs(v) for v in yaw):.0f} deg"
                  f"  net {sum(yaw):+.0f} deg")

        # CONTROL: does the steering reverse with the direction at all?
        flips = 0
        held = 0
        adds = 0
        cancels = 0
        for a, b in zip(moving, moving[1:], strict=False):
            if not a["steer"] or not b["steer"]:
                continue
            sa, sb = _sign(statistics.mean(a["steer"])), _sign(statistics.mean(b["steer"]))
            if sa != 0 and sb != 0:
                if sa == sb:
                    held += 1
                else:
                    flips += 1
            ya, yb = a["yaw"], b["yaw"]
            if _sign(ya) != 0 and _sign(yb) != 0:
                if _sign(ya) == _sign(yb):
                    adds += 1
                else:
                    cancels += 1
        pairs = adds + cancels
        print(f"  steer sign across a reversal: HELD {held}, FLIPPED {flips}"
              "   <- a ratchet needs FLIPPED; HELD means the reverse undoes the forward")
        print(f"  consecutive-leg yaw: ADDS {adds}, CANCELS {cancels}"
              f"  ({cancels / pairs:.0%} cancelling)" if pairs else "  no yaw pairs")


if __name__ == "__main__":
    main()

r"""Does the bay-exit wheel break away just after a leg reversal, then stall?

`diag_bag_drive_response` shows the bay exit stalling 74% of its ticks while
ordinary driving stalls under 2%, but the bay is the ONLY place that commands
0.10 m/s and it is also the only place holding full lock, so between-run
comparisons cannot say whether speed or load binds.

This is a WITHIN-leg measurement, which does not care about that confound: it
bins encoder speed by time since the current leg started. Static friction
predicts a breakaway profile -- the wheel moves just after a reversal (the
motor is applying torque against a stopped wheel and wins briefly) and stalls
as the leg continues. Pure load predicts a flat, uniformly stalled profile.

If the breakaway is real, the fix is to make the ratchet reverse MORE often
(shorter legs), not merely faster -- which is testable against the observed
split: the runs with 0.33-0.50 s legs got out, the ones with 2-14 s legs
never did.

Usage::
    pixi run -e dev python scripts/bag/diag_bag_bay_breakaway.py data/live/runs/run_20260908_*
"""
from __future__ import annotations

import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader

ZERO_DEG_S = 1.0
AGE_BINS = [0.0, 0.2, 0.5, 1.0, 2.0, 4.0, 8.0, 1e9]


def age_bin(a: float) -> str:
    for lo, hi in zip(AGE_BINS, AGE_BINS[1:]):
        if lo <= a < hi:
            return f"{lo:>4.1f}-{hi:<5.1f}" if hi < 1e8 else f"{lo:>4.1f}+     "
    return "?"


def main() -> None:
    by_age: dict[str, list[float]] = defaultdict(list)
    leg_lengths: list[float] = []
    per_run: dict[str, list[float]] = defaultdict(list)

    for a in sys.argv[1:]:
        d = Path(a)
        if not d.is_dir():
            continue
        reader = open_reader(d)
        t0 = None
        last_drive: float | None = None
        leg_kind: bool | None = None
        leg_start: float | None = None
        while reader.has_next():
            topic, data, t = reader.read_next()
            if t0 is None:
                t0 = t
            ts = elapsed_seconds(t, t0)
            if topic == Topics.MOTOR_DRIVE_SPEED:
                last_drive = float(deserialize_message(data, Float32).data)
                continue
            if topic != Topics.NAV_DEBUG or last_drive is None:
                continue
            snap = decode_nav_debug(data)
            if str(getattr(snap.phase, "value", snap.phase)) != "bay_exit":
                leg_kind, leg_start = None, None
                continue
            rev = snap.bay_leg_is_reverse
            if rev is None:
                continue
            # Exclude the deliberate standstill: _begin_leg sets a servo-slew
            # settle that COMMANDS zero, and counting those as stalls would
            # manufacture the very profile this is testing for.
            cmd = snap.commanded_speed_mps
            if cmd is None or abs(cmd) < 1e-6:
                continue
            if leg_kind is None or rev != leg_kind:
                if leg_start is not None:
                    leg_lengths.append(ts - leg_start)
                leg_kind, leg_start = rev, ts
            by_age[age_bin(ts - leg_start)].append(abs(last_drive))
            per_run[d.name[-6:]].append(abs(last_drive))

    if not by_age:
        print("no bay_exit legs found")
        return

    hdr = f"{'leg age s':14}{'n':>7}{'stall%':>8}{'encMed':>9}{'encMean':>9}"
    print(hdr)
    print("-" * len(hdr))
    for k in sorted(by_age, key=lambda s: float(s.split("-")[0].replace("+", ""))):
        v = by_age[k]
        stall = 100.0 * sum(1 for e in v if e <= ZERO_DEG_S) / len(v)
        print(f"{k:14}{len(v):7}{stall:8.1f}{statistics.median(v):9.1f}{statistics.mean(v):9.1f}")

    if leg_lengths:
        print(f"\nleg durations s: n={len(leg_lengths)} median {statistics.median(leg_lengths):.2f} "
              f"p90 {sorted(leg_lengths)[int(0.9 * len(leg_lengths))]:.2f} max {max(leg_lengths):.2f}")


if __name__ == "__main__":
    main()

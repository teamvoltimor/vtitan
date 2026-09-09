r"""What speed does the drivetrain actually deliver for a commanded speed?

The bay exit commands 0.10 m/s and the encoder reads zero on a median 73% of
its ticks (``diag_bag_bay_deadband``). That alone does not say the speed is
too low -- a chassis wedged at full lock stalls at ANY commanded speed, and
the two want opposite fixes: a deadband wants a higher number, a wedge wants
a different manoeuvre.

The separator is whether the shortfall is specific to LOW commands. This bins
every tick of a session by commanded speed and reports what
``/motor/drive_speed`` did in each bin, so the response curve is visible:

* a DEADBAND shows a floor -- near-total stall below some commanded speed and
  clean tracking above it, with the knee at the floor,
* a LOAD problem shows shortfall spread across all bins, or concentrated in
  the phases that steer at full lock rather than in the low-speed bins.

Bins are also split by phase, because the bay exit is the only phase that
commands 0.10 m/s AND holds full lock -- without the split, "slow at 0.10"
and "slow while scrubbing" are the same column.

Encoder is deg/s (``bag_drive_speed_is_deg_per_sec``), converted on a 7 cm
wheel. Both series are compared as magnitudes: ``/motor/drive_speed`` sign
conventions and the reverse legs would otherwise cancel
(``signed_wheel_odometry_cancels_under_a_ratchet``).

Usage::

    pixi run -e dev python scripts/bag/diag_bag_drive_response.py \
        data/live/runs/run_20260908_*
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

WHEEL_CIRCUM_M = math.pi * 0.07
ZERO_DEG_S = 1.0
BINS = [0.0, 0.05, 0.09, 0.11, 0.15, 0.20, 0.30, 0.40, 0.60]


def deg_s_to_mps(deg_s: float) -> float:
    return (deg_s / 360.0) * WHEEL_CIRCUM_M


def bin_of(v: float) -> str:
    for lo, hi in zip(BINS, BINS[1:]):
        if lo <= v < hi:
            return f"{lo:.2f}-{hi:.2f}"
    return f">={BINS[-1]:.2f}"


def collect(bag_dir: Path, by_bin: dict, by_phase: dict, by_steer: dict) -> None:
    reader = open_reader(bag_dir)
    t0 = None
    last_drive: float | None = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic == Topics.MOTOR_DRIVE_SPEED:
            last_drive = float(deserialize_message(data, Float32).data)
            continue
        if topic != Topics.NAV_DEBUG or last_drive is None:
            continue
        snap = decode_nav_debug(data)
        cmd = snap.commanded_speed_mps
        if cmd is None or abs(cmd) < 1e-6:
            continue
        enc = abs(last_drive)
        key = bin_of(abs(cmd))
        by_bin[key].append(enc)
        ph = str(getattr(snap.phase, "value", snap.phase))
        by_phase[ph].append((abs(cmd), enc))
        st = snap.commanded_steering_norm
        if st is not None:
            lock = "full_lock" if abs(st) >= 0.95 else "part_lock"
            by_steer[lock].append((abs(cmd), enc))
            # The separating cell: the 0.10 bin, the bay_exit phase and full
            # lock are nearly the same ticks, so "too slow", "scrubbing" and
            # "wedged" are collinear. Full lock at a HIGHER command is the
            # only cell that tells them apart.
            band = "lo(<0.12)" if abs(cmd) < 0.12 else "hi(>=0.12)"
            by_steer[f"{lock} {band}"].append((abs(cmd), enc))
            by_steer[f"{lock} {band} {ph}"].append((abs(cmd), enc))


def report(title: str, groups: dict) -> None:
    print(f"\n== {title}")
    hdr = f"{'group':22}{'n':>7}{'stall%':>8}{'encMed':>9}{'encMed m/s':>12}{'cmdMed':>9}{'deliver%':>10}"
    print(hdr)
    print("-" * len(hdr))
    for k in sorted(groups):
        vals = groups[k]
        if isinstance(vals[0], tuple):
            cmds = [c for c, _ in vals]
            encs = [e for _, e in vals]
        else:
            cmds, encs = [], vals
        stall = 100.0 * sum(1 for e in encs if e <= ZERO_DEG_S) / len(encs)
        med = statistics.median(encs)
        cmd_med = statistics.median(cmds) if cmds else float("nan")
        med_mps = deg_s_to_mps(med)
        deliver = 100.0 * med_mps / cmd_med if cmds and cmd_med else float("nan")
        print(f"{k:22}{len(encs):7}{stall:8.1f}{med:9.1f}{med_mps:12.3f}{cmd_med:9.3f}{deliver:10.1f}")


def main() -> None:
    by_bin: dict[str, list] = defaultdict(list)
    by_phase: dict[str, list] = defaultdict(list)
    by_steer: dict[str, list] = defaultdict(list)
    for a in sys.argv[1:]:
        p = Path(a)
        if p.is_dir():
            collect(p, by_bin, by_phase, by_steer)

    if not by_bin:
        print("no ticks with a nonzero commanded speed")
        return

    report("BY COMMANDED SPEED BIN (m/s)", by_bin)
    report("BY PHASE", by_phase)
    report("BY STEERING (full lock = |norm| >= 0.95)", by_steer)
    print("\nstall% = ticks where |/motor/drive_speed| <= 1 deg/s")
    print("deliver% = median encoder m/s as a share of median commanded m/s")


if __name__ == "__main__":
    main()

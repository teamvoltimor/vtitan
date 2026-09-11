r"""Why did the robot stop making progress, at the one place it stopped?

Measured on the 2026-09-11 Obstacles rounds, all of which ended 0 laps of 3:
two of the three wedged at the SAME physical point, (2.52, 1.27) and
(2.57, 1.27), five centimetres apart, and one of them spent every escape it
ever fired inside a 0.19 m radius. A failure that repeats at one spot has a
cause; a distribution over 78 bags only has a shape. This is for the cause.

It finds the longest window in which the chassis stayed inside
``WEDGE_RADIUS_M``, and then separates the three explanations that look
identical from the outside and want completely different fixes:

1. **COMMANDING AND NOT MOVING.** Nonzero commanded speed, wheel encoder near
   zero. Motor deadband, stall under steering load, or a speed below the floor.
   Nothing in the sign lane or the escape logic can help.
2. **MOVING AND COMING BACK.** Wheel travelling, pose not. The pendulum: legs
   that cancel. ``ESCAPE_MIRRORS_REVERSE`` shipped for Obstacles today, so this
   is also the check on whether that fix is reaching this case.
3. **NEVER COMMITTED TO THE SIGN.** The router held no claim through the
   wedge, so the chassis was reacting to an obstacle nothing had planned
   around. That points back at the anticipation budget rather than at control.

Reported alongside what the robot BELIEVED at the time -- forward and rear
clearance, the escape's own trigger bearing and range, whether it read itself
as stuck -- because the belief is what it acted on, right or wrong.

TRAP this script exists to avoid: `travelled_m`-style signed odometry cancels
under a ratchet, so ABSOLUTE wheel travel is integrated separately from signed.
The ratio between them is the pendulum's signature (measured 3.6x on the open
track) and neither number alone shows it.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_wedge_trace.py \
        data/live/runs/run_20260911_110734
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from scripts.common.bag_io import (  # noqa: E402
    create_bags_parser,
    read_motion_streams,
    read_vision_rows_and_scans,
)
from scripts.common.stats import percentile  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402

WEDGE_RADIUS_M = 0.30
"""How tightly the chassis must be held to count as wedged.

Comfortably larger than the pose noise and smaller than a corridor, so a robot
merely driving slowly does not qualify while one rocking in place does.
"""
MIN_WEDGE_TICKS = 50
"""Five seconds at 10 Hz. Shorter than that is a manoeuvre, not a wedge."""


def _series_in(series: list[tuple[float, float]], t0: float, t1: float) -> list[float]:
    return [v for t, v in series if t0 <= t <= t1]


def find_wedge(rows) -> tuple[int, int] | None:  # noqa: ANN001
    """Longest run of ticks whose poses all fit inside WEDGE_RADIUS_M.

    A simple expanding window rather than clustering: the question is "did it
    stop going anywhere", which is a property of a contiguous stretch of time,
    and a cluster would happily merge two separate visits to the same corner.
    """
    # The bay exit is EXCLUDED. It is a deliberately tight manoeuvre inside a
    # pocket, so it always wins a "held still longest" search and buries the
    # thing being looked for -- on run_20260911_110734 it took the whole answer
    # (369 ticks, 41% of the run, a 0.14 x 0.16 m box) while the escapes that
    # actually ended the round clustered a metre and a half away.
    poses = [
        (i, d.pose_x, d.pose_y)
        for i, (_, d) in enumerate(rows)
        if d.pose_x is not None and "bay" not in str(getattr(d, "phase", "")).lower()
    ]
    best: tuple[int, int] | None = None
    best_len = 0
    start = 0
    for end in range(len(poses)):
        # Shrink from the left until the window fits again.
        while start < end:
            xs = [p[1] for p in poses[start : end + 1]]
            ys = [p[2] for p in poses[start : end + 1]]
            if (max(xs) - min(xs)) <= 2 * WEDGE_RADIUS_M and (max(ys) - min(ys)) <= 2 * WEDGE_RADIUS_M:
                break
            start += 1
        if end - start + 1 > best_len:
            best_len = end - start + 1
            best = (poses[start][0], poses[end][0])
    if best is None or best_len < MIN_WEDGE_TICKS:
        return None
    return best


def main() -> None:
    args = create_bags_parser(__doc__).parse_args()

    for bag in args.bag_dirs:
        path = Path(bag)
        rows, _frames, _scans = read_vision_rows_and_scans(path, with_scans=False)
        if len(rows) < MIN_WEDGE_TICKS:
            print(f"== {path.name}: too short")
            continue
        window = find_wedge(rows)
        print(f"== {path.name}  ({len(rows)} ticks)")
        if window is None:
            print("   never held still long enough to count as wedged")
            continue
        lo, hi = window
        t0, t1 = rows[lo][0], rows[hi][0]
        held = rows[lo : hi + 1]
        print(f"   WEDGE: ticks {lo}-{hi}, t={t0:.1f}-{t1:.1f} s ({t1 - t0:.1f} s, "
              f"{100 * len(held) / len(rows):.0f}% of the run)")

        xs = [d.pose_x for _, d in held if d.pose_x is not None]
        ys = [d.pose_y for _, d in held if d.pose_y is not None]
        print(f"   held at ({sum(xs) / len(xs):.2f}, {sum(ys) / len(ys):.2f}), "
              f"box {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m")
        print()

        # 1. Commanding and not moving?
        motion = read_motion_streams(path)
        cmd = _series_in(motion.cmd_speed_mps, t0, t1)
        wheel = _series_in(motion.drive_speed_mps(), t0, t1)
        commanding = sum(1 for v in cmd if abs(v) > 0.01)
        turning = sum(1 for v in wheel if abs(v) > 0.01)
        print("   1. IS IT COMMANDING AND NOT MOVING?")
        print(f"      ticks commanding a nonzero speed : {commanding}/{len(cmd)}"
              f"  ({100 * commanding / len(cmd):.0f}%)" if cmd else "      no commands in window")
        print(f"      ticks the WHEEL actually turned  : {turning}/{len(wheel)}"
              f"  ({100 * turning / len(wheel):.0f}%)" if wheel else "      no encoder in window")
        if cmd and wheel:
            print(f"      |commanded| p50 {percentile([abs(v) for v in cmd], 0.5):.3f} m/s   "
                  f"|wheel| p50 {percentile([abs(v) for v in wheel], 0.5):.3f} m/s")

        # 2. Moving and coming back? Absolute vs signed wheel travel.
        dt = (t1 - t0) / max(1, len(wheel) - 1) if len(wheel) > 1 else 0.0
        abs_travel = sum(abs(v) for v in wheel) * dt
        signed_travel = abs(sum(wheel) * dt)
        displacement = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        print()
        print("   2. IS IT MOVING AND COMING BACK?")
        print(f"      ABSOLUTE wheel travel {abs_travel:6.3f} m")
        print(f"      SIGNED   wheel travel {signed_travel:6.3f} m"
              f"   ratio {abs_travel / signed_travel:.1f}x" if signed_travel > 1e-6 else
              f"      SIGNED   wheel travel {signed_travel:6.3f} m")
        print(f"      net POSE displacement {displacement:6.3f} m")

        # 3. Did the sign lane ever have a claim here?
        committed = sum(1 for _, d in held if getattr(d, "committed_sign_x", None) is not None)
        signs = [d.active_sign_count for _, d in held if d.active_sign_count is not None]
        print()
        print("   3. DID THE ROUTER HOLD A SIGN THROUGH IT?")
        print(f"      ticks with a committed sign : {committed}/{len(held)}"
              f"  ({100 * committed / len(held):.0f}%)")
        if signs:
            print(f"      believed sign count         : p50 {percentile(signs, 0.5):.0f}  "
                  f"max {max(signs)}   (the track holds at most 8)")

        # What it believed while it sat there.
        print()
        print("   WHAT IT BELIEVED")
        fields = [
            ("forward_clearance_m", "forward clearance"),
            ("rear_clearance_m", "rear clearance"),
            ("min_lidar_range_m", "min lidar range"),
            ("escape_trigger_range_m", "escape trigger range"),
            ("recent_movement_m", "recent movement"),
        ]
        rows_b = []
        for attr, label in fields:
            vals = [getattr(d, attr) for _, d in held if getattr(d, attr, None) is not None]
            rows_b.append([
                label,
                f"{percentile(vals, 0.1):.3f}" if vals else "--",
                f"{percentile(vals, 0.5):.3f}" if vals else "--",
                f"{percentile(vals, 0.9):.3f}" if vals else "--",
            ])
        print_table(rows_b, ["belief", "p10", "p50", "p90"])

        stuck = sum(1 for _, d in held if d.is_stuck)
        phases: dict[str, int] = {}
        for _, d in held:
            name = str(getattr(d, "phase", None) or getattr(d, "navigator_phase", None))
            phases[name] = phases.get(name, 0) + 1
        man: dict[str, int] = {}
        for _, d in held:
            if d.active_maneuver_type is not None:
                man[str(d.active_maneuver_type)] = man.get(str(d.active_maneuver_type), 0) + 1
        print(f"      read itself as STUCK on {stuck}/{len(held)} ticks")
        print(f"      phases    : {dict(sorted(phases.items(), key=lambda kv: -kv[1])[:4])}")
        print(f"      manoeuvres: {dict(sorted(man.items(), key=lambda kv: -kv[1]))}")
        print()


if __name__ == "__main__":
    main()

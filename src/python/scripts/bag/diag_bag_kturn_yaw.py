r"""How much heading does a K-turn actually deliver, and how much does a BURST of them?

``escape.toml`` budgets the K-turn in SECONDS (``k_turn_min_s`` 0.54 at OBSTACLE risk,
``k_turn_max_s`` 1.08 at CRITICAL, escalating to ``max_escape_s`` 1.8) and the rear-gap
fit truncates it in METRES. Nothing anywhere budgets radians: ``controller.py:824`` picks
the duration from the risk level alone. So the heading a manoeuvre delivers is whatever
its duration multiplied by the chassis's reverse yaw rate happens to be.

That makes two things worth separating, and this script prints both:

* PER EPISODE -- is the yaw repeatable? Split by the steering that was COMMANDED, because
  ``controller.py:822`` makes the OBSTACLE-risk K-turn a straight reverse BY DESIGN, and
  an episode that delivers no yaw is only interesting if lock was actually asked for.
* PER BURST -- one locked K-turn is about 48 deg, so a heading flip needs several. Bursts
  accumulate SIGNED yaw across consecutive episodes: signed, because two opposite legs
  cancel, and an unsigned sum would manufacture a flip the chassis never performed.

Yaw comes from the IMU and is accumulated as wrapped per-sample increments rather than as
an endpoint difference, so an episode is free to exceed 180 deg without aliasing back.
``pose_yaw`` is deliberately not used: it is localizer-fused and damped, which understates
the achieved rate (see ``bag_io.quaternion_yaw``).

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_kturn_yaw.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
import statistics
from collections import Counter

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows, read_motion_streams
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import ManeuverType


def accumulated_yaw(samples, t0, t1):
    """Signed yaw swept between t0 and t1, summing wrapped per-sample increments."""
    window = [(t, y) for t, y in samples if t0 <= t <= t1]
    if len(window) < 2:
        return None
    return sum(math.atan2(math.sin(b - a), math.cos(b - a)) for (_, a), (_, b) in zip(window, window[1:]))


def k_turn_episodes(rows):
    """Contiguous runs of ticks whose latched manoeuvre is a K-turn, as (start, end)."""
    episodes, start, last = [], None, None
    for ts, snap in rows:
        is_k = snap.active_maneuver_type == ManeuverType.K_TURN
        if is_k and start is None:
            start = ts
        elif not is_k and start is not None:
            episodes.append((start, last if last is not None else ts))
            start = None
        if is_k:
            last = ts
    if start is not None and last is not None:
        episodes.append((start, last))
    return episodes


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--burst-gap-s",
        type=float,
        default=2.0,
        help="Gap below which consecutive K-turns count as one burst (default 2.0, the re-fire window).",
    )
    args = parser.parse_args()

    escape = NavigationTuning.load_default().escape
    print(
        f"rev_steer_deg={escape.rev_steer_deg}  rev_speed={escape.rev_speed}  "
        f"k_turn_min_s={escape.k_turn_min_s}  k_turn_max_s={escape.k_turn_max_s}  "
        f"max_escape_s={escape.max_escape_s}  burst_gap_s={args.burst_gap_s}"
    )

    all_eps, all_bursts = [], []
    for bag_dir in args.bag_dirs:
        rows, _ = load_nav_debug_rows(bag_dir)
        streams = read_motion_streams(bag_dir)
        imu = streams.imu_yaw_rad
        if not imu:
            print(f"\n=== {bag_dir.name}: no IMU stream")
            continue
        pose = [(ts, snap.pose_x, snap.pose_y) for ts, snap in rows if snap.pose_x is not None]

        measured = []
        for t0, t1 in k_turn_episodes(rows):
            swept = accumulated_yaw(imu, t0, t1)
            if swept is None:
                continue
            leg = [(x, y) for t, x, y in pose if t0 <= t <= t1]
            cmd = [abs(math.degrees(v)) for t, v in streams.cmd_steer_rad if t0 <= t <= t1]
            measured.append(
                {
                    "t0": t0,
                    "t1": t1,
                    "dur": t1 - t0,
                    "yaw": math.degrees(swept),
                    "travel": math.dist(leg[0], leg[-1]) if len(leg) >= 2 else 0.0,
                    "cmd": max(cmd) if cmd else 0.0,
                }
            )
        all_eps.extend(measured)

        if not measured:
            print(f"\n=== {bag_dir.name}: {len(rows)} ticks, no K_TURN episodes")
            continue

        print(f"\n=== {bag_dir.name}  episodes={len(measured)}")
        for label, group in (
            ("straight (cmd 0) ", [e for e in measured if e["cmd"] < 1.0]),
            ("locked   (cmd 44)", [e for e in measured if e["cmd"] >= 1.0]),
        ):
            if not group:
                continue
            yaws = [abs(e["yaw"]) for e in group]
            print(
                f"  {label}  n={len(group):<3}"
                f" dur_s p50 {statistics.median([e['dur'] for e in group]):.2f}"
                f"  |yaw| p50 {statistics.median(yaws):>5.1f}  max {max(yaws):>5.1f}"
                f"  travel_m p50 {statistics.median([e['travel'] for e in group]):.3f}"
            )

        bursts, current = [], None
        for e in measured:
            if current is not None and e["t0"] - current["end"] <= args.burst_gap_s:
                current["n"] += 1
                current["signed"] += e["yaw"]
                current["abs"] += abs(e["yaw"])
                current["end"] = e["t1"]
            else:
                if current is not None:
                    bursts.append(current)
                current = {"n": 1, "signed": e["yaw"], "abs": abs(e["yaw"]), "start": e["t0"], "end": e["t1"]}
        if current is not None:
            bursts.append(current)
        all_bursts.extend(bursts)
        nets = [abs(b["signed"]) for b in bursts]
        print(
            f"  bursts={len(bursts)}  multi={sum(1 for b in bursts if b['n'] > 1)}"
            f"  |net heading| p50 {statistics.median(nets):.0f}  max {max(nets):.0f} deg"
        )

    if all_bursts:
        multi = [b for b in all_bursts if b["n"] > 1]
        share = 100 * len(multi) / len(all_bursts)
        print(f"\n=== BURSTS pooled (gap <= {args.burst_gap_s}s)  n={len(all_bursts)}   multi-episode {len(multi)} ({share:.0f}%)")
        sizes = Counter(b["n"] for b in all_bursts)
        print("  episodes per burst: " + "  ".join(f"{k}:{v}" for k, v in sorted(sizes.items())))
        for lo, hi, label in ((0, 60, "  <60  "), (60, 120, " 60-120"), (120, 170, "120-170"), (170, 1e9, "170+   ")):
            n = sum(1 for b in all_bursts if lo <= abs(b["signed"]) < hi)
            bar = "#" * int(50 * n / len(all_bursts))
            print(f"  |net heading| {label} deg  {n:>3}  {100 * n / len(all_bursts):>5.1f}%  {bar}")
        worst = max(all_bursts, key=lambda b: abs(b["signed"]))
        print(f"  worst: {worst['n']} episodes, net {worst['signed']:+.0f} deg, swept {worst['abs']:.0f} deg over {worst['end'] - worst['start']:.1f} s")
        eff = [abs(b["signed"]) / b["abs"] for b in multi if b["abs"] > 1e-6]
        if eff:
            print(f"  multi-episode net/swept p50 {statistics.median(eff):.2f}  (1.0 = all legs turn the same way, 0 = perfect cancellation)")

    if all_eps:
        print(f"\n=== POOLED episodes={len(all_eps)}")
        for lo, hi in ((0, 10), (10, 20), (20, 40), (40, 55), (55, 80), (80, 120), (120, 1e9)):
            n = sum(1 for e in all_eps if lo <= abs(e["yaw"]) < hi)
            label = f"{lo:>3}-{hi:<4.0f}" if hi < 1e9 else f"{lo:>3}+   "
            bar = "#" * int(50 * n / len(all_eps))
            print(f"  {label} deg  {n:>4}  {100 * n / len(all_eps):>5.1f}%  {bar}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

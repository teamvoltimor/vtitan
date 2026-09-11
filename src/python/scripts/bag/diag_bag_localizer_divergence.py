"""Per-tick localizer divergence trace: are the LIDAR beams consistent with the believed pose?

Method (the project's documented divergence post-mortem, run as a time series
rather than a single verdict): take each ``/nav_debug`` tick's BELIEVED pose,
take the nearest real ``/scan``, project every informative ray to its endpoint
and count the endpoints that land OUTSIDE the track's free space. A beam that
ends inside a wall is physically impossible, so a converged pose scores near
zero and a diverged one sprays.

Prints a per-second time series plus annotations from the same ``/nav_debug``
stream (phase, escape count, direction adoption, lap credit, corridor change,
waypoint index) so a step in the metric can be lined up against what the
navigator thought was happening.

Usage (from ``src/python``, with PYTHONPATH=.)::

    python scripts/bag/diag_bag_localizer_divergence.py data/live/runs/run_XXXX --bin 1.0
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning.blind_nav import LocalizationParams

from scripts.bag.diag_localizer_guard_replay import _final_walls, _read_bag, _scan_to_ranges_angles
from scripts.common.bag_io import create_bags_parser
from scripts.common.stats import nearest_by_time
from src.navigation.localization import LidarLocalizer

_BEAM_PULLBACK_M = 0.05
_MAT_CENTRE = (1.5, 1.5)


def _off_track_beam_fraction(walls, x, y, yaw, ranges, angles) -> tuple[float, int]:
    informative = ranges < RobotSpecs.LIDAR_MAX_RANGE
    if not informative.any():
        return 0.0, 0
    r = ranges[informative] - _BEAM_PULLBACK_M
    bearings = yaw + angles[informative]
    sx = x + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(yaw)
    sy = y + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(yaw)
    ex = sx + r * np.cos(bearings)
    ey = sy + r * np.sin(bearings)
    outside = sum(1 for a, b in zip(ex, ey, strict=True) if not walls.point_in_free_space(float(a), float(b)))
    return outside / len(r), len(r)


def analyse(bag_dir: Path, stride: int, bin_s: float, tmax: float | None) -> None:
    nav_debug, scans = _read_bag(bag_dir)
    if len(nav_debug) < 2 or not scans:
        print(f"{bag_dir.name}: NOT ENOUGH DATA nav_debug={len(nav_debug)} scan={len(scans)}")
        return
    walls = _final_walls(nav_debug)
    if walls is None:
        print(f"{bag_dir.name}: no converged belief widths, skipping")
        return
    params = LocalizationParams()
    localizer = LidarLocalizer(walls, params)
    scan_times = [t for t, _ in scans]
    t0 = nav_debug[0][0]

    rows = []
    prev = {}
    events: list[tuple[float, str]] = []
    for i in range(0, len(nav_debug), stride):
        t, s = nav_debug[i]
        el = (t - t0) / 1e9
        if tmax is not None and el > tmax:
            break
        if s.pose_yaw is None:
            continue
        scan = nearest_by_time(scans, scan_times, t)
        ranges, angles = _scan_to_ranges_angles(scan)
        frac, n = _off_track_beam_fraction(walls, s.pose_x, s.pose_y, s.pose_yaw, ranges, angles)
        cost = localizer._fit_cost(s.pose_x, s.pose_y, s.pose_yaw, ranges, angles)  # noqa: SLF001
        ang = math.degrees(math.atan2(s.pose_y - _MAT_CENTRE[1], s.pose_x - _MAT_CENTRE[0]))
        rows.append((el, s.pose_x, s.pose_y, s.pose_yaw, frac, cost, n, ang, s.commanded_speed_mps))
        for f in ("phase", "direction", "current_corridor", "laps_completed", "escape_count", "active_maneuver_type"):
            v = getattr(s, f)
            if f in prev and prev[f] != v:
                events.append((el, f"{f}: {prev[f]} -> {v}"))
            prev[f] = v

    if not rows:
        print(f"{bag_dir.name}: no posed ticks")
        return
    fr = np.array([r[4] for r in rows])
    cs = np.array([r[5] for r in rows])
    print(f"\n===== {bag_dir.name} =====  {len(rows)} ticks (stride {stride}), dur {rows[-1][0]:.1f}s, {len(scans)} scans")
    print(f"  walls belief N/S/E/W from final snapshot; beams/tick median={int(np.median([r[6] for r in rows]))}")
    print(f"  off_beams p10={np.percentile(fr,10):.3f} p50={np.percentile(fr,50):.3f} p90={np.percentile(fr,90):.3f} max={fr.max():.3f}")
    print(f"  fit_cost  p10={np.percentile(cs,10):.4f} p50={np.percentile(cs,50):.4f} p90={np.percentile(cs,90):.4f}")

    # binned series
    print(f"  -- {bin_s:.1f}s bins: t  off_beams(mean)  fit_cost(mean)  pose  ang_deg  cmd_v")
    b = 0.0
    cur = []
    def flush(bstart, cur):
        if not cur:
            return
        f = np.mean([c[4] for c in cur])
        c_ = np.mean([c[5] for c in cur])
        last = cur[-1]
        bar = "#" * int(round(f * 40))
        print(f"   {bstart:6.1f} {f:6.3f} {c_:8.4f}  ({last[1]:5.2f},{last[2]:5.2f}) yaw{math.degrees(last[3]):7.1f} ang{last[7]:7.1f} v={last[8]}  {bar}")
    for r in rows:
        while r[0] >= b + bin_s:
            flush(b, cur)
            cur = []
            b += bin_s
        cur.append(r)
    flush(b, cur)
    print("  -- events:")
    for el, txt in events:
        print(f"   {el:6.1f}  {txt}")


def main() -> None:
    p = create_bags_parser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stride", type=int, default=10)
    p.add_argument("--bin", type=float, default=1.0, dest="bin_s")
    p.add_argument("--tmax", type=float, default=None)
    a = p.parse_args()
    for d in a.bag_dirs:
        analyse(d, a.stride, a.bin_s, a.tmax)


if __name__ == "__main__":
    main()

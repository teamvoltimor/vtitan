"""Compare the shipped relocalization trigger against a non-saturating one, on real bags.

The shipped trigger (``LidarLocalizer.estimate_position``) fires the global
rescue when the mean CLIPPED squared residual exceeds
``RELOCALIZE_COST_THRESHOLD`` for ``RELOCALIZE_AFTER_SCANS`` consecutive
scans. Every ray's contribution is capped at ``RESIDUAL_CLIP_M ** 2`` =
0.0625 m^2, so a pose 3 m from the truth cannot score worse than one 30 cm
off: the metric saturates, and the threshold (0.03) sits at less than half
its own ceiling. run_20260907_205830 held a 1.5-3 m pose error for 48 s
without the rescue ever recovering it.

This script measures whether that is because the trigger never tripped, by
replaying each tick's REPORTED pose (``/nav_debug``, i.e. the wrong one the
run actually used) against the nearest real ``/scan`` and recomputing:

1. ``_fit_cost`` -- the production metric, verbatim, at that pose.
2. off-track beams -- the alternative the divergence post-mortem used: project
   every informative ray to its endpoint and count the ones landing outside
   the track's free space. A beam that ends inside a wall is PHYSICALLY
   IMPOSSIBLE, not "a bit worse", so this measure does not saturate.

Both are evaluated per tick, plus the streak logic of the real trigger
(``off_track OR cost > threshold``), so the output answers one question
directly: during the divergence, did the shipped detector ever accumulate
``RELOCALIZE_AFTER_SCANS`` consecutive bad scans?

Usage (from ``src/python``, with PYTHONPATH=.)::

    python scripts/bag/diag_localizer_detector_saturation.py data/live/runs/run_20260907_205830
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

# How far back along a ray its endpoint is pulled before the free-space test.
# A ray that correctly hits a wall lands exactly ON the boundary, where
# point_in_free_space is a coin flip; pulling back by more than the LIDAR's
# own range sigma (0.03 m, see corridor_estimator) makes a correct hit
# unambiguously inside, so anything still outside went THROUGH a wall.
_BEAM_PULLBACK_M = 0.05


def _off_track_beam_fraction(walls, x: float, y: float, yaw: float, ranges: np.ndarray, angles: np.ndarray) -> float:
    """Fraction of informative rays whose endpoint lies outside the track.

    Non-return rays are excluded for the same reason _fit_cost excludes them:
    sanitize_lidar_ranges reports them at max range, which is not a measured
    endpoint and would land outside the track from almost any pose.
    """
    informative = ranges < RobotSpecs.LIDAR_MAX_RANGE
    if not informative.any():
        return 0.0
    r = ranges[informative] - _BEAM_PULLBACK_M
    bearings = yaw + angles[informative]
    sensor_x = x + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(yaw)
    sensor_y = y + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(yaw)
    end_x = sensor_x + r * np.cos(bearings)
    end_y = sensor_y + r * np.sin(bearings)
    outside = sum(1 for ex, ey in zip(end_x, end_y, strict=True) if not walls.point_in_free_space(float(ex), float(ey)))
    return outside / len(r)


def analyse(bag_dir: Path, stride: int) -> None:
    nav_debug, scans = _read_bag(bag_dir)
    if len(nav_debug) < 2 or not scans:
        print(f"{bag_dir.name}: not enough data (nav_debug={len(nav_debug)}, scan={len(scans)})")
        return

    walls = _final_walls(nav_debug)
    if walls is None:
        print(f"{bag_dir.name}: no belief widths recorded, skipping")
        return

    params = LocalizationParams()
    localizer = LidarLocalizer(walls, params)
    scan_times = [t for t, _ in scans]
    t0 = nav_debug[0][0]

    rows = []
    streak = 0
    max_streak = 0
    streak_start_s = None
    max_streak_start_s = None
    would_fire = 0
    for i in range(0, len(nav_debug), stride):
        t, snap = nav_debug[i]
        if snap.pose_yaw is None:
            continue
        scan = nearest_by_time(scans, scan_times, t)
        ranges, angles = _scan_to_ranges_angles(scan)
        x, y, yaw = snap.pose_x, snap.pose_y, snap.pose_yaw
        cost = localizer._fit_cost(x, y, yaw, ranges, angles)  # noqa: SLF001 - diagnostic replay
        off_track = not walls.point_in_free_space(x, y)
        off_beams = _off_track_beam_fraction(walls, x, y, yaw, ranges, angles)
        elapsed = (t - t0) / 1e9

        bad = off_track or cost > params.RELOCALIZE_COST_THRESHOLD
        if bad:
            if streak == 0:
                streak_start_s = elapsed
            streak += 1
            if streak > max_streak:
                max_streak = streak
                max_streak_start_s = streak_start_s
            if streak == params.RELOCALIZE_AFTER_SCANS:
                would_fire += 1
        else:
            streak = 0
        rows.append((elapsed, x, y, cost, off_track, off_beams, streak))

    costs = np.array([r[3] for r in rows])
    beams = np.array([r[5] for r in rows])
    n = len(rows)
    print(f"\n== {bag_dir.name} ==  {n} ticks analysed (stride {stride}), {len(scans)} scans")
    print(
        f"  ceiling per ray = RESIDUAL_CLIP_M^2 = {params.RESIDUAL_CLIP_M ** 2:.4f} m^2   "
        f"threshold = {params.RELOCALIZE_COST_THRESHOLD}   after_scans = {params.RELOCALIZE_AFTER_SCANS}",
    )
    print(
        f"  fit_cost      p10={np.percentile(costs, 10):.4f}  p50={np.percentile(costs, 50):.4f}  "
        f"p90={np.percentile(costs, 90):.4f}  max={costs.max():.4f}",
    )
    print(
        f"  off_beams     p10={np.percentile(beams, 10):.3f}  p50={np.percentile(beams, 50):.3f}  "
        f"p90={np.percentile(beams, 90):.3f}  max={beams.max():.3f}",
    )
    over = int((costs > params.RELOCALIZE_COST_THRESHOLD).sum())
    print(f"  ticks over threshold: {over}/{n} ({100 * over / n:.1f}%)")
    print(
        f"  longest bad-fit streak: {max_streak} "
        f"(needs {params.RELOCALIZE_AFTER_SCANS}; starts at t={max_streak_start_s if max_streak_start_s is None else round(max_streak_start_s, 1)}s)  "
        f"-> rescue would fire {would_fire} time(s)",
    )

    print("   t(s)      pose            fit_cost  over?  off_beams  streak")
    for elapsed, x, y, cost, off_track, off_beams, streak in rows:
        flag = "OVER" if cost > params.RELOCALIZE_COST_THRESHOLD else "    "
        mark = " OFFTRACK" if off_track else ""
        print(
            f"  {elapsed:6.1f}  ({x:6.2f},{y:6.2f})  {cost:8.4f}  {flag}  {off_beams:8.3f}  {streak:4d}{mark}",
        )


def main() -> None:
    parser = create_bags_parser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stride", type=int, default=10, help="analyse every Nth nav_debug tick")
    args = parser.parse_args()
    for bag_dir in args.bag_dirs:
        analyse(bag_dir, args.stride)


if __name__ == "__main__":
    main()

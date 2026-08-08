"""Test whether the direction gate's single side ray is what starved it.

``infer_direction`` reads the side ranges through ``_nearest_ray``, which
returns ONE ray -- the single beam closest to +/-90 deg. A Slamtec emits no
return off dark or shallow-incidence surfaces and the gateway substitutes max
range, so one unlucky beam turns a legitimate 2.5 m opening into a 12 m
"dropout" that the estimator then rejects outright.

This replays the raw ``/scan`` messages and, for each one, compares the single
ray against a windowed reading (median of the VALID returns within a few degrees
of +/-90 deg). If the windowed reading recovers in-track distances where the
single ray reads max range, the gate was starved by beam-level noise rather than
by an absent signal.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_side_ray_robustness.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX [--window-deg 5]
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import NavigationTuning

from scripts.common.bag_io import create_bag_parser, open_reader, read_bag
from scripts.common.tables import print_table
from src.navigation.utils import _wrap
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

if TYPE_CHECKING:
    from collections.abc import Sequence

_MIN_VALID_M = RobotSpecs.LIDAR_MIN_RANGE
_MAX_RANGE_M = RobotSpecs.LIDAR_MAX_RANGE - 0.1
_MIN_VOTES = 5
_MAT_CENTRE_X = 1.5
_MAT_CENTRE_Y = 1.5


def _single(ranges: Sequence[float], angles: Sequence[float], target: float) -> float:
    idx = min(range(len(angles)), key=lambda i: abs(_wrap(angles[i] - target)))
    return ranges[idx]


def _windowed(ranges: Sequence[float], angles: Sequence[float], target: float, half_width: float) -> float | None:
    """Median of the in-track returns within ``half_width`` of ``target``.

    Returns None when every beam in the window is a dropout -- that is a
    genuinely absent reading, not a recoverable one.
    """
    vals = sorted(
        r
        for r, a in zip(ranges, angles, strict=False)
        if abs(_wrap(a - target)) <= half_width and _MIN_VALID_M < r < _MAX_RANGE_M
    )
    if not vals:
        return None
    return vals[len(vals) // 2]


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    parser.add_argument("--window-deg", type=float, default=5.0)
    args = parser.parse_args()
    half = math.radians(args.window_deg)

    tuning = NavigationTuning.load_default()
    estimator = tuning.direction_estimator

    reader = open_reader(args.bag_dir)
    # Same rotation ROS2HardwareGateway._lidar_callback applies -- the LIDAR is
    # mounted inverted, so raw bearings are 180 deg out and a replay that skips
    # this reads left as right and infers the mirror image of the direction
    # the node actually inferred.
    scans, nav_rows = read_bag(reader, _LIDAR_YAW_OFFSET_RAD)
    yaws: list[tuple[float, float]] = [(t, snap.pose_yaw) for t, snap in nav_rows if snap.pose_yaw is not None]
    poses: list[tuple[float, float, float]] = [
        (t, snap.pose_x, snap.pose_y) for t, snap in nav_rows if snap.pose_x is not None and snap.pose_y is not None
    ]

    print(f"== {args.bag_dir.name}  scans={len(scans)}  window=+/-{args.window_deg:.0f}deg")

    if not scans:
        print("no /scan messages")
        return

    beams = len(scans[0][1].ranges_m)
    total = sum(len(scan.ranges_m) for _, scan in scans)
    dropouts = sum(1 for _, scan in scans for v in scan.ranges_m if v >= _MAX_RANGE_M)
    print(f"beams/scan={beams}  overall max-range fraction: {100.0 * dropouts / total:.1f}%")

    def nearest_yaw(t: float) -> float | None:
        if not yaws:
            return None
        return min(yaws, key=lambda p: abs(p[0] - t))[1]

    recovered = Counter()
    rows = []
    for t, scan in scans:
        yaw = nearest_yaw(t)
        if yaw is None:
            continue
        s_l = _single(scan.ranges_m, scan.angles_rad, math.pi / 2)
        s_r = _single(scan.ranges_m, scan.angles_rad, -math.pi / 2)
        w_l = _windowed(scan.ranges_m, scan.angles_rad, math.pi / 2, half)
        w_r = _windowed(scan.ranges_m, scan.angles_rad, -math.pi / 2, half)
        for single, win in ((s_l, w_l), (s_r, w_r)):
            if single >= _MAX_RANGE_M:
                recovered["single max-range"] += 1
                if win is None:
                    recovered["  window also empty (real dropout)"] += 1
                else:
                    recovered["  window recovers a reading"] += 1
                    if win > estimator.PLAUSIBLE_SPAN_THRESHOLD_M:
                        recovered["    ...and it is an OPEN side (>1.25m)"] += 1
        rows.append((t, yaw, s_l, s_r, w_l, w_r))

    print("\nside-ray recovery:")
    if recovered:
        recovery_rows = [(name, count) for name, count in recovered.items()]
        print_table(recovery_rows, ["metric", "count"])

    # Winding sense of the pose trace, as ground truth.
    total_ang = 0.0
    prev = None
    for _, x, y in poses:
        ang = math.atan2(y - _MAT_CENTRE_Y, x - _MAT_CENTRE_X)
        if prev is not None:
            total_ang += _wrap(ang - prev)
        prev = ang
    truth = "counterclockwise" if total_ang > 0 else "clockwise"
    print(f"\npose winding: {total_ang / (2 * math.pi):+.2f} turns -> travelling {truth}")

    def settle(use_window: bool) -> tuple[float, str, int] | None:
        votes: Counter[str] = Counter()
        cast = 0
        for t, yaw, s_l, s_r, w_l, w_r in rows:
            left, right = (w_l, w_r) if use_window else (s_l, s_r)
            if left is None or right is None:
                continue
            if left > estimator.MAX_IN_TRACK_RANGE_M or right > estimator.MAX_IN_TRACK_RANGE_M:
                continue
            axis_error = abs(_wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
            if axis_error > estimator.ALIGNMENT_TOLERANCE_RAD:
                continue
            if left + right <= estimator.PLAUSIBLE_SPAN_THRESHOLD_M:
                continue
            if abs(left - right) < estimator.MIN_ASYMMETRY_M:
                continue
            inferred = "clockwise" if right > left else "counterclockwise"
            votes[inferred] += 1
            cast += 1
            if votes[inferred] >= _MIN_VOTES:
                return t, inferred, cast
        return None

    print("\nsettle with shipped thresholds:")
    for label, use_window in (("single ray (shipped)", False), ("windowed median", True)):
        result = settle(use_window)
        if result is None:
            print(f"  {label:24s} never settles")
        else:
            t, direction, cast = result
            mark = "OK" if direction == truth else "WRONG"
            print(f"  {label:24s} settles {direction} at {t:6.1f}s ({cast} votes) [{mark}]")


if __name__ == "__main__":
    main()

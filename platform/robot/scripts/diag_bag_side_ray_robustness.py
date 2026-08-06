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
    pixi run -e dev python scripts/diag_bag_side_ray_robustness.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX [--window-deg 5]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from src.navigation.direction_estimator import (
    _MAX_IN_TRACK_RANGE_M,
    _MAX_PLAUSIBLE_SPAN_M,
    _MIN_ASYMMETRY_M,
)
from src.navigation.utils import _ALIGNMENT_TOLERANCE_RAD, _wrap
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

MAX_RANGE_M = 11.9
MIN_VALID_M = 0.05
MIN_VOTES = 5


def _single(ranges: list[float], angles: list[float], target: float) -> float:
    idx = min(range(len(angles)), key=lambda i: abs(_wrap(angles[i] - target)))
    return ranges[idx]


def _windowed(ranges: list[float], angles: list[float], target: float, half_width: float) -> float | None:
    """Median of the in-track returns within ``half_width`` of ``target``.

    Returns None when every beam in the window is a dropout -- that is a
    genuinely absent reading, not a recoverable one.
    """
    vals = sorted(
        r
        for r, a in zip(ranges, angles, strict=False)
        if abs(_wrap(a - target)) <= half_width and MIN_VALID_M < r < MAX_RANGE_M
    )
    if not vals:
        return None
    return vals[len(vals) // 2]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--window-deg", type=float, default=5.0)
    args = parser.parse_args()
    half = math.radians(args.window_deg)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t0 = None
    scans: list[tuple[float, list[float], list[float]]] = []
    yaws: list[tuple[float, float]] = []
    poses: list[tuple[float, float, float]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = (t - t0) / 1e9
        if topic == "/scan":
            msg = deserialize_message(data, LaserScan)
            # Same rotation ROS2HardwareGateway._lidar_callback applies -- the
            # LIDAR is mounted inverted, so raw bearings are 180 deg out and a
            # replay that skips this reads left as right and infers the mirror
            # image of the direction the node actually inferred.
            ranges = [
                v if math.isfinite(v) else MAX_RANGE_M + 0.1
                for v in msg.ranges
            ]
            n = len(ranges)
            angles = [
                msg.angle_min + i * (msg.angle_max - msg.angle_min) / max(n - 1, 1) + _LIDAR_YAW_OFFSET_RAD
                for i in range(n)
            ]
            scans.append((rel, ranges, angles))
        elif topic == "/nav_debug":
            p = json.loads(deserialize_message(data, String).data)
            if isinstance(p.get("pose_yaw"), (int, float)):
                yaws.append((rel, p["pose_yaw"]))
            if isinstance(p.get("pose_x"), (int, float)) and isinstance(p.get("pose_y"), (int, float)):
                poses.append((rel, p["pose_x"], p["pose_y"]))

    print(f"== {args.bag_dir.name}  scans={len(scans)}  window=+/-{args.window_deg:.0f}deg")

    if not scans:
        print("no /scan messages")
        return

    beams = len(scans[0][1])
    total = sum(len(r) for _, r, _ in scans)
    dropouts = sum(1 for _, r, _ in scans for v in r if v >= MAX_RANGE_M)
    print(f"beams/scan={beams}  overall max-range fraction: {100.0 * dropouts / total:.1f}%")

    def nearest_yaw(t: float) -> float | None:
        if not yaws:
            return None
        return min(yaws, key=lambda p: abs(p[0] - t))[1]

    recovered = Counter()
    rows = []
    for t, ranges, angles in scans:
        yaw = nearest_yaw(t)
        if yaw is None:
            continue
        s_l = _single(ranges, angles, math.pi / 2)
        s_r = _single(ranges, angles, -math.pi / 2)
        w_l = _windowed(ranges, angles, math.pi / 2, half)
        w_r = _windowed(ranges, angles, -math.pi / 2, half)
        for single, win in ((s_l, w_l), (s_r, w_r)):
            if single >= MAX_RANGE_M:
                recovered["single max-range"] += 1
                if win is None:
                    recovered["  window also empty (real dropout)"] += 1
                else:
                    recovered["  window recovers a reading"] += 1
                    if win > _MAX_PLAUSIBLE_SPAN_M:
                        recovered["    ...and it is an OPEN side (>1.25m)"] += 1
        rows.append((t, yaw, s_l, s_r, w_l, w_r))

    print("\nside-ray recovery:")
    for name, count in recovered.items():
        print(f"  {name:44s} {count}")

    # Winding sense of the pose trace, as ground truth.
    total_ang = 0.0
    prev = None
    for _, x, y in poses:
        ang = math.atan2(y - 1.5, x - 1.5)
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
            if left > _MAX_IN_TRACK_RANGE_M or right > _MAX_IN_TRACK_RANGE_M:
                continue
            axis_error = abs(_wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
            if axis_error > _ALIGNMENT_TOLERANCE_RAD:
                continue
            if left + right <= _MAX_PLAUSIBLE_SPAN_M:
                continue
            if abs(left - right) < _MIN_ASYMMETRY_M:
                continue
            inferred = "clockwise" if right > left else "counterclockwise"
            votes[inferred] += 1
            cast += 1
            if votes[inferred] >= MIN_VOTES:
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

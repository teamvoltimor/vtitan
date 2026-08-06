"""Compare candidate "is the corridor ending?" tests against the bags.

``follow_corridor`` commits to a hard-over corner turn whenever the forward
clearance -- the MINIMUM over a narrow +/-8 deg cone -- drops below
``turn_clearance_m``. That cone is too narrow to tell a corridor that is ending
from a chassis pointed obliquely at the wall beside it: 0.24 m off a wall at 30
deg puts the whole cone on that wall at 0.24/sin(30) = 0.48 m, under the 0.60 m
threshold, mid-corridor.

The physically right question is whether there is anywhere ahead to go, which
is a MAXIMUM over a WIDER arc: at a real corner the end wall blocks every
bearing in the arc, while an oblique chassis still has the corridor's own axis
inside it, reading metres.

This scores both tests against a pose-derived notion of actually being near a
corner, on any bag, so the arc and threshold are chosen from data rather than
guessed.

Usage:
    pixi run -e dev python scripts/diag_bag_turn_trigger.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from src.navigation.corridor_follower import TURN_CLEARANCE_M
from src.navigation.utils import _forward_clearance, _wrap
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

MIN_VALID_M = 0.05
MAX_RANGE_M = 11.9
# Mat corners; the loop turns at each one.
CORNERS = ((0.0, 0.0), (3.0, 0.0), (0.0, 3.0), (3.0, 3.0))
CORNER_RADIUS_M = 1.0


def _arc_max(ranges: list[float], angles: list[float], half_fov: float) -> float:
    """Furthest in-track return within ``half_fov`` of straight ahead.

    Max, not min: this asks whether ANY bearing ahead still has room, which is
    what distinguishes a corridor that has ended from one seen at an angle.
    """
    vals = [
        r
        for r, a in zip(ranges, angles, strict=False)
        if abs(_wrap(a)) <= half_fov and MIN_VALID_M < r < MAX_RANGE_M
    ]
    return max(vals) if vals else 0.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t0 = None
    scans = []
    poses: list[tuple[float, float, float]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = (t - t0) / 1e9
        if topic == "/scan":
            msg = deserialize_message(data, LaserScan)
            ranges = [v if math.isfinite(v) else 12.0 for v in msg.ranges]
            n = len(ranges)
            angles = [
                msg.angle_min + i * (msg.angle_max - msg.angle_min) / max(n - 1, 1) + _LIDAR_YAW_OFFSET_RAD
                for i in range(n)
            ]
            scans.append((rel, ranges, angles))
        elif topic == "/nav_debug":
            p = json.loads(deserialize_message(data, String).data)
            if isinstance(p.get("pose_x"), (int, float)) and isinstance(p.get("pose_y"), (int, float)):
                poses.append((rel, p["pose_x"], p["pose_y"]))

    print(f"== {args.bag_dir.name}  scans={len(scans)}")
    if not scans or not poses:
        print("insufficient data")
        return

    rows = []
    for t, ranges, angles in scans:
        _, x, y = min(poses, key=lambda p: abs(p[0] - t))
        near = min(math.hypot(x - cx, y - cy) for cx, cy in CORNERS) < CORNER_RADIUS_M
        rows.append(
            {
                "near": near,
                "min8": _forward_clearance(ranges, angles),
                **{
                    f"max{arc}": _arc_max(ranges, angles, math.radians(arc))
                    for arc in (12, 15, 20, 30)
                },
            }
        )

    truth = sum(1 for r in rows if r["near"])
    n = len(rows)
    print(f"pose says near a corner (<{CORNER_RADIUS_M:.1f}m): {truth}/{n} = {100.0 * truth / n:.1f}%\n")

    def episodes(mask: list[bool]) -> int:
        """Contiguous firing runs. One sustained episode per corner per lap is
        what a working turn trigger looks like; a high tick count spread over
        dozens of episodes is a weave, not cornering.
        """
        return sum(1 for i, on in enumerate(mask) if on and (i == 0 or not mask[i - 1]))

    candidates: list[tuple[str, object]] = [
        ("min +/-8 < 0.60 (shipped)", lambda r: r["min8"] < TURN_CLEARANCE_M),
    ]
    for arc in (12, 15, 20, 30):
        for thr in (0.70, 0.80):
            candidates.append(
                (f"max +/-{arc} < {thr:.2f}", lambda r, a=arc, t=thr: r[f"max{a}"] < t)
            )
    for arc in (12, 15, 20):
        for thr in (0.80, 1.00):
            candidates.append(
                (
                    f"min+/-8<0.60 AND max+/-{arc}<{thr:.2f}",
                    lambda r, a=arc, t=thr: r["min8"] < TURN_CLEARANCE_M and r[f"max{a}"] < t,
                )
            )

    print(f"{'test':>32s} {'fires':>7s} {'%':>6s} {'prec':>7s} {'recall':>7s} {'episodes':>9s}")
    for label, pred in candidates:
        mask = [bool(pred(r)) for r in rows]
        fires = [r for r, on in zip(rows, mask, strict=True) if on]
        hit = sum(1 for r in fires if r["near"])
        precision = 100.0 * hit / len(fires) if fires else float("nan")
        recall = 100.0 * hit / truth if truth else float("nan")
        print(
            f"{label:>32s} {len(fires):7d} {100.0 * len(fires) / n:5.1f}% "
            f"{precision:6.1f}% {recall:6.1f}% {episodes(mask):9d}"
        )


if __name__ == "__main__":
    main()

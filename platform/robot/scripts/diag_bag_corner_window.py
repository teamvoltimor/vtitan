"""Did the robot ever enter the window where travel direction is readable?

The direction signal only exists in a narrow band: far enough up the corridor
that the inner block has ended (so the sideways ray runs off down the next
corridor) but not yet far enough that the corridor follower has begun its turn
and swung the heading past the alignment gate. If a blind run never settles, the
first thing to establish is whether that band was ever entered at all -- a gate
cannot refuse a reading that was never presented.

For every scan this prints the forward clearance, the axis error, and both side
ranges, then summarises how often the three preconditions held together.

Usage:
    pixi run -e dev python scripts/diag_bag_corner_window.py \
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
from shared.config.constants import CorridorDimensions, RobotSpecs
from std_msgs.msg import String

from src.navigation.corridor_follower import TURN_CLEARANCE_M
from src.navigation.direction_estimator import _MAX_PLAUSIBLE_SPAN_M, CORNER_CLEARANCE_M
from src.navigation.utils import _ALIGNMENT_TOLERANCE_RAD, _forward_clearance, _nearest_ray, _wrap
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD


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
    yaws: list[tuple[float, float]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = (t - t0) / 1e9
        if topic == "/scan":
            msg = deserialize_message(data, LaserScan)
            # Same rotation ROS2HardwareGateway._lidar_callback applies; see the
            # note in diag_bag_side_ray_robustness.py.
            ranges = [v if math.isfinite(v) else 12.0 for v in msg.ranges]
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

    print(f"== {args.bag_dir.name}  scans={len(scans)}")
    if not scans or not yaws:
        print("insufficient data")
        return

    rows = []
    for t, ranges, angles in scans:
        yaw = min(yaws, key=lambda p: abs(p[0] - t))[1]
        fwd = _forward_clearance(ranges, angles)
        axis = abs(_wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
        left = _nearest_ray(ranges, angles, math.pi / 2)
        right = _nearest_ray(ranges, angles, -math.pi / 2)
        rows.append((t, fwd, axis, left, right))

    finite = [r[1] for r in rows if math.isfinite(r[1])]
    finite.sort()
    if finite:
        print(
            f"forward clearance: min={finite[0]:.2f} "
            f"p10={finite[len(finite) // 10]:.2f} median={finite[len(finite) // 2]:.2f} "
            f"max={finite[-1]:.2f}   (corner_clearance_m={CORNER_CLEARANCE_M:.2f})"
        )

    near_corner = [r for r in rows if r[1] < CORNER_CLEARANCE_M]
    aligned = [r for r in rows if r[2] <= _ALIGNMENT_TOLERANCE_RAD]
    open_side = [r for r in rows if max(r[3], r[4]) > _MAX_PLAUSIBLE_SPAN_M]
    both = [r for r in near_corner if r[2] <= _ALIGNMENT_TOLERANCE_RAD]
    all_three = [r for r in both if max(r[3], r[4]) > _MAX_PLAUSIBLE_SPAN_M]

    n = len(rows)
    print(f"\nof {n} scans:")
    print(f"  near a corner (fwd < {CORNER_CLEARANCE_M:.2f}m):      {len(near_corner):5d}  {100.0 * len(near_corner) / n:5.1f}%")
    print(f"  axis-aligned (err <= {math.degrees(_ALIGNMENT_TOLERANCE_RAD):.0f}deg):        {len(aligned):5d}  {100.0 * len(aligned) / n:5.1f}%")
    print(f"  a side reads open (> {_MAX_PLAUSIBLE_SPAN_M:.2f}m):     {len(open_side):5d}  {100.0 * len(open_side) / n:5.1f}%")
    print(f"  near corner AND aligned:              {len(both):5d}  {100.0 * len(both) / n:5.1f}%")
    print(f"  all three together (votable):         {len(all_three):5d}  {100.0 * len(all_three) / n:5.1f}%")

    # Which branch of follow_corridor each scan would have taken. The corner
    # branch steers hard over at +/-MAX_CENTERING_STEER, which swings the
    # heading past the alignment gate -- so a run that sits in it is a run that
    # cannot read its own direction, and the two reinforce each other.
    turning = [r for r in rows if r[1] < TURN_CLEARANCE_M]
    backing = [r for r in turning if r[1] < RobotSpecs.LENGTH]
    held = [r for r in rows if r[1] >= TURN_CLEARANCE_M and max(r[3], r[4]) > CorridorDimensions.WIDE + 0.35]
    centring = n - len(turning) - len(held)
    print(f"\nfollow_corridor branch occupancy (turn_clearance_m={TURN_CLEARANCE_M:.2f}):")
    print(f"  corner turn (hard over):              {len(turning):5d}  {100.0 * len(turning) / n:5.1f}%")
    print(f"    of which backing off:               {len(backing):5d}  {100.0 * len(backing) / n:5.1f}%")
    print(f"  hold straight (a side disqualified):  {len(held):5d}  {100.0 * len(held) / n:5.1f}%")
    print(f"  centring:                             {centring:5d}  {100.0 * centring / n:5.1f}%")

    if near_corner:
        print("\nnearest-corner scans (lowest forward clearance), 15 shown:")
        print("      t     fwd   axis_deg    left   right")
        for t, fwd, axis, left, right in sorted(near_corner, key=lambda r: r[1])[:15]:
            print(f"  {t:6.1f}  {fwd:6.2f}  {math.degrees(axis):8.1f}  {left:6.2f}  {right:6.2f}")


if __name__ == "__main__":
    main()

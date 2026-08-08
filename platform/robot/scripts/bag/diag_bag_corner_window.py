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
    pixi run -e dev python scripts/bag/diag_bag_corner_window.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning

from scripts.common.bag_io import create_bag_parser, open_reader, read_bag
from scripts.common.tables import print_table
from src.navigation.utils import _forward_clearance, _nearest_ray, _wrap
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD


def main() -> None:
    parser = create_bag_parser("Check direction inference window conditions")
    args = parser.parse_args()

    tuning = NavigationTuning.load_default()
    alignment_tol = tuning.direction_estimator.ALIGNMENT_TOLERANCE_RAD
    corner_clearance_m = tuning.direction_estimator.CORNER_CLEARANCE_M
    turn_clearance_m = tuning.corridor_follower.TURN_CLEARANCE_M
    plausible_span = tuning.direction_estimator.PLAUSIBLE_SPAN_THRESHOLD_M

    reader = open_reader(args.bag_dir)
    scans, nav_rows = read_bag(reader, _LIDAR_YAW_OFFSET_RAD)
    yaws: list[tuple[float, float]] = [(t, snap.pose_yaw) for t, snap in nav_rows if snap.pose_yaw is not None]

    print(f"== {args.bag_dir.name}  scans={len(scans)}")
    if not scans or not yaws:
        print("insufficient data")
        return

    rows = []
    for t, scan in scans:
        yaw = min(yaws, key=lambda p: abs(p[0] - t))[1]
        fwd = _forward_clearance(scan.ranges_m, scan.angles_rad, tuning)
        axis = abs(_wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
        left = _nearest_ray(scan.ranges_m, scan.angles_rad, math.pi / 2)
        right = _nearest_ray(scan.ranges_m, scan.angles_rad, -math.pi / 2)
        rows.append((t, fwd, axis, left, right))

    finite = [r[1] for r in rows if math.isfinite(r[1])]
    finite.sort()
    if finite:
        print(
            f"forward clearance: min={finite[0]:.2f} "
            f"p10={finite[len(finite) // 10]:.2f} median={finite[len(finite) // 2]:.2f} "
            f"max={finite[-1]:.2f}   (corner_clearance_m={corner_clearance_m:.2f})"
        )

    near_corner = [r for r in rows if r[1] < corner_clearance_m]
    aligned = [r for r in rows if r[2] <= alignment_tol]
    open_side = [r for r in rows if max(r[3], r[4]) > plausible_span]
    both = [r for r in near_corner if r[2] <= alignment_tol]
    all_three = [r for r in both if max(r[3], r[4]) > plausible_span]

    n = len(rows)
    print(f"\nof {n} scans:")
    print(f"  near a corner (fwd < {corner_clearance_m:.2f}m):      {len(near_corner):5d}  {100.0 * len(near_corner) / n:5.1f}%")
    print(f"  axis-aligned (err <= {math.degrees(alignment_tol):.0f}deg):        {len(aligned):5d}  {100.0 * len(aligned) / n:5.1f}%")
    print(f"  a side reads open (> {plausible_span:.2f}m):     {len(open_side):5d}  {100.0 * len(open_side) / n:5.1f}%")
    print(f"  near corner AND aligned:              {len(both):5d}  {100.0 * len(both) / n:5.1f}%")
    print(f"  all three together (votable):         {len(all_three):5d}  {100.0 * len(all_three) / n:5.1f}%")

    # Which branch of follow_corridor each scan would have taken. The corner
    # branch steers hard over at +/-MAX_CENTERING_STEER, which swings the
    # heading past the alignment gate -- so a run that sits in it is a run that
    # cannot read its own direction, and the two reinforce each other.
    turning = [r for r in rows if r[1] < turn_clearance_m]
    backing = [r for r in turning if r[1] < RobotSpecs.LENGTH]
    held = [r for r in rows if r[1] >= turn_clearance_m and max(r[3], r[4]) > CorridorDimensions.WIDE + 0.35]
    centring = n - len(turning) - len(held)
    print(f"\nfollow_corridor branch occupancy (turn_clearance_m={turn_clearance_m:.2f}):")
    print(f"  corner turn (hard over):              {len(turning):5d}  {100.0 * len(turning) / n:5.1f}%")
    print(f"    of which backing off:               {len(backing):5d}  {100.0 * len(backing) / n:5.1f}%")
    print(f"  hold straight (a side disqualified):  {len(held):5d}  {100.0 * len(held) / n:5.1f}%")
    print(f"  centring:                             {centring:5d}  {100.0 * centring / n:5.1f}%")

    if near_corner:
        print("\nnearest-corner scans (lowest forward clearance), 15 shown:")
        rows = [
            (t, fwd, math.degrees(axis), left, right)
            for t, fwd, axis, left, right in sorted(near_corner, key=lambda r: r[1])[:15]
        ]
        print_table(rows, ["t", "fwd", "axis_deg", "left", "right"])


if __name__ == "__main__":
    main()

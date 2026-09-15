"""Print every escape/maneuver tick from a recorded race bag.

Shows the K-turn steering sign, threat direction, escape escalation counter and
the pose the stuck detector is being fed, so a wedged run can be read as a
sequence of maneuver decisions rather than a sampled timeline.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_escape.py data/live/runs/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import NO_TIME_LIMIT_S, create_bag_parser, load_nav_debug_rows


def _pose(snap: object) -> str:
    """The pose triple, or ``unposed`` for a tick recorded before the navigator had a fix.

    Those ticks are the point of this script rather than noise to drop: the
    stuck detector is fed them too, so formatting them as None is what shows a
    wedge that began before localisation settled.
    """
    x, y, yaw = snap.pose_x, snap.pose_y, snap.pose_yaw
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return "unposed"
    yaw_s = f"{yaw:.2f}" if isinstance(yaw, (int, float)) else "none"
    return f"({x:.3f},{y:.3f},{yaw_s})"


def main() -> int:
    parser = create_bag_parser("Print every escape/maneuver tick with the pose the stuck detector is fed.")
    parser.add_argument("--until", type=float, default=NO_TIME_LIMIT_S)
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    prev_key = None
    for ts, snap in rows:
        if ts > args.until:
            continue
        key = (
            snap.phase,
            snap.active_maneuver_type,
            snap.maneuver_steering,
            snap.maneuver_speed_mps,
            snap.escape_count,
            snap.direction,
        )
        if key == prev_key:
            continue
        prev_key = key
        print(
            f"{ts:7.2f}s phase={snap.phase!s:22} man={snap.active_maneuver_type!s:16} "
            f"steer={snap.maneuver_steering!s:7} spd={snap.maneuver_speed_mps!s:7} "
            f"frames={snap.maneuver_frames_left!s:4} esc={snap.escape_count!s:3} "
            f"dir={snap.direction!s:18} pose={_pose(snap):26} "
            f"min_r={snap.min_lidar_range_m!s:8} stuck={snap.stuck_count}",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

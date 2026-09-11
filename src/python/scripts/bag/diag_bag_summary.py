"""Quick summary of a recorded race bag: robot_state transitions, nav_debug
trajectory (direction/section changes, pose extent), and drive command stats.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_summary.py data/live/runs/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import (
    Topics,
    append_if_changed,
    create_bag_parser,
    decode_nav_debug,
    elapsed_seconds,
    open_reader,
)
from scripts.common.tables import print_table


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)

    t_start = None
    state_transitions: list[tuple[float, str]] = []
    nav_debug_first = None
    nav_debug_last = None
    section_transitions: list[tuple[float, str]] = []
    direction_transitions: list[tuple[float, str]] = []
    drive_count = 0
    max_speed = 0.0
    min_speed = 0.0
    last_pose = None
    total_dist = 0.0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = elapsed_seconds(t, t_start)

        if topic == Topics.ROBOT_STATE:
            msg = deserialize_message(data, String)
            append_if_changed(state_transitions, ts, msg.data)
        elif topic == Topics.NAV_DEBUG:
            snap = decode_nav_debug(data)
            if nav_debug_first is None:
                nav_debug_first = (ts, snap)
            nav_debug_last = (ts, snap)
            # "section" == current_corridor (a Section value: north/south/east/west).
            sect = snap.current_corridor
            if sect is not None:
                append_if_changed(section_transitions, ts, sect)
            dirn = snap.direction
            if dirn is not None:
                append_if_changed(direction_transitions, ts, dirn)
            px, py = snap.pose_x, snap.pose_y
            if px is not None and py is not None:
                if last_pose is not None:
                    total_dist += ((px - last_pose[0]) ** 2 + (py - last_pose[1]) ** 2) ** 0.5
                last_pose = (px, py)
        elif topic == Topics.ACKERMANN_CMD:
            msg = deserialize_message(data, AckermannDriveStamped)
            drive_count += 1
            max_speed = max(max_speed, msg.drive.speed)
            min_speed = min(min_speed, msg.drive.speed)

    print(f"bag: {args.bag_dir}")
    print(f"duration: {ts:.1f}s")

    if state_transitions:
        print(f"\nrobot_state transitions ({len(state_transitions)}):")
        print_table(state_transitions, ["t", "state"])

    if section_transitions:
        print(f"\nsection transitions ({len(section_transitions)}):")
        print_table(section_transitions, ["t", "section"])

    if direction_transitions:
        print(f"\ndirection transitions ({len(direction_transitions)}):")
        print_table(direction_transitions, ["t", "direction"])
    print(f"\ndrive cmds: {drive_count}, speed range [{min_speed:.3f}, {max_speed:.3f}] m/s")
    print(f"total pose travel distance (nav_debug): {total_dist:.2f} m")
    if nav_debug_first:
        print(f"\nfirst nav_debug @ {nav_debug_first[0]:.2f}s: {nav_debug_first[1]!r:.400}")
    if nav_debug_last:
        print(f"\nlast nav_debug @ {nav_debug_last[0]:.2f}s: {nav_debug_last[1]!r:.400}")


if __name__ == "__main__":
    main()

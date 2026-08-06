"""Quick summary of a recorded race bag: robot_state transitions, nav_debug
trajectory (direction/section changes, pose extent), and drive command stats.

Usage:
    pixi run -e dev python scripts/diag_bag_summary.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

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
    guard_holds = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = (t - t_start) / 1e9

        if topic == "/robot_state":
            msg = deserialize_message(data, String)
            if not state_transitions or state_transitions[-1][1] != msg.data:
                state_transitions.append((ts, msg.data))
        elif topic == "/nav_debug":
            payload = json.loads(deserialize_message(data, String).data)
            if nav_debug_first is None:
                nav_debug_first = (ts, payload)
            nav_debug_last = (ts, payload)
            sect = payload.get("section")
            if sect is not None and (not section_transitions or section_transitions[-1][1] != sect):
                section_transitions.append((ts, sect))
            dirn = payload.get("direction")
            if dirn is not None and (not direction_transitions or direction_transitions[-1][1] != dirn):
                direction_transitions.append((ts, dirn))
            px, py = payload.get("pose_x"), payload.get("pose_y")
            if px is not None and py is not None:
                if last_pose is not None:
                    total_dist += ((px - last_pose[0]) ** 2 + (py - last_pose[1]) ** 2) ** 0.5
                last_pose = (px, py)
            if payload.get("localizer_guard_hold"):
                guard_holds += 1
        elif topic == "/ackermann_cmd":
            msg = deserialize_message(data, AckermannDriveStamped)
            drive_count += 1
            max_speed = max(max_speed, msg.drive.speed)
            min_speed = min(min_speed, msg.drive.speed)

    print(f"bag: {args.bag_dir}")
    print(f"duration: {ts:.1f}s")
    print(f"\nrobot_state transitions ({len(state_transitions)}):")
    for t_, s in state_transitions:
        print(f"  {t_:7.2f}s  {s}")
    print(f"\nsection transitions ({len(section_transitions)}):")
    for t_, s in section_transitions:
        print(f"  {t_:7.2f}s  {s}")
    print(f"\ndirection transitions ({len(direction_transitions)}):")
    for t_, s in direction_transitions:
        print(f"  {t_:7.2f}s  {s}")
    print(f"\ndrive cmds: {drive_count}, speed range [{min_speed:.3f}, {max_speed:.3f}] m/s")
    print(f"total pose travel distance (nav_debug): {total_dist:.2f} m")
    print(f"localizer_guard_hold ticks: {guard_holds}")
    if nav_debug_first:
        print(f"\nfirst nav_debug @ {nav_debug_first[0]:.2f}s: {json.dumps(nav_debug_first[1])[:400]}")
    if nav_debug_last:
        print(f"\nlast nav_debug @ {nav_debug_last[0]:.2f}s: {json.dumps(nav_debug_last[1])[:400]}")


if __name__ == "__main__":
    main()

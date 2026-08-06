"""Replay a race bag through LapDetector's two gates to see which one blocked.

A lap only counts when the waypoint index has wrapped AND the robot crosses the
start line's normal from negative to non-negative while inside the start
section. When the counter stays at zero through a race the robot visibly
completed, exactly one of those gates is at fault -- this prints both, per
candidate crossing, so it is clear which.

Usage:
    pixi run -e dev python scripts/diag_bag_lap_gate.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from src.navigation.race_tracker import TRAVEL_DIRS
from shared.domain.enums import Direction, Section


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
    rows: list[dict] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic != "/nav_debug":
            continue
        p = json.loads(deserialize_message(data, String).data)
        p["_t"] = (t - t0) / 1e9
        rows.append(p)

    posed = [r for r in rows if isinstance(r.get("pose_x"), (int, float))]
    if not posed:
        print("no posed samples")
        return

    direction = next((r["direction"] for r in rows if r.get("direction")), None)
    print(f"== {args.bag_dir.name}  direction={direction}  samples={len(posed)}")

    xs = [r["pose_x"] for r in posed]
    ys = [r["pose_y"] for r in posed]
    print(f"pose extent: x {min(xs):.2f}..{max(xs):.2f}   y {min(ys):.2f}..{max(ys):.2f}")

    # Gate 2: did the waypoint index ever wrap?
    idx = [(r["_t"], r.get("waypoint_index")) for r in posed if r.get("waypoint_index") is not None]
    wraps = [(t1, a, b) for (t0_, a), (t1, b) in zip(idx, idx[1:]) if b < a - 1]
    print(f"waypoint_index range: {min(i for _, i in idx)}..{max(i for _, i in idx)}")
    print(f"waypoint wraps: {len(wraps)}  " + ", ".join(f"{t:.0f}s({a}->{b})" for t, a, b in wraps[:12]))

    # Gate 1: geometric crossings, using the run's own start pose/section.
    start = posed[0]
    origin = (start["pose_x"], start["pose_y"])
    print(f"assumed start origin (first posed sample): ({origin[0]:.2f}, {origin[1]:.2f})")

    dir_enum = Direction.COUNTERCLOCKWISE if direction == "counterclockwise" else Direction.CLOCKWISE
    for section in (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST):
        nx, ny = TRAVEL_DIRS[(section, dir_enum)]
        prev = None
        crossings = 0
        in_section_crossings = 0
        for r in posed:
            dot = (r["pose_x"] - origin[0]) * nx + (r["pose_y"] - origin[1]) * ny
            if prev is not None and prev < 0.0 <= dot:
                crossings += 1
                if str(r.get("current_corridor")) == section.value:
                    in_section_crossings += 1
            prev = dot
        print(
            f"  start_section={section.value:6} normal=({nx:+.0f},{ny:+.0f})  "
            f"neg->pos crossings={crossings:3}  with corridor=={section.value}: {in_section_crossings}"
        )

    # How much time was actually spent in each corridor?
    from collections import Counter

    counts = Counter(str(r.get("current_corridor")) for r in posed)
    print("corridor sample counts: " + ", ".join(f"{k}={v}" for k, v in counts.most_common()))


if __name__ == "__main__":
    main()

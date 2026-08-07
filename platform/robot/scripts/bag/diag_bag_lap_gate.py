"""Replay a race bag through LapDetector's two gates to see which one blocked.

A lap only counts when the waypoint index has wrapped AND the robot crosses the
start line's normal from negative to non-negative while inside the start
section. When the counter stays at zero through a race the robot visibly
completed, exactly one of those gates is at fault -- this prints both, per
candidate crossing, so it is clear which.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_lap_gate.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Direction, Section

from scripts.common.bag_io import load_nav_debug_rows, print_table
from src.navigation.race_tracker import TRAVEL_DIRS


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    posed = [(t, snap) for t, snap in rows if isinstance(snap.pose_x, (int, float))]
    if not posed:
        print("no posed samples")
        return

    direction = next((snap.direction for _, snap in rows if snap.direction), None)
    print(f"== {args.bag_dir.name}  direction={direction}  samples={len(posed)}")

    xs = [snap.pose_x for _, snap in posed]
    ys = [snap.pose_y for _, snap in posed]
    print(f"pose extent: x {min(xs):.2f}..{max(xs):.2f}   y {min(ys):.2f}..{max(ys):.2f}")

    # Gate 2: did the waypoint index ever wrap?
    idx = [(t, snap.waypoint_index) for t, snap in posed if snap.waypoint_index is not None]
    wraps = [(t1, a, b) for (t0_, a), (t1, b) in zip(idx, idx[1:]) if b < a - 1]
    print(f"waypoint_index range: {min(i for _, i in idx)}..{max(i for _, i in idx)}")
    print(f"waypoint wraps: {len(wraps)}  " + ", ".join(f"{t:.0f}s({a}->{b})" for t, a, b in wraps[:12]))

    # Gate 1: geometric crossings, using the run's own start pose/section.
    _, start = posed[0]
    origin = (start.pose_x, start.pose_y)
    print(f"assumed start origin (first posed sample): ({origin[0]:.2f}, {origin[1]:.2f})")

    dir_enum = Direction.COUNTERCLOCKWISE if direction == "counterclockwise" else Direction.CLOCKWISE
    rows = []
    for section in (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST):
        nx, ny = TRAVEL_DIRS[(section, dir_enum)]
        prev = None
        crossings = 0
        in_section_crossings = 0
        for _, snap in posed:
            dot = (snap.pose_x - origin[0]) * nx + (snap.pose_y - origin[1]) * ny
            if prev is not None and prev < 0.0 <= dot:
                crossings += 1
                if str(snap.current_corridor) == section.value:
                    in_section_crossings += 1
            prev = dot
        rows.append((section.value, f"({nx:+.0f},{ny:+.0f})", crossings, in_section_crossings))
    print_table(rows, ["start_section", "normal", "neg->pos_crossings", "in_section"])

    # How much time was actually spent in each corridor?
    counts = Counter(str(snap.current_corridor) for _, snap in posed)
    print("corridor sample counts: " + ", ".join(f"{k}={v}" for k, v in counts.most_common()))


if __name__ == "__main__":
    main()

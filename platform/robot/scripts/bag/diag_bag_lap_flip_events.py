"""Show every sign flip of the lap-line dot test, and the label it happened under.

The detector needs the negative-to-non-negative flip and the start-section label
to coincide on the same sample. This lists the flips for a given origin together
with the section label the robot was carrying at that instant, which is what
separates "the robot never crossed the line" from "it crossed it while the label
said it was somewhere else".

Usage:
    pixi run -e dev python scripts/bag/diag_bag_lap_flip_events.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --origin measured
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Direction

from scripts.common.bag_io import load_nav_debug_rows, print_table
from src.navigation.race_tracker import TRAVEL_DIRS
from src.navigation.start_conditions import CANONICAL_SECTION, assumed_start_conditions


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--origin", choices=("assumed", "measured"), default="measured")
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)
    posed = [
        (t, snap) for t, snap in rows if isinstance(snap.pose_x, (int, float)) and isinstance(snap.pose_y, (int, float))
    ]
    direction_name = next((snap.direction for _, snap in rows if snap.direction), None)
    if direction_name is None or not posed:
        print("nothing to measure")
        return
    direction = Direction(direction_name)
    section = CANONICAL_SECTION
    nx, ny = TRAVEL_DIRS[(section, direction)]

    if args.origin == "assumed":
        pos = assumed_start_conditions(direction)["position"]
        origin = (pos["x"], pos["y"])
    else:
        origin = next(
            (
                (snap.start_measured_x, snap.start_measured_y)
                for _, snap in rows
                if isinstance(getattr(snap, "start_measured_x", None), (int, float))
            ),
            None,
        )
        if origin is None:
            print("bag has no measured start")
            return

    print(f"== {args.bag_dir.name}  direction={direction.value}  origin={args.origin} ({origin[0]:.3f}, {origin[1]:.3f})")

    out = []
    prev_dot = None
    for t, snap in posed:
        dot = (snap.pose_x - origin[0]) * nx + (snap.pose_y - origin[1]) * ny
        if prev_dot is not None and prev_dot < 0.0 <= dot:
            out.append(
                (
                    f"{t:.1f}",
                    snap.current_corridor or "-",
                    "YES" if snap.current_corridor == section.value else "no",
                    f"{prev_dot:.3f}",
                    f"{dot:.3f}",
                    f"({snap.pose_x:.2f}, {snap.pose_y:.2f})",
                ),
            )
        prev_dot = dot
    if not out:
        print("no negative-to-non-negative flips at all")
        return
    print_table(out, ["t", "label_at_flip", "in_start_section", "prev_dot", "dot", "pose"])


if __name__ == "__main__":
    main()

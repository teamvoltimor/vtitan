"""Ask whether the lap line is even reachable inside the section it is gated to.

LapDetector fires on a sign flip of ``(pose - origin) . normal`` that happens
while ``current_section`` equals the start section. The origin therefore has to
sit somewhere the robot is still LABELLED as being in the start corridor when it
gets there. An origin measured far down the corridor can land past the point
where the section label has already flipped to the next corridor, which makes
the gate unsatisfiable no matter how many laps are driven.

This prints, per section, the along-normal extent the robot actually occupied
while carrying that section's label -- so an origin can be checked against it.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_lap_line_reach.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Direction, Section

from scripts.common.bag_io import load_nav_debug_rows, print_table
from src.navigation.race_tracker import TRAVEL_DIRS
from src.navigation.start_conditions import CANONICAL_SECTION, assumed_start_conditions

_SECTIONS = {s.value: s for s in Section}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
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

    assumed_xy = assumed_start_conditions(direction)["position"]
    assumed = (assumed_xy["x"], assumed_xy["y"])
    measured = next(
        (
            (snap.start_measured_x, snap.start_measured_y)
            for _, snap in rows
            if isinstance(getattr(snap, "start_measured_x", None), (int, float))
        ),
        None,
    )

    print(f"== {args.bag_dir.name}  direction={direction.value}  start_section={section.value}")
    print(f"lap-line normal = ({nx}, {ny})")

    # Projection along the normal, in the same units the dot test uses.
    out = []
    for name in _SECTIONS:
        proj = [snap.pose_x * nx + snap.pose_y * ny for _, snap in posed if snap.current_corridor == name]
        if not proj:
            out.append((name, 0, "-", "-"))
            continue
        out.append((name, len(proj), f"{min(proj):.3f}", f"{max(proj):.3f}"))
    print_table(out, ["section_label", "samples", "min_proj", "max_proj"])

    print()
    for label, origin in (("ASSUMED", assumed), ("MEASURED", measured)):
        if origin is None:
            continue
        print(f"{label:>8} origin ({origin[0]:.3f}, {origin[1]:.3f}) -> lap line at proj = {origin[0] * nx + origin[1] * ny:.3f}")


if __name__ == "__main__":
    main()

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
        data/live/runs/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Direction, Section

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows, measured_start, posed_rows, settled_direction
from scripts.common.tables import print_table
from src.navigation.race_tracker import TRAVEL_DIRS
from src.navigation.start_conditions import CANONICAL_SECTION, assumed_start_conditions


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)
    posed = posed_rows(rows)
    direction = settled_direction(rows)
    if direction is None or not posed:
        print("nothing to measure")
        return
    section = CANONICAL_SECTION
    nx, ny = TRAVEL_DIRS[(section, direction)]

    assumed_xy = assumed_start_conditions(direction)["position"]
    assumed = (assumed_xy["x"], assumed_xy["y"])
    measured = measured_start(rows)

    print(f"== {args.bag_dir.name}  direction={direction.value}  start_section={section.value}")
    print(f"lap-line normal = ({nx}, {ny})")

    # Projection along the normal, in the same units the dot test uses.
    out = []
    for section in Section:
        proj = [snap.pose_x * nx + snap.pose_y * ny for _, snap in posed if snap.current_corridor is section]
        if not proj:
            out.append((section.value, 0, "-", "-"))
            continue
        out.append((section.value, len(proj), f"{min(proj):.3f}", f"{max(proj):.3f}"))
    print_table(out, ["section_label", "samples", "min_proj", "max_proj"])

    print()
    for label, origin in (("ASSUMED", assumed), ("MEASURED", measured)):
        if origin is None:
            continue
        print(f"{label:>8} origin ({origin[0]:.3f}, {origin[1]:.3f}) -> lap line at proj = {origin[0] * nx + origin[1] * ny:.3f}")


if __name__ == "__main__":
    main()

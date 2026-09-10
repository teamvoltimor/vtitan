"""Look for the fingerprint of _commit_direction's ``if changed`` branch.

That branch, and only that branch, calls
``correct_heading_for_direction_change`` -- a step of exactly pi in the heading
estimate -- and rebuilds the LapDetector at the measured start. The ``elif``
branch leaves both alone. So a yaw discontinuity near pi at the moment the state
leaves blind creep says the launch direction was overturned, which is what
decides which origin the lap detector ended up anchored at.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_commit_branch.py \
        data/live/runs/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows
from scripts.common.tables import print_table
from src.navigation.utils import wrap_angle


def main() -> None:
    parser = create_bag_parser("Look for _commit_direction branch fingerprint in yaw discontinuities")
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)
    if not rows:
        print("empty bag")
        return

    states = []
    prev_state = object()
    for t, snap in rows:
        state = snap.phase
        if state != prev_state:
            states.append((f"{t:.2f}", str(state), str(snap.direction), str(snap.waypoint_index)))
            prev_state = state
    print(f"== {args.bag_dir.name}   first sample t={rows[0][0]:.2f}s")
    print("state transitions:")
    print_table(states[:12], ["t", "state", "direction", "waypoint_index"])

    jumps = []
    prev = None
    for t, snap in rows:
        yaw = snap.pose_yaw
        if not isinstance(yaw, (int, float)):
            continue
        if prev is not None:
            d = abs(wrap_angle(yaw - prev[1]))
            if d > 1.0:
                jumps.append((f"{prev[0]:.2f}", f"{t:.2f}", f"{prev[1]:.3f}", f"{yaw:.3f}", f"{d:.3f}"))
        prev = (t, yaw)
    print(f"\nyaw discontinuities > 1.0 rad between consecutive samples: {len(jumps)}")
    print_table(jumps[:10], ["t_before", "t_after", "yaw_before", "yaw_after", "delta"])


if __name__ == "__main__":
    main()

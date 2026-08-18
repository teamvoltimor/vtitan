"""Test whether a run's heading frame is rotated off the track axes.

The direction gate needs the heading within _ALIGNMENT_TOLERANCE_RAD of a
multiple of 90 deg. A run that never satisfies it has two very different
possible causes:

* the robot is genuinely wandering -- axis error is spread across the whole
  0-45 deg range, and the fix is in the controller; or
* the heading REFERENCE is rotated -- axis error clusters tightly around a
  non-zero constant, because every corridor-aligned heading reads as that
  constant. Then nothing the controller does can help, and the fix is in how
  the reference is established (or in how the robot is placed).

A tight cluster away from zero is the signature of the second. This prints the
distribution so the two can be told apart rather than guessed at.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_yaw_frame_offset.py \
        vtitan_runs_pulled/run_A vtitan_runs_pulled/run_B ...
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows
from scripts.common.tables import print_table
from src.navigation.utils import axis_error_rad

_BUCKET_DEG = 5.0
"""Histogram resolution. Axis error spans 0-45 deg, so this gives 9 buckets."""


def _axis_error_deg(yaw: float) -> float:
    """The gate's own measure, in degrees for reading."""
    return math.degrees(axis_error_rad(yaw))


def main() -> None:
    parser = create_bags_parser("Analyze multiple bags")
    args = parser.parse_args()

    rows_out = []
    for bag_dir in args.bag_dirs:
        rows, _topics = load_nav_debug_rows(bag_dir)
        errors = [_axis_error_deg(s.pose_yaw) for _, s in rows if isinstance(s.pose_yaw, (int, float))]
        if not errors:
            continue
        # Spread, not centre, is what separates the two causes: a rotated frame
        # is tight around its offset, wandering is broad.
        rows_out.append(
            (
                bag_dir.name.replace("run_2026", ""),
                len(errors),
                f"{statistics.median(errors):.1f}",
                f"{statistics.mean(errors):.1f}",
                f"{statistics.pstdev(errors):.1f}",
                f"{100.0 * sum(1 for e in errors if e < 8.0) / len(errors):.0f}%",
                f"{100.0 * sum(1 for e in errors if e > 20.0) / len(errors):.0f}%",
            ),
        )
    print("axis error = distance from the nearest 90 deg multiple, the gate's own measure")
    print_table(rows_out, ["run", "samples", "median", "mean", "stdev", "<8deg (gate ok)", ">20deg"])

    for bag_dir in args.bag_dirs:
        rows, _topics = load_nav_debug_rows(bag_dir)
        errors = [_axis_error_deg(s.pose_yaw) for _, s in rows if isinstance(s.pose_yaw, (int, float))]
        if not errors:
            continue
        print(f"\n{bag_dir.name} histogram:")
        for low in range(0, 45, int(_BUCKET_DEG)):
            n = sum(1 for e in errors if low <= e < low + _BUCKET_DEG)
            bar = "#" * round(60.0 * n / len(errors))
            print(f"  {low:2d}-{low + int(_BUCKET_DEG):2d} deg {100.0 * n / len(errors):5.1f}% {bar}")


if __name__ == "__main__":
    main()

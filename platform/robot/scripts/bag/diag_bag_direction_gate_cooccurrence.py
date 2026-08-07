"""Why the direction gate starves: which conditions pass, and whether ever together.

``diag_bag_direction_gate.py`` reports the FIRST gate that refuses each scan,
which answers "what stopped this reading". It cannot answer the question a run
that never settles actually raises: each condition passes often enough on its
own, so is the problem one hostile gate, or four reasonable gates that never
line up on the same scan?

``infer_direction`` needs alignment, no dropout, an open span and an asymmetry
all true at once. This evaluates each independently over the recorded scans,
then reports the pass rate of each and of the conjunction -- so a conjunction
far below the product of the marginals says the conditions are anti-correlated,
which is a different defect from any single threshold being wrong.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_direction_gate_cooccurrence.py \
        vtitan_runs_pulled/run_A vtitan_runs_pulled/run_B ...
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import open_reader, print_table, read_bag

from src.navigation.direction_estimator import (
    _MAX_IN_TRACK_RANGE_M,
    _MAX_PLAUSIBLE_SPAN_M,
    _MIN_ASYMMETRY_M,
)
from src.navigation.utils import _ALIGNMENT_TOLERANCE_RAD, _nearest_ray, axis_error_rad
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD


def _gates(scan, yaw: float) -> dict[str, bool]:
    """Each of infer_direction's conditions, evaluated independently."""
    left = _nearest_ray(scan.ranges_m, scan.angles_rad, math.pi / 2)
    right = _nearest_ray(scan.ranges_m, scan.angles_rad, -math.pi / 2)
    return {
        "aligned": axis_error_rad(yaw) <= _ALIGNMENT_TOLERANCE_RAD,
        "no_dropout": left <= _MAX_IN_TRACK_RANGE_M and right <= _MAX_IN_TRACK_RANGE_M,
        "span_open": left + right > _MAX_PLAUSIBLE_SPAN_M,
        "asymmetric": abs(left - right) >= _MIN_ASYMMETRY_M,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dirs", type=Path, nargs="+")
    args = parser.parse_args()

    names = ("aligned", "no_dropout", "span_open", "asymmetric")
    out = []
    for bag_dir in args.bag_dirs:
        scans, nav = read_bag(open_reader(bag_dir), _LIDAR_YAW_OFFSET_RAD)
        poses = {round(t, 1): s for t, s in nav if isinstance(s.pose_yaw, (int, float))}
        evaluated = [_gates(sc, poses[round(t, 1)].pose_yaw) for t, sc in scans if round(t, 1) in poses]
        if not evaluated:
            continue
        n = len(evaluated)
        marginals = {k: sum(1 for g in evaluated if g[k]) / n for k in names}
        both = sum(1 for g in evaluated if all(g.values())) / n
        # If the conditions were independent, this is what the conjunction would
        # be. Far below it means they actively exclude each other.
        expected = 1.0
        for v in marginals.values():
            expected *= v
        # The pair the geometry ties together: you are square to the corridor
        # mid-corridor, and the span only opens at the very end of it.
        align_and_span = sum(1 for g in evaluated if g["aligned"] and g["span_open"]) / n
        out.append(
            (
                bag_dir.name.replace("run_2026", ""),
                n,
                *(f"{100 * marginals[k]:.0f}%" for k in names),
                f"{100 * align_and_span:.1f}%",
                f"{100 * both:.1f}%",
                f"{100 * expected:.1f}%",
            ),
        )
    print_table(
        out,
        ["run", "scans", *names, "aligned&span", "ALL", "if independent"],
    )


if __name__ == "__main__":
    main()

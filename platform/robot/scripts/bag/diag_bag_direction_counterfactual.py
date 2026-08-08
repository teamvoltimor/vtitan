"""Ask what the direction estimator would have decided under relaxed gates.

``diag_bag_direction_gate.py`` says which gate refused each scan. This says
whether relaxing a gate would have produced the RIGHT answer or merely a fast
wrong one -- the only question that matters, since a confidently wrong direction
is worse than never settling.

It replays the side ranges the node already logged (``direction_left_range_m`` /
``direction_right_range_m``) plus ``pose_yaw`` through the same gate arithmetic
as :func:`src.navigation.direction_estimator.infer_direction`, under the shipped
thresholds and under variants, and reports for each variant when it would have
settled and on which direction. The true direction is derived independently from
the pose trace's winding about the mat centre.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_direction_counterfactual.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import NavigatorDebugSnapshot

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows
from scripts.common.tables import print_table
from src.navigation.utils import _wrap

_MIN_VOTES = 5
_MAX_RANGE_FILL_M = RobotSpecs.LIDAR_MAX_RANGE - 0.1
_MAT_CENTRE_X = 1.5
_MAT_CENTRE_Y = 1.5
_LARGE_MAX_RANGE_M = 12.5
_ALIGNMENT_TOL_MULTIPLIER = 2


def settle(
    rows: list[tuple[float, NavigatorDebugSnapshot]],
    *,
    max_range: float,
    max_span: float,
    min_asym: float,
    align_tol: float,
) -> tuple[float, str, int] | None:
    """Replay the gate with these thresholds; return (time, direction, votes-cast)."""
    votes: Counter[str] = Counter()
    cast = 0
    for t, snap in rows:
        left = snap.direction_left_range_m
        right = snap.direction_right_range_m
        yaw = snap.pose_yaw
        if left is None or right is None or yaw is None:
            continue
        if left > max_range or right > max_range:
            continue
        axis_error = abs(_wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
        if axis_error > align_tol:
            continue
        if left + right <= max_span:
            continue
        if abs(left - right) < min_asym:
            continue
        inferred = "clockwise" if right > left else "counterclockwise"
        votes[inferred] += 1
        cast += 1
        if votes[inferred] >= _MIN_VOTES:
            return t, inferred, cast
    return None


def true_direction(rows: list[tuple[float, NavigatorDebugSnapshot]]) -> tuple[str, float]:
    """Winding sense of the pose trace about the mat centre (1.5, 1.5)."""
    total = 0.0
    prev = None
    for _, snap in rows:
        x, y = snap.pose_x, snap.pose_y
        if x is None or y is None:
            continue
        ang = math.atan2(y - _MAT_CENTRE_Y, x - _MAT_CENTRE_X)
        if prev is not None:
            total += _wrap(ang - prev)
        prev = ang
    laps = total / (2 * math.pi)
    return ("counterclockwise" if total > 0 else "clockwise"), laps


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    truth, laps = true_direction(rows)
    print(f"== {args.bag_dir.name}  samples={len(rows)}")
    print(f"pose winding: {laps:+.2f} turns about mat centre -> travelling {truth}")

    print("\nside-range population (both sides pooled):")
    pool = [
        v
        for _, snap in rows
        for v in (snap.direction_left_range_m, snap.direction_right_range_m)
        if v is not None
    ]
    buckets = [
        (0, 0.5),
        (0.5, 1.0),
        (1.0, 1.25),
        (1.25, 2.0),
        (2.0, 4.5),
        (4.5, _MAX_RANGE_FILL_M),
        (_MAX_RANGE_FILL_M, 99),
    ]
    table_rows = []
    for lo, hi in buckets:
        n = sum(1 for v in pool if lo <= v < hi)
        pct = 100.0 * n / len(pool) if pool else 0.0
        table_rows.append((f"[{lo:.2f}, {hi:.2f})", n, pct))
    if table_rows:
        print_table(table_rows, ["range", "count", "percent"])

    print("\nticks where exactly one side reads max-range (the rejected signal):")
    one_side_max = [
        snap
        for _, snap in rows
        if snap.direction_left_range_m is not None
        and snap.direction_right_range_m is not None
        and (snap.direction_left_range_m > _MAX_RANGE_FILL_M) != (snap.direction_right_range_m > _MAX_RANGE_FILL_M)
    ]
    print(f"  {len(one_side_max)} ticks")
    implied = Counter(
        "counterclockwise" if snap.direction_left_range_m > _MAX_RANGE_FILL_M else "clockwise"
        for snap in one_side_max
    )
    for name, count in implied.most_common():
        mark = "  <- matches pose" if name == truth else "  <- WRONG"
        print(f"    would imply {name}: {count}{mark}")

    print("\ncounterfactual settles:")
    tuning = NavigationTuning.load_default()
    estimator = tuning.direction_estimator
    shipped = dict(
        max_range=estimator.MAX_IN_TRACK_RANGE_M,
        max_span=estimator.PLAUSIBLE_SPAN_THRESHOLD_M,
        min_asym=estimator.MIN_ASYMMETRY_M,
        align_tol=estimator.ALIGNMENT_TOLERANCE_RAD,
    )
    wide_tol = _ALIGNMENT_TOL_MULTIPLIER * estimator.ALIGNMENT_TOLERANCE_RAD
    variants = [
        ("shipped", shipped),
        ("max_range=12.5 (accept max-range as open)",
         {**shipped, "max_range": _LARGE_MAX_RANGE_M}),
        ("align_tol x2", {**shipped, "align_tol": wide_tol}),
        ("max_range=12.5 AND align_tol x2",
         {**shipped, "max_range": _LARGE_MAX_RANGE_M, "align_tol": wide_tol}),
    ]
    table_rows = []
    for name, kwargs in variants:
        result = settle(rows, **kwargs)
        if result is None:
            table_rows.append((name, "never settles", "", "", ""))
        else:
            t, direction, cast = result
            mark = "OK" if direction == truth else "WRONG"
            table_rows.append((name, direction, t, cast, mark))
    if table_rows:
        print_table(table_rows, ["variant", "result", "time_s", "votes", "status"])


if __name__ == "__main__":
    main()

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
    pixi run -e dev python scripts/diag_bag_direction_counterfactual.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from src.navigation.direction_estimator import (
    _MAX_IN_TRACK_RANGE_M,
    _MAX_PLAUSIBLE_SPAN_M,
    _MIN_ASYMMETRY_M,
)
from src.navigation.utils import _ALIGNMENT_TOLERANCE_RAD, _wrap

MIN_VOTES = 5


def settle(
    rows: list[dict],
    *,
    max_range: float,
    max_span: float,
    min_asym: float,
    align_tol: float,
) -> tuple[float, str, int] | None:
    """Replay the gate with these thresholds; return (time, direction, votes-cast)."""
    votes: Counter[str] = Counter()
    cast = 0
    for row in rows:
        left = row.get("direction_left_range_m")
        right = row.get("direction_right_range_m")
        yaw = row.get("pose_yaw")
        if not isinstance(left, (int, float)) or not isinstance(right, (int, float)):
            continue
        if not isinstance(yaw, (int, float)):
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
        if votes[inferred] >= MIN_VOTES:
            return row["_t"], inferred, cast
    return None


def true_direction(rows: list[dict]) -> tuple[str, float]:
    """Winding sense of the pose trace about the mat centre (1.5, 1.5)."""
    total = 0.0
    prev = None
    for row in rows:
        x, y = row.get("pose_x"), row.get("pose_y")
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            continue
        ang = math.atan2(y - 1.5, x - 1.5)
        if prev is not None:
            total += _wrap(ang - prev)
        prev = ang
    laps = total / (2 * math.pi)
    return ("counterclockwise" if total > 0 else "clockwise"), laps


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

    truth, laps = true_direction(rows)
    print(f"== {args.bag_dir.name}  samples={len(rows)}")
    print(f"pose winding: {laps:+.2f} turns about mat centre -> travelling {truth}")

    print("\nside-range population (both sides pooled):")
    pool = [
        v
        for r in rows
        for v in (r.get("direction_left_range_m"), r.get("direction_right_range_m"))
        if isinstance(v, (int, float))
    ]
    buckets = [(0, 0.5), (0.5, 1.0), (1.0, 1.25), (1.25, 2.0), (2.0, 4.5), (4.5, 11.9), (11.9, 99)]
    for lo, hi in buckets:
        n = sum(1 for v in pool if lo <= v < hi)
        pct = 100.0 * n / len(pool) if pool else 0.0
        print(f"  [{lo:5.2f}, {hi:5.2f})  {n:6d}  {pct:5.1f}%")

    print("\nticks where exactly one side reads max-range (the rejected signal):")
    one_side_max = [
        r
        for r in rows
        if isinstance(r.get("direction_left_range_m"), (int, float))
        and isinstance(r.get("direction_right_range_m"), (int, float))
        and (r["direction_left_range_m"] > 11.9) != (r["direction_right_range_m"] > 11.9)
    ]
    print(f"  {len(one_side_max)} ticks")
    implied = Counter(
        "counterclockwise" if r["direction_left_range_m"] > 11.9 else "clockwise"
        for r in one_side_max
    )
    for name, count in implied.most_common():
        mark = "  <- matches pose" if name == truth else "  <- WRONG"
        print(f"    would imply {name}: {count}{mark}")

    print("\ncounterfactual settles:")
    variants = [
        ("shipped", dict(max_range=_MAX_IN_TRACK_RANGE_M, max_span=_MAX_PLAUSIBLE_SPAN_M,
                         min_asym=_MIN_ASYMMETRY_M, align_tol=_ALIGNMENT_TOLERANCE_RAD)),
        ("max_range=12.5 (accept max-range as open)",
         dict(max_range=12.5, max_span=_MAX_PLAUSIBLE_SPAN_M,
              min_asym=_MIN_ASYMMETRY_M, align_tol=_ALIGNMENT_TOLERANCE_RAD)),
        ("align_tol x2",
         dict(max_range=_MAX_IN_TRACK_RANGE_M, max_span=_MAX_PLAUSIBLE_SPAN_M,
              min_asym=_MIN_ASYMMETRY_M, align_tol=2 * _ALIGNMENT_TOLERANCE_RAD)),
        ("max_range=12.5 AND align_tol x2",
         dict(max_range=12.5, max_span=_MAX_PLAUSIBLE_SPAN_M,
              min_asym=_MIN_ASYMMETRY_M, align_tol=2 * _ALIGNMENT_TOLERANCE_RAD)),
    ]
    for name, kwargs in variants:
        result = settle(rows, **kwargs)
        if result is None:
            print(f"  {name:44s} never settles")
        else:
            t, direction, cast = result
            mark = "OK" if direction == truth else "WRONG"
            print(f"  {name:44s} settles {direction} at {t:6.1f}s  ({cast} votes cast)  [{mark}]")


if __name__ == "__main__":
    main()

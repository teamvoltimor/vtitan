"""Score the in-bay placement read against the rounds that actually started in it.

``direction_from_parking_bay`` decides, before the chassis moves, which way round
the lap goes -- and getting it wrong reflects the pass-side rule for every sign
in the round. Until now it read a SINGLE ray per side on a SINGLE scan, and both
of those turned out to be load-bearing in a way no test could show: the failures
are properties of the recorded LIDAR, not of the arithmetic.

This replays the first scans of each bag through the real function under three
variants and scores them against an independently derived truth:

* ``single/1``  -- one ray per side, one look. What shipped.
* ``sector/1``  -- sector median per side, one look.
* ``sector/N``  -- sector median, retried over the first ``bay_start_max_checks``
  scans. What ships now.

An ABSTENTION IS NOT AN ERROR and is counted separately. A wrong answer is far
worse than no answer -- the caller has ``assume_bay_start`` to fall back on, but
nothing to recover a direction committed backwards -- so a variant that trades
abstentions for errors is a regression however its headline count reads.

Truth is taken from the winding of the pose trace about the mat centre, and
falls back to the direction the round committed when the round died before
completing a quarter turn. Both are printed, so a disagreement is visible rather
than averaged away.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_bay_start_direction.py \
        ../../other/data/live/runs/run_2026*
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.domain.enums import Direction

from scripts.common.bag_io import (
    Topics,
    create_bags_parser,
    decode_scan,
    load_nav_debug_rows,
    open_reader,
    settled_direction,
)
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning, tuning_with_overrides
from src.navigation.direction_estimator import direction_from_parking_bay
from src.navigation.utils import wrap_angle
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

_MAT_CENTRE = (1.5, 1.5)
_MIN_TURN_FOR_WINDING = 0.25
"""Turns of pose winding below which the trace is too short to name a sense."""


def _truth(bag_dir: Path) -> tuple[Direction | None, str, float]:
    """The direction the round really travelled, and where that came from."""
    rows, _ = load_nav_debug_rows(bag_dir)
    total = 0.0
    prev = None
    for _, snap in rows:
        x, y = snap.pose_x, snap.pose_y
        if x is None or y is None:
            continue
        ang = math.atan2(y - _MAT_CENTRE[1], x - _MAT_CENTRE[0])
        if prev is not None:
            total += wrap_angle(ang - prev)
        prev = ang
    laps = total / (2 * math.pi)
    if abs(laps) >= _MIN_TURN_FOR_WINDING:
        return (Direction.COUNTERCLOCKWISE if laps > 0 else Direction.CLOCKWISE), "winding", laps
    return settled_direction(rows), "committed", laps


def _first_scans(bag_dir: Path, n: int):
    """The first ``n`` scans of the bag, decoded exactly as the gateway does."""
    reader = open_reader(bag_dir)
    out = []
    while reader.has_next() and len(out) < n:
        topic, data, _t = reader.read_next()
        if topic != Topics.SCAN:
            continue
        out.append(decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD))
    return out


def _verdict(scans, tuning, max_checks: int) -> Direction | None:
    """What the rule answers, retried over at most ``max_checks`` scans."""
    for scan in scans[:max_checks]:
        answer = direction_from_parking_bay(scan.ranges_m, scan.angles_rad, tuning)
        if answer is not None:
            return answer
    return None


def main() -> int:
    parser = create_bags_parser(__doc__.split("\n\n")[0])
    parser.add_argument(
        "--max-checks",
        type=int,
        default=None,
        help="Retry budget for the sector/N variant (default: the shipped bay_start_max_checks)",
    )
    args = parser.parse_args()

    shipped = get_tuning(None)
    max_checks = args.max_checks or shipped.corridor_follower.bay_start_max_checks
    single = tuning_with_overrides({"bay_start_sector_deg": 0.0})
    variants = (("single/1", single, 1), ("sector/1", shipped, 1), (f"sector/{max_checks}", shipped, max_checks))

    tally = {name: [0, 0, 0] for name, _, _ in variants}  # right, wrong, abstained
    rows = []
    for bag_dir in args.bag_dirs:
        if not (bag_dir / "metadata.yaml").exists():
            continue
        try:
            scans = _first_scans(bag_dir, max_checks)
        except RuntimeError:
            # An aborted recording leaves a metadata file the reader cannot
            # parse. Skipping is right: there is no placement to score.
            continue
        if not scans:
            continue
        try:
            truth, source, laps = _truth(bag_dir)
        except Exception:  # noqa: BLE001 - a truncated snapshot is not a finding
            truth, source, laps = None, "unreadable", 0.0
        answers = []
        for name, tuning, checks in variants:
            answer = _verdict(scans, tuning, checks)
            answers.append("-" if answer is None else answer.value[:4])
            slot = 2 if answer is None else (0 if truth is not None and answer is truth else 1)
            # A bag with no truth cannot score an answer either way; it is
            # counted as abstained so it never inflates the right column.
            tally[name][2 if truth is None else slot] += 1
        rows.append(
            [
                bag_dir.name.replace("run_2026", ""),
                "-" if truth is None else truth.value[:4],
                f"{source}({laps:+.2f})",
                *answers,
            ]
        )

    print_table(rows, ["run", "truth", "source", *(name for name, _, _ in variants)])
    print()
    print_table(
        [[name, *tally[name]] for name, _, _ in variants],
        ["variant", "right", "WRONG", "abstained"],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

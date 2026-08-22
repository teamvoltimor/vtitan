"""Replay measure_start_pose against every scan in a bag, second by second.

``measure_start_pose`` runs exactly once per round, at the moment the travel
direction is committed, and a refusal there costs the whole round: the node
logs "falling back to the assumed start" and races from (1.50, 0.40) while the
real placement is 0.5-0.9 m away. Two of the four rounds on 2026-08-08 failed
that way.

The question this answers is whether the refusal is *transient* -- an operator's
hand standing in one of the four cardinal rays for a second -- or permanent. If
a valid measurement appears a second later, retrying is worth doing; if the scan
never supports one, the fix has to be somewhere else. So this replays the same
function over the whole scan stream and reports, per scan, which of its two
refusal criteria fired (a wedge with no valid return, or opposite rays that do
not span the mat) and what pose it would have produced.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_start_measurement.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX [--section south] [--window 15]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.domain.enums import Direction, Section

from scripts.common.bag_io import (
    Topics,
    create_bag_parser,
    decode_nav_debug,
    decode_scan,
    elapsed_seconds,
    open_reader,
    settled_direction,
)
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning
from src.navigation.race_tracker import TRAVEL_DIRS
from src.navigation.start_measurement import _wedge_median, measure_start_pose
from src.navigation.utils import _wrap
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

if TYPE_CHECKING:
    from shared.domain.models import NavigatorDebugSnapshot

    from src.navigation.ports import LidarScan


def _read(bag_dir: Path) -> tuple[list[tuple[float, LidarScan]], list[tuple[float, NavigatorDebugSnapshot]]]:
    """Replay a bag into its (time, scan) and (time, nav_debug) streams."""
    reader = open_reader(bag_dir)
    scans: list[tuple[float, LidarScan]] = []
    rows: list[tuple[float, NavigatorDebugSnapshot]] = []
    t0: int | None = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if t0 is None:
            t0 = stamp
        rel = elapsed_seconds(stamp, t0)
        if topic == Topics.SCAN:
            scans.append((rel, decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD)))
        elif topic == Topics.NAV_DEBUG:
            rows.append((rel, decode_nav_debug(data)))
    return scans, rows


def _print_phases(rows: list[tuple[float, NavigatorDebugSnapshot]]) -> None:
    """Print when the round changed phase -- the commit is the transition out of blind creep."""
    phases = []
    previous = object()
    for t, snap in rows:
        if snap.phase != previous:
            phases.append((f"{t:.2f}", str(snap.phase), str(snap.direction)))
            previous = snap.phase
    print("\nphase timeline:")
    print_table(phases[:10], ["t", "phase", "direction"])


def _verdict(
    scan: LidarScan,
    half_width_rad: float,
    closing_tolerance_m: float,
) -> tuple[str, dict[str, float | None]]:
    """Which refusal criterion fires for this scan, and the four wedge medians."""
    ranges = np.asarray(scan.ranges_m, dtype=float)
    angles = np.asarray(scan.angles_rad, dtype=float)
    rays = {
        "fwd": _wedge_median(ranges, angles, 0.0, half_width_rad),
        "back": _wedge_median(ranges, angles, math.pi, half_width_rad),
        "left": _wedge_median(ranges, angles, math.pi / 2, half_width_rad),
        "right": _wedge_median(ranges, angles, -math.pi / 2, half_width_rad),
    }
    blocked = [name for name, value in rays.items() if value is None]
    if blocked:
        return f"blocked:{'+'.join(blocked)}", rays
    closing = (rays["fwd"] + rays["back"]) - TrackDimensions.MAX_COORD
    if abs(closing) > closing_tolerance_m:
        return f"closing:{closing:+.2f}", rays
    return "ok", rays


def main() -> None:
    """Print the per-scan start-measurement replay for the bag named on the command line."""
    parser = create_bag_parser("Replay measure_start_pose over a bag's whole scan stream")
    parser.add_argument("--section", default=Section.SOUTH.value, help="Start section (default south)")
    parser.add_argument("--direction", default=None, help="Override the direction read from /nav_debug")
    parser.add_argument("--window", type=float, default=15.0, help="Seconds of scans to tabulate")
    args = parser.parse_args()

    tuning = get_tuning(None)
    half_width_rad = math.radians(tuning.start_measurement.RAY_HALF_WIDTH_DEG)
    closing_tolerance_m = tuning.start_measurement.CLOSING_TOLERANCE_M

    scans, rows = _read(args.bag_dir)
    if not scans:
        print("bag carries no /scan")
        return

    direction = Direction(args.direction) if args.direction else settled_direction(rows)
    if direction is None:
        print("bag never settled a direction and none was given with --direction; assuming counterclockwise")
        direction = Direction.COUNTERCLOCKWISE
    section = Section(args.section)

    print(f"== {args.bag_dir.name}  direction={direction}  section={section}  scans={len(scans)}")
    print(f"   ray half-width {tuning.start_measurement.RAY_HALF_WIDTH_DEG}deg, "
          f"closing tolerance {closing_tolerance_m} m, LIDAR min {RobotSpecs.LIDAR_MIN_RANGE} m")

    # What the round actually did: the phase timeline says when the commit that
    # took the one real measurement happened.
    _print_phases(rows)

    posed = [(t, s) for t, s in rows if isinstance(s.pose_x, (int, float))]
    travel = TRAVEL_DIRS[(section, direction)]
    travel_yaw = math.atan2(travel[1], travel[0])

    def believed(t: float) -> tuple[str, str]:
        """What the node thought its pose and its alignment were, at the scan nearest ``t``.

        Alignment is against the starting corridor's travel bearing, which is
        the gate ``_retry_start_measurement`` applies before believing a
        retried reading -- a chassis turned through 180 degrees passes the
        closing check and comes out mirrored about the mat's centre.
        """
        if not posed:
            return "-", "-"
        nearest = min(posed, key=lambda r: abs(r[0] - t))[1]
        pose = f"({nearest.pose_x:.2f}, {nearest.pose_y:.2f})"
        if not isinstance(nearest.pose_yaw, (int, float)):
            return pose, "-"
        return pose, f"{math.degrees(abs(_wrap(nearest.pose_yaw - travel_yaw))):.0f}"

    table = []
    first_ok: float | None = None
    verdict_counts: dict[str, int] = {}
    for t, scan in scans:
        verdict, rays = _verdict(scan, half_width_rad, closing_tolerance_m)
        key = verdict.split(":")[0]
        verdict_counts[key] = verdict_counts.get(key, 0) + 1
        if verdict == "ok" and first_ok is None:
            first_ok = t
        if t <= args.window:
            measured = measure_start_pose(scan.ranges_m, scan.angles_rad, direction, section, tuning=tuning)
            table.append((
                f"{t:.2f}",
                *(f"{rays[k]:.2f}" if rays[k] is not None else "--" for k in ("fwd", "back", "left", "right")),
                verdict,
                f"({measured.x:.2f}, {measured.y:.2f})" if measured else "-",
                *believed(t),
            ))

    print(f"\nfirst {args.window:.0f} s of scans:")
    print_table(table, ["t", "fwd", "back", "left", "right", "verdict", "measured", "believed", "off_deg"])
    print(f"\nverdicts over the whole bag: {verdict_counts}")
    print(f"first scan a measurement would have succeeded on: "
          f"{f'{first_ok:.2f} s' if first_ok is not None else 'never'}")


if __name__ == "__main__":
    main()

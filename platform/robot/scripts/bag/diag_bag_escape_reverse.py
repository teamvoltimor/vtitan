r"""Did the longer escape reverse actually fit on the mat?

``MAX_ESCAPE_S`` went 1.0 -> 1.8 on 2026-09-07 (``72e7172b``), which takes the
simulated escape timeouts 9 -> 0 but roughly doubles the reverse: at
``REV_SPEED`` 0.2 m/s, 0.20 m becomes 0.36 m. The corpus CANNOT price that risk
-- its contact model never slides along a wall, so it cannot show what a longer
reverse does against one -- and reverse has never been live-verified on this
chassis at all. Predicted from the pre-change bags: room for 0.20 m on 94% of
escape triggers, for 0.36 m on only 73%.

This checks the prediction against runs recorded on each side of the change.

Per escape episode (a contiguous run of ticks with a reversing maneuver):

* **duration** -- how long the episode actually ran, against the cap.
* **rotation** -- yaw turned over the episode, the thing the escape exists to
  produce. The pre-change complaint was 27.4 deg median against the 90 deg+ a
  corner needs.
* **rear room at trigger** -- rear-arc clearance the tick the episode began.

TRAP, and the reason this script measures the arc rather than
``rear_clearance_m``: the chassis rear face sits 0.272 m BEHIND the LIDAR, so
returns closer than that are the robot's own body. Taking a plain minimum over
the rear arc reports ~0.02 m -- physically inside the chassis -- and reads as
"no room anywhere" on every single trigger. Self-returns must be excluded
before the number means anything.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_escape_reverse.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs  # noqa: E402

from scripts.common.bag_io import create_bags_parser, decode_scan, elapsed_seconds, open_reader  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402

REAR_OVERHANG_M = 0.272
"""Distance from the LIDAR back to the chassis rear face. Returns nearer than
this are the robot's own structure, not an obstacle -- see the module docstring."""

REAR_ARC_RAD = math.radians(40.0)
"""Half-width of the rear arc searched, centred on 180 deg."""

REV_SPEED_MPS = 0.2
"""Escape reverse speed, for turning a duration into a distance."""

CONTACT_M = 0.05
"""Rear room below this at the trigger means the reverse had nowhere to go."""


@dataclass
class Episode:
    """One contiguous reversing-maneuver episode."""

    run: str
    duration_s: float
    rotation_deg: float
    rear_room_m: float | None

    @property
    def reverse_m(self) -> float:
        """Path the episode backed through, at the fixed reverse speed."""
        return self.duration_s * REV_SPEED_MPS


def _rear_room(ranges: list[float], angles: list[float]) -> float | None:
    """Nearest REAL return in the rear arc, or None when the arc is empty.

    Excludes anything inside ``REAR_OVERHANG_M``: that is the chassis, and
    including it reports ~0.02 m on every trigger.
    """
    best: float | None = None
    for rng, ang in zip(ranges, angles):
        if rng is None or not math.isfinite(rng) or rng <= 0.0:
            continue
        if abs(abs(ang) - math.pi) > REAR_ARC_RAD:
            continue
        if rng < REAR_OVERHANG_M:
            continue
        if best is None or rng < best:
            best = rng
    return None if best is None else best - REAR_OVERHANG_M


def _episodes(run: str, rows: list, scans: list) -> list[Episode]:
    """Split a run's ticks into reversing episodes and reduce each."""
    out: list[Episode] = []
    start: tuple[float, float] | None = None  # (time, yaw)
    last: tuple[float, float] | None = None
    room: float | None = None
    scan_i = 0

    for rel, d in rows:
        speed = d.maneuver_speed_mps
        reversing = speed is not None and speed < 0.0
        if reversing and start is None:
            start = (rel, d.pose_yaw if d.pose_yaw is not None else 0.0)
            while scan_i + 1 < len(scans) and scans[scan_i + 1][0] <= rel:
                scan_i += 1
            if scans:
                scan = scans[min(scan_i, len(scans) - 1)][1]
                room = _rear_room(list(scan.ranges_m), list(scan.angles_rad))
        if reversing:
            last = (rel, d.pose_yaw if d.pose_yaw is not None else 0.0)
        elif start is not None and last is not None:
            turned = math.degrees(abs(math.atan2(math.sin(last[1] - start[1]), math.cos(last[1] - start[1]))))
            out.append(Episode(run, last[0] - start[0], turned, room))
            start, last, room = None, None, None
    return out


def main() -> None:
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()

    episodes: list[Episode] = []
    for bag in args.bag_dirs:
        reader = open_reader(Path(bag))
        t0 = None
        rows, scans = [], []
        from scripts.common.bag_io import Topics, decode_nav_debug  # noqa: PLC0415

        while reader.has_next():
            topic, data, t = reader.read_next()
            if t0 is None:
                t0 = t
            rel = elapsed_seconds(t, t0)
            if topic == Topics.NAV_DEBUG:
                rows.append((rel, decode_nav_debug(data)))
            elif topic == Topics.SCAN:
                from sensor_msgs.msg import LaserScan  # noqa: PLC0415
                from rclpy.serialization import deserialize_message  # noqa: PLC0415

                scans.append((rel, decode_scan(deserialize_message(data, LaserScan), 0.0, RobotSpecs.LIDAR_MAX_RANGE)))
        episodes.extend(_episodes(Path(bag).name.replace("run_", ""), rows, scans))

    if not episodes:
        print("No reversing escape episodes in these bags.")
        return

    table = []
    for run in sorted({e.run for e in episodes}):
        group = [e for e in episodes if e.run == run]
        rooms = [e.rear_room_m for e in group if e.rear_room_m is not None]
        table.append(
            [
                run,
                len(group),
                round(statistics.median(e.duration_s for e in group), 2),
                round(max(e.duration_s for e in group), 2),
                round(statistics.median(e.rotation_deg for e in group), 1),
                round(statistics.median(rooms), 2) if rooms else None,
                sum(1 for r in rooms if r < CONTACT_M),
            ]
        )
    print(f"== ESCAPE REVERSES  ({len(episodes)} episodes)")
    print_table(
        table,
        ["run", "episodes", "med dur s", "max dur s", "med rot deg", "med rear m", "no room"],
    )

    rooms = [e.rear_room_m for e in episodes if e.rear_room_m is not None]
    fits = [e for e in episodes if e.rear_room_m is not None and e.rear_room_m >= e.reverse_m]
    print()
    print(f"  episodes with a usable rear reading: {len(rooms)}/{len(episodes)}")
    if rooms:
        print(f"  rear room at trigger m: p10 {sorted(rooms)[len(rooms) // 10]:.2f} / median {statistics.median(rooms):.2f}")
        print(f"  reverse actually FIT the measured room: {len(fits)}/{len(rooms)}")
        print(f"  triggers with under {CONTACT_M} m behind: {sum(1 for r in rooms if r < CONTACT_M)}")
    print(f"  rotation deg: median {statistics.median(e.rotation_deg for e in episodes):.1f}")
    print(f"  duration s:   median {statistics.median(e.duration_s for e in episodes):.2f}")


if __name__ == "__main__":
    main()

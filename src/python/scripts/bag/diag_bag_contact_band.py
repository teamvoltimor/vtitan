"""Pick ``obstacles_contact_dist`` from the bags, not from the filter's edge.

``obstacles_contact_dist = 0.04`` is inert: the sector filter keeps only
``r > min_valid_range_m = 0.044``, so the side term of ``already_touching``
can never fire (measured: 5200 of 5216 side sectors qualify unfiltered, 0
filtered). It has to move up into the observable band, and the band is narrow
-- above ~0.045 to be visible, below ~0.10 because 0.10 fires on geometry the
router chose on purpose: the planner routes PAST a pillar at roughly 0.175 m
from its surface, and 0.10 produced ~180 escapes per run on subset128.

Choosing the bottom edge of that band because it is the bottom edge is the same
mistake that produced the inert value. So this prices every candidate instead.

For each candidate threshold the side term is replayed over the recorded scans
and the firings are SPLIT by whether a sign was committed at that moment:

  * while COMMITTED   -- the robot is working a pillar the router aimed it
                         past. A firing here is probably a false alarm on
                         intended geometry, and it evicts the commitment
                         (the router stops being ticked during a manoeuvre).
  * while FREE        -- no sign in play, so a close side reading is a wall or
                         an unplanned obstacle. A firing here is the one the
                         threshold exists for.

A good threshold maximises FREE firings per COMMITTED firing. The control is
the whole distribution, printed first: if almost no ticks land in the band at
all, no value in it can matter and the side term should be retired rather than
retuned.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_contact_band.py BAG [BAG ...]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from scripts.common.bag_io import (  # noqa: E402
    Topics,
    decode_scan,
    elapsed_seconds,
    load_nav_debug_rows,
    open_reader,
)
from scripts.common.stats import nearest_by_time  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD  # noqa: E402

CANDIDATES = (0.045, 0.050, 0.055, 0.060, 0.070, 0.080, 0.090, 0.100)

BANDS = (0.044, 0.050, 0.060, 0.070, 0.080, 0.100, 0.150, 0.250, 1e9)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    tuning = get_tuning(None)
    min_valid = tuning.lidar_sectors.MIN_VALID_RANGE_M
    half_fov = math.radians(tuning.lidar_sectors.THREAT_HALF_FOV_DEG)
    print(f"min_valid_range_m = {min_valid}   threat half-FOV = {math.degrees(half_fov):.0f} deg")
    print(f"shipped obstacles_contact_dist = {tuning.clearance.OBSTACLES_CONTACT_DIST} (inert)\n")

    band_counts = [0] * (len(BANDS) - 1)
    fire_committed = dict.fromkeys(CANDIDATES, 0)
    fire_free = dict.fromkeys(CANDIDATES, 0)
    total_scans = committed_scans = 0

    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        if not rows:
            print(f"  {bag.name}: no nav_debug, skipped")
            continue
        times = [t for t, _ in rows]

        reader = open_reader(bag)
        t0 = None
        while reader.has_next():
            topic, data, raw_t = reader.read_next()
            if topic != Topics.SCAN:
                continue
            if t0 is None:
                t0 = raw_t
            rel = elapsed_seconds(raw_t, t0)
            scan = decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD)

            # The tightest of the two side sectors is what the guard reacts to.
            best = None
            for centre in (math.pi / 2, -math.pi / 2):
                kept = [
                    r
                    for r, a in zip(scan.ranges_m, scan.angles_rad, strict=False)
                    if abs(_wrap(a - centre)) <= half_fov and math.isfinite(r) and r > min_valid
                ]
                if kept:
                    m = min(kept)
                    best = m if best is None else min(best, m)
            if best is None:
                continue

            total_scans += 1
            for i in range(len(BANDS) - 1):
                if BANDS[i] <= best < BANDS[i + 1]:
                    band_counts[i] += 1
                    break

            snap = nearest_by_time(rows, times, rel, tolerance=0.2)
            committed = snap is not None and snap.committed_sign_x_m is not None
            committed_scans += int(committed)
            for c in CANDIDATES:
                if best < c:
                    if committed:
                        fire_committed[c] += 1
                    else:
                        fire_free[c] += 1

    if not total_scans:
        print("no scans with a valid side sector -- nothing to decide")
        return

    print(f"== WHERE THE SIDE MINIMUM ACTUALLY SITS  ({total_scans} scans)")
    for i in range(len(BANDS) - 1):
        hi = "inf" if BANDS[i + 1] > 1e8 else f"{BANDS[i + 1]:.3f}"
        n = band_counts[i]
        print(f"  [{BANDS[i]:.3f}, {hi:>5})  {n:6d}  ({100 * n / total_scans:5.2f}%)")
    print(f"\n  a sign was committed on {committed_scans} of {total_scans} scans "
          f"({100 * committed_scans / total_scans:.1f}%)  <- the base rate any split must beat")

    print("\n== WHAT EACH CANDIDATE WOULD FIRE ON")
    print(f"  {'contact_dist':>13} {'fires':>7} {'while FREE':>12} {'while COMMITTED':>17} {'free per committed':>20}")
    for c in CANDIDATES:
        free, comm = fire_free[c], fire_committed[c]
        total = free + comm
        ratio = f"{free / comm:.2f}" if comm else ("inf" if free else "-")
        print(f"  {c:>13.3f} {total:>7} {free:>12} {comm:>17} {ratio:>20}")

    print(
        "\n  Read: a candidate is only worth shipping if it fires ENOUGH to matter and"
        "\n  fires mostly while FREE. A high committed share means it would trip on"
        "\n  pillars the router already aimed past, and each trip evicts the commitment."
    )


if __name__ == "__main__":
    main()

"""Can the Obstacles contact threshold be OBSERVED, or is it below the floor?

``clearance.toml`` sets ``obstacles_contact_dist = 0.04`` and warns in a
comment that "0.04 only works PAIRED with min_valid_range_m = 0.044. Alone it
is a threshold the guard can never observe". But 0.044 is ABOVE 0.04: the
sector filter keeps only ``r > 0.044``, so the smallest side clearance the
guard can ever see is ~0.044, and ``_side_clearance() < 0.04`` may still be
unreachable even with the pairing in place.

That matters because ``already_touching`` -- the flag that turns a forward
SIDE_CORRECTION into a REVERSE one -- is
``_side_clearance(...) < contact_dist or _forward_touching(...)``. On hardware
99.4% of side_correction ticks are reverse. If the side term is unobservable,
every one of those reverses came from the FORWARD term, and tuning the side
threshold is wasted work.

This replays the real sector filter over the recorded scans and reports the
distribution of the filtered side-sector minimum against the threshold. The
control is the same statistic computed UNFILTERED: if the unfiltered minimum
dips below 0.04 and the filtered one never does, the threshold is measuring
the filter rather than the track.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_contact_threshold_observable.py BAG [BAG ...]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from scripts.common.bag_io import Topics, decode_scan, open_reader  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD  # noqa: E402


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    tuning = get_tuning(None)
    contact = tuning.clearance.OBSTACLES_CONTACT_DIST
    min_valid = tuning.lidar_sectors.MIN_VALID_RANGE_M
    half_fov = math.radians(tuning.lidar_sectors.THREAT_HALF_FOV_DEG)

    print(f"obstacles_contact_dist = {contact}   min_valid_range_m = {min_valid}")
    print(f"threat half-FOV = {math.degrees(half_fov):.0f} deg")
    if min_valid >= contact:
        print(
            f"\n  STATIC READING: the filter keeps only r > {min_valid}, which is ABOVE the"
            f"\n  {contact} threshold. No filtered reading can satisfy `< {contact}`."
            "\n  Measuring anyway -- a static argument is not a measurement."
        )

    for bag in bags:
        below_filtered = below_unfiltered = ticks = 0
        min_filtered = min_unfiltered = None
        reader = open_reader(bag)
        while reader.has_next():
            topic, data, _t = reader.read_next()
            if topic != Topics.SCAN:
                continue
            scan = decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD)
            ticks += 1
            for centre in (math.pi / 2, -math.pi / 2):
                sector = [
                    r
                    for r, a in zip(scan.ranges_m, scan.angles_rad, strict=False)
                    if abs(_wrap(a - centre)) <= half_fov and math.isfinite(r) and r > 0.0
                ]
                if not sector:
                    continue
                raw_min = min(sector)
                min_unfiltered = raw_min if min_unfiltered is None else min(min_unfiltered, raw_min)
                below_unfiltered += int(raw_min < contact)

                kept = [r for r in sector if r > min_valid]
                if not kept:
                    continue
                f_min = min(kept)
                min_filtered = f_min if min_filtered is None else min(min_filtered, f_min)
                below_filtered += int(f_min < contact)

        print(f"\n### {bag.name}  ({ticks} scans, 2 side sectors each)")
        print(f"  UNFILTERED side min: smallest {min_unfiltered}  below {contact} on {below_unfiltered} sectors")
        print(f"  FILTERED   side min: smallest {min_filtered}  below {contact} on {below_filtered} sectors")
        if below_unfiltered and not below_filtered:
            print(
                "  -> The side term of `already_touching` is UNOBSERVABLE: the track does go"
                f"\n     below {contact}, but the filter removes exactly those returns. Every"
                "\n     reverse side_correction therefore came from _forward_touching."
            )
        elif below_filtered:
            print(f"  -> Observable: the side term fired on {below_filtered} sectors.")


if __name__ == "__main__":
    main()

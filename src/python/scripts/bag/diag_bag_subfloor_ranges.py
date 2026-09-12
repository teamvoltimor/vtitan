"""Are the sub-floor LIDAR ranges real contact, or garbage firing phantom escapes?

The 2026-09-12 rounds report ``min_lidar_range_m`` of 0.008-0.013 m. The C1
cannot produce that: ``robot.toml`` sets ``lidar.min_range = 0.045`` and the
sector filter keeps only ``r > min_valid_range_m = 0.044``, with the note that
"the sensor REPORTS 0.045 for anything closer". A reading of 8 mm is therefore
either a real value the filter never saw, or an artefact.

Which one it is decides completely different work, and the two are separable
because the bag records both the filtered risk input and the ray the escape
verdict was actually minimised over:

  * ``min_lidar_range_m``      -- the scan minimum the navigator saw.
  * ``escape_trigger_range_m`` -- the raw range of the ray that CAUSED the
                                  escape, with its bearing.

If escapes fire on sub-floor ranges, the recovery is chasing readings the
sensor cannot make, and 99.4%-reverse side_correction is a phantom. If the
triggers sit above the floor while only the reported minimum dips below it,
then the minimum is a cosmetic reporting bug and the contact is real.

The control is the RAW /scan topic: sub-floor values present there too means
the sensor (or driver) emits them, and the filter is what fails; absent there
means the field is computed somewhere the filter does not reach.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_subfloor_ranges.py BAG [BAG ...]
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from scripts.common.bag_io import (  # noqa: E402
    Topics,
    decode_scan,
    load_nav_debug_rows,
    open_reader,
)
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD  # noqa: E402

SENSOR_FLOOR_M = 0.045
"""robot.toml lidar.min_range -- the smallest range the C1 can report."""

VALID_FILTER_M = 0.044
"""lidar_sectors.toml min_valid_range_m -- the navigator's own keep threshold."""


def _hist(values: list[float]) -> str:
    if not values:
        return "none"
    v = sorted(values)
    return (
        f"n={len(v)} min={v[0]:.4f} p05={v[len(v) // 20]:.4f} p50={v[len(v) // 2]:.4f} max={v[-1]:.4f}"
    )


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        mins = [s.min_lidar_range_m for _t, s in rows if s.min_lidar_range_m is not None]
        trig = [s.escape_trigger_range_m for _t, s in rows if s.escape_trigger_range_m is not None]
        sub_min = [v for v in mins if v < VALID_FILTER_M]
        sub_trig = [v for v in trig if v < VALID_FILTER_M]

        print(f"\n### {bag.name}")
        print(f"  min_lidar_range_m       {_hist(mins)}")
        print(
            f"    below the {VALID_FILTER_M} filter: {len(sub_min)} of {len(mins)}"
            f" ({100 * len(sub_min) / len(mins):.1f}%)" if mins else "    no values"
        )
        print(f"  escape_trigger_range_m  {_hist(trig)}")
        if trig:
            # The verdict has to follow the count, not the hypothesis that
            # prompted the script: the first version printed the accusation
            # unconditionally and read as damning at a count of zero.
            note = (
                "   <- escapes fired on impossible ranges"
                if sub_trig
                else "   <- CLEAN: the escape path filters correctly"
            )
            print(
                f"    below the {VALID_FILTER_M} filter: {len(sub_trig)} of {len(trig)}"
                f" ({100 * len(sub_trig) / len(trig):.1f}%){note}"
            )

        # CONTROL: the raw topic. If the driver never emits sub-floor values,
        # the navigator is manufacturing them downstream.
        raw_sub = raw_total = 0
        raw_min = None
        reader = open_reader(bag)
        while reader.has_next():
            topic, data, _t = reader.read_next()
            if topic != Topics.SCAN:
                continue
            # Same mount correction the real gateway applies; the angles are
            # unused here but decode_scan owns the dropout substitution.
            scan = decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD)
            for r in scan.ranges_m:
                if r is None or r <= 0.0:
                    continue
                raw_total += 1
                raw_min = r if raw_min is None else min(raw_min, r)
                raw_sub += int(r < VALID_FILTER_M)
        if raw_total:
            print(
                f"  CONTROL raw /scan: {raw_sub} of {raw_total} returns below {VALID_FILTER_M}"
                f" ({100 * raw_sub / raw_total:.3f}%)  raw min {raw_min:.4f}"
            )
        else:
            print("  CONTROL raw /scan: no returns decoded -- control is broken, ignore the above")

    print(
        f"\nReading: sensor floor {SENSOR_FLOOR_M} m, navigator keeps r > {VALID_FILTER_M} m."
        "\n  sub-floor in raw /scan AND in the triggers -> the filter is not being applied"
        "\n  sub-floor only in min_lidar_range_m        -> cosmetic reporting bug, contact is real"
        "\n  sub-floor in the triggers                  -> escapes fire on impossible ranges"
    )


if __name__ == "__main__":
    main()

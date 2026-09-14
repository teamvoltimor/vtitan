"""How wide is the FRONT contact trigger's observable window, on real scans?

``obstacles_contact_dist`` is compared against a BUMPER GAP, not a raw range:
``gap = range - LIDAR_TO_FRONT_BUMPER`` (0.0278 m). The sector filter keeps
only ``r > min_valid_range_m`` (0.044), and the C1 REPORTS 0.045 for anything
closer than it can measure. So a candidate threshold can only ever fire on a
return inside a narrow band:

    live window = ( min_valid_range_m , contact_dist + LIDAR_TO_FRONT_BUMPER ]

which is 24 mm wide at 0.04 and 84 mm at 0.10. Inside that band the sensor is
at its least reliable -- measured on hardware, 25% of rays do not return at all
and 6.9% come back BELOW the filter floor, where they are discarded as invalid.

The question this answers is not "is the threshold observable" (it is, for
every candidate: the floor in gap terms is 0.045-0.0278 = 0.0172 m). It is
whether a SMALLER threshold buys a narrower window than the sensor can
reliably fill -- i.e. how often the robot is genuinely close to something and
the trigger cannot see it because the return landed under the floor.

Per bag it reports, over the forward threat cone:

  * ray-level: share of forward rays that are no-return, sub-floor, or valid
  * tick-level FIRE:  ticks whose FILTERED min gap is below each candidate
  * tick-level BLIND: ticks where the UNFILTERED scan says something is inside
                      the floor (genuinely close) but the FILTERED scan does
                      not fire the candidate -- a miss caused by the filter,
                      not by the geometry.

BLIND rising as the candidate falls is the cost of a narrow window. If BLIND is
flat across candidates, window width is not the binding constraint and the
choice can be made on the corpus alone.

Usage:
    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
    pixi run -e dev python scripts/bag/diag_bag_front_trigger_window.py BAG [BAG ...]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from scripts.common.bag_io import Topics, decode_scan, open_reader  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD  # noqa: E402

CANDIDATES = (0.04, 0.05, 0.06, 0.07, 0.10)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    tuning = get_tuning(None)
    min_valid = tuning.lidar_sectors.min_valid_range_m
    half_fov = math.radians(tuning.lidar_sectors.threat_half_fov_deg)
    offset = RobotSpecs.LIDAR_TO_FRONT_BUMPER

    print(f"min_valid_range_m = {min_valid}   LIDAR_TO_FRONT_BUMPER = {offset:.4f}")
    print(f"forward threat half-FOV = {math.degrees(half_fov):.0f} deg")
    print(f"smallest observable gap = {min_valid - offset:+.4f} m")
    for cd in CANDIDATES:
        print(f"  contact_dist {cd:.2f} -> live window ({min_valid:.3f}, {cd + offset:.4f}] m, "
              f"width {1000 * (cd + offset - min_valid):.0f} mm")

    tot_rays = tot_noret = tot_sub = tot_valid = 0
    tot_ticks = 0
    fire = dict.fromkeys(CANDIDATES, 0)
    blind = dict.fromkeys(CANDIDATES, 0)
    close_ticks = 0

    for bag in bags:
        reader = open_reader(bag)
        while reader.has_next():
            topic, data, _t = reader.read_next()
            if topic != Topics.SCAN:
                continue
            msg = deserialize_message(data, LaserScan)
            scan = decode_scan(msg, _LIDAR_YAW_OFFSET_RAD)
            # decode_scan SUBSTITUTES max_range for a dropout (bag_io.py:99),
            # which is what the real gateway does -- so the decoded scan is
            # right for what the navigator SEES, but it cannot tell a dropout
            # from a genuine far reading. Classify the rays from the RAW
            # message and keep the decoded scan only for the firing decision.
            raw = [
                r for r, a in zip(msg.ranges, scan.angles_rad, strict=False)
                if abs(_wrap(a)) <= half_fov
            ]
            fwd = [
                r for r, a in zip(scan.ranges_m, scan.angles_rad, strict=False)
                if abs(_wrap(a)) <= half_fov
            ]
            if not fwd:
                continue
            tot_ticks += 1
            tot_rays += len(raw)
            tot_noret += sum(1 for r in raw if not math.isfinite(r) or r <= 0.0)
            sub = [r for r in raw if math.isfinite(r) and 0.0 < r <= min_valid]
            kept = [r for r in fwd if math.isfinite(r) and r > min_valid]
            tot_sub += len(sub)
            tot_valid += len(kept)

            # Something is genuinely inside the filter floor this tick.
            genuinely_close = bool(sub)
            close_ticks += int(genuinely_close)

            filt_gap = (min(kept) - offset) if kept else None
            for cd in CANDIDATES:
                fired = filt_gap is not None and filt_gap < cd
                fire[cd] += int(fired)
                if genuinely_close and not fired:
                    blind[cd] += 1

    if not tot_ticks:
        print("\nNo forward scans found. Wrong topic or empty bags?")
        return 1

    print(f"\n=== {len(bags)} bag(s), {tot_ticks} forward scans, {tot_rays} forward rays ===")
    print(f"  no-return      {tot_noret:>9} ({100 * tot_noret / tot_rays:5.1f}%)")
    print(f"  <= floor       {tot_sub:>9} ({100 * tot_sub / tot_rays:5.1f}%)  discarded as invalid")
    print(f"  valid          {tot_valid:>9} ({100 * tot_valid / tot_rays:5.1f}%)")
    print(f"\n  ticks with something inside the floor: {close_ticks} "
          f"({100 * close_ticks / tot_ticks:.1f}%)")
    print(f"\n  {'cand':>5} {'FIRE':>9} {'%ticks':>8} {'BLIND':>8} {'%close':>8}")
    for cd in CANDIDATES:
        pct_close = (100 * blind[cd] / close_ticks) if close_ticks else 0.0
        print(f"  {cd:>5.2f} {fire[cd]:>9} {100 * fire[cd] / tot_ticks:>7.1f}% "
              f"{blind[cd]:>8} {pct_close:>7.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

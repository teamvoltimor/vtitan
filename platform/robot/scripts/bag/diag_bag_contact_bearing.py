r"""When the robot is touching something, can the escape SEE it?

Obstacles runs collide repeatedly and rarely reach half a lap. Two candidate
explanations, and they want opposite fixes:

* the escape sees the obstacle and reacts badly, or
* the escape cannot see it at all, and reacts only once the geometry has
  already changed.

``CollisionAvoidanceController`` judges risk over the FORWARD DRIVING LANE, a
band of ``path_half_width`` (0.197 m) either side of the robot's axis, ahead of
it -- deliberately, because treating corridor side walls as obstacles pins the
speed to a crawl for a whole lap. The consequence is geometric: an object
beside the chassis is outside the lane BY CONSTRUCTION, however close it is.

So this asks, of every tick where something is within touching distance, what
BEARING it is at and whether that bearing falls inside the lane. A contact the
lane cannot contain is one no escape can ever fire on, and the fix is then the
sensing window rather than the manoeuvre.

Reads ``/scan`` directly with the gateway's own mount correction, not
``nav_debug.min_lidar_range_m`` -- that field bottoms out near 0.006 m on every
run because the gateway leaves invalid near-zero returns in the scan.

The chassis half-width is 0.097 m, so a return at 0.10 m and 70 deg off the
nose is 0.094 m of lateral clearance: beside the robot, not in front of it.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_contact_bearing.py \
        data/live/runs/run_20260908_0047*
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from pathlib import Path

from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, decode_scan, open_reader
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

PATH_HALF_WIDTH_M = 0.197
"""``CollisionAvoidanceController.path_half_width`` -- the lane's half width."""

CONTACT_M = 0.12
"""A return this close is touching or about to. The measured escape trigger
ceiling over the 2026-09-08 session was 0.128 m, so this is the regime every
escape actually fired in."""

MIN_VALID_M = 0.05
"""``min_valid_range_m``. Below it the sensor is not measuring anything."""

# The chassis, in the LIDAR's own frame. The sensor sits 0.1222 m FORWARD of
# centre on a 0.300 x 0.194 m body, so its rear edge is 0.2722 m behind the
# sensor and its sides are 0.097 m away -- well inside the 0.12 m this script
# calls contact. Returns landing in that box are the ROBOT, not an obstacle.
# Without this filter 77.7% of "contacts" sat at 120-180 deg, which is the
# self-return problem this project has already been bitten by once.
SELF_X_MIN, SELF_X_MAX = -0.2722, 0.0278
SELF_Y_ABS = 0.097
SELF_MARGIN_M = 0.01
"""Grown slightly: the body is not a perfect rectangle and the mount has
tolerance, so a return a few millimetres outside the nominal box is still the
chassis rather than a pillar."""

BEARING_EDGES = (0, 15, 30, 45, 60, 75, 90, 120, 180)
"""|bearing| bands in degrees, finer where the lane boundary falls."""


def _band(deg: float) -> str:
    for lo, hi in zip(BEARING_EDGES, BEARING_EDGES[1:]):
        if lo <= deg < hi:
            return f"{lo:>3}-{hi:<3}"
    return "180+"


def analyse(bag_dir: Path) -> tuple[int, int, Counter, list[float]]:
    reader = open_reader(bag_dir)
    in_lane = out_lane = 0
    bands: Counter[str] = Counter()
    steer_at_contact: list[float] = []
    last_steer: float | None = None

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            try:
                snap = decode_nav_debug(data)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(snap.commanded_steering_norm, (int, float)):
                last_steer = abs(snap.commanded_steering_norm)
            continue
        if topic != Topics.SCAN:
            continue
        from rclpy.serialization import deserialize_message

        scan = decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD)

        # The single closest VALID return this tick that is NOT the robot.
        best_r, best_a = math.inf, 0.0
        for r, a in zip(scan.ranges_m, scan.angles_rad):
            if not (MIN_VALID_M < r < best_r):
                continue
            px, py = r * math.cos(a), r * math.sin(a)
            if (
                SELF_X_MIN - SELF_MARGIN_M <= px <= SELF_X_MAX + SELF_MARGIN_M
                and abs(py) <= SELF_Y_ABS + SELF_MARGIN_M
            ):
                continue
            best_r, best_a = r, a
        if best_r > CONTACT_M:
            continue

        # Lane membership is a LATERAL test, not an angular one -- that is what
        # the controller does, and the difference is the whole point: at 0.10 m
        # the lane spans +-80 deg, at 0.40 m only +-30.
        lateral = abs(best_r * math.sin(best_a))
        forward = best_r * math.cos(best_a)
        if forward > 0 and lateral <= PATH_HALF_WIDTH_M:
            in_lane += 1
        else:
            out_lane += 1
        # WRAP first. The gateway's mount correction is ADDED to the raw
        # sweep, so bearings run past pi and an unwrapped |angle| lands every
        # rear return in a >180 deg bucket -- 921 of 1085 ticks vanished that
        # way before this line existed. The lane test above uses sin/cos and is
        # unaffected, which is exactly why the discrepancy was visible.
        wrapped = math.atan2(math.sin(best_a), math.cos(best_a))
        bands[_band(abs(math.degrees(wrapped)))] += 1
        if last_steer is not None:
            steer_at_contact.append(last_steer)

    return in_lane, out_lane, bands, steer_at_contact


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()

    total_in = total_out = 0
    all_bands: Counter[str] = Counter()
    all_steers: list[float] = []
    for bag_dir in args.bag_dirs:
        try:
            i, o, bands, steers = analyse(Path(bag_dir))
        except Exception as exc:  # noqa: BLE001
            print(f"{Path(bag_dir).name:<26} unreadable: {ascii(exc)[:90]}")
            continue
        total_in += i
        total_out += o
        all_bands += bands
        all_steers.extend(steers)
        if i + o:
            print(f"{Path(bag_dir).name:<26} contact ticks={i + o:<6} in lane {100 * i / (i + o):5.1f}%")

    total = total_in + total_out
    print(f"\n== ALL RUNS: {total} ticks with something inside {CONTACT_M} m")
    if not total:
        return 0
    print(f"  inside the forward lane (escape CAN see it):  {total_in:>6} ({100 * total_in / total:.1f}%)")
    print(f"  outside it (escape is structurally blind):    {total_out:>6} ({100 * total_out / total:.1f}%)")
    print("\n  |bearing| of the closest return:")
    for lo, hi in zip(BEARING_EDGES, BEARING_EDGES[1:]):
        key = f"{lo:>3}-{hi:<3}"
        n = all_bands.get(key, 0)
        if n:
            print(f"    {key} deg  {n:>6} ({100 * n / total:5.1f}%)")

    if all_steers:
        ordered = sorted(all_steers)
        def q(f: float) -> float:
            return ordered[min(len(ordered) - 1, int(f * len(ordered)))]
        turning = sum(1 for v in all_steers if v >= 0.5)
        print()
        print(
            "  |steering| at the moment of contact: "
            f"p25={q(0.25):.2f} p50={q(0.50):.2f} p75={q(0.75):.2f} p95={q(0.95):.2f}"
        )
        print(f"    at or past half lock: {turning}/{len(all_steers)} ({100 * turning / len(all_steers):.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

r"""Does the rear sector actually MEASURE anything on hardware?

``K_TURN_FIT_REAR_GAP`` (shipped d2599906, ON for Obstacles) caps a reversing
escape at the rear room the LIDAR reports. It is deliberately conservative
about blindness: a sector that measured NOTHING is left alone rather than
capped to zero, so the manoeuvre is never silently deleted on a mount with no
rear slot.

That safety property has a cost nobody has priced. Every scan where the rear
sector is unmeasured is a scan where the cap does nothing, and if that is most
of them, the fix is shipping as a no-op while being counted as a fix. The
mount makes this a live worry rather than a hypothetical: the occlusion wedges
leave only a ~25 deg slot straight back, the chassis rear face sits 0.2722 m
behind the sensor, and on run_20260906_192424 the rear minimum was the CHASSIS
on 100% of scans.

So this replays ``CollisionAvoidanceController.rear_sector`` -- the shipped
code, not a restatement of it -- over the recorded sweeps and counts.

TWO POPULATIONS, and only the second one decides anything:

* EVERY scan, which says how blind the mount is in general;
* the scans at the moment a REVERSE was launched, which is the only moment
  ``K_TURN_FIT_REAR_GAP`` can act. A cap that is blind 90% of the time overall
  but sighted whenever it matters is a working cap.

The reverse moments are found from ``/nav_debug``: the tick where
``active_maneuver_type`` becomes STUCK_REVERSE, matched to its nearest sweep.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_rear_sector_measured.py \
        $(cat corpus_obstacles.txt)
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import create_bags_parser, read_vision_rows_and_scans  # noqa: E402
from scripts.common.stats import fmt_p50_p90, nearest_by_time  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.control.controllers.collision_avoidance import CollisionAvoidanceController  # noqa: E402


def _is_reverse(row) -> bool:  # noqa: ANN001
    """A latched manoeuvre that drives BACKWARDS.

    Read from the commanded speed rather than the manoeuvre name: the name
    distinguishes intent (STUCK_REVERSE vs STUCK_FORWARD) but the cap acts on
    direction, and a forward-named leg at negative speed would still be capped.
    """
    return row.active_maneuver_type is not None and (row.maneuver_speed_mps or 0.0) < 0.0


def main() -> None:
    args = create_bags_parser(__doc__).parse_args()
    controller = CollisionAvoidanceController.from_tuning(get_tuning(None))

    all_measured = all_total = 0
    rev_measured = rev_total = 0
    # Controls. A reverse count of zero is a claim, and an unproven null here
    # would read as "the cap is never reachable" when the real cause could be
    # an unpopulated field or a corpus with no escapes at all.
    latched_any = 0
    latched_by_name: dict[str, int] = {}
    speed_unpublished = 0
    all_gaps: list[float] = []
    rev_gaps: list[float] = []
    skipped: list[str] = []

    for bag in args.bag_dirs:
        try:
            rows, _frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            skipped.append(f"{Path(bag).name} ({type(exc).__name__})")
            continue
        if not scans:
            continue
        scan_times = [t for t, _ in scans]

        for rel, msg in scans:
            ranges, angles = _scan_to_ranges_angles(deserialize_message(msg, LaserScan))
            sector = controller.rear_sector(ranges, angles)
            all_total += 1
            if sector.measured:
                all_measured += 1
                all_gaps.append(sector.min_range_m)
            del rel

        # Only the FIRST tick of each latched reverse: a manoeuvre held for 30
        # frames would otherwise count 30 times and drown the population that
        # actually decides.
        was_reversing = False
        for rel, row in rows:
            if row.active_maneuver_type is not None:
                latched_any += 1
                name = str(row.active_maneuver_type)
                latched_by_name[name] = latched_by_name.get(name, 0) + 1
                if row.maneuver_speed_mps is None:
                    speed_unpublished += 1
            reversing = _is_reverse(row)
            if reversing and not was_reversing:
                msg = nearest_by_time(scans, scan_times, rel)
                ranges, angles = _scan_to_ranges_angles(deserialize_message(msg, LaserScan))
                sector = controller.rear_sector(ranges, angles)
                rev_total += 1
                if sector.measured:
                    rev_measured += 1
                    rev_gaps.append(sector.min_range_m)
            was_reversing = reversing

    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(skipped[:6])}")
        print()
    if not all_total:
        print("No sweeps found in these bags.")
        return

    def pct(n: int, d: int) -> str:
        return f"{100 * n / d:5.1f}%" if d else "    --"

    print("== IS THE REAR SECTOR MEASURED?")
    print_table(
        [
            ["every sweep", f"{all_measured} / {all_total}", pct(all_measured, all_total)],
            ["at a reverse launch", f"{rev_measured} / {rev_total}", pct(rev_measured, rev_total)],
        ],
        ["population", "measured / total", "share"],
    )
    print()
    print(f"  rear gap when measured, every sweep:   {fmt_p50_p90(all_gaps) if all_gaps else 'never'}")
    print(f"  rear gap when measured, at a reverse:  {fmt_p50_p90(rev_gaps) if rev_gaps else 'never'}")
    print()
    print(f"  (sensor frame. The rear bumper sits {RobotSpecs.LIDAR_TO_REAR_BUMPER:.4f} m behind the")
    print("   sensor, so subtract that to read these as room behind the CHASSIS.)")
    print()
    print("== CONTROLS (so a zero above is a fact, not a broken read)")
    print(f"  ticks with a latched manoeuvre:        {latched_any}")
    print(f"  of those, maneuver_speed_mps is None:  {speed_unpublished}")
    for name, count in sorted(latched_by_name.items(), key=lambda kv: -kv[1]):
        print(f"    {name:>28}  {count}")
    print()
    if rev_total:
        blind = rev_total - rev_measured
        print(f"== VERDICT: K_TURN_FIT_REAR_GAP can act on {rev_measured} of {rev_total} reverse launches.")
        print(f"   It is INERT on the other {blind} ({pct(blind, rev_total).strip()}), by design -- an")
        print("   unmeasured sector is left alone rather than capped to zero.")
    else:
        print("== No reverse launches in this corpus, so the cap was never reachable here.")


if __name__ == "__main__":
    main()

r"""When the chassis is close enough to shove something, is it going FORWARD or BACK?

`scoring.py` accumulates displacement as the component of travel pointing AT an
obstacle while touching it, and ends the run past
``TrafficSignSpecs.MAX_LEGAL_DISPLACEMENT_M`` (59.4 mm). It does not care which
way the chassis was pointing. So "reverse more slowly" is only a fix for pushes
if the pushes happen in REVERSE, and that is an empirical question this answers
rather than assumes -- the corpus says the sim's failures are collisions in a
k_turn's reverse leg, but the operator's read of the track is that the real
shoves happen driving forward.

METHOD

For every tick, take the minimum LIDAR range in a front cone and in a rear cone
and call the tick CONTACT-CLOSE when either is under ``--contact-m``. The
direction is the ENCODER's sign, not the command: a commanded reverse that the
drivetrain never delivered pushes nothing.

The pairing is what matters. A tick is only counted as a push opportunity when
the close side and the travel direction AGREE -- something close in front while
driving forward, or close behind while reversing. Close in front while reversing
is the chassis driving AWAY from the obstacle, which cannot displace it.

CONTROLS, because this would otherwise just measure where the robot spends time:

* bay_exit is reported separately. It is a confined pocket where everything is
  close, it commands full lock, and it is a known separate failure -- pooling it
  would swamp the open-track figure.
* ticks with the encoder at rest are excluded from the direction split, since a
  stationary chassis displaces nothing either way.
* the REAR cone on this chassis is the LIDAR's occluded band, so rear ranges are
  sparse; the count of usable rear samples is printed so a rear figure built on
  nothing is visible as such.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_contact_direction.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from scripts.common.bag_io import (
    LIDAR_YAW_OFFSET_RAD,
    create_bags_parser,
    open_reader,
    read_bag,
    read_motion_streams,
)

FRONT_HALF_FOV_RAD = math.pi / 4
REAR_HALF_FOV_RAD = math.pi / 4


def cone_min(ranges, angles, centre_rad, half_fov_rad, floor_m):
    """Closest valid return inside a cone, or None when the cone saw nothing."""
    if ranges is None or len(ranges) == 0:
        return None
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    delta = np.arctan2(np.sin(angles - centre_rad), np.cos(angles - centre_rad))
    inside = (np.abs(delta) <= half_fov_rad) & np.isfinite(ranges) & (ranges > floor_m)
    return float(np.min(ranges[inside])) if bool(np.any(inside)) else None


def nearest(series, ts):
    """Value of a (timestamp, value) series nearest to ts."""
    if not series:
        return None
    return min(series, key=lambda tv: abs(tv[0] - ts))[1]


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--contact-m", type=float, default=0.12, help="Range under which a cone counts as contact-close.")
    parser.add_argument("--range-floor-m", type=float, default=0.05, help="Returns at or below this are self-returns from the chassis.")
    parser.add_argument("--rest-dps", type=float, default=20.0, help="Wheel speed under which the chassis counts as at rest.")
    args = parser.parse_args()

    totals = {"fwd": 0, "rev": 0, "bay_fwd": 0, "bay_rev": 0}
    wtotals = {"fwd": 0.0, "rev": 0.0, "bay_fwd": 0.0, "bay_rev": 0.0}
    attrib = {"fwd_maneuver": 0, "fwd_unattended": 0}
    for bag_dir in args.bag_dirs:
        # Angles carry production's own mount-rotation correction, so the rear
        # cone here is the same rear the controller reasons about.
        scans, rows = read_bag(open_reader(bag_dir), LIDAR_YAW_OFFSET_RAD)
        wheel = read_motion_streams(bag_dir).drive_speed_dps
        if not wheel:
            print(f"\n=== {bag_dir.name}: no wheel stream")
            continue

        counts = {"fwd": 0, "rev": 0, "bay_fwd": 0, "bay_rev": 0}
        weights = {"fwd": 0.0, "rev": 0.0, "bay_fwd": 0.0, "bay_rev": 0.0}
        moving = front_seen = rear_seen = 0
        for ts, scan in scans:
            speed = nearest(wheel, ts)
            if speed is None or abs(speed) < args.rest_dps:
                continue
            moving += 1
            front = cone_min(scan.ranges_m, scan.angles_rad, 0.0, FRONT_HALF_FOV_RAD, args.range_floor_m)
            rear = cone_min(scan.ranges_m, scan.angles_rad, math.pi, REAR_HALF_FOV_RAD, args.range_floor_m)
            front_seen += front is not None
            rear_seen += rear is not None
            snap = min(rows, key=lambda r: abs(r[0] - ts))[1] if rows else None
            in_bay = getattr(snap, "phase", None) is not None and "bay" in str(getattr(snap, "phase", "")).lower()
            # Only an approach counts. Close in front while reversing is the
            # chassis leaving, and it cannot displace anything.
            # Weight by wheel speed as well as counting ticks. scoring.py
            # accumulates DISTANCE travelled into the obstacle, and a forward
            # tick covers more ground than a reverse one, so a tick count alone
            # understates the forward share of the displacement actually scored.
            if speed > 0 and front is not None and front < args.contact_m:
                counts["bay_fwd" if in_bay else "fwd"] += 1
                weights["bay_fwd" if in_bay else "fwd"] += abs(speed)
                # Was the robot DOING anything about it? A forward push while no
                # manoeuvre is latched means the reactive layer never engaged,
                # which is a different defect from one that engaged and failed.
                if not in_bay:
                    if getattr(snap, "active_maneuver_type", None) is not None:
                        attrib["fwd_maneuver"] += 1
                    else:
                        attrib["fwd_unattended"] += 1
            if speed < 0 and rear is not None and rear < args.contact_m:
                counts["bay_rev" if in_bay else "rev"] += 1
                weights["bay_rev" if in_bay else "rev"] += abs(speed)

        for key in counts:
            totals[key] += counts[key]
            wtotals[key] += weights[key]
        open_total = counts["fwd"] + counts["rev"]
        print(f"\n=== {bag_dir.name}   moving ticks={moving}   front cone seen {front_seen}, rear cone seen {rear_seen}")
        if open_total:
            print(f"  open track  approach-into-contact ticks: {open_total}"
                  f"   FORWARD {counts['fwd']} ({100 * counts['fwd'] / open_total:.0f}%)"
                  f"   REVERSE {counts['rev']} ({100 * counts['rev'] / open_total:.0f}%)")
            wopen = weights["fwd"] + weights["rev"]
            if wopen:
                print(f"              by DISTANCE into contact:"
                      f"   FORWARD {100 * weights['fwd'] / wopen:.0f}%"
                      f"   REVERSE {100 * weights['rev'] / wopen:.0f}%")
        else:
            print("  open track  approach-into-contact ticks: 0")
        print(f"  bay_exit    FORWARD {counts['bay_fwd']}   REVERSE {counts['bay_rev']}")

    pooled = totals["fwd"] + totals["rev"]
    if pooled:
        print(f"\n=== POOLED, open track   n={pooled}"
              f"   FORWARD {totals['fwd']} ({100 * totals['fwd'] / pooled:.0f}%)"
              f"   REVERSE {totals['rev']} ({100 * totals['rev'] / pooled:.0f}%)")
        wpooled = wtotals["fwd"] + wtotals["rev"]
        if wpooled:
            print(f"    by DISTANCE into contact:   FORWARD {100 * wtotals['fwd'] / wpooled:.0f}%"
                  f"   REVERSE {100 * wtotals['rev'] / wpooled:.0f}%")
        print(f"    bay_exit, reported apart: FORWARD {totals['bay_fwd']}  REVERSE {totals['bay_rev']}")
        att = attrib["fwd_maneuver"] + attrib["fwd_unattended"]
        if att:
            print(f"    of the FORWARD pushes: {attrib['fwd_maneuver']} ({100 * attrib['fwd_maneuver'] / att:.0f}%) had a manoeuvre latched,"
                  f" {attrib['fwd_unattended']} ({100 * attrib['fwd_unattended'] / att:.0f}%) had NONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

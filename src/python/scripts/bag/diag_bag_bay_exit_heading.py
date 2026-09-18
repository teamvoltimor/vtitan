r"""Which WAY does the car leave the bay, and how long does the pocket cost it?

The operator's account of the in-bay starts is that the car does get out, nose
first toward the open side, but that it sometimes leaves running the wrong way
round the track and then has to turn back. That is not a rotation-sign question
-- the nose does come out correctly -- it is about the heading the exit LEAVES
the chassis in, relative to the placement it started from.

Measured from the IMU, not the pose: the exit is exactly where the localizer is
worst, and the placement heading is the only reference the manoeuvre itself has.

* ``settled`` -- when the round's travel direction first appears in
  ``/nav_debug``. A bay start cannot answer it until the exit is over, so this
  doubles as the cost of the pocket.
* ``bay ticks`` -- how long the exit manoeuvre held the chassis.
* ``net yaw`` -- IMU heading 30 s in, minus the heading at the first sample.
  Read it as the EXIT'S OWN ANGLE, not as the travel sense. A first cut called
  two rounds "left in the opposite sense" on this number alone and was WRONG:
  30 s covers the exit's 45-66 degrees plus a 90 degree corner, and that sum
  crosses any threshold a reversal would.
* ``travelled`` -- the honest sense measure: cumulative angular progress of the
  believed pose about the mat centre. Corners do not change its sign, the
  corridor classifier's flapping cannot reach it, and 15 cm of pose error at a
  1.5 m radius is noise. ``backtrack`` is the largest retreat from its own
  running extreme, which is what a turn-around actually looks like.

A round with no bay ticks is the control: those settle in under 3 s and are
already a full lap into the round (net yaw 270-372 degrees) while a bay start is
still deciding which way it faces.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_bay_exit_heading.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows, read_motion_streams

MAT_CENTRE_M = 1.5
"""Centre of the 3 x 3 m mat, the origin angular progress is measured about."""


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--window-s", type=float, default=30.0, help="Seconds after the first sample to judge the heading over.")
    args = parser.parse_args()

    flipped = bayed = 0
    for bag_dir in args.bag_dirs:
        try:
            rows, _counts = load_nav_debug_rows(bag_dir)
            imu = read_motion_streams(bag_dir).imu_yaw_rad
        except Exception as exc:  # noqa: BLE001  (a bad bag is a finding, not a crash)
            print(f"{bag_dir.name}: unreadable ({type(exc).__name__})")
            continue
        if not rows or not imu:
            print(f"{bag_dir.name}: no nav_debug or no IMU")
            continue

        t0 = rows[0][0]
        settled = next((ts - t0 for ts, snap in rows if snap.direction), None)
        direction = next((str(snap.direction).split(".")[-1] for _ts, snap in rows if snap.direction), "?")
        bay_ticks = sum(1 for _ts, snap in rows if "bay" in str(getattr(snap, "phase", "")).lower())

        times = np.array([t for t, _ in imu])
        yaw = np.unwrap([y for _t, y in imu])
        window = (times >= t0) & (times <= t0 + args.window_s)
        if int(window.sum()) < 3:
            print(f"{bag_dir.name}: no IMU inside the window")
            continue
        net = math.degrees(yaw[window][-1] - yaw[window][0])

        posed = [(snap.pose_x, snap.pose_y) for _ts, snap in rows if snap.pose_x is not None and snap.pose_y is not None]
        travelled = backtrack = float("nan")
        if len(posed) >= 50:
            angle = np.unwrap(
                np.arctan2(
                    np.array([y for _x, y in posed]) - MAT_CENTRE_M,
                    np.array([x for x, _y in posed]) - MAT_CENTRE_M,
                )
            )
            progress = np.degrees(angle - angle[0])
            travelled = float(progress[-1])
            signed = progress if travelled > 0 else -progress
            backtrack = float(np.max(np.maximum.accumulate(signed) - signed))
        sense = "ccw" if travelled > 0 else "cw"
        agrees = direction.startswith("counter") == (sense == "ccw")
        bayed += bay_ticks > 0
        flipped += bay_ticks > 0 and not agrees
        print(
            f"{bag_dir.name}: dir={direction:17s} settled={'n/a' if settled is None else f'{settled:5.1f}s':>6s} "
            f"bay_ticks={bay_ticks:4d} net_yaw={net:+7.0f} deg  travelled={travelled:+7.0f} deg ({sense}, "
            f"{abs(travelled) / 360:.2f} laps) backtrack={backtrack:4.0f} deg"
            f"{'  TRAVELLED AGAINST ITS OWN BELIEF' if (bay_ticks and not agrees) else ''}"
        )
    if bayed:
        print(f"SUMMARY in-bay rounds={bayed} travelled_against_belief={flipped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

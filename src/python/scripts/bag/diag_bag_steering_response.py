r"""How long does the chassis actually take to turn after a steering step?

The reactive escape commands `REV_STEER_DEG` 44 deg and lasts a median 0.49 s.
At the shipped `MAX_STEERING_RATE` 1.2 rad/s, centre to 44 deg is 0.77 rad =
**0.64 s** -- longer than the escape. If that constant is honest the escape can
never reach the angle it asks for, which is why it extracts only 10.5 deg of
rotation, leaves pointing where it entered, and re-approaches the same pillar
after 2-8 cm ([[escape_loop_is_cleared_then_drove_back_in]]).

`MAX_STEERING_RATE` 1.2 has NEVER been measured; a real 35 kg servo is ~5 rad/s.
But the wheel angle has no sensor -- `get_steering_position()` returns the last
COMMAND -- so the usual answer is a bench test with a high-speed camera.

The gyro can answer it instead, and answers a BETTER question. Yaw rate is
`v * tan(delta) * (1 + rear_ratio) * yaw_gain / wheelbase`, so with the chassis
driving at a steady speed a step in commanded steering makes the yaw rate ramp
to a new plateau, and the ramp time is what the escape actually has to live
with. That is servo AND chassis together, which is the thing that matters: not
where the wheel is, but whether the robot has turned.

Method: find ticks in `normal_drive` where commanded steering jumps by at least
``--step`` while the commanded speed is above ``--min-speed`` and steady within
``--speed-tol``. Track yaw rate after each, and report the time to reach 63% and
90% of the plateau it settles at.

CONTROLS, because a null here is otherwise unreadable:

* ``usable steps`` -- if this is near zero nothing below means anything, and the
  reasons are printed separately (too slow, speed not steady, no plateau).
* ``plateau yaw rate`` -- if the robot barely turned after the step, the rise
  time is fitted to noise. Steps whose plateau is under ``--min-rate`` are
  dropped and counted.
* The prediction from the shipped constant is printed alongside the measurement,
  so the comparison is explicit rather than left to the reader.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_steering_response.py \
        ../../data/live/runs/run_20260910_12*
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, elapsed_seconds, open_reader

_MAX_STEERING_RATE = 1.2  # rad/s, the shipped constant this exists to check
_REV_STEER_RAD = math.radians(44.0)


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--step", type=float, default=0.30, help="min jump in commanded steering_norm to count as a step")
    parser.add_argument("--min-speed", type=float, default=0.15, help="m/s below which yaw rate is too small to read")
    parser.add_argument("--speed-tol", type=float, default=0.05, help="m/s the speed may drift during the window")
    parser.add_argument("--window", type=float, default=1.2, help="seconds tracked after a step")
    parser.add_argument("--min-rate", type=float, default=0.15, help="rad/s plateau below which the fit is noise")
    args = parser.parse_args()

    rows: list[tuple[float, float, float, float]] = []
    dropped_slow = dropped_unsteady = dropped_flat = 0
    steps_seen = 0

    for bag_dir in args.bag_dirs:
        reader = open_reader(Path(bag_dir))
        t0 = None
        samples: list[tuple[float, float, float, float]] = []  # t, steer, speed, yaw
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic != Topics.NAV_DEBUG:
                continue
            if t0 is None:
                t0 = stamp
            snap = decode_nav_debug(data)
            if snap is None or str(getattr(snap.phase, "value", snap.phase)) != "normal_drive":
                continue
            if snap.commanded_steering_norm is None or snap.commanded_speed_mps is None or snap.pose_yaw is None:
                continue
            samples.append((elapsed_seconds(stamp, t0), snap.commanded_steering_norm, snap.commanded_speed_mps, snap.pose_yaw))

        # yaw RATE by central difference on the unwrapped yaw
        rates: list[float] = [0.0] * len(samples)
        for i in range(1, len(samples)):
            dt = samples[i][0] - samples[i - 1][0]
            if dt <= 0:
                continue
            d = samples[i][3] - samples[i - 1][3]
            rates[i] = math.atan2(math.sin(d), math.cos(d)) / dt

        for i in range(1, len(samples)):
            if abs(samples[i][1] - samples[i - 1][1]) < args.step:
                continue
            steps_seen += 1
            t_step = samples[i][0]
            win = [(t, st, sp, r) for (t, st, sp, _), r in zip(samples, rates, strict=False)
                   if t_step <= t <= t_step + args.window]
            if len(win) < 4:
                continue
            speeds = [w[2] for w in win]
            if min(speeds) < args.min_speed:
                dropped_slow += 1
                continue
            if max(speeds) - min(speeds) > args.speed_tol:
                dropped_unsteady += 1
                continue
            tail = [abs(w[3]) for w in win[len(win) // 2:]]
            plateau = statistics.median(tail) if tail else 0.0
            if plateau < args.min_rate:
                dropped_flat += 1
                continue
            t63 = t90 = None
            for t, _, _, r in win:
                if t63 is None and abs(r) >= 0.63 * plateau:
                    t63 = t - t_step
                if t90 is None and abs(r) >= 0.90 * plateau:
                    t90 = t - t_step
                    break
            if t63 is not None and t90 is not None:
                rows.append((t63, t90, plateau, statistics.mean(speeds)))

    print(f"steering steps of >= {args.step} seen in normal_drive: {steps_seen}")
    print(f"  dropped: too slow {dropped_slow}, speed not steady {dropped_unsteady}, no yaw plateau {dropped_flat}")
    print(f"  USABLE: {len(rows)}"
          "   <- if this is near zero, nothing below means anything")
    if not rows:
        return
    t63 = sorted(r[0] for r in rows)
    t90 = sorted(r[1] for r in rows)
    pl = [r[2] for r in rows]
    sp = [r[3] for r in rows]
    print()
    print(f"  time to 63% of the plateau: p50 {statistics.median(t63):.3f} s   p90 {t63[int(0.9 * (len(t63) - 1))]:.3f} s")
    print(f"  time to 90% of the plateau: p50 {statistics.median(t90):.3f} s   p90 {t90[int(0.9 * (len(t90) - 1))]:.3f} s")
    print(f"  plateau yaw rate p50 {statistics.median(pl):.2f} rad/s, at speed p50 {statistics.median(sp):.2f} m/s")
    print()
    predicted = _REV_STEER_RAD / _MAX_STEERING_RATE
    print(f"  SHIPPED MAX_STEERING_RATE {_MAX_STEERING_RATE} rad/s predicts {predicted:.3f} s to reach 44 deg")
    print(f"  the escape has {0.49:.2f} s (measured median duration)")
    print("  -> if the measured 90% time is well UNDER the prediction, the constant is pessimistic")
    print("     and raising it gives the escape the angle it already asks for.")


if __name__ == "__main__":
    main()

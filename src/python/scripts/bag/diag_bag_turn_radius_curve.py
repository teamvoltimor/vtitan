"""The chassis turn radius as a function of speed, MEASURED at race speed.

The navigation stack reasons against `MIN_TURN_RADIUS_M`, which was fitted as
a speed curve `R = 0.053 + 1.86 v` from pocket/bay manoeuvres -- i.e. from
data taken at 0.1-0.2 m/s and then EXTRAPOLATED across the whole Open ladder
up to 0.55 m/s. Every "the path demands a radius the chassis lacks" claim in
this project rests on that extrapolation, and nobody has checked it where it
matters. The 2026-09-12 Open runs can: three clean laps each at the Open
ladder, twelve corners per run.

Two traps found the hard way while writing this, both of which silently
produce an empty or wrong answer rather than an error:

1. `/imu/data` angular_velocity is IDENTICALLY ZERO in these bags -- the
   BNO08x runs in RVC mode and publishes orientation only. A gyro-based yaw
   rate reads 0.000 rad/s on every one of 10855 samples and would report an
   INFINITE turn radius at every speed. The yaw rate here is differentiated
   from `pose_yaw` instead, and the raw-gyro check is kept as an assertion so
   a future bag that does carry a gyro is not silently ignored.

2. Filtering to full-lock ticks finds nothing usable. The car only commands
   |steering| >= 0.85 when it is nearly stopped (median speed on those ticks:
   0.017 m/s). That is itself a finding -- the controller never asks for the
   tightest circle at speed -- but it means the radius envelope has to be read
   from what was ACHIEVED per speed bin, not from what was commanded.

So: bin every moving tick by speed, and report the radius at the high
percentiles of |yaw rate|, which is the tightest circle the chassis actually
drove in that bin. This is an envelope, not a limit: it can only show the
chassis turning at least this tight. A bin whose measured envelope is TIGHTER
than the fitted curve refutes the fit at that speed.

`/motor/drive_speed` is DEG/S of WHEEL rotation, not m/s -- a trap this
project has been burned by -- so it is converted with the wheel radius here.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_turn_radius_curve.py \
        ../../data/live/runs/run_20260912_074544 ../../data/live/runs/run_20260912_083126
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader

WHEEL_RADIUS_M = 0.035
"""Half of the 7 cm wheel diameter recorded in robot.toml."""

FITTED_INTERCEPT = 0.053
FITTED_SLOPE = 1.86
"""The curve in force, for the comparison column. Fitted on bay-speed data."""


def _wheel_deg_s_to_mps(deg_s: float) -> float:
    return math.radians(deg_s) * WHEEL_RADIUS_M


def collect(bag_dir: Path) -> tuple[list[tuple[float, float, float]], bool]:
    """Return (speed_mps, |yaw_rate|, |steer_norm|) per nav tick, and a gyro-alive flag.

    The yaw rate is differentiated between CONSECUTIVE nav ticks, wrapped into
    (-pi, pi] so a heading crossing +/-pi does not register as a 6 rad jump.
    The speed is held from the last `/motor/drive_speed`, which publishes at a
    similar rate.
    """
    reader = open_reader(bag_dir)
    rows: list[tuple[float, float, float]] = []
    speed_mps: float | None = None
    prev: tuple[float, float] | None = None
    gyro_alive = False
    t0: int | None = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t

        if topic == Topics.MOTOR_DRIVE_SPEED:
            speed_mps = _wheel_deg_s_to_mps(deserialize_message(data, Float32).data)

        elif topic == Topics.IMU_DATA:
            if abs(deserialize_message(data, Imu).angular_velocity.z) > 1e-6:
                gyro_alive = True

        elif topic == Topics.NAV_DEBUG:
            snap = decode_nav_debug(data)
            if snap.pose_yaw is None or speed_mps is None:
                continue
            now = elapsed_seconds(t, t0)
            if prev is not None:
                dt = now - prev[0]
                if 0.01 < dt < 0.25:
                    dyaw = (snap.pose_yaw - prev[1] + math.pi) % (2 * math.pi) - math.pi
                    rows.append(
                        (abs(speed_mps), abs(dyaw / dt), abs(snap.commanded_steering_norm or 0.0))
                    )
            prev = (now, snap.pose_yaw)

    return rows, gyro_alive


def _percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))
    return ordered[idx]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--bin", type=float, default=0.05, help="Speed bin width (m/s)")
    parser.add_argument("--min-speed", type=float, default=0.08)
    args = parser.parse_args()

    pooled: list[tuple[float, float, float]] = []
    any_gyro = False
    for bag in args.bags:
        try:
            rows, gyro_alive = collect(bag)
        except Exception as exc:  # noqa: BLE001 - one bad bag must not hide the rest
            print(f"{bag.name}: FAILED {exc}")
            continue
        any_gyro = any_gyro or gyro_alive
        print(f"{bag.name}: {len(rows)} moving ticks, gyro {'ALIVE' if gyro_alive else 'DEAD (all zero)'}")
        pooled.extend(rows)

    if not pooled:
        print("\nNO usable ticks. Check the bag paths before concluding anything.")
        return
    if not any_gyro:
        print("\nNOTE: /imu/data angular_velocity was zero in every bag. Yaw rate is")
        print("differentiated from pose_yaw, so it carries the localizer's noise.")

    bins: dict[int, list[tuple[float, float, float]]] = {}
    for speed, rate, steer in pooled:
        if speed < args.min_speed:
            continue
        bins.setdefault(int(speed / args.bin), []).append((speed, rate, steer))

    print(f"\npooled moving ticks: {len(pooled)}")
    header = (
        f"{'speed bin':<14}{'n':>7}{'v mean':>8}{'|w| p90':>9}{'|w| p99':>9}"
        f"{'R@p90':>8}{'R@p99':>8}{'R fitted':>10}{'ratio90':>9}{'|steer| p95':>12}"
    )
    print(header)
    print("-" * len(header))
    for key in sorted(bins):
        rows = bins[key]
        label = f"{key * args.bin:.2f}-{(key + 1) * args.bin:.2f}"
        if len(rows) < 30:
            print(f"{label:<14}{len(rows):>7}   (thin bin, not reported)")
            continue
        speeds = [s for s, _, _ in rows]
        rates = [w for _, w, _ in rows]
        steers = [c for _, _, c in rows]
        v_mean = statistics.mean(speeds)
        w90, w99 = _percentile(rates, 0.90), _percentile(rates, 0.99)
        r90 = v_mean / w90 if w90 > 1e-6 else float("inf")
        r99 = v_mean / w99 if w99 > 1e-6 else float("inf")
        r_fit = FITTED_INTERCEPT + FITTED_SLOPE * v_mean
        print(
            f"{label:<14}{len(rows):>7}{v_mean:>8.3f}{w90:>9.3f}{w99:>9.3f}"
            f"{r90:>8.3f}{r99:>8.3f}{r_fit:>10.3f}{r90 / r_fit:>9.2f}{_percentile(steers, 0.95):>12.3f}"
        )

    print("\nR@pNN = v_mean / |w| pNN: the tightest circle actually driven in that bin.")
    print("R fitted = 0.053 + 1.86 v, the curve the planner reasons against.")
    print("ratio90 < 1 => the chassis turns TIGHTER at that speed than the fit claims,")
    print("               so every 'impossible radius' finding resting on the fit is")
    print("               overstated there by roughly that factor.")
    print("|steer| p95 shows how hard the wheel was being asked in that bin: a bin that")
    print("never approaches 1.0 has NOT been driven to its limit, so its envelope is a")
    print("lower bound on the chassis capability, not the capability itself.")


if __name__ == "__main__":
    main()

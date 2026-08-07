#!/usr/bin/env python3
"""Calibrate the drive encoder's counts-per-wheel-revolution from driven runs.

Reads RAW quadrature counts across a driven run and divides by the distance you
measure with a tape. Raw counts matter: deriving the ratio by integrating the
smoothed RPM feedback (as an earlier ad-hoc test did) folds in the estimator's
exponential smoothing and the sampling interval, on top of the real error.
``get_drive_counts()`` is the hardware counter, with none of that.

Why it matters: counts_per_rev is currently 194 (assumed 11 PPR x4 quadrature
x 4.4 gear). Paired with the measured 70 mm wheel that implies 22.0 cm of
travel per reported revolution, but the robot actually covers ~11.9 cm, so
counts_to_distance() over-reports by ~1.85x. That is harmless while nothing
consumes distance_m, but becomes a real error the moment the drive loop closes
on m/s -- run_drive_at_rpm() measures its feedback through the same constant,
so the loop would converge to a speed that is wrong by the same factor.

Wheel slip is the one contaminant left, and it only ever inflates the count for
a given distance -- so every reading is an UPPER bound on the true ratio, and
the lowest reading across speeds is the best estimate. Run at the slowest speed
that moves reliably, over the longest run you have room for (start/stop
marking error shrinks as a fraction of a longer run). Comparing two speeds is
the built-in check: if they agree, slip is negligible; if the faster one reads
higher, that gap IS the slip.

Run ON Pi 5 (it publishes to /ackermann_cmd like any other controller):

    cd ~/vtitan/platform/robot
    . ros2_ws/install/setup.bash
    python3 scripts/hardware/calibrate_encoder.py --speed 3.0 --duration-s 5.0

Mark the robot's start position, let it run, then measure how far it actually
travelled and type that in. Repeat with --speed at a couple of values.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from shared.config.constants import RobotSpecs
from std_msgs.msg import Float32

from scripts.common.motor_hold import publish_hold


class _Probe(Node):
    """Publishes drive commands and captures encoder feedback."""

    def __init__(self) -> None:
        super().__init__("encoder_calibration_probe")
        self.counts: int | None = None
        self.speed_samples: list[float] = []
        self.pub = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        # Raw counts ride on /motor/status rather than a topic of their own.
        self.create_subscription(DiagnosticStatus, "/motor/status", self._on_status, 10)
        self.create_subscription(Float32, "/motor/drive_speed", self._on_speed, 10)

    def _on_status(self, msg: DiagnosticStatus) -> None:
        for kv in msg.values:
            if kv.key == "encoder_counts":
                self.counts = int(kv.value)
                return

    def _on_speed(self, msg: Float32) -> None:
        self.speed_samples.append(msg.data)

    def drive(self, speed_mps: float, duration_s: float) -> None:
        """Hold ``speed_mps`` for ``duration_s``, then stop and let it coast out.

        Republishes at 10 Hz throughout: ackermann_motor_node stops the drive if
        it goes ~1 s without a command, which would silently truncate the run.
        """
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = speed_mps
        publish_hold(self, self.pub, msg, duration_s)

        stop = AckermannDriveStamped()
        stop.drive.steering_angle = 0.0
        stop.drive.speed = 0.0
        publish_hold(self, self.pub, stop, 2.5, publish_interval_s=0.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=3.0, help="Commanded speed (current fake m/s units)")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument(
        "--wheel-diameter-m",
        type=float,
        default=2.0 * RobotSpecs.WHEEL_RADIUS,
        help="Measured wheel diameter",
    )
    parser.add_argument("--counts-per-rev-config", type=float, default=194.0, help="Value currently in the driver")
    args = parser.parse_args()

    circumference_m = math.pi * args.wheel_diameter_m
    print(f"Wheel {args.wheel_diameter_m * 1000:.0f} mm -> circumference {circumference_m * 100:.2f} cm")
    print("Clear a straight run and mark the start position.\n")

    rclpy.init()
    node = _Probe()
    time.sleep(1.0)

    have_counts = node.counts is not None
    if not have_counts:
        print("NOTE: no raw-counts topic found; falling back to integrating the")
        print("      smoothed /motor/drive_speed feedback, which is less exact.\n")

    results: list[float] = []
    try:
        for run in range(1, args.runs + 1):
            input(f"[run {run}/{args.runs}] Robot on the start mark, press Enter to drive...")

            start_counts = node.counts
            node.speed_samples.clear()
            t0 = time.monotonic()
            node.drive(args.speed, args.duration_s)
            elapsed = time.monotonic() - t0

            if have_counts and start_counts is not None and node.counts is not None:
                counts = abs(node.counts - start_counts)
                source = "raw counts"
            else:
                moving = [abs(v) for v in node.speed_samples if abs(v) >= 5.0]
                if not moving:
                    print("  no motion detected -- speed too low? skipping\n")
                    continue
                revs = statistics.fmean(moving) * elapsed / 360.0
                counts = revs * args.counts_per_rev_config
                source = "integrated speed (approx)"

            raw = input(f"  Measure the distance travelled, in cm (blank to skip): ").strip()
            if not raw:
                print("  skipped\n")
                continue
            distance_m = float(raw) / 100.0

            wheel_revs = distance_m / circumference_m
            cpr = counts / wheel_revs
            results.append(cpr)
            print(f"  {counts:.0f} counts ({source}) over {distance_m * 100:.1f} cm")
            print(f"  -> {wheel_revs:.3f} wheel revs -> {cpr:.1f} counts per wheel revolution\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not results:
        print("No usable runs.")
        sys.exit(1)

    best = min(results)  # slip only inflates, so the lowest reading is closest to truth
    print("=" * 62)
    for i, r in enumerate(results, 1):
        print(f"  run {i}: {r:.1f}")
    if len(results) > 1:
        spread = (max(results) - min(results)) / statistics.fmean(results) * 100
        print(f"  spread {spread:.1f}%  (this is your wheel-slip signal)")
    print(f"\nBest estimate (lowest, least slip): counts_per_rev = {best:.0f}")
    print(f"  currently configured: {args.counts_per_rev_config:.0f}  ({best / args.counts_per_rev_config:.2f}x off)")
    print("=" * 62)
    print("\nSet _DEFAULT_COUNTS_PER_REV in src/hardware/motors/dc_encoder/driver.py")


if __name__ == "__main__":
    main()

"""Instrumented track run: start a race, record what the navigator commands, stop.

Written for diagnosing real-track behaviour, where the useful evidence is what
the navigator *decided* rather than where the robot ended up. Watching a robot
misbehave tells you it steered wrong; this tells you which way, how hard, and
what it thought it saw at the time.

Bounded by construction: it E-STOPs after ``--seconds`` whatever happens, so a
run cannot outlive the person watching it.

Usage (on the Pi 5):
    pixi run -e vision python scripts/diag_track_run.py --seconds 20
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

_QOS_STATE = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class TrackRunProbe(Node):
    """Drives one bounded race and records the drive commands it produces."""

    def __init__(self, seconds: float) -> None:
        super().__init__("track_run_probe")
        self.seconds = seconds
        self.samples: list[tuple[float, float, float]] = []
        """(t, steering_rad, speed) as published on /ackermann_cmd."""
        self.states: list[tuple[float, str]] = []
        self.clearances: list[tuple[float, float, float, float]] = []
        """(t, front, left, right) in metres, from the raw scan."""

        self.button = self.create_publisher(String, "/button/event", 10)
        self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self._on_cmd, 10)
        self.create_subscription(String, "/robot_state", self._on_state, _QOS_STATE)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.t0 = time.monotonic()

    def _on_cmd(self, msg: AckermannDriveStamped) -> None:
        self.samples.append((time.monotonic() - self.t0, msg.drive.steering_angle, msg.drive.speed))

    def _on_state(self, msg: String) -> None:
        stamp = time.monotonic() - self.t0
        if not self.states or self.states[-1][1] != msg.data:
            self.states.append((stamp, msg.data))

    def _on_scan(self, msg: LaserScan) -> None:
        # Sectors in the ROBOT frame, i.e. after the same mount correction the
        # navigator applies -- otherwise "front" here would mean something
        # different from "front" there and the two logs could not be compared.
        from shared.config.constants import RobotSpecs

        offset = math.radians(RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG)
        n = len(msg.ranges)
        if n == 0:
            return
        step = (msg.angle_max - msg.angle_min) / max(1, n - 1)

        def sector(center_deg: float, half_deg: float = 20.0) -> float:
            vals = []
            for i, r in enumerate(msg.ranges):
                if not (0.05 < r < 12.0):
                    continue
                ang = math.degrees(msg.angle_min + i * step + offset)
                delta = (ang - center_deg + 540) % 360 - 180
                if abs(delta) <= half_deg:
                    vals.append(r)
            return min(vals) if vals else float("nan")

        self.clearances.append(
            (time.monotonic() - self.t0, sector(0.0), sector(90.0), sector(-90.0)),
        )

    def press(self, event: str) -> None:
        msg = String()
        msg.data = event
        self.button.publish(msg)
        self.get_logger().info(f"published {event}")


def _summarise(probe: TrackRunProbe) -> None:
    print("\n--- states ---")
    for t, s in probe.states:
        print(f"  t={t:5.1f}s  {s}")

    print("\n--- drive commands (steering + = left) ---")
    driving = [s for s in probe.samples if abs(s[2]) > 1e-6]
    if not driving:
        print("  none with non-zero speed -- the robot never drove")
    else:
        step = max(1, len(driving) // 12)
        for t, steer, speed in driving[::step]:
            bar = "L" * int(max(0.0, steer) * 20) or "R" * int(max(0.0, -steer) * 20)
            print(f"  t={t:5.1f}s  steer={math.degrees(steer):+6.1f}deg  speed={speed:+.2f}  {bar}")
        steers = [s[1] for s in driving]
        print(
            f"\n  steering: mean={math.degrees(statistics.fmean(steers)):+.1f}deg  "
            f"min={math.degrees(min(steers)):+.1f}  max={math.degrees(max(steers)):+.1f}",
        )
        left = sum(1 for s in steers if s > 0.02)
        right = sum(1 for s in steers if s < -0.02)
        print(f"  bias: {left} samples left, {right} right (of {len(steers)})")

    print("\n--- clearances (m) ---")
    if probe.clearances:
        step = max(1, len(probe.clearances) // 8)
        for t, f, left_m, right_m in probe.clearances[::step]:
            print(f"  t={t:5.1f}s  front={f:5.2f}  left={left_m:5.2f}  right={right_m:5.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=20.0, help="How long to let it drive")
    args = parser.parse_args()

    rclpy.init()
    probe = TrackRunProbe(args.seconds)
    try:
        # Let discovery settle before pressing anything, or the start event is
        # published into a graph the state machine has not joined yet.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)

        probe.press("short_press")
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)

        # Always, even if the run looked fine: nothing here should be able to
        # leave the robot driving once this script exits.
        probe.press("long_press")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
    finally:
        _summarise(probe)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

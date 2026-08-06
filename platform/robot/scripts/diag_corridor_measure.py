"""Feed a live LIDAR scan into measure_corridor_width and print what it decides.

The estimator starts from a narrow (60 cm) prior and only leaves it after
repeated agreeing measurements. On a real 1 m corridor it therefore matters a
great deal whether the measurement function actually reads 1 m -- a run that
keeps voting "narrow" in a wide corridor never corrects, and the navigator
plans against a corridor half the width of the one it is driving in.

This isolates that one function against real hardware data, rather than
inferring its behaviour from how the robot drove.

Usage (on the Pi 5, robot stationary in a corridor):
    pixi run -e vision python scripts/diag_corridor_measure.py
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from shared.config.constants import CorridorDimensions, RobotSpecs

from src.navigation.corridor_estimator import measure_corridor_width

_OFFSET_RAD = math.radians(
    (180.0 if RobotSpecs.LIDAR_INVERTED else 0.0) + RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG
)
_BOUNDARY = (CorridorDimensions.NARROW + CorridorDimensions.WIDE) / 2.0


class Probe(Node):
    """Collects one scan and one yaw, then measures."""

    def __init__(self) -> None:
        super().__init__("corridor_measure_probe")
        self.scan: LaserScan | None = None
        self.yaw: float | None = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main() -> None:
    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline and (probe.scan is None or probe.yaw is None):
        rclpy.spin_once(probe, timeout_sec=0.2)

    if probe.scan is None or probe.yaw is None:
        print(f"missing input: scan={probe.scan is not None} imu={probe.yaw is not None}")
        return

    msg = probe.scan
    ranges = np.array(msg.ranges, dtype=float)
    ranges[~np.isfinite(ranges)] = RobotSpecs.LIDAR_MAX_RANGE
    ranges = np.clip(ranges, 0.0, RobotSpecs.LIDAR_MAX_RANGE)
    angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) + _OFFSET_RAD

    def sector(center_deg: float, half_deg: float = 10.0) -> float:
        deltas = (np.degrees(angles) - center_deg + 540) % 360 - 180
        sel = (np.abs(deltas) <= half_deg) & (ranges > 0.05) & (ranges < 12.0)
        return float(np.min(ranges[sel])) if sel.any() else float("nan")

    left, right, front = sector(90.0), sector(-90.0), sector(0.0)
    print(f"yaw={math.degrees(probe.yaw):+.1f}deg")
    print(f"raw sectors: front={front:.2f}m left={left:.2f}m right={right:.2f}m")
    print(f"left+right+chassis = {left + right + RobotSpecs.WIDTH:.2f}m")

    result = measure_corridor_width(ranges.tolist(), angles.tolist(), probe.yaw)
    print(f"\nNARROW={CorridorDimensions.NARROW:.2f}m  WIDE={CorridorDimensions.WIDE:.2f}m  boundary={_BOUNDARY:.2f}m")
    if result is None:
        print("measure_corridor_width -> None (no usable measurement)")
    else:
        verdict = "WIDE" if result.width_m >= _BOUNDARY else "NARROW"
        print(f"measure_corridor_width -> {result.width_m:.2f}m  => votes {verdict}")

    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

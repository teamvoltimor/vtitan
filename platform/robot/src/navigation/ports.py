"""Hardware port owned by the navigation domain.

Declared here — not in ``hardware/`` or ``simulation/`` — so the dependency
direction matches hexagonal architecture: adapters (``ROS2HardwareGateway``,
``SimulatedHardwareGateway``) import this port, the domain never imports an
adapter package.

``LidarScan`` and ``DriveCommand`` replace the previous anonymous
``tuple[list[float], list[float]]`` scan and the twist-shaped ``Velocity``
command, which let a single steering-unit contract mean three different things
across the sim, the real motor node, and Gazebo (see CRIT-1). Every adapter
now decodes/encodes the same ``steering_norm: float  # [-1, 1], + = left``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from shared.domain.models import IMUReading, Pose, TrafficSignObservation


@dataclass(frozen=True, slots=True, repr=False)
class LidarScan:
    """A single LIDAR sweep in the robot frame (0 rad = forward, +pi/2 = left)."""

    ranges_m: tuple[float, ...]
    angles_rad: tuple[float, ...]

    def __repr__(self) -> str:
        """Ray count + range span, not every ray -- the default dataclass repr
        of a 720-ray scan is ~10,000 characters, unusable in a test failure or
        a debug print."""
        n = len(self.ranges_m)
        if n == 0:
            return "LidarScan(rays=0)"
        finite = [r for r in self.ranges_m if math.isfinite(r)]
        span = f"{min(finite):.2f}..{max(finite):.2f}m" if finite else "no finite returns"
        return f"LidarScan(rays={n}, ranges={span})"


@dataclass(frozen=True, slots=True)
class DriveCommand:
    """Motor command in the one contract every adapter must decode identically."""

    speed_mps: float
    steering_norm: float  # [-1, 1], + = left


@dataclass(frozen=True, slots=True)
class WheelOdometry:
    """Distance travelled at the wheel, and the rate it is travelling.

    Deliberately not a pose. The encoder measures wheel rotation and nothing
    else -- turning that into a position needs a heading, which lives with the
    IMU, and fusing it with a position fix needs the LIDAR. Publishing a pose
    from the wheel alone would put dead reckoning in the component with the
    least information and create a second, worse answer competing with the
    localizer's.

    ``distance_m`` accumulates from an arbitrary zero, so only differences
    between samples are meaningful. ``stamp_s`` is required for the same
    reason: integrating a distance between LIDAR scans means nothing without
    knowing when each sample was taken.
    """

    distance_m: float
    speed_mps: float
    stamp_s: float


class HardwareGateway(Protocol):
    """Interface for robot hardware interaction (ROS2 or simulation)."""

    def publish_drive(self, command: DriveCommand) -> None:
        """Command the robot to drive at the given speed and steering angle."""

    def get_current_pose(self) -> Pose | None:
        """Get the current estimated pose of the robot."""

    def get_lidar_scan(self) -> LidarScan | None:
        """Get the latest LIDAR sweep."""

    def get_imu_reading(self) -> IMUReading | None:
        """Get the latest IMU orientation."""

    def get_vision_detections(self) -> list[TrafficSignObservation]:
        """Get the latest sign observations from the camera."""

    def get_localizer_inputs(self) -> tuple[float, float, float] | None:
        """(yaw, prior_x, prior_y) last handed to the LIDAR localizer.

        Diagnostic only. The localizer solves for position alone and trusts
        the yaw it is given, so a heading wrong by pi yields a confidently
        tracked but wrong position, and the fused pose that comes back cannot
        show the mismatch. ``None`` before the first scan.
        """

    def get_wheel_odometry(self) -> WheelOdometry | None:
        """Get the latest wheel travel and speed, or ``None`` if unavailable.

        ``None`` is a normal state, not an error: a drive backend without an
        encoder has nothing to report, and on the real robot nothing has
        arrived until the first ``/joint_states`` message.
        """

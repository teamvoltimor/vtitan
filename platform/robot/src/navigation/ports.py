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

from dataclasses import dataclass
from typing import Protocol

from shared.domain.models import Detection, IMUReading, Pose


@dataclass(frozen=True, slots=True)
class LidarScan:
    """A single LIDAR sweep in the robot frame (0 rad = forward, +pi/2 = left)."""

    ranges_m: tuple[float, ...]
    angles_rad: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class DriveCommand:
    """Motor command in the one contract every adapter must decode identically."""

    speed_mps: float
    steering_norm: float  # [-1, 1], + = left


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

    def get_vision_detections(self) -> list[Detection]:
        """Get the latest object detections from the camera."""

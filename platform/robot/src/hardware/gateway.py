"""Hardware gateway protocol for dependency inversion.

Decouples core navigation logic from ROS2, allowing pure Python testing
and simulation injection.
"""

from typing import Protocol

from shared.domain.models import Detection, IMUReading, Pose, Velocity


class HardwareGateway(Protocol):
    """Interface for robot hardware interaction (ROS2 or Simulation)."""

    def publish_velocity(self, velocity: Velocity) -> None:
        """Command the robot to move with specified velocities."""

    def get_current_pose(self) -> Pose | None:
        """Get the current estimated pose of the robot."""

    def get_lidar_scan(self) -> tuple[list[float], list[float]] | None:
        """Get the latest LIDAR ranges and corresponding angles."""

    def get_imu_reading(self) -> IMUReading | None:
        """Get the latest IMU orientation."""

    def get_vision_detections(self) -> list[Detection]:
        """Get the latest object detections from the camera."""

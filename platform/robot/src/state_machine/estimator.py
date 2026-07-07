"""State estimator combining IMU heading with a LIDAR-derived world position.

Real hardware has no wheel odometry, so position comes from
:class:`~src.navigation.localization.LidarLocalizer` (an absolute fix against
known track wall geometry, not a relative delta), while heading comes from the
IMU. This replaces an earlier IMU+wheel-odometry complementary filter design;
that filter is not reinstated here because reintroducing an unfused odometry
input would resurrect the same silent-drift risk that motivated the LIDAR fix.
"""

import math

from shared.domain.models import IMUReading, Pose


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.remainder(angle, 2 * math.pi)


class StateEstimator:
    """Combines IMU heading and LIDAR-fixed position into a world-frame Pose."""

    def __init__(self, start_x: float, start_y: float, start_yaw: float) -> None:
        """Initialize the state estimator.

        Args:
            start_x: Starting X coordinate in world frame (also the position
                estimate until the first LIDAR fix arrives).
            start_y: Starting Y coordinate in world frame.
            start_yaw: Starting Yaw (heading) in world frame (also the heading
                estimate until the first IMU reading arrives).
        """
        self._start_yaw = start_yaw
        self._imu_yaw_offset: float | None = None
        self._relative_imu_yaw: float | None = None

        self._x = start_x
        self._y = start_y

    def update_imu(self, reading: IMUReading) -> None:
        """Process incoming IMU orientation data."""
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = reading.yaw

        self._relative_imu_yaw = wrap_angle(reading.yaw - self._imu_yaw_offset)

    def update_position(self, x: float, y: float) -> None:
        """Process an absolute world-frame position fix (from LIDAR localization)."""
        self._x = x
        self._y = y

    def estimate_pose(self) -> Pose:
        """Calculate and return the current fused Pose in the world frame."""
        if self._relative_imu_yaw is not None:
            world_yaw = wrap_angle(self._relative_imu_yaw + self._start_yaw)
        else:
            world_yaw = self._start_yaw

        return Pose(x=self._x, y=self._y, yaw=world_yaw)

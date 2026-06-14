"""State estimator for Sensor Fusion (IMU + Odometry).

Calculates fused yaw and world-frame position based on starting conditions
and relative odometry ticks.
"""

import math

from shared.domain.models import IMUReading, Pose


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.remainder(angle, 2 * math.pi)


class StateEstimator:
    """Fuses IMU and Odometry readings into a single world-frame Pose.

    Adheres to SOLID (Single Responsibility) by decoupling estimation math
    from the main ROS2 node logic.
    """

    def __init__(self, start_x: float, start_y: float, start_yaw: float, alpha: float = 0.1) -> None:
        """Initialize the state estimator.

        Args:
            start_x: Starting X coordinate in world frame.
            start_y: Starting Y coordinate in world frame.
            start_yaw: Starting Yaw (heading) in world frame.
            alpha: Complementary filter weight (IMU vs Odom).
        """
        self._start_x = start_x
        self._start_y = start_y
        self._start_yaw = start_yaw
        self._cos_start_yaw = math.cos(start_yaw)
        self._sin_start_yaw = math.sin(start_yaw)
        self._alpha = alpha

        self._imu_yaw_offset: float | None = None
        self._relative_imu_yaw: float | None = None

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

    def update_imu(self, reading: IMUReading) -> None:
        """Process incoming IMU orientation data."""
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = reading.yaw

        self._relative_imu_yaw = wrap_angle(reading.yaw - self._imu_yaw_offset)

    def update_odom(self, x: float, y: float, yaw: float) -> None:
        """Process incoming raw odometry from wheel encoders."""
        self._odom_x = x
        self._odom_y = y
        self._odom_yaw = yaw

    def estimate_pose(self) -> Pose:
        """Calculate and return the current fused Pose in the world frame."""
        world_x = self._start_x + self._odom_x * self._cos_start_yaw - self._odom_y * self._sin_start_yaw
        world_y = self._start_y + self._odom_x * self._sin_start_yaw + self._odom_y * self._cos_start_yaw

        if self._relative_imu_yaw is not None:
            diff = wrap_angle(self._odom_yaw - self._relative_imu_yaw)
            fused_relative_yaw = wrap_angle(self._relative_imu_yaw + self._alpha * diff)
            world_yaw = wrap_angle(fused_relative_yaw + self._start_yaw)
        else:
            # Fall back to pure odometry if IMU is offline (resiliency)
            world_yaw = wrap_angle(self._odom_yaw + self._start_yaw)

        return Pose(x=world_x, y=world_y, yaw=world_yaw)

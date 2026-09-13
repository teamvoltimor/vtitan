"""Typed IMU sensor readings.

Each reading is a frozen, slotted dataclass, matching the DTO convention used
across the rest of the hardware layer (these were ``typing.NamedTuple``).
``as_tuple()`` exists because ``base.Data`` still carries plain tuples, the
shape the ROS2 nodes unpack into ``sensor_msgs/Imu``.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Vector3Reading:
    """A 3-axis sensor reading (accelerometer, gyroscope or magnetometer)."""

    x: float
    y: float
    z: float

    def as_tuple(self) -> tuple[float, float, float]:
        """This reading as ``(x, y, z)``."""
        return (self.x, self.y, self.z)


class AccelerometerReading(Vector3Reading):
    """Accelerometer sensor Reading in m/s²."""


class GyroscopeReading(Vector3Reading):
    """Gyroscope sensor Reading in rad/s."""


class MagnetometerReading(Vector3Reading):
    """Magnetometer sensor Reading in µT."""


@dataclass(frozen=True, slots=True)
class QuaternionReading:
    """Fused quaternion Reading (w, x, y, z)."""

    w: float
    x: float
    y: float
    z: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        """This quaternion as ``(w, x, y, z)``."""
        return (self.w, self.x, self.y, self.z)


@dataclass(frozen=True, slots=True)
class EulerReading:
    """Fused Euler angles in degrees (pitch, roll, yaw)."""

    pitch: float
    roll: float
    yaw: float

    def as_tuple(self) -> tuple[float, float, float]:
        """These angles as ``(pitch, roll, yaw)``."""
        return (self.pitch, self.roll, self.yaw)


LinearAccelerometerReading = AccelerometerReading
"""Linear acceleration (gravity removed) shares the accelerometer shape."""

LinearAccelelerometerReading = LinearAccelerometerReading
"""Deprecated alias retaining the historical triple-l misspelling; use
:data:`LinearAccelerometerReading`."""


@dataclass(frozen=True, slots=True)
class RVCReading:
    """IMU Reading for ROS 2 RVC message format."""

    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    x_accel: float
    y_accel: float
    z_accel: float
    quaternion: QuaternionReading

from typing import NamedTuple


class AccelerometerReading(NamedTuple):
    """Accelerometer sensor Reading in m/s²."""

    x: float
    y: float
    z: float


class GyroscopeReading(NamedTuple):
    """Gyroscope sensor Reading in rad/s."""

    x: float
    y: float
    z: float


class MagnetometerReading(NamedTuple):
    """Magnetometer sensor Reading in µT."""

    x: float
    y: float
    z: float


class QuaternionReading(NamedTuple):
    """Fused quaternion Reading (w, x, y, z)."""

    w: float
    x: float
    y: float
    z: float


class EulerReading(NamedTuple):
    """Fused Euler angles in degrees (pitch, roll, yaw)."""

    pitch: float
    roll: float
    yaw: float


LinearAccelelerometerReading = AccelerometerReading  # Alias for linear acceleration Reading (gravity removed)


class RVCReading(NamedTuple):
    """IMU Reading for ROS 2 RVC message format."""

    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    x_accel: float
    y_accel: float
    z_accel: float
    quaternion: QuaternionReading

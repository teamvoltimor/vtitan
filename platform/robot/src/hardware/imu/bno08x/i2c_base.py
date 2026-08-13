"""Shared BNO08x driver logic for the two I2C backends (direct bus, MCP2221A bridge)."""

import logging
import time
from typing import TYPE_CHECKING, override

from adafruit_bno08x import (
    BNO08X,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

from src.hardware.imu.base import (
    Data,
    Driver as ABC_Driver,
)
from src.hardware.imu.readings import (
    AccelerometerReading,
    EulerReading,
    GyroscopeReading,
    LinearAccelelerometerReading,
    MagnetometerReading,
    QuaternionReading,
)
from src.logger.constants import DETAILS_KEY

if TYPE_CHECKING:
    import busio


class BNO08xI2CDriver(ABC_Driver):
    """Sensor-reading logic shared by both I2C backends.

    Subclasses own `config`, `connect()`, and `close()` -- those are the
    pieces that genuinely differ between talking to the I2C bus directly and
    going through the MCP2221A USB bridge.
    """

    def __init__(self, enable_sensors_delay: float = 0.1) -> None:
        self._imu: BNO08X | None = None
        self._i2c: busio.I2C | None = None
        self._enable_sensors_delay = enable_sensors_delay
        self.logger = logging.getLogger(type(self).__module__)

    @property
    def imu(self) -> BNO08X | None:
        """Get IMU instance."""
        if self._imu is None:
            self.connect()

        return self._imu

    @override
    def enable_sensors(self) -> None:
        """Enable all sensors."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot enable sensors")
            return

        self.imu.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        self.imu.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)  # Standard Quaternion
        self.imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)  # Z-axis gravity removed
        self.imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        # Allow sensors to stabilize after enabling
        time.sleep(self._enable_sensors_delay)
        self.logger.info("Sensors enabled")

    @override
    def get_accelerometer(self) -> AccelerometerReading:
        """Get accelerometer data (m/s²)."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read accelerometer")
            return AccelerometerReading(0.0, 0.0, 0.0)

        accel = self.imu.acceleration
        self.logger.debug(
            "Accelerometer read",
            extra={DETAILS_KEY: {"x": accel[0], "y": accel[1], "z": accel[2]}},
        )
        return AccelerometerReading(x=accel[0], y=accel[1], z=accel[2])

    @override
    def get_gyroscope(self) -> GyroscopeReading:
        """Get gyroscope data (rad/s)."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read gyroscope")
            return GyroscopeReading(0.0, 0.0, 0.0)

        gyro = self.imu.gyro
        self.logger.debug(
            "Gyroscope read",
            extra={DETAILS_KEY: {"x": gyro[0], "y": gyro[1], "z": gyro[2]}},
        )
        return GyroscopeReading(x=gyro[0], y=gyro[1], z=gyro[2])

    @override
    def get_magnetometer(self) -> MagnetometerReading:
        """Get magnetometer data (µT)."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read magnetometer")
            return MagnetometerReading(0.0, 0.0, 0.0)

        mag = self.imu.magnetic
        self.logger.debug(
            "Magnetometer read",
            extra={DETAILS_KEY: {"x": mag[0], "y": mag[1], "z": mag[2]}},
        )
        return MagnetometerReading(x=mag[0], y=mag[1], z=mag[2])

    @override
    def get_quaternion(self) -> QuaternionReading:
        """Get fused quaternion (w, x, y, z)."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read quaternion")
            return QuaternionReading(w=1.0, x=0.0, y=0.0, z=0.0)  # Identity quaternion as default

        # Adafruit's BNO08x .quaternion property returns (x, y, z, w).
        x, y, z, w = self.imu.quaternion

        self.logger.debug(
            "Quaternion read",
            extra={DETAILS_KEY: {"x": x, "y": y, "z": z, "w": w}},
        )
        # QuaternionReading is (w, x, y, z) -- keyword args so the fields
        # line up correctly regardless of Adafruit's (x, y, z, w) order.
        return QuaternionReading(w=w, x=x, y=y, z=z)

    @override
    def get_euler(self) -> EulerReading:
        """Get fused Euler angles (pitch, roll, yaw) in degrees."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read Euler angles")
            return EulerReading(0.0, 0.0, 0.0)

        euler = self.imu.euler
        self.logger.debug(
            "Euler read",
            extra={DETAILS_KEY: {"pitch": euler[0], "roll": euler[1], "yaw": euler[2]}},
        )
        return EulerReading(pitch=euler[0], roll=euler[1], yaw=euler[2])

    @override
    def get_linear_acceleration(self) -> LinearAccelelerometerReading:
        """Get linear acceleration (m/s², gravity removed)."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read linear acceleration")
            return AccelerometerReading(0.0, 0.0, 0.0)

        linear = self.imu.linear_acceleration
        self.logger.debug(
            "Linear accel read",
            extra={DETAILS_KEY: {"x": linear[0], "y": linear[1], "z": linear[2]}},
        )
        return AccelerometerReading(x=linear[0], y=linear[1], z=linear[2])

    @override
    def get_all_data(self) -> Data:
        """Get all sensor data."""
        return Data(
            accelerometer=self.get_accelerometer(),
            gyroscope=self.get_gyroscope(),
            magnetometer=self.get_magnetometer(),
            quaternion=self.get_quaternion(),
            euler=self.get_euler(),
            linear_accel=self.get_linear_acceleration(),
        )

    def get_calibration_status(self) -> dict | None:
        """Get calibration status."""
        if self.imu is None:
            self.logger.warning("IMU not connected, cannot read calibration status")
            return None

        try:
            cal = self.imu.calibration_status
            self.logger.info("Calibration status read", extra={DETAILS_KEY: {"calibration": cal}})
        except AttributeError:
            return None
        else:
            # cal is an untyped Adafruit attribute.
            return cal  # type: ignore[no-any-return]

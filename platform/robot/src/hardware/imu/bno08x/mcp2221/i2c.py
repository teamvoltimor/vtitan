"""Hardware driver for BNO08x IMU via MCP2221A I2C bridge."""

import logging
import time
from dataclasses import dataclass
from typing import override

import board
import busio
from adafruit_bno08x import (
    BNO08X,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

from src.env import EnvVar
from src.hardware.imu.base import (
    Data,
    Driver as ABC_Driver,
)
from src.logger import configure_json_logging

configure_json_logging()


IMU_I2C_ADDRESS = EnvVar[int](
    key="IMU_I2C_ADDRESS",
    default=0x4A,
    cast=lambda x: int(x, 0) if x.startswith("0x") else int(x),
)
"""
I2C address for the BNO08x IMU. The default address is 0x4A when the ADR pin is high, and 0x4B when the ADR pin is low.
Ensure that the ADR pin on your BNO08x board is set accordingly to match this address.
"""


@dataclass
class Config:
    """Configuration for BNO08x via MCP2221A I2C."""

    i2c_address: int = IMU_I2C_ADDRESS.value


class Driver(ABC_Driver):
    """Driver for BNO08x IMU via MCP2221A I2C bridge."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._imu = None
        self._i2c = None
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to IMU via MCP2221A I2C using Blinka."""
        self.logger.info(
            "Connecting to BNO08x via MCP2221A I2C (Blinka)",
            extra={"details": {"i2c_address": hex(self.config.i2c_address)}},
        )

        try:
            # Blinka automatically detects and uses MCP2221A via USB HID
            # Create I2C bus using Blinka (handles MCP2221A USB communication)
            self._i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize BNO08x with the I2C bus
            self._imu: BNO08X = BNO08X_I2C(self._i2c, address=self.config.i2c_address)
            self.logger.info("Connected to BNO08x IMU via MCP2221A I2C")
        except (RuntimeError, OSError) as e:
            self.logger.error(f"Failed to connect to IMU: {type(e).__name__}: {e}")
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error connecting to IMU: {e}", exc_info=True)
            raise

    @override
    def close(self) -> None:
        """Release the I2C bus and clear the IMU handle."""
        self.logger.info("Closing connection to BNO08x I2C")
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                self.logger.warning("Error during I2C deinit", exc_info=True)
        self._i2c = None
        self._imu = None
        self.logger.info("Connection closed")

    @property
    def imu(self):
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

        time.sleep(0.1)
        self.logger.info("Sensors enabled")

    @override
    def get_accelerometer(self) -> tuple[float, float, float]:
        """Get accelerometer data (m/s²)."""
        accel = self.imu.acceleration
        self.logger.debug("Accelerometer read", extra={"details": {"x": accel[0], "y": accel[1], "z": accel[2]}})
        return tuple(accel)

    @override
    def get_gyroscope(self) -> tuple[float, float, float]:
        """Get gyroscope data (rad/s)."""
        gyro = self.imu.gyro
        self.logger.debug("Gyroscope read", extra={"details": {"x": gyro[0], "y": gyro[1], "z": gyro[2]}})
        return tuple(gyro)

    @override
    def get_magnetometer(self) -> tuple[float, float, float]:
        """Get magnetometer data (µT)."""
        mag = self.imu.magnetic
        self.logger.debug("Magnetometer read", extra={"details": {"x": mag[0], "y": mag[1], "z": mag[2]}})
        return tuple(mag)

    @override
    def get_quaternion(self) -> tuple[float, float, float, float]:
        """Get fused quaternion (x, y, z, w)."""
        # Unpack correctly based on Adafruit's return order
        x, y, z, w = self.imu.quaternion

        self.logger.debug(
            "Quaternion read",
            extra={"details": {"x": x, "y": y, "z": z, "w": w}},
        )
        # Return explicitly in the ROS 2 expected order
        return (x, y, z, w)

    @override
    def get_euler(self) -> tuple[float, float, float]:
        """Get fused Euler angles (pitch, roll, yaw) in degrees."""
        euler = self.imu.euler
        self.logger.debug("Euler read", extra={"details": {"pitch": euler[0], "roll": euler[1], "yaw": euler[2]}})
        return tuple(euler)

    @override
    def get_linear_acceleration(self) -> tuple[float, float, float]:
        """Get linear acceleration (m/s², gravity removed)."""
        linear = self.imu.linear_acceleration
        self.logger.debug("Linear accel read", extra={"details": {"x": linear[0], "y": linear[1], "z": linear[2]}})
        return tuple(linear)

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
        try:
            cal = self.imu.calibration_status
            self.logger.info("Calibration status read", extra={"details": {"calibration": cal}})
            return cal
        except AttributeError:
            return None

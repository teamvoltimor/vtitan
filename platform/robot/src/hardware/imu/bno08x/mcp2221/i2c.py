"""
Hardware driver for BNO08x IMU via MCP2221A I2C bridge.
"""

import logging
import time
from dataclasses import dataclass
from typing import override

import serial
import serial.tools.list_ports

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
IMU_I2C_PORT = EnvVar[str](key="IMU_I2C_PORT", default="")
IMU_I2C_MCP2221_VID = EnvVar[int](
    key="IMU_I2C_MCP2221_VID", default=0x04D8, cast=lambda x: int(x, 0) if x.startswith("0x") else int(x)
)
IMU_I2C_MCP2221_PID = EnvVar[int](
    key="IMU_I2C_MCP2221_PID", default=0x00DD, cast=lambda x: int(x, 0) if x.startswith("0x") else int(x)
)


@dataclass
class Config:
    """Configuration for BNO08x via MCP2221A I2C."""

    i2c_address: int = IMU_I2C_ADDRESS.value
    port: str = IMU_I2C_PORT.value
    mcp2221_vid: int = IMU_I2C_MCP2221_VID.value
    mcp2221_pid: int = IMU_I2C_MCP2221_PID.value


class Driver(ABC_Driver):
    """Driver for BNO08x IMU via MCP2221A I2C bridge."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._imu = None
        self._i2c = None
        self.logger = logging.getLogger(__name__)

    def find_mcp2221_port(self) -> str | None:
        """Auto-detect MCP2221 USB bridge port."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == self.config.mcp2221_vid and port.pid == self.config.mcp2221_pid:
                self.logger.info("Found MCP2221", extra={"details": {"port": port.device}})
                return port.device
        return None

    def connect(self) -> None:
        """Connect to IMU via MCP2221A I2C."""
        try:
            from pyftdi.i2c import I2cController
        except ImportError:
            self.logger.error("pyftdi not installed. Install with: pip install pyftdi")
            raise

        port = self.config.port
        if not port:
            port = self.find_mcp2221_port()
            if not port:
                self.logger.warning("MCP2221 auto-detect failed, using /dev/ttyACM0")
                port = "/dev/ttyACM0"

        self.logger.info(
            "Connecting to BNO08x via MCP2221",
            extra={"details": {"port": port, "i2c_address": hex(self.config.i2c_address)}},
        )

        try:
            # Initialize I2C controller via MCP2221
            i2c_controller = I2cController()
            i2c_controller.configure(port)
            self._i2c = i2c_controller.get_port(self.config.i2c_address)

            from adafruit_bno08x import BNO08X_I2C

            self._imu = BNO08X_I2C(self._i2c, address=self.config.i2c_address)
            self.logger.info("Connected to BNO08x IMU")
        except Exception as e:
            self.logger.error(f"Failed to connect to IMU: {e}")
            raise

    @property
    def imu(self):
        """Get IMU instance."""
        if self._imu is None:
            self.connect()
        return self._imu

    @override
    def enable_sensors(self) -> None:
        """Enable all sensors."""
        self.imu.enable_accel()
        self.imu.enable_gyro()
        self.imu.enable_mag()
        self.imu.enable_quaternion()
        self.imu.enable_game_rotation_vector()
        self.imu.enable_linear_accel()
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
        """Get fused quaternion (w, x, y, z)."""
        quat = self.imu.quaternion
        w, x, y, z = quat
        magnitude = (w**2 + x**2 + y**2 + z**2) ** 0.5
        self.logger.debug(
            "Quaternion read", extra={"details": {"w": w, "x": x, "y": y, "z": z, "magnitude": magnitude}}
        )
        return tuple(quat)

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

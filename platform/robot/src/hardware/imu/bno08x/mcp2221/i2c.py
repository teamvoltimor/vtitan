"""Hardware driver for BNO08x IMU via MCP2221A I2C bridge."""

import logging
import time
from typing import Any, override

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
from pydantic import Field, field_validator
from pydantic_settings import SettingsConfigDict

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
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging

configure_json_logging()


class Config(HardwareBaseSettings):
    """Configuration for BNO08x via MCP2221A I2C."""

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "imu" / "bno08x_mcp2221_i2c.toml")

    i2c_address: int = Field(default=0x4A, validation_alias="IMU_I2C_ADDRESS")
    """I2C address for the BNO08x IMU. The default address is 0x4A when the ADR pin is high, and 0x4B when
    the ADR pin is low. Ensure that the ADR pin on your BNO08x board is set accordingly to match this address."""

    @field_validator("i2c_address", mode="before")
    @classmethod
    def _parse_int_literal(cls, value: Any) -> Any:
        """Accept "0x4A"-style hex literals as well as plain decimal strings."""
        return int(value, 0) if isinstance(value, str) else value


class Driver(ABC_Driver):
    """Driver for BNO08x IMU via MCP2221A I2C bridge."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._imu: BNO08X | None = None
        self._i2c: busio.I2C | None = None
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
            self._imu = BNO08X_I2C(self._i2c, address=self.config.i2c_address)
            self.logger.info("Connected to BNO08x IMU via MCP2221A I2C")
        except (RuntimeError, OSError):
            self.logger.exception("Failed to connect to IMU")
            raise
        except Exception:
            self.logger.exception("Unexpected error connecting to IMU")
            raise

    @override
    def close(self) -> None:
        """Release the I2C bus and clear the IMU handle."""
        self.logger.info("Closing connection to BNO08x I2C")
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:  # noqa: BLE001 - cleanup must never raise
                self.logger.warning("Error during I2C deinit", exc_info=True)
        self._i2c = None
        self._imu = None
        self.logger.info("Connection closed")

    @property
    def imu(self) -> BNO08X | None:
        """Get IMU instance."""
        if self._imu is None:
            self.connect()
        return self._imu

    @override
    def enable_sensors(self) -> None:
        """Enable all sensors."""
        imu = self.imu
        if imu is None:
            self.logger.warning("IMU not connected, cannot enable sensors")
            return

        imu.enable_feature(BNO_REPORT_ACCELEROMETER)
        imu.enable_feature(BNO_REPORT_GYROSCOPE)
        imu.enable_feature(BNO_REPORT_MAGNETOMETER)
        imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)  # Standard Quaternion
        imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)  # Z-axis gravity removed
        imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        time.sleep(0.1)
        self.logger.info("Sensors enabled")

    @override
    def get_accelerometer(self) -> AccelerometerReading:
        """Get accelerometer data (m/s²)."""
        assert self.imu is not None
        accel = self.imu.acceleration
        self.logger.debug("Accelerometer read", extra={"details": {"x": accel[0], "y": accel[1], "z": accel[2]}})
        return AccelerometerReading(*accel)

    @override
    def get_gyroscope(self) -> GyroscopeReading:
        """Get gyroscope data (rad/s)."""
        assert self.imu is not None
        gyro = self.imu.gyro
        self.logger.debug("Gyroscope read", extra={"details": {"x": gyro[0], "y": gyro[1], "z": gyro[2]}})
        return GyroscopeReading(*gyro)

    @override
    def get_magnetometer(self) -> MagnetometerReading:
        """Get magnetometer data (µT)."""
        assert self.imu is not None
        mag = self.imu.magnetic
        self.logger.debug("Magnetometer read", extra={"details": {"x": mag[0], "y": mag[1], "z": mag[2]}})
        return MagnetometerReading(*mag)

    @override
    def get_quaternion(self) -> QuaternionReading:
        """Get fused quaternion (w, x, y, z)."""
        assert self.imu is not None
        # Adafruit's BNO08x .quaternion property returns (x, y, z, w).
        x, y, z, w = self.imu.quaternion

        self.logger.debug(
            "Quaternion read",
            extra={"details": {"x": x, "y": y, "z": z, "w": w}},
        )
        # QuaternionReading is (w, x, y, z) -- keyword args so the fields
        # line up correctly regardless of Adafruit's (x, y, z, w) order.
        return QuaternionReading(w=w, x=x, y=y, z=z)

    @override
    def get_euler(self) -> EulerReading:
        """Get fused Euler angles (pitch, roll, yaw) in degrees."""
        assert self.imu is not None
        euler = self.imu.euler
        self.logger.debug("Euler read", extra={"details": {"pitch": euler[0], "roll": euler[1], "yaw": euler[2]}})
        return EulerReading(*euler)

    @override
    def get_linear_acceleration(self) -> LinearAccelelerometerReading:
        """Get linear acceleration (m/s², gravity removed)."""
        assert self.imu is not None
        linear = self.imu.linear_acceleration
        self.logger.debug("Linear accel read", extra={"details": {"x": linear[0], "y": linear[1], "z": linear[2]}})
        return LinearAccelelerometerReading(*linear)

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
        imu = self.imu
        if imu is None:
            return None
        try:
            cal = imu.calibration_status
            self.logger.info("Calibration status read", extra={"details": {"calibration": cal}})
        except AttributeError:
            return None
        else:
            # cal is an untyped Adafruit attribute.
            return cal  # type: ignore[no-any-return]

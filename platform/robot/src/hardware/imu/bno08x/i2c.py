"""Hardware driver for BNO08x IMU via I2C."""

import logging
import threading
import time
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
from pydantic import Field, field_validator
from pydantic_settings import SettingsConfigDict

from src.hardware.exceptions import IMUConnectionError
from src.hardware.imu.base import (
    Driver as ABC_Driver,
)
from src.hardware.imu.config import QuaternionConfig
from src.hardware.imu.readings import (
    AccelerometerReading,
    EulerReading,
    GyroscopeReading,
    MagnetometerReading,
    QuaternionReading,
)
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

configure_json_logging()


class Config(HardwareBaseSettings):
    """Configuration for BNO08x IMU over I2C."""

    model_config = SettingsConfigDict(
        env_prefix="bno08x_i2c_",
        # "__" so nested leaves with underscores parse, e.g.
        # BNO08X_I2C_QUATERNION__NEGATE_YAW -> quaternion.negate_yaw.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "imu" / "bno08x_i2c.toml",
    )

    # QuaternionConfig's negate_yaw/pitch/roll have no defaults -- this factory
    # only succeeds when the nested QUATERNION__* env vars are set; mypy
    # can't see that env resolution, hence the ignore.
    quaternion: QuaternionConfig = Field(
        default_factory=lambda: QuaternionConfig(),  # type: ignore[call-arg]
    )

    address: int
    """I2C address of the BNO08x IMU (0x4A or 0x4B depending on ADR pin)"""

    enable_sensors_delay: float = 0.5
    """Delay in seconds after enabling sensors to allow them to stabilize"""

    @field_validator("address", mode="before")
    @classmethod
    def _parse_address(cls, value: object) -> object:
        """Coerce a hex/decimal string address (e.g. "0x4A") to int."""
        if isinstance(value, str):
            return int(value, 0)
        return value


class Driver(ABC_Driver):
    """Driver for BNO08x IMU via I2C bridge."""

    def __init__(self, config: Config | None = None):
        """
        Initialize driver with optional configuration.

        Args:
            config (I2CConfig | None): Configuration for I2C connection and sensor settings.
                If None, resolved from env vars (address is required, no default).
        """
        # address is required with no default -- resolved from an env var
        # when config isn't passed explicitly; mypy can't see that.
        self.config: Config = config or Config()  # type: ignore[call-arg]
        self._imu: BNO08X | None = None
        self._i2c: busio.I2C | None = None
        self._conn_lock: threading.Lock = threading.Lock()
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to IMU via I2C."""
        # Acquire lock to prevent concurrent connections
        self._conn_lock.acquire()

        self.logger.info(
            "Connecting to BNO08x via I2C (Blinka)",
            extra={DETAILS_KEY: {"i2c_address": hex(self.config.address)}},
        )

        try:
            # If using MCP2221A, Blinka will automatically detect it as a USB HID device and provide I2C access via busio
            # Remember that if that's the case, BLINKA_MCP2221 env var must be set to "1" and the MCP2221A must be properly connected to the I2C bus with correct wiring and power.
            self._i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize BNO08x with the I2C bus
            self._imu = BNO08X_I2C(self._i2c, address=self.config.address)
            self.logger.info("Connected to BNO08x IMU via I2C")
        except (RuntimeError, OSError) as e:
            self.logger.exception("Failed to connect to IMU", extra={DETAILS_KEY: {"error": str(e)}})
            raise IMUConnectionError(
                hex(self.config.address),
                "Connection failed (check wiring, power, and I2C address)",
            ) from e
        except Exception as e:
            self.logger.exception(
                "Unexpected error connecting to IMU",
                extra={DETAILS_KEY: {"error": str(e)}},
            )
            raise IMUConnectionError(
                hex(self.config.address),
                f"Unexpected connection error: {type(e).__name__}",
            ) from e
        finally:
            # Release lock regardless of success or failure to allow retries
            self._conn_lock.release()

    @property
    def imu(self) -> BNO08X | None:
        """Get IMU instance."""
        if self._imu is None:
            self.connect()

        return self._imu

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
        time.sleep(self.config.enable_sensors_delay)
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
    def get_linear_acceleration(self) -> AccelerometerReading:
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

    @override
    def close(self) -> None:
        """Close connection."""
        # Acquire lock to prevent concurrent access during close
        self._conn_lock.acquire()
        self.logger.info("Closing connection to IMU")

        if self._i2c is not None:
            self._i2c.deinit()
            self._i2c = None

        self._imu = None
        self.logger.info("Connection closed")

        # Release lock after closing connection
        self._conn_lock.release()

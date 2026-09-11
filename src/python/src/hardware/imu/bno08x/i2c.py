"""Hardware driver for BNO08x IMU via I2C."""

import threading
from typing import override

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from pydantic import Field
from pydantic_settings import SettingsConfigDict

from src.hardware.exceptions import IMUConnectionError
from src.hardware.imu.bno08x.i2c_base import BNO08xI2CDriver
from src.hardware.imu.config import QuaternionConfig
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings, HexInt
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

    quaternion: QuaternionConfig = Field(default_factory=QuaternionConfig)

    address: HexInt
    """I2C address of the BNO08x IMU (0x4A or 0x4B depending on ADR pin)"""

    enable_sensors_delay: float = 0.5
    """Delay in seconds after enabling sensors to allow them to stabilize"""


class Driver(BNO08xI2CDriver):
    """Driver for BNO08x IMU via I2C bridge."""

    def __init__(self, config: Config | None = None):
        """
        Initialize driver with optional configuration.

        Args:
            config (Config | None): Configuration for I2C connection and sensor settings.
                If None, resolved from env vars (address is required, no default).
        """
        # address is required with no default -- resolved from an env var
        # when config isn't passed explicitly; mypy can't see that.
        self.config: Config = config or Config()  # type: ignore[call-arg]
        super().__init__(enable_sensors_delay=self.config.enable_sensors_delay)
        self._conn_lock: threading.Lock = threading.Lock()

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

"""Hardware driver for BNO08x IMU via MCP2221A I2C bridge."""

from typing import override

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.imu.bno08x_mcp2221_i2c_schema import (
    HardwareImuBno08xMcp2221I2c,
)

from src.hardware.imu.bno08x.i2c_base import BNO08xI2CDriver
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings, parse_hex_int
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

configure_json_logging()


class Config(HardwareBaseSettings, HardwareImuBno08xMcp2221I2c):
    """Configuration for BNO08x via MCP2221A I2C.

    Subclasses the generated DTO: ``imu_i2c_address`` is the schema's hex
    string (the TOML spelling). The int the Blinka/Adafruit call needs is
    exposed as :attr:`imu_i2c_address_int`.
    """

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "imu" / "bno08x_mcp2221_i2c.toml")

    @property
    def imu_i2c_address_int(self) -> int:
        """``imu_i2c_address`` parsed as an int (accepts hex/octal/decimal strings)."""
        return parse_hex_int(self.imu_i2c_address)


class Driver(BNO08xI2CDriver):
    """Driver for BNO08x IMU via MCP2221A I2C bridge."""

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config.load()
        super().__init__()

    def connect(self) -> None:
        """Connect to IMU via MCP2221A I2C using Blinka."""
        self.logger.info(
            "Connecting to BNO08x via MCP2221A I2C (Blinka)",
            extra={DETAILS_KEY: {"i2c_address": hex(self.config.imu_i2c_address_int)}},
        )

        try:
            # Blinka automatically detects and uses MCP2221A via USB HID
            # Create I2C bus using Blinka (handles MCP2221A USB communication)
            self._i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize BNO08x with the I2C bus
            self._imu = BNO08X_I2C(self._i2c, address=self.config.imu_i2c_address_int)
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
            except Exception:
                self.logger.warning("Error during I2C deinit", exc_info=True)
        self._i2c = None
        self._imu = None
        self.logger.info("Connection closed")

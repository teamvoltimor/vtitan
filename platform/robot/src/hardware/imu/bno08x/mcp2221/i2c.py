"""Hardware driver for BNO08x IMU via MCP2221A I2C bridge."""

from typing import override

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict

from src.hardware.imu.bno08x.i2c_base import BNO08xI2CDriver
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings, HexInt
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

configure_json_logging()


class Config(HardwareBaseSettings):
    """Configuration for BNO08x via MCP2221A I2C."""

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "imu" / "bno08x_mcp2221_i2c.toml")

    i2c_address: HexInt = Field(default=0x4A, validation_alias=AliasChoices("IMU_I2C_ADDRESS", "imu_i2c_address"))
    """I2C address for the BNO08x IMU. The default address is 0x4A when the ADR pin is high, and 0x4B when
    the ADR pin is low. Ensure that the ADR pin on your BNO08x board is set accordingly to match this address."""


class Driver(BNO08xI2CDriver):
    """Driver for BNO08x IMU via MCP2221A I2C bridge."""

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config()
        super().__init__()

    def connect(self) -> None:
        """Connect to IMU via MCP2221A I2C using Blinka."""
        self.logger.info(
            "Connecting to BNO08x via MCP2221A I2C (Blinka)",
            extra={DETAILS_KEY: {"i2c_address": hex(self.config.i2c_address)}},
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
            except Exception:
                self.logger.warning("Error during I2C deinit", exc_info=True)
        self._i2c = None
        self._imu = None
        self.logger.info("Connection closed")

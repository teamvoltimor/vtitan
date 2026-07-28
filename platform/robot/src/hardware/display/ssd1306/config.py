from typing import Annotated

from pydantic import BeforeValidator
from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


def _parse_int(value: object) -> object:
    """Parse ints given as hex/octal/binary strings (e.g. env var "0x3C")."""
    if isinstance(value, str):
        return int(value, 0)
    return value


class Config(HardwareBaseSettings):
    """Configuration for SSD1306 OLED display driver."""

    model_config = SettingsConfigDict(
        env_prefix="SSD1306_",
        toml_file=CONFIG_DIR / "display" / "ssd1306.toml",
    )

    width: int = 128
    """"Display width in pixels. Default is 128 for SSD1306."""

    height: int = 64
    """Display height in pixels. Default is 64 for SSD1306."""

    i2c_address: Annotated[int, BeforeValidator(_parse_int)] = 0x3C
    """I2C address of the display. Default is 0x3C."""

    i2c_bus: int = 1
    """I2C bus number. Default is 1 (typically /dev/i2c-1 on Raspberry Pi)."""

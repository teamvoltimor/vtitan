from pydantic_settings import BaseSettings, SettingsConfigDict


class Config(BaseSettings):
    """Configuration for SSD1306 OLED display driver."""

    model_config = SettingsConfigDict(
        env_prefix="SSD1306_",
    )

    width: int = 128
    """"Display width in pixels. Default is 128 for SSD1306."""

    height: int = 64
    """Display height in pixels. Default is 64 for SSD1306."""

    i2c_address: str | int = 0x3C
    """I2C address of the display. Default is 0x3C."""

    i2c_bus: int = 1
    """I2C bus number. Default is 1 (typically /dev/i2c-1 on Raspberry Pi)."""

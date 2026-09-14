from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.display.ssd1306_schema import HardwareDisplaySsd1306

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings, HardwareDisplaySsd1306):
    """Configuration for SSD1306 OLED display driver.

    Subclasses the generated DTO purely to attach the TOML/env settings
    wiring; the field declarations (and their descriptions) are generated. The
    schema types ``i2c_address`` as a hex string (the TOML spelling); the int
    the I2C callers need is exposed as :attr:`i2c_address_int`.
    """

    model_config = SettingsConfigDict(
        env_prefix="SSD1306_",
        toml_file=CONFIG_DIR / "display" / "ssd1306.toml",
    )

    @property
    def i2c_address_int(self) -> int:
        """``i2c_address`` parsed as an int (accepts hex/octal/decimal strings)."""
        return int(self.i2c_address, 0)

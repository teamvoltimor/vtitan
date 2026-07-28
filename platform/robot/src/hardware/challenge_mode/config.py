from pydantic import Field
from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings):
    """Configuration for the challenge-mode jumper GPIO driver."""

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "challenge_mode.toml")

    gpio_pin: int = Field(default=23, validation_alias="CHALLENGE_MODE_GPIO_PIN")
    """GPIO pin (BCM numbering) for the challenge-mode jumper (default: GPIO23 / physical pin 16)."""

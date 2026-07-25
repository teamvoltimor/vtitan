from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Config(BaseSettings):
    """Configuration for the challenge-mode jumper GPIO driver."""

    model_config = SettingsConfigDict(env_prefix="")

    gpio_pin: int = Field(default=23, validation_alias="CHALLENGE_MODE_GPIO_PIN")
    """GPIO pin (BCM numbering) for the challenge-mode jumper (default: GPIO23 / physical pin 16)."""

from pydantic import Field
from pydantic_settings import SettingsConfigDict

from src.hardware.button.config import Config as ButtonBaseConfig
from src.hardware.mcp2221.config import MCP2221Config
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings):
    """Configuration for MCP2221A button driver."""

    model_config = SettingsConfigDict(
        env_prefix="",
        # "__" so nested leaves with underscores parse, e.g.
        # BUTTON__PULL_UP -> button.pull_up, MCP2221__VID -> mcp2221.vid.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "button" / "mcp2221.toml",
    )

    mcp2221: MCP2221Config = Field(default_factory=MCP2221Config)
    """MCP2221 USB bridge configuration."""

    gpio_pin: int = Field(validation_alias="MCP2221_BUTTON_GPIO_PIN")
    """MCP2221A GPIO channel the button is wired to (0-3, i.e. GP0-GP3)."""

    button: ButtonBaseConfig = Field(default_factory=ButtonBaseConfig)
    """Button-specific configuration."""

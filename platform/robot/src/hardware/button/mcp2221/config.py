from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from src.hardware.button.config import Config as ButtonBaseConfig
from src.hardware.mcp2221.config import MCP2221Config


class Config(BaseSettings):
    """Configuration for MCP2221A button driver."""

    model_config = SettingsConfigDict(
        env_prefix="",
        # "__" so nested leaves with underscores parse, e.g.
        # BUTTON__PULL_UP -> button.pull_up, MCP2221__VID -> mcp2221.vid.
        env_nested_delimiter="__",
    )

    mcp2221: MCP2221Config = Field(default_factory=MCP2221Config)
    """MCP2221 USB bridge configuration."""

    gpio_pin: int = Field(validation_alias="MCP2221_BUTTON_GPIO_PIN")
    """MCP2221A GPIO channel the button is wired to (0-3, i.e. GP0-GP3)."""

    # ButtonBaseConfig has no defaults for pull_up/debounce_ms/long_press_threshold_sec
    # -- this factory only succeeds when the nested BUTTON__* env vars are
    # set; mypy can't see that env resolution, hence the ignore.
    button: ButtonBaseConfig = Field(default_factory=lambda: ButtonBaseConfig())  # type: ignore[call-arg]
    """Button-specific configuration."""

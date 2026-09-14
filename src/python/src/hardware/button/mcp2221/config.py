"""MCP2221A button driver configuration."""

from pydantic import Field
from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.button.mcp2221_schema import HardwareButtonMcp2221

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings, HardwareButtonMcp2221):
    """Configuration for MCP2221A button driver.

    Subclasses the generated DTO for the TOML-backed ``mcp2221``/``button``
    groups. ``gpio_pin`` has no key in button/mcp2221.toml (it is deliberately
    env-only), so it stays a wrapper field.
    """

    model_config = SettingsConfigDict(
        env_prefix="",
        # "__" so nested leaves with underscores parse, e.g.
        # BUTTON__PULL_UP -> button.pull_up, MCP2221__VID -> mcp2221.vid.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "button" / "mcp2221.toml",
    )

    gpio_pin: int = Field(validation_alias="MCP2221_BUTTON_GPIO_PIN")
    """MCP2221A GPIO channel the button is wired to (0-3, i.e. GP0-GP3)."""

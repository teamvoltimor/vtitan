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

    button: ButtonBaseConfig = Field(default_factory=ButtonBaseConfig)
    """Button-specific configuration."""

"""Shared MCP2221A USB-bridge configuration."""

from __future__ import annotations

from typing import Any, ClassVar

from shared.config.defaults_model import DefaultsModel
from shared.config.generated._models import Mcp2221

from src.hardware.settings_base import parse_hex_int


class MCP2221Config(DefaultsModel, Mcp2221):
    """Base configuration shared by all MCP2221A drivers.

    Subclasses the generated ``Mcp2221`` DTO, whose ``vid``/``pid`` are the
    schema's hex strings (the TOML spelling). The ints the USB-port matchers
    need are exposed as :attr:`vid_int` / :attr:`pid_int`; the old hand-written
    defaults are re-applied as wrapper fallbacks, not as DTO fields.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {"vid": "0x04D8", "pid": "0x00DD"}

    @property
    def vid_int(self) -> int:
        """USB vendor ID parsed as an int (accepts hex/octal/decimal strings)."""
        return parse_hex_int(self.vid)

    @property
    def pid_int(self) -> int:
        """USB product ID parsed as an int (accepts hex/octal/decimal strings)."""
        return parse_hex_int(self.pid)

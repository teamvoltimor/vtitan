"""Shared MCP2221A USB-bridge configuration."""

from __future__ import annotations

from shared.config.generated._models import Mcp2221

from src.hardware.settings_base import parse_hex_int


class MCP2221Config(Mcp2221):
    """Base configuration shared by all MCP2221A drivers.

    Subclasses the generated ``Mcp2221`` DTO, whose ``vid``/``pid`` are the
    schema's hex strings (the TOML spelling). The ints the USB-port matchers
    need are exposed as :attr:`vid_int` / :attr:`pid_int`. Values come from the
    owning driver's TOML ``[mcp2221]`` section.
    """

    @property
    def vid_int(self) -> int:
        """USB vendor ID parsed as an int (accepts hex/octal/decimal strings)."""
        return parse_hex_int(self.vid)

    @property
    def pid_int(self) -> int:
        """USB product ID parsed as an int (accepts hex/octal/decimal strings)."""
        return parse_hex_int(self.pid)

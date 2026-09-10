from pydantic import BaseModel, Field

from src.hardware.settings_base import HexInt


class MCP2221Config(BaseModel):
    """Base configuration shared by all MCP2221A drivers."""

    vid: HexInt = Field(default=0x04D8)
    """USB Vendor ID (VID) for MCP2221. Default is 0x04D8 (Microchip). Accepts hex strings."""

    pid: HexInt = Field(default=0x00DD)
    """USB Product ID (PID) for MCP2221. Default is 0x00DD. Accepts hex strings."""

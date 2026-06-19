from typing import Annotated

from pydantic import BaseModel, BeforeValidator, Field


def _parse_int(value: object) -> object:
    """Coerce hex/octal/decimal strings (e.g. "0x04D8") to int; pass through ints."""
    if isinstance(value, str):
        return int(value, 0)
    return value


HexInt = Annotated[int, BeforeValidator(_parse_int)]


class MCP2221Config(BaseModel):
    """Base configuration shared by all MCP2221A drivers."""

    vid: HexInt = Field(default=0x04D8)
    """USB Vendor ID (VID) for MCP2221. Default is 0x04D8 (Microchip). Accepts hex strings."""

    pid: HexInt = Field(default=0x00DD)
    """USB Product ID (PID) for MCP2221. Default is 0x00DD. Accepts hex strings."""

from pydantic import BaseModel, Field


class MCP2221Config(BaseModel):
    """Base configuration shared by all MCP2221A drivers."""

    vid: int = Field(default=0x04D8)
    """USB Vendor ID (VID) for MCP2221. Default is 0x04D8 (Microchip)."""

    pid: int = Field(default=0x00DD)
    """USB Product ID (PID) for MCP2221. Default is 0x00DD."""

    def __post_init__(self):
        if self.vid is not None:
            self.vid = int(self.vid, 0) if isinstance(self.vid, str) else self.vid
        if self.pid is not None:
            self.pid = int(self.pid, 0) if isinstance(self.pid, str) else self.pid

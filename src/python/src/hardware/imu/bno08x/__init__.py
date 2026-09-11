"""
Hardware drivers for BNO08x IMU.

Hierarchy:
    IMU (abstract)
        └── BNO08x (sensor chip)
            ├── MCP2221A (USB bridge)
            ├── I2C (protocol)
            └── UART RVC (protocol)
"""

from src.hardware.imu.bno08x.i2c import (
    Config as I2CConfig,
    Driver as I2CDriver,
)
from src.hardware.imu.bno08x.mcp2221 import (
    UART_RVCConfig as MCP2221_UART_RVCConfig,
    UART_RVCDriver as MCP2221_UART_RVCDriver,
)
from src.hardware.imu.bno08x.uart_rvc import (
    Config as UART_RVCConfig,
    Driver as UART_RVCDriver,
)

__all__ = [
    "I2CConfig",
    "I2CDriver",
    "MCP2221_UART_RVCConfig",
    "MCP2221_UART_RVCDriver",
    "UART_RVCConfig",
    "UART_RVCDriver",
]

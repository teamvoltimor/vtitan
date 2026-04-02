"""
Hardware drivers for BNO08x IMU.

Hierarchy:
    IMU (abstract)
        └── BNO08x (sensor chip)
"""

from src.hardware.imu.bno08x.mcp2221 import (
    I2CConfig as MCP2221_I2CConfig,
    I2CDriver as MCP2221_I2CDriver,
    UART_RVCConfig as MCP2221_UART_RVCConfig,
    UART_RVCDriver as MCP2221_UART_RVCDriver,
)

__all__ = [
    "MCP2221_I2CConfig",
    "MCP2221_I2CDriver",
    "MCP2221_UART_RVCDriver",
    "MCP2221_UART_RVCConfig",
]

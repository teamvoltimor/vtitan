"""
Hardware drivers for BNO08x IMU via MCP2221A USB bridge.

Hierarchy:
    IMU (abstract)
        └── BNO08x (sensor chip)
            └── MCP2221A (USB bridge)
                  ├── i2c (protocol)
                  └── uart_rvc (protocol)
"""

from src.hardware.imu.bno08x.mcp2221.i2c import (
    Config as I2CConfig,
    Driver as I2CDriver,
)
from src.hardware.imu.bno08x.mcp2221.uart_rvc import (
    Config as UART_RVCConfig,
    Driver as UART_RVCDriver,
)

__all__ = [
    "I2CConfig",
    "I2CDriver",
    "UART_RVCConfig",
    "UART_RVCDriver",
]

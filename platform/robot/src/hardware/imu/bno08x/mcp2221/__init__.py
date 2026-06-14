"""
Hardware drivers for BNO08x IMU via MCP2221A USB bridge.

Hierarchy:
    IMU (abstract)
        └── BNO08x (sensor chip)
            └── MCP2221A (USB bridge)
                  └── uart_rvc (protocol)
"""

from src.hardware.imu.bno08x.mcp2221.uart_rvc import (
    Config as UART_RVCConfig,
    Driver as UART_RVCDriver,
)

__all__ = [
    "UART_RVCConfig",
    "UART_RVCDriver",
]

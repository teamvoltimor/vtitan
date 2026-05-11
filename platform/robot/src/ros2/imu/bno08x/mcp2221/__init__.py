"""
ROS2 nodes for BNO08x IMU via MCP2221A USB bridge.

Hierarchy:
    IMU (abstract)
        └── BNO08x (sensor chip)
            └── MCP2221A (USB bridge)
                  └── UART RVC (protocol)
"""

from src.ros2.imu.bno08x.mcp2221.uart_rvc_node import IMU_MCP2221_UART_RVCNode

__all__ = [
    "IMU_MCP2221_UART_RVCNode",
]

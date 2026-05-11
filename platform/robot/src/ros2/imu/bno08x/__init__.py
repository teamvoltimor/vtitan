"""
ROS2 nodes for BNO08x IMU.

Hierarchy:
    IMU (abstract)
        └── BNO08x (sensor chip)
            ├── I2C (protocol)
            └── UART RVC (protocol)
"""

from src.ros2.imu.bno08x.i2c_node import IMU_I2CNode
from src.ros2.imu.bno08x.uart_rvc_node import IMU_UART_RVCNode

__all__ = [
    "IMU_I2CNode",
    "IMU_UART_RVCNode",
]

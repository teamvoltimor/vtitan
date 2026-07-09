from src.ros2.imu.bno08x.mcp2221.uart_rvc_node import IMU_UART_RVCNode, main

# Backwards-compatible alias for the previous local class name.
IMU_RVCNode = IMU_UART_RVCNode

__all__ = ["IMU_RVCNode", "IMU_UART_RVCNode", "main"]

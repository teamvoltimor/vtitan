"""ROS2 node for BNO08x IMU via MCP2221A UART RVC mode."""

from src.hardware.imu.bno08x.mcp2221.uart_rvc import Driver as IMU_MCP2221_UART_RVCDriver
from src.ros2.imu.bno08x.uart_rvc_node import IMU_UART_RVCNode


class IMU_MCP2221_UART_RVCNode(IMU_UART_RVCNode):
    """ROS2 node publishing IMU data from BNO08x over MCP2221A UART RVC."""

    def __init__(self) -> None:
        super().__init__(node_name="bno08x_mcp2221_uart_rvc_node", driver=IMU_MCP2221_UART_RVCDriver())
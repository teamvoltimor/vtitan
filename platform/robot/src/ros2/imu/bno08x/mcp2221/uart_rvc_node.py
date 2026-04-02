"""ROS2 node for BNO085 IMU via MCP2221A UART RVC mode.

Run on: Raspberry Pi 5

Usage:
    ros2 run klevor_robot uart_rvc_node

Topics:
    Published: imu/data (sensor_msgs.msg.Imu)
"""

from typing import TYPE_CHECKING, cast, override

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from src.hardware.imu.bno08x.mcp2221 import UART_RVCDriver

if TYPE_CHECKING:
    from geometry_msgs.msg import Quaternion, Vector3
    from rclpy.publisher import Publisher
    from rclpy.timer import Timer
    from std_msgs.msg import Header


NODE_NAME = "bno08x_uart_rvc_node"
"""ROS2 node for BNO085 IMU via MCP2221A UART RVC mode."""

PUBLISHER_TOPIC = "imu/data"
"""ROS2 topic for publishing IMU data."""

PUBLISHER_QUEUE_SIZE = 10
"""Queue size for the IMU data publisher."""

PUBLISHER_RATE_HZ = 100
"""Rate (in Hz) for publishing IMU data."""

FRAME_ID = "imu_link"
"""Frame ID for the IMU data. Should match the TF frame used for the IMU in the robot's URDF."""


class IMU_RVCNode(Node):
    """ROS2 node that interfaces with the BNO085 IMU using the MCP2221A UART RVC driver and publishes IMU data to the "imu/data" topic."""

    def __init__(self) -> None:
        """Initialize BNO085 RVC node."""
        super().__init__(NODE_NAME)

        _ = self.get_logger().info("Initializing BNO085 RVC node")

        # Create publisher for IMU data and initialize the BNO085 RVC driver
        self.publisher_: Publisher[Imu] = self.create_publisher(Imu, PUBLISHER_TOPIC, PUBLISHER_QUEUE_SIZE)
        self.driver: UART_RVCDriver = UART_RVCDriver()

        try:
            # Connect to the IMU and start polling for data
            self.driver.connect()
            self.driver.start_polling()

            _ = self.get_logger().info("Connected to BNO085 RVC")
        except Exception as e:
            _ = self.get_logger().error(f"Could not connect to IMU: {e}")
            raise

        self.timer: Timer = self.create_timer(1.0 / PUBLISHER_RATE_HZ, self.publish_imu)

    def publish_imu(self) -> None:
        """Publish IMU data to the imu/data topic."""
        data = self.driver.get_data()
        if data is None:
            return

        msg = Imu()

        # Explicitly cast the nested structures to their actual types
        # This clears the "partially unknown" status for Pyright
        header = cast("Header", msg.header)
        orientation = cast("Quaternion", msg.orientation)
        accel = cast("Vector3", msg.linear_acceleration)

        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = FRAME_ID

        # Unpack with the cast we fixed earlier
        qx, qy, qz, qw = data.quaternion
        orientation.x = qx
        orientation.y = qy
        orientation.z = qz
        orientation.w = qw

        accel.x = data.x_accel
        accel.y = data.y_accel
        accel.z = data.z_accel

        msg.angular_velocity_covariance[0] = -1.0

        self.publisher_.publish(msg)

    @override
    def destroy_node(self) -> None:
        """Shut down the BNO085 RVC node."""
        _ = self.get_logger().info("Shutting down BNO085 RVC node")
        self.driver.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Main entry point for the IMU RVC node."""
    _ = rclpy.init(args=args)
    node = IMU_RVCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""ROS2 node for BNO08x IMU via MCP2221A UART RVC mode."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from src.hardware.exceptions import IMUConnectionError
from src.hardware.imu.bno08x.mcp2221.uart_rvc import Driver as IMU_UART_RVCDriver
from src.ros2.params import declare_and_get_float_param, declare_and_get_str_param


class IMU_UART_RVCNode(Node):
    """ROS2 node publishing IMU data from BNO08x over UART RVC."""

    def __init__(self) -> None:
        super().__init__("bno08x_uart_rvc_node")

        # Declare and get parameters
        publish_rate = declare_and_get_float_param(self, "publish_rate", 100.0)
        self.frame_id = declare_and_get_str_param(self, "frame_id", "imu_link")
        topic = declare_and_get_str_param(self, "topic", "imu/data")

        # Initialize hardware driver
        self.driver = IMU_UART_RVCDriver()
        self._hardware_ready = False

        try:
            self.driver.connect()
            self.driver.start_polling()
            self.get_logger().info("IMU driver connected and polling started.")
            self._hardware_ready = True
        except IMUConnectionError as e:
            self.get_logger().error(f"IMU hardware not available on port {e.port}: {e}")
            # Continue gracefully — IMU data is not critical for motor control
        except (RuntimeError, ValueError, ImportError, OSError, TimeoutError, AttributeError) as e:
            # Unexpected failures (vs. a clean IMUConnectionError) are fatal — surface them.
            self.get_logger().error(f"Unexpected IMU initialization error: {e}")
            raise

        # Setup publisher
        self.publisher_ = self.create_publisher(Imu, topic, 10)

        # Setup timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu)

    def destroy_node(self) -> bool:
        """Clean up hardware driver."""
        self.driver.close()
        return super().destroy_node()

    def publish_imu(self) -> None:
        """Read data from driver and publish as sensor_msgs/Imu."""
        if not self._hardware_ready:
            return

        data = self.driver.get_data()
        if data is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # The driver returns quaternion as (qx, qy, qz, qw)
        qx, qy, qz, qw = data.quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance = [-1.0] + [0.0] * 8

        # Angular velocity is not provided by RVC mode, set to 0 and covariance to -1
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.angular_velocity_covariance = [-1.0] + [0.0] * 8

        # Linear acceleration (from RVC)
        msg.linear_acceleration.x = float(data.x_accel)
        msg.linear_acceleration.y = float(data.y_accel)
        msg.linear_acceleration.z = float(data.z_accel)

        # Linear acceleration covariance (approximate 0.01 diagonal)
        msg.linear_acceleration_covariance = [0.0] * 9
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.publisher_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IMU_UART_RVCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""ROS2 node for BNO085 IMU via I2C.

Run on: Raspberry Pi 5

Usage:
    ros2 run klevor_robot bno08x_i2c_node

Topics:
    Published: imu/data (sensor_msgs.msg.Imu)
"""

from typing import TYPE_CHECKING, override

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import Imu

from src.hardware.imu.bno08x.mcp2221.i2c import Driver as IMU_I2CDriver

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class IMU_I2CNode(Node):
    """ROS2 node that interfaces with the BNO085 IMU using I2C and publishes IMU data to the "imu/data" topic."""

    timer: Timer

    def __init__(self) -> None:
        """Initialize BNO085 I2C node."""
        super().__init__("bno08x_i2c_node")

        self.get_logger().info("Initializing BNO085 I2C node")

        self.publisher_: Publisher[Imu] = self.create_publisher(Imu, "imu/data", 10)

        self.driver: IMU_I2CDriver = IMU_I2CDriver()
        try:
            self.driver.connect()
            self.driver.enable_sensors()
            self.get_logger().info("Connected to BNO085 I2C")
        except Exception as e:
            self.get_logger().error(f"Could not connect to IMU: {e}")
            raise

        self.timer = self.create_timer(0.01, self.publish_imu)

    def publish_imu(self) -> None:
        """Publish IMU data to the imu/data topic."""
        data = self.driver.get_all_data()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()  # type: ignore[union-attr]
        msg.header.frame_id = "imu_link"  # type: ignore[union-attr]

        qw, qx, qy, qz = data.quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        ax, ay, az = data.linear_accel
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        gx, gy, gz = data.gyroscope
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.publisher_.publish(msg)

    @override
    def destroy_node(self) -> None:
        """Shut down the BNO085 I2C node."""
        self.get_logger().info("Shutting down BNO085 I2C node")
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Main entry point for the IMU I2C node."""
    rclpy.init(args=args)
    node = IMU_I2CNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

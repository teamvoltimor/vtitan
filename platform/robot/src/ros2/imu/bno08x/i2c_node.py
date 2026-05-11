"""ROS2 node for BNO08x IMU via I2C."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from src.hardware.imu.bno08x.i2c import Driver as IMU_I2CDriver


class IMU_I2CNode(Node):
    """ROS2 node publishing IMU data from BNO08x over I2C."""

    def __init__(self) -> None:
        super().__init__("bno08x_i2c_node")

        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "imu/data")

        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.driver = IMU_I2CDriver()
        try:
            self.driver.connect()
            self.driver.enable_sensors()
            self.get_logger().info("IMU driver connected and sensors enabled.")
        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().error(f"Failed to initialize IMU driver: {type(e).__name__}: {e}")
            raise

        self.publisher_ = self.create_publisher(Imu, topic, 10)

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu)

    def publish_imu(self) -> None:
        """Read data from driver and publish as sensor_msgs/Imu."""
        try:
            data = self.driver.get_all_data()
        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().warning(f"Failed to read IMU data: {type(e).__name__}: {e}")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        qw, qx, qy, qz = data.quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance = [-1.0] + [0.0] * 8

        msg.angular_velocity.x = data.gyroscope[0]
        msg.angular_velocity.y = data.gyroscope[1]
        msg.angular_velocity.z = data.gyroscope[2]

        msg.angular_velocity_covariance = [0.0] * 9
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration.x = data.linear_accel[0]
        msg.linear_acceleration.y = data.linear_accel[1]
        msg.linear_acceleration.z = data.linear_accel[2]

        msg.linear_acceleration_covariance = [0.0] * 9
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.publisher_.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Main entry point for IMU I2C node."""
    rclpy.init(args=args)
    node = IMU_I2CNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

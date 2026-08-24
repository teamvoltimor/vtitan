"""ROS2 lifecycle node for BNO08x IMU via MCP2221A I2C bridge.

Hardware connects in on_configure() and publishing starts in on_activate(),
matching the driver lifecycle pattern used by every hardware node in this
package (see button_node.py for the reference implementation).
"""

from __future__ import annotations

from typing import override

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Imu
from shared.config.constants import TfFrames
from shared.config.ros_topics import RosTopicConfig

from src.hardware.imu.bno08x.mcp2221.i2c import Driver as IMU_I2CDriver
from src.ros2.hardware_node import LifecycleHardwareNode


class IMU_I2CNode(LifecycleHardwareNode[IMU_I2CDriver]):
    """ROS2 lifecycle node publishing IMU data from BNO08x over I2C."""

    def __init__(self) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__(
            "bno08x_i2c_node",
            Imu,
            publish_rate_default=50.0,  # Hz
            topic_default=RosTopicConfig.load_default().sensors.imu,
            frame_id_default=TfFrames.IMU_LINK,
        )

    @override
    def _create_driver(self) -> IMU_I2CDriver:
        return IMU_I2CDriver()

    @override
    def _configure_driver(self, driver: IMU_I2CDriver) -> TransitionCallbackReturn:
        try:
            driver.connect()
            driver.enable_sensors()
            self.get_logger().info("IMU driver connected and sensors enabled.")
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Failed to initialize IMU driver: {type(e).__name__}: {e}")
            self.driver = None
        return TransitionCallbackReturn.SUCCESS

    @override
    def publish_imu(self) -> None:
        """Read data from driver and publish as sensor_msgs/Imu."""
        if self.driver is None or self.publisher_ is None:
            return
        try:
            data = self.driver.get_all_data()
        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().warning(f"Failed to read IMU data: {type(e).__name__}: {e}")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # The test expects driver.get_all_data().quaternion to be (qw, qx, qy, qz)
        qw, qx, qy, qz = data.quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Orientation covariance (unused/unknown)
        msg.orientation_covariance = [-1.0] + [0.0] * 8

        # Angular velocity
        msg.angular_velocity.x = data.gyroscope[0]
        msg.angular_velocity.y = data.gyroscope[1]
        msg.angular_velocity.z = data.gyroscope[2]

        # Angular velocity covariance (approximate 0.01 diagonal)
        msg.angular_velocity_covariance = [0.0] * 9
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        # Linear acceleration
        msg.linear_acceleration.x = data.linear_accel[0]
        msg.linear_acceleration.y = data.linear_accel[1]
        msg.linear_acceleration.z = data.linear_accel[2]

        # Linear acceleration covariance (approximate 0.01 diagonal)
        msg.linear_acceleration_covariance = [0.0] * 9
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.publisher_.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Run the I2C IMU node, auto-configuring and auto-activating on launch."""
    rclpy.init(args=args)
    node = IMU_I2CNode()
    try:
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

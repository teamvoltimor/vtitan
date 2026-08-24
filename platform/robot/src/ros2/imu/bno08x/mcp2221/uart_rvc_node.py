"""ROS2 lifecycle node for BNO08x IMU via MCP2221A UART RVC mode.

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

from src.hardware.exceptions import IMUConnectionError
from src.hardware.imu.bno08x.mcp2221.uart_rvc import Driver as IMU_UART_RVCDriver
from src.ros2.hardware_node import LifecycleHardwareNode


class IMU_UART_RVCNode(LifecycleHardwareNode[IMU_UART_RVCDriver]):
    """ROS2 lifecycle node publishing IMU data from BNO08x over UART RVC."""

    def __init__(self) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__(
            "bno08x_uart_rvc_node",
            Imu,
            publish_rate_default=100.0,
            topic_default=RosTopicConfig.load_default().sensors.imu,
            frame_id_default=TfFrames.IMU_LINK,
        )
        self._hardware_ready = False

    @override
    def _create_driver(self) -> IMU_UART_RVCDriver:
        return IMU_UART_RVCDriver()

    @override
    def _configure_driver(self, driver: IMU_UART_RVCDriver) -> TransitionCallbackReturn:
        self._hardware_ready = False

        try:
            driver.connect()
            driver.start_polling()
            self.get_logger().info("IMU driver connected and polling started.")
            self._hardware_ready = True
        except IMUConnectionError as e:
            self.get_logger().error(f"IMU hardware not available on port {e.port}: {e}")
            # Continue gracefully — IMU data is not critical for motor control
        except (RuntimeError, ValueError, ImportError, OSError, TimeoutError, AttributeError) as e:
            # Unexpected failures (vs. a clean IMUConnectionError) are a real bug,
            # not an absent-hardware condition -- fail configuration instead of
            # silently degrading.
            self.get_logger().error(f"Unexpected IMU initialization error: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    @override
    def _on_driver_disconnected(self) -> None:
        self._hardware_ready = False

    @override
    def publish_imu(self) -> None:
        """Read data from driver and publish as sensor_msgs/Imu."""
        if not self._hardware_ready or self.publisher_ is None or self.driver is None:
            return

        data = self.driver.get_data()
        if data is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # QuaternionReading's field order is (w, x, y, z); ROS 2's
        # geometry_msgs/Quaternion is (x, y, z, w).
        qw, qx, qy, qz = data.quaternion
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


def main(args: list[str] | None = None) -> None:
    """Run the UART RVC IMU node, auto-configuring and auto-activating on launch."""
    rclpy.init(args=args)
    node = IMU_UART_RVCNode()
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

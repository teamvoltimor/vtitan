"""ROS2 lifecycle node for BNO08x IMU via MCP2221A UART RVC mode.

Hardware connects in on_configure() and publishing starts in on_activate(),
matching the driver lifecycle pattern used by every hardware node in this
package (see button_node.py for the reference implementation).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, override

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Imu

from src.hardware.exceptions import IMUConnectionError
from src.hardware.imu.bno08x.mcp2221.uart_rvc import Driver as IMU_UART_RVCDriver
from src.ros2.params import declare_and_get_float_param, declare_and_get_str_param

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.timer import Timer


class IMU_UART_RVCNode(LifecycleNode):
    """ROS2 lifecycle node publishing IMU data from BNO08x over UART RVC."""

    def __init__(self) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__("bno08x_uart_rvc_node")
        self.get_logger().info("IMU UART RVC Node constructed (unconfigured)")

        declare_and_get_float_param(self, "publish_rate", 100.0)
        declare_and_get_str_param(self, "frame_id", "imu_link")
        declare_and_get_str_param(self, "topic", "imu/data")

        self.driver: IMU_UART_RVCDriver | None = None
        self.publisher_: Publisher | None = None
        self.timer: Timer | None = None
        self.frame_id: str = ""
        self._hardware_ready = False

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the IMU driver and create the publisher."""
        self.get_logger().info("Configuring IMU UART RVC Node")

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.publisher_ = self.create_lifecycle_publisher(Imu, topic, 10)

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
            # Unexpected failures (vs. a clean IMUConnectionError) are a real bug,
            # not an absent-hardware condition -- fail configuration instead of
            # silently degrading.
            self.get_logger().error(f"Unexpected IMU initialization error: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the publish timer."""
        self.get_logger().info("Activating IMU UART RVC Node")
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_imu)
        return super().on_activate(state)

    @override
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop the publish timer."""
        self.get_logger().info("Deactivating IMU UART RVC Node")
        self._destroy_timer()
        return super().on_deactivate(state)

    @override
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Disconnect the driver and tear down the publisher."""
        self.get_logger().info("Cleaning up IMU UART RVC Node")
        self._disconnect_driver()
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info("Shutting down IMU UART RVC Node")
        self._destroy_timer()
        self._disconnect_driver()
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None
        return TransitionCallbackReturn.SUCCESS

    def _destroy_timer(self) -> None:
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

    def _disconnect_driver(self) -> None:
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception as e:  # noqa: BLE001 - cleanup must never fail node teardown
                self.get_logger().error(f"Error closing IMU driver: {e}")
            self.driver = None
        self._hardware_ready = False

    @override
    def destroy_node(self) -> bool:
        """Release hardware directly rather than trigger an on_shutdown transition.

        Handles a node destroyed without a clean lifecycle shutdown (e.g.
        process killed mid-active, or a test that never triggers shutdown).
        """
        self._destroy_timer()
        self._disconnect_driver()
        self.publisher_ = None
        return super().destroy_node()

    def publish_imu(self) -> None:
        """Read data from driver and publish as sensor_msgs/Imu."""
        if not self._hardware_ready or self.publisher_ is None:
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

"""ROS2 lifecycle node for BNO08x IMU via MCP2221A I2C bridge.

Hardware connects in on_configure() and publishing starts in on_activate(),
matching the driver lifecycle pattern used by every hardware node in this
package (see button_node.py for the reference implementation).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, override

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Imu

from src.hardware.imu.bno08x.mcp2221.i2c import Driver as IMU_I2CDriver

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.timer import Timer


class IMU_I2CNode(LifecycleNode):
    """ROS2 lifecycle node publishing IMU data from BNO08x over I2C."""

    def __init__(self) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__("bno08x_i2c_node")
        self.get_logger().info("IMU I2C Node constructed (unconfigured)")

        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "imu/data")

        self.driver: IMU_I2CDriver | None = None
        self.publisher_: Publisher | None = None
        self.timer: Timer | None = None
        self.frame_id: str = ""

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the IMU driver and create the publisher."""
        self.get_logger().info("Configuring IMU I2C Node")

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.publisher_ = self.create_lifecycle_publisher(Imu, topic, 10)

        self.driver = IMU_I2CDriver()
        try:
            self.driver.connect()
            self.driver.enable_sensors()
            self.get_logger().info("IMU driver connected and sensors enabled.")
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"Failed to initialize IMU driver: {type(e).__name__}: {e}")
            self.driver = None

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the publish timer."""
        self.get_logger().info("Activating IMU I2C Node")
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_imu)
        return super().on_activate(state)

    @override
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop the publish timer."""
        self.get_logger().info("Deactivating IMU I2C Node")
        self._destroy_timer()
        return super().on_deactivate(state)

    @override
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Disconnect the driver and tear down the publisher."""
        self.get_logger().info("Cleaning up IMU I2C Node")
        self._disconnect_driver()
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info("Shutting down IMU I2C Node")
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

    @override
    def destroy_node(self) -> None:
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

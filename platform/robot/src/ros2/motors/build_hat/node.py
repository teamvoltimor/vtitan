"""ROS2 node for Build HAT motor driver."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from shared.config.constants import RobotSpecs

from src.hardware.exceptions import MotorCalibrationError, MotorConnectionError
from src.hardware.motors import BuildHatDriver
from src.ros2.params import declare_and_get_str_param


class SimulatedMotorDriver:
    """Fallback simulated motor driver for graceful degradation."""

    def stop_drive(self) -> None:
        """No-op drive stop."""
        pass

    def center_steering(self) -> None:
        """No-op steering center."""
        pass

    def run_drive_forward(self, speed: int | None = None) -> None:
        """No-op forward."""
        pass

    def run_drive_reverse(self, speed: int | None = None) -> None:
        """No-op reverse."""
        pass

    def move_steering_to(self, position: float, speed: int = 20) -> None:
        """No-op steering movement."""
        pass


class BuildHatNode(Node):
    """ROS2 node controlling Build HAT motors."""

    def __init__(self) -> None:
        super().__init__("build_hat_node")

        cmd_vel_topic = declare_and_get_str_param(self, "cmd_vel_topic", "/wro_robot/cmd_vel")

        self.driver = BuildHatDriver()
        self._hardware_ready = False

        # Phase 1: Connect to motors
        try:
            self.driver.connect()
            self.get_logger().info("Build HAT motors connected.")
            self._hardware_ready = True
        except MotorConnectionError as e:
            self.get_logger().error(f"Motor hardware missing on port {e.port} (using simulated driver): {e}")
            self.driver = SimulatedMotorDriver()
            return

        # Phase 2: Load calibration (recoverable error)
        try:
            self.driver.load_calibration()
            self.driver.center_steering()
            self.get_logger().info("Build HAT motors calibrated and centered.")
        except MotorCalibrationError as e:
            self.get_logger().warning(f"Using default calibration (file missing): {e}")
            # Continue with defaults — not fatal

        self.subscription = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        # Max speed for conversion
        self.max_linear_speed = 1.0  # m/s approximate
        self.max_steering_angle = RobotSpecs.MAX_STEERING_ANGLE  # radians
        self.max_motor_speed = 100  # percentage

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Convert Twist message to motor commands."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Control drive motor
        if abs(linear_x) < 0.01:
            self.driver.stop_drive()
        else:
            # Map linear velocity to motor speed (-100 to 100)
            speed_pct = int(min(1.0, abs(linear_x) / self.max_linear_speed) * self.max_motor_speed)
            if linear_x > 0:
                self.driver.run_drive_forward(speed=speed_pct)
            else:
                self.driver.run_drive_reverse(speed=speed_pct)

        # Control steering motor
        # Map angular velocity (Ackermann steering angle) to motor position
        # Driver expects position in degrees. Our calibration left_limit / right_limit usually map to degrees.
        # Ackermann angles are in radians.
        angle_deg = math.degrees(angular_z)

        # Invert if necessary, assuming positive z is left (standard ROS),
        # so positive angle is left turn. The driver's move_steering_to takes absolute position in degrees.
        # We assume 0 is center.
        self.driver.move_steering_to(-angle_deg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuildHatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop_drive()
        node.driver.center_steering()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

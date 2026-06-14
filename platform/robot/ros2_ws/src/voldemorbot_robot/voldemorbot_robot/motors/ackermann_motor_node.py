"""ROS2 node for Ackermann motor control via Build HAT.

Run on: Raspberry Pi Zero (connected to Raspberry Pi 5 via network)

Usage:
    ros2 run voldemorbot_robot ackermann_motor_node

Topics:
    Subscribed:
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - Ackermann drive commands
    Published:
        - /motor/steering_position (std_msgs/Float32) - Current steering position in degrees
        - /motor/drive_speed (std_msgs/Float32) - Current drive speed in degrees/s
        - /motor/status (diagnostic_msgs/DiagnosticStatus) - Motor status diagnostics

Environment Variables:
    MOTOR_STEERING_PORT: Build HAT port for steering motor (default: A)
    MOTOR_DRIVE_PORT: Build HAT port for drive motor (default: B)
    MOTOR_STEERING_OFFSET: Steering center angle offset in degrees (default: 0.0)
    MOTOR_REVERSE_DRIVE: Reverse drive motor direction (default: False)
    MOTOR_MAX_SPEED: Maximum drive speed 0-100 (default: 50)
    MOTOR_MAX_STEERING_ANGLE: Maximum steering angle in degrees (default: 45.0)
    MOTOR_SPEED_SCALE: Scale factor for velocity to motor speed (default: 30.0)
"""

import math
from typing import TYPE_CHECKING, override

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Float32

from src.env import EnvVar
from src.hardware.motors.build_hat import Driver as MotorDriver

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer


NODE_NAME = "ackermann_motor_node"
"""ROS2 node name for Ackermann motor controller."""

PUBLISHER_RATE_HZ = 20.0
"""Rate for publishing motor state and diagnostics."""

# Environment variables for motor configuration
MOTOR_STEERING_OFFSET = EnvVar[float](key="MOTOR_STEERING_OFFSET", default=0.0, cast=float)
"""Steering center angle offset in degrees for calibration. Positive = bias right, Negative = bias left."""

MOTOR_REVERSE_DRIVE = EnvVar[bool](
    key="MOTOR_REVERSE_DRIVE",
    default=False,
    cast=lambda x: str(x).lower() in ("true", "1", "yes"),
)
"""Reverse drive motor direction. Set to True if motor is mounted backwards."""

MOTOR_MAX_SPEED = EnvVar[int](key="MOTOR_MAX_SPEED", default=50, cast=int)
"""Maximum drive motor speed (0-100 scale). Limits top speed for safety."""

MOTOR_MAX_STEERING_ANGLE = EnvVar[float](key="MOTOR_MAX_STEERING_ANGLE", default=45.0, cast=float)
"""Maximum steering angle in degrees. Commands beyond this are clamped."""

MOTOR_SPEED_SCALE = EnvVar[float](key="MOTOR_SPEED_SCALE", default=30.0, cast=float)
"""Scale factor for converting Ackermann velocity (m/s) to motor speed (0-100).
Formula: motor_speed = velocity * MOTOR_SPEED_SCALE"""


class AckermannMotorNode(Node):
    """ROS2 node that controls motors via Ackermann drive commands.

    Responsibilities:
    - Subscribe to Ackermann drive commands (/ackermann_cmd)
    - Apply steering offset calibration
    - Handle drive motor direction reversal
    - Clamp steering and speed to safe limits
    - Publish motor feedback (position, speed, status)
    - Convert velocity (m/s) to motor speed percentage
    """

    def __init__(self) -> None:
        """Initialize Ackermann motor node."""
        super().__init__(NODE_NAME)

        self.get_logger().info("Initializing Ackermann Motor Control Node")

        # Load configuration from environment
        self.steering_offset = MOTOR_STEERING_OFFSET.value
        self.reverse_drive = MOTOR_REVERSE_DRIVE.value
        self.max_speed = MOTOR_MAX_SPEED.value
        self.max_steering_angle = MOTOR_MAX_STEERING_ANGLE.value
        self.speed_scale = MOTOR_SPEED_SCALE.value

        self.get_logger().info(
            f"Configuration: steering_offset={self.steering_offset}°, "
            f"reverse_drive={self.reverse_drive}, "
            f"max_speed={self.max_speed}, "
            f"max_steering={self.max_steering_angle}°, "
            f"speed_scale={self.speed_scale}",
        )

        # Motor driver
        try:
            self.motor_driver = MotorDriver()
            self.motor_driver.connect()
            self.get_logger().info("Motor driver connected")

            # Center steering on startup
            self.motor_driver.center_steering()
            self.get_logger().info("Steering centered")

        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect motor driver: {e}")
            self.motor_driver = None  # type: ignore[assignment]

        # Current command tracking
        self.current_speed: float = 0.0
        self.current_steering_angle: float = 0.0
        self.last_command_time: float = 0.0

        # Publishers
        self.steering_pos_pub: Publisher[Float32] = self.create_publisher(Float32, "/motor/steering_position", 10)
        self.drive_speed_pub: Publisher[Float32] = self.create_publisher(Float32, "/motor/drive_speed", 10)
        self.status_pub: Publisher[DiagnosticStatus] = self.create_publisher(DiagnosticStatus, "/motor/status", 10)

        # Subscriber
        self.ackermann_sub: Subscription[AckermannDriveStamped] = self.create_subscription(
            AckermannDriveStamped,
            "/ackermann_cmd",
            self._ackermann_callback,
            10,
        )

        # Timers
        self.feedback_timer: Timer = self.create_timer(1.0 / PUBLISHER_RATE_HZ, self._publish_feedback)
        self.watchdog_timer: Timer = self.create_timer(0.5, self._watchdog_check)  # 500ms watchdog

        self.get_logger().info("Ackermann Motor Node initialized and ready")

    def _ackermann_callback(self, msg: AckermannDriveStamped) -> None:
        """Handle incoming Ackermann drive commands.

        Args:
            msg: Ackermann drive command with speed and steering angle.
        """
        if self.motor_driver is None:
            return

        # Extract velocity and steering angle from message
        velocity = msg.drive.speed  # m/s
        steering_angle_rad = msg.drive.steering_angle  # radians

        # Convert steering angle from radians to degrees
        steering_angle_deg = math.degrees(steering_angle_rad)

        # Apply steering offset calibration
        calibrated_steering = steering_angle_deg + self.steering_offset

        # Clamp steering to safe limits
        clamped_steering = max(-self.max_steering_angle, min(self.max_steering_angle, calibrated_steering))

        if abs(calibrated_steering) > self.max_steering_angle:
            self.get_logger().warning(
                f"Steering angle {calibrated_steering:.2f}° exceeds limit ±{self.max_steering_angle}°, "
                f"clamped to {clamped_steering:.2f}°",
            )

        # Convert velocity to motor speed percentage
        motor_speed = int(velocity * self.speed_scale)

        # Apply drive reversal if configured
        if self.reverse_drive:
            motor_speed = -motor_speed

        # Clamp motor speed to safe limits
        clamped_speed = max(-self.max_speed, min(self.max_speed, motor_speed))

        if abs(motor_speed) > self.max_speed:
            self.get_logger().warning(
                f"Motor speed {motor_speed} exceeds limit ±{self.max_speed}, clamped to {clamped_speed}",
            )

        # Update tracking variables
        self.current_speed = clamped_speed
        self.current_steering_angle = clamped_steering
        self.last_command_time = self.get_clock().now().nanoseconds / 1e9

        # Execute motor commands
        try:
            # Set steering position
            self.motor_driver.move_steering_to(clamped_steering, speed=30)

            # Set drive motor speed
            if clamped_speed > 0:
                self.motor_driver.run_drive_forward(abs(clamped_speed))
            elif clamped_speed < 0:
                self.motor_driver.run_drive_reverse(abs(clamped_speed))
            else:
                self.motor_driver.stop_drive()

            self.get_logger().debug(
                f"Motor command: speed={clamped_speed}, steering={clamped_steering:.2f}° "
                f"(offset={self.steering_offset}°, reverse={self.reverse_drive})",
            )

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().error(f"Failed to execute motor command: {e}")

    def _publish_feedback(self) -> None:
        """Publish motor position and speed feedback."""
        if self.motor_driver is None:
            return

        try:
            # Get current motor states
            steering_pos = self.motor_driver.get_steering_position()
            drive_speed = self.motor_driver.get_drive_speed()

            # Publish steering position
            steering_msg = Float32()
            steering_msg.data = steering_pos
            self.steering_pos_pub.publish(steering_msg)

            # Publish drive speed
            speed_msg = Float32()
            speed_msg.data = drive_speed
            self.drive_speed_pub.publish(speed_msg)

            # Publish status diagnostics
            status_msg = DiagnosticStatus()
            status_msg.name = "Ackermann Motors"
            status_msg.level = DiagnosticStatus.OK
            status_msg.message = "Motors operational"
            status_msg.hardware_id = "BuildHAT"

            status_msg.values.append(KeyValue(key="steering_position", value=f"{steering_pos:.2f}"))
            status_msg.values.append(KeyValue(key="drive_speed", value=f"{drive_speed:.2f}"))
            status_msg.values.append(KeyValue(key="commanded_speed", value=f"{self.current_speed}"))
            status_msg.values.append(KeyValue(key="commanded_steering", value=f"{self.current_steering_angle:.2f}"))
            status_msg.values.append(KeyValue(key="steering_offset", value=f"{self.steering_offset:.2f}"))
            status_msg.values.append(KeyValue(key="reverse_drive", value=str(self.reverse_drive)))

            self.status_pub.publish(status_msg)

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().warning(f"Failed to read motor feedback: {e}")

    def _watchdog_check(self) -> None:
        """Watchdog to stop motors if no commands received recently."""
        if self.motor_driver is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_command = current_time - self.last_command_time

        # If no command received in 1 second, ensure motors are stopped
        if time_since_last_command > 1.0 and self.current_speed != 0.0:
            self.get_logger().warning(
                f"No Ackermann commands received for {time_since_last_command:.2f}s - stopping motors for safety",
            )
            try:
                self.motor_driver.stop_drive()
                self.current_speed = 0.0
            except (RuntimeError, OSError, ValueError) as e:
                self.get_logger().error(f"Failed to stop motors in watchdog: {e}")

    @override
    def destroy_node(self) -> None:
        """Shutdown the Ackermann motor node."""
        self.get_logger().info("Shutting down Ackermann Motor Node")

        # Stop motors safely
        if self.motor_driver is not None:
            try:
                self.motor_driver.stop_drive()
                self.motor_driver.center_steering()
                self.get_logger().info("Motors stopped and steering centered")
            except (RuntimeError, OSError, ValueError) as e:
                self.get_logger().error(f"Error stopping motors during shutdown: {e}")

        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Main entry point for Ackermann motor node."""
    rclpy.init(args=args)
    node = AckermannMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

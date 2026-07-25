"""ROS2 lifecycle node for Ackermann motor control with selectable actuator backends.

Run on: Raspberry Pi Zero (connected to Raspberry Pi 5 via network)

Steering and drive are independent backends, chosen at runtime:
    - Default: servo steering + DC-encoder drive (two split-interface drivers).
    - Build HAT: one combined steering+drive object, reused for both sides.

Hardware connects in on_configure() and command handling starts in
on_activate(), matching the driver lifecycle pattern used by every hardware
node in this package (see button_node.py for the reference implementation).
Safety-critical: on_deactivate(), on_cleanup(), on_shutdown() and
destroy_node() all route through _stop_motors_safely() so the motors are
always commanded to stop and steering re-centered, regardless of which
teardown path the node takes.

Usage:
    ros2 run voldemorbot_drivers ackermann_motor_node

Topics:
    Subscribed:
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - Ackermann drive commands
    Published:
        - /motor/steering_position (std_msgs/Float32) - Current steering position in degrees
        - /motor/drive_speed (std_msgs/Float32) - Current drive speed in degrees/s
        - /motor/status (diagnostic_msgs/DiagnosticStatus) - Motor status diagnostics

Environment Variables:
    STEERING_BACKEND: servo | build_hat (default: servo)
    DRIVE_BACKEND: dc_encoder | build_hat (default: dc_encoder)
    MOTOR_STEERING_OFFSET: Steering center angle offset in degrees (default: 0.0)
    MOTOR_REVERSE_DRIVE: Reverse drive motor direction (default: False)
    MOTOR_MAX_SPEED: Maximum drive speed 0-100 (default: 50)
    MOTOR_MAX_STEERING_ANGLE: Maximum steering angle in degrees (default: 45.0)
    MOTOR_SPEED_SCALE: Scale factor for velocity to motor speed (default: 30.0)
    DC-encoder drive pins: MOTOR_PWM_PIN (ENB), MOTOR_IN3_PIN, MOTOR_IN4_PIN,
        MOTOR_ENCODER_A_PIN, MOTOR_ENCODER_B_PIN
    Servo steering: SERVO_* (see src.hardware.motors.servo.config.ServoConfig)
    Build HAT (when selected): MOTOR_STEERING_PORT, MOTOR_DRIVE_PORT, ...
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, override

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32

from src.hardware.motors.config import Config
from src.hardware.motors.enums import DriveBackend, SteeringBackend
from src.logger import configure_json_logging
from src.ros2.params import declare_and_get_int_param, declare_and_get_str_param

# .env loading is a side effect of importing src.logger.config -- the
# servo/dc_encoder backends (this node's defaults) never import src.logger
# themselves (only build_hat's driver does), and even then only inside
# on_configure()'s try block, well after Config() below would already have
# run. Trigger it explicitly here so Config() sees MOTOR_STEERING__*/
# MOTOR_DRIVE__* regardless of which backend is selected.
configure_json_logging()

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer

    from src.hardware.motors.base import (
        DriveDriver,
        Driver as CombinedDriver,
        SteeringDriver,
    )


NODE_NAME = "ackermann_motor_node"
"""ROS2 node name for Ackermann motor controller."""

PUBLISHER_RATE_HZ = 100.0
"""Rate for publishing motor state and diagnostics."""

STEERING_COMMAND_SPEED = 30
"""Steering move speed (deg/s) commanded per update. Used by geared backends; the servo self-paces."""


# Backend selection (from environment)
def _parse_steering_backend(value: str) -> SteeringBackend:
    """Parse STEERING_BACKEND env var."""
    try:
        return SteeringBackend(value)
    except ValueError:
        return SteeringBackend.SERVO


def _parse_drive_backend(value: str) -> DriveBackend:
    """Parse DRIVE_BACKEND env var."""
    try:
        return DriveBackend(value)
    except ValueError:
        return DriveBackend.DC_ENCODER


@dataclass(frozen=True, slots=True)
class _DcEncoderPins:
    """GPIO pin assignment for the DC-encoder drive backend (L298N/TB6612)."""

    pwm_pin: int = 13
    dir_a_pin: int = 5
    dir_b_pin: int = 6
    encoder_a_pin: int = 16
    encoder_b_pin: int = 20


class _DriverFactory:
    """Build the steering/drive drivers for the configured backends.

    The Build HAT driver is a single combined steering+drive object, so when
    both backends select it the same instance is reused (one GPIO/serial open).
    Driver classes are imported lazily so an unused backend need not be
    installed on the host.
    """

    def __init__(self, config: Config, dc_encoder_pins: _DcEncoderPins | None = None) -> None:
        self._config = config
        self._dc_encoder_pins = dc_encoder_pins or _DcEncoderPins()
        self._build_hat: CombinedDriver | None = None

    def _shared_build_hat(self) -> CombinedDriver:
        """Return the combined Build HAT driver, building it at most once."""
        if self._build_hat is None:
            from src.hardware.motors.build_hat import Driver  # noqa: PLC0415 - lazy: only when selected

            self._build_hat = Driver(self._config)
        return self._build_hat

    def steering(self, backend: SteeringBackend) -> SteeringDriver:
        """Build the steering driver for ``backend``."""
        if backend is SteeringBackend.BUILD_HAT:
            return self._shared_build_hat()
        from src.hardware.motors.servo import Driver, ServoConfig  # noqa: PLC0415 - lazy: only when selected

        return Driver(ServoConfig())

    def drive(self, backend: DriveBackend) -> DriveDriver:
        """Build the drive driver for ``backend``."""
        if backend is DriveBackend.BUILD_HAT:
            return self._shared_build_hat()  # combined object also satisfies DriveDriver
        from src.hardware.motors.dc_encoder.driver import Driver  # noqa: PLC0415 - lazy: only when selected

        pins = self._dc_encoder_pins
        return Driver(
            pwm_pin=pins.pwm_pin,
            dir_a_pin=pins.dir_a_pin,
            dir_b_pin=pins.dir_b_pin,
            encoder_a_pin=pins.encoder_a_pin,
            encoder_b_pin=pins.encoder_b_pin,
            standby_pin=None,  # L298N has no STBY line
        )


class AckermannMotorNode(LifecycleNode):
    """ROS2 lifecycle node that controls motors via Ackermann drive commands.

    Responsibilities:
    - Subscribe to Ackermann drive commands (/ackermann_cmd)
    - Apply steering offset calibration
    - Handle drive motor direction reversal
    - Clamp steering and speed to safe limits
    - Publish motor feedback (position, speed, status)
    - Convert velocity (m/s) to motor speed percentage
    """

    def __init__(self) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__(NODE_NAME)
        self.get_logger().info("Ackermann Motor Node constructed (unconfigured)")

        self.config: Config | None = None
        self.steering_backend: SteeringBackend | None = None
        self.drive_backend: DriveBackend | None = None
        self.steering: SteeringDriver | None = None
        self.drive: DriveDriver | None = None

        self.steering_pos_pub: Publisher | None = None
        self.drive_speed_pub: Publisher | None = None
        self.status_pub: Publisher | None = None
        self.ackermann_sub: Subscription | None = None
        self.feedback_timer: Timer | None = None
        self.watchdog_timer: Timer | None = None

        # Current command tracking
        self.current_speed: float = 0.0
        self.current_steering_angle: float = 0.0
        self.last_command_time: float = 0.0

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the motor drivers, center steering, and create publishers."""
        self.get_logger().info("Configuring Ackermann Motor Node")

        # Load configuration from environment (pydantic-settings via Config).
        # test_duration is required with no default -- resolved from an env
        # var; mypy can't see that.
        config = Config()  # type: ignore[call-arg]
        self.config = config

        # Backend selection (with fallback), exposed as ROS2 params so
        # `ros2 param get/set` and launch-time YAML overrides work.
        self.steering_backend = _parse_steering_backend(
            declare_and_get_str_param(self, "steering_backend", "servo"),
        )
        self.drive_backend = _parse_drive_backend(
            declare_and_get_str_param(self, "drive_backend", "dc_encoder"),
        )

        self.get_logger().info(
            f"Configuration: steering_backend={self.steering_backend.value}, "
            f"drive_backend={self.drive_backend.value}, "
            f"steering_offset={config.steering.offset}°, "
            f"reverse_drive={config.drive.reversed}, "
            f"max_speed={config.drive.max_speed}, "
            f"max_steering={config.steering.max_steering_angle}°, "
            f"speed_scale={config.drive.speed_scale}",
        )

        # DC-encoder GPIO pin assignment, exposed as ROS2 params.
        _pin_defaults = _DcEncoderPins()
        dc_encoder_pins = _DcEncoderPins(
            pwm_pin=declare_and_get_int_param(self, "motor_pwm_pin", _pin_defaults.pwm_pin),
            dir_a_pin=declare_and_get_int_param(self, "motor_in3_pin", _pin_defaults.dir_a_pin),
            dir_b_pin=declare_and_get_int_param(self, "motor_in4_pin", _pin_defaults.dir_b_pin),
            encoder_a_pin=declare_and_get_int_param(self, "motor_encoder_a_pin", _pin_defaults.encoder_a_pin),
            encoder_b_pin=declare_and_get_int_param(self, "motor_encoder_b_pin", _pin_defaults.encoder_b_pin),
        )

        # Motor drivers (steering and drive may be one combined object or two)
        self.steering = None
        self.drive = None
        try:
            factory = _DriverFactory(config, dc_encoder_pins)
            steering = factory.steering(self.steering_backend)
            drive = factory.drive(self.drive_backend)

            steering.connect()
            if drive is not steering:  # combined Build HAT: connect once
                drive.connect()
            self.get_logger().info("Motor drivers connected")

            # Center steering on startup
            steering.center_steering()
            self.get_logger().info("Steering centered")

            self.steering = steering
            self.drive = drive

        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect motor drivers: {e}")
            self.steering = None
            self.drive = None

        self.steering_pos_pub = self.create_lifecycle_publisher(Float32, "/motor/steering_position", 10)
        self.drive_speed_pub = self.create_lifecycle_publisher(Float32, "/motor/drive_speed", 10)
        self.status_pub = self.create_lifecycle_publisher(DiagnosticStatus, "/motor/status", 10)

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start accepting Ackermann commands and start the feedback/watchdog timers."""
        self.get_logger().info("Activating Ackermann Motor Node")

        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            "/ackermann_cmd",
            self._ackermann_callback,
            10,
        )
        self.feedback_timer = self.create_timer(1.0 / PUBLISHER_RATE_HZ, self._publish_feedback)
        self.watchdog_timer = self.create_timer(0.5, self._watchdog_check)  # 500ms watchdog

        return super().on_activate(state)

    @override
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop accepting commands and stop the motors.

        Order matters: the motors are stopped BEFORE the subscription is torn
        down, so a command that's already mid-callback still lands on a live
        publisher/driver, and no further commands can arrive once this returns.
        """
        self.get_logger().info("Deactivating Ackermann Motor Node")
        self._stop_motors_safely()
        self._destroy_sub_and_timers()
        return super().on_deactivate(state)

    @override
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop the motors (defensive) and tear down publishers."""
        self.get_logger().info("Cleaning up Ackermann Motor Node")
        self._stop_motors_safely()
        self._destroy_sub_and_timers()
        self._destroy_publishers()
        self.steering = None
        self.drive = None
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info("Shutting down Ackermann Motor Node")
        self._stop_motors_safely()
        self._destroy_sub_and_timers()
        self._destroy_publishers()
        self.steering = None
        self.drive = None
        return TransitionCallbackReturn.SUCCESS

    def _stop_motors_safely(self) -> None:
        """Command the motors to a safe stopped state. Never raises."""
        if self.steering is not None and self.drive is not None:
            try:
                self.drive.stop_drive()
                self.steering.center_steering()
                self.get_logger().info("Motors stopped and steering centered")
            except (RuntimeError, OSError, ValueError) as e:
                self.get_logger().error(f"Error stopping motors: {e}")

    def _destroy_sub_and_timers(self) -> None:
        if self.ackermann_sub is not None:
            self.destroy_subscription(self.ackermann_sub)
            self.ackermann_sub = None
        for timer_attr in ("feedback_timer", "watchdog_timer"):
            timer = getattr(self, timer_attr)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                setattr(self, timer_attr, None)

    def _destroy_publishers(self) -> None:
        for pub_attr in ("steering_pos_pub", "drive_speed_pub", "status_pub"):
            pub = getattr(self, pub_attr)
            if pub is not None:
                self.destroy_publisher(pub)
                setattr(self, pub_attr, None)

    @override
    def destroy_node(self) -> None:
        """Stop the motors and release everything, regardless of lifecycle state.

        Handles a node destroyed without a clean lifecycle shutdown (e.g.
        process killed mid-active, or a test that never triggers shutdown) --
        this is the final safety net, so it must never skip the motor stop.
        """
        self._stop_motors_safely()
        self._destroy_sub_and_timers()
        self._destroy_publishers()
        return super().destroy_node()

    def _ackermann_callback(self, msg: AckermannDriveStamped) -> None:
        """Handle incoming Ackermann drive commands.

        Args:
            msg: Ackermann drive command with speed and steering angle.
        """
        if self.steering is None or self.drive is None or self.config is None:
            return

        # Extract velocity and steering angle from message
        velocity = msg.drive.speed  # m/s
        steering_angle_rad = msg.drive.steering_angle  # radians

        # Convert steering angle from radians to degrees
        steering_angle_deg = math.degrees(steering_angle_rad)

        # Apply steering offset calibration
        calibrated_steering = steering_angle_deg + self.config.steering.offset

        # Clamp steering to safe limits
        max_angle = self.config.steering.max_steering_angle
        clamped_steering = max(-max_angle, min(max_angle, calibrated_steering))

        if abs(calibrated_steering) > max_angle:
            self.get_logger().warning(
                f"Steering angle {calibrated_steering:.2f}° exceeds limit ±{max_angle}°, "
                f"clamped to {clamped_steering:.2f}°",
            )

        # Convert velocity to motor speed percentage
        motor_speed = int(velocity * self.config.drive.speed_scale)

        # Apply drive reversal if configured
        if self.config.drive.reversed:
            motor_speed = -motor_speed

        # Clamp motor speed to safe limits
        max_speed = self.config.drive.max_speed
        clamped_speed = max(-max_speed, min(max_speed, motor_speed))

        if abs(motor_speed) > max_speed:
            self.get_logger().warning(
                f"Motor speed {motor_speed} exceeds limit ±{max_speed}, clamped to {clamped_speed}",
            )

        # Update tracking variables
        self.current_speed = clamped_speed
        self.current_steering_angle = clamped_steering
        self.last_command_time = self.get_clock().now().nanoseconds / 1e9

        # Execute motor commands
        try:
            # Set steering position
            self.steering.move_steering_to(clamped_steering, speed=STEERING_COMMAND_SPEED)

            # Set drive motor speed
            if clamped_speed > 0:
                self.drive.run_drive_forward(abs(clamped_speed))
            elif clamped_speed < 0:
                self.drive.run_drive_reverse(abs(clamped_speed))
            else:
                self.drive.stop_drive()

            self.get_logger().debug(
                f"Motor command: speed={clamped_speed}, steering={clamped_steering:.2f}° "
                f"(offset={self.config.steering.offset}°, reverse={self.config.drive.reversed})",
            )

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().error(f"Failed to execute motor command: {e}")

    def _publish_feedback(self) -> None:
        """Publish motor position and speed feedback."""
        if (
            self.steering is None
            or self.drive is None
            or self.config is None
            or self.steering_pos_pub is None
            or self.drive_speed_pub is None
            or self.status_pub is None
            or self.steering_backend is None
            or self.drive_backend is None
        ):
            return

        try:
            # Get current motor states
            steering_pos = self.steering.get_steering_position()
            drive_speed = self.drive.get_drive_speed()

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
            status_msg.hardware_id = f"{self.steering_backend.value}+{self.drive_backend.value}"

            status_msg.values.append(KeyValue(key="steering_position", value=f"{steering_pos:.2f}"))
            status_msg.values.append(KeyValue(key="drive_speed", value=f"{drive_speed:.2f}"))
            status_msg.values.append(KeyValue(key="commanded_speed", value=f"{self.current_speed}"))
            status_msg.values.append(KeyValue(key="commanded_steering", value=f"{self.current_steering_angle:.2f}"))
            status_msg.values.append(KeyValue(key="steering_offset", value=f"{self.config.steering.offset:.2f}"))
            status_msg.values.append(KeyValue(key="reverse_drive", value=str(self.config.drive.reversed)))

            self.status_pub.publish(status_msg)

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().warning(f"Failed to read motor feedback: {e}")

    def _watchdog_check(self) -> None:
        """Watchdog to stop motors if no commands received recently."""
        if self.drive is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_command = current_time - self.last_command_time

        # If no command received in 1 second, ensure motors are stopped
        if time_since_last_command > 1.0 and self.current_speed != 0.0:
            self.get_logger().warning(
                f"No Ackermann commands received for {time_since_last_command:.2f}s - stopping motors for safety",
            )
            try:
                self.drive.stop_drive()
                self.current_speed = 0.0
            except (RuntimeError, OSError, ValueError) as e:
                self.get_logger().error(f"Failed to stop motors in watchdog: {e}")


def main(args: list[str] | None = None) -> None:
    """Run the Ackermann motor node, auto-configuring and auto-activating on launch."""
    rclpy.init(args=args)
    node = AckermannMotorNode()

    try:
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

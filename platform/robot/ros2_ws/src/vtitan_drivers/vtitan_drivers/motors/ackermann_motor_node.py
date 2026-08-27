"""ROS2 lifecycle node for Ackermann motor control with selectable actuator backends.

Run on: Raspberry Pi Zero (connected to Raspberry Pi 5 via network)

Steering and drive are independent backends, chosen at runtime:
    - Default: servo steering + L298N/BTS7960 drive H-bridge, each paired
      with an independently-wired quadrature encoder
      (``src.hardware.motors.encoder``) for closed-loop RPM control via
      ``ClosedLoopDrive``. The encoder is a separate physical part from the
      H-bridge, so it is wired the same way regardless of which H-bridge is
      selected.
    - Build HAT: one combined steering+drive object, reused for both sides.
      Needs no external encoder -- its LEGO motors report their own
      position/speed through the Build HAT protocol.

Hardware connects in on_configure() and command handling starts in
on_activate(), matching the driver lifecycle pattern used by every hardware
node in this package (see button_node.py for the reference implementation).
Safety-critical: on_deactivate(), on_cleanup(), on_shutdown() and
destroy_node() all route through _stop_motors_safely() so the motors are
always commanded to stop and steering re-centered, regardless of which
teardown path the node takes.

Usage:
    ros2 run vtitan_drivers ackermann_motor_node

Topics:
    Subscribed:
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - Ackermann drive commands
    Published:
        - /motor/steering_position (std_msgs/Float32) - Current steering position in degrees
        - /motor/drive_speed (std_msgs/Float32) - Current drive speed in degrees/s
        - /motor/status (diagnostic_msgs/DiagnosticStatus) - Motor status diagnostics
        - /joint_states (sensor_msgs/JointState) - Wheel angle + rate and steering
          angle, in SI units. Carries a header timestamp and an accumulating
          wheel angle, neither of which the Float32 topics above can express;
          wheel distance is angle x wheel radius. Published alongside them
          rather than replacing them, because drive_speed and steering_position
          are telemetry payload fields reaching the proto, OpenAPI contract and
          frontend dials.

Environment Variables:
    Note the DOUBLE underscore in the MOTOR_* names: they are nested
    pydantic-settings fields (Config.steering / Config.drive), so the
    delimiter is ``__``. ``MOTOR_DRIVE_REVERSED`` (single) is silently
    ignored; it must be ``MOTOR_DRIVE__REVERSED``.

    STEERING_BACKEND: servo | build_hat (default: servo)
    DRIVE_BACKEND: l298n | bts7960 | build_hat (default: l298n)
    MOTOR_STEERING__OFFSET: Steering center angle offset in degrees
    MOTOR_STEERING__MAX_STEERING_ANGLE: Maximum steering angle in degrees
    MOTOR_STEERING__REVERSED: Invert steering direction
    MOTOR_DRIVE__REVERSED: Invert drive motor direction (true on this robot --
        see .env.example and docs/sensor-verification.md)
    MOTOR_DRIVE__MIN_SPEED / MOTOR_DRIVE__MAX_SPEED: Drive speed clamp
    MOTOR_DRIVE__SPEED_SCALE: motor_speed = velocity_m_s * scale
    See src.hardware.motors.config.Config for the full set and defaults.

    H-bridge/encoder pin numbers are NOT read here -- they live on
    L298nPwmConfig/Bts7960PwmConfig/EncoderConfig (TOML + env, same pattern
    as the servo's ServoConfig.gpio_pin):
        L298N drive pins: config/hardware/motors/l298n.toml
            (env: L298N_PWM_PWM_PIN, L298N_PWM_DIR_A_PIN, L298N_PWM_DIR_B_PIN)
        BTS7960 drive pins: config/hardware/motors/bts7960.toml
            (env: BTS7960_PWM_FORWARD_PWM_PIN, BTS7960_PWM_REVERSE_PWM_PIN,
            BTS7960_PWM_R_EN_PIN, BTS7960_PWM_L_EN_PIN; see
            docs/bts7960-ibt2-wiring.md)
        Encoder pins (either H-bridge backend): config/hardware/motors/encoder.toml
            (env: ENCODER_PIN_A, ENCODER_PIN_B)
    Servo steering: SERVO_* (see src.hardware.motors.servo.config.ServoConfig)
    Build HAT (when selected): MOTOR_STEERING__PORT, MOTOR_DRIVE__PORT, ...
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, override

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import JointState
from shared.config.constants import RobotSpecs
from shared.config.ros_topics import RosTopicConfig
from std_msgs.msg import Float32

from src.hardware.motors.base import ClosedLoopDrive
from src.hardware.motors.config import Config
from src.hardware.motors.encoder import EncoderConfig, PIDController, QuadratureEncoder
from src.hardware.motors.encoder.calibration import DEFAULT_WHEEL_DIAMETER_M
from src.hardware.motors.enums import (
    DRIVE_JOINT,
    STEERING_JOINT,
    DriveBackend,
    SteeringBackend,
)
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging
from src.ros2.params import declare_and_get_str_param
from src.ros2.qos import QOS_STREAM

# .env loading is a side effect of importing src.logger.config -- the
# servo/l298n backends (this node's defaults) never import src.logger
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


class NodeConfig(HardwareBaseSettings):
    """Node-level timing, configurable via config/hardware/motors/ackermann_motor_node.toml.

    Matches every hardware driver's Config pattern -- separate from
    motors.toml/l298n.toml/bts7960.toml/encoder.toml/servo.toml alongside it,
    which are driver-level config (offsets, PID gains, pins), not this node's
    timer rates.
    """

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "motors" / "ackermann_motor_node.toml")

    publisher_rate_hz: float = Field(
        default=20.0, validation_alias=AliasChoices("PUBLISHER_RATE_HZ", "publisher_rate_hz"),
    )
    """Rate for publishing motor state (steering position, drive speed, joint states).

    Was 100Hz -- telemetry consumed by a UI dial or an occasional motion
    prior doesn't need 10ms latency, and rebuilding + publishing 3 messages
    that often was a measurable, unnecessary CPU cost on the Pi Zero this
    node runs on (alongside the same-shaped fix already applied to
    /ui/telemetry_summary, /robot_state and /system_status). 20Hz (50ms) is
    still well under human perception for a dial and far above what a motion
    prior integrates against.
    """

    diagnostics_rate_hz: float = Field(
        default=2.0, validation_alias=AliasChoices("DIAGNOSTICS_RATE_HZ", "diagnostics_rate_hz"),
    )
    """Rate for publishing /motor/status diagnostics.

    Deliberately much slower than publisher_rate_hz: DiagnosticStatus is for
    a human or a monitoring dashboard, not a control loop, and building it
    involves 7 KeyValue allocations + f-string formats per call -- paying
    that cost 100x/s for a value that changes on human timescales was pure
    waste.
    """


_node_config = NodeConfig()
PUBLISHER_RATE_HZ = _node_config.publisher_rate_hz
DIAGNOSTICS_RATE_HZ = _node_config.diagnostics_rate_hz

DRIVE_CONTROL_RATE_HZ = 50.0
"""Rate of the closed-loop drive step.

Free to change: ``ClosedLoopDrive.run_drive_at_rpm()`` and
``QuadratureEncoder.get_rpm()`` both measure the real wall-clock interval
between calls rather than assuming a fixed one (the node measures its own
elapsed time for the PID's ``dt``; the encoder measures its own separately
for the speed estimator's ``dt`` -- see ``encoder/driver.py``'s
``_elapsed()``), specifically so the PID gains and speed estimate stay
correct if this rate ever changes.
"""

_NOMINAL_PID_DT_S = 1.0 / DRIVE_CONTROL_RATE_HZ
"""Assumed PID step on the first closed-loop tick, before a real interval can be measured."""

_MIN_PID_DT_S = 0.001
_MAX_PID_DT_S = 0.5
"""Bounds on a measured PID step, so a duplicate call or a stall can't blow up the loop."""



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
        return DriveBackend.L298N


class _DriverFactory:
    """Build the steering/drive/encoder drivers for the configured backends.

    The Build HAT driver is a single combined steering+drive object, so when
    both backends select it the same instance is reused (one GPIO/serial open).
    Driver classes are imported lazily so an unused backend need not be
    installed on the host.

    Pin numbers and PWM wiring are NOT constructor args here -- each backend
    driver reads them from its own TOML/env-backed config
    (``L298nPwmConfig``/``Bts7960PwmConfig``/``EncoderConfig``), the same way
    ``ServoConfig`` already owns the servo's ``gpio_pin``. This factory only
    owns which backend to build and the software-level ``reversed`` flags
    (``Config.drive``), which are not wiring facts.
    """

    def __init__(self, config: Config) -> None:
        self._config = config
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
        """Build the drive H-bridge driver for ``backend``. Never a ``ClosedLoopDrive``.

        # Direction lives in the driver, not the node: the closed loop runs
        # PID -> run_drive_forward/reverse() and never passes through the
        # node, so a negation applied there would be silently skipped (it
        # was, and the robot drove backwards on the first closed-loop run).
        """
        if backend is DriveBackend.BUILD_HAT:
            return self._shared_build_hat()  # combined object also satisfies DriveDriver
        if backend is DriveBackend.BTS7960:
            from src.hardware.motors.bts7960 import Driver  # noqa: PLC0415 - lazy: only when selected

            return Driver(invert=self._config.drive.reversed)

        from src.hardware.motors.l298n import Driver  # noqa: PLC0415 - lazy: only when selected

        return Driver(invert=self._config.drive.reversed)

    def encoder(self, backend: DriveBackend) -> QuadratureEncoder | None:
        """Build the drive encoder, unless ``backend`` reports its own feedback (Build HAT)."""
        if backend is DriveBackend.BUILD_HAT:
            return None
        encoder_config = EncoderConfig()
        return QuadratureEncoder(
            pin_a=encoder_config.pin_a,
            pin_b=encoder_config.pin_b,
            counts_per_rev=encoder_config.counts_per_rev,
            wheel_diameter_m=DEFAULT_WHEEL_DIAMETER_M,
            # DEFAULT_WHEEL_DIAMETER_M derives from RobotSpecs.WHEEL_RADIUS,
            # not restated as a config literal -- see encoder/config.py.
            invert=self._config.drive.encoder_reversed,
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

        self._topics = RosTopicConfig.load_default()

        self.config: Config | None = None
        self.steering_backend: SteeringBackend | None = None
        self.drive_backend: DriveBackend | None = None
        self.steering: SteeringDriver | None = None
        self.drive: DriveDriver | ClosedLoopDrive | None = None

        self.steering_pos_pub: Publisher | None = None
        self.drive_speed_pub: Publisher | None = None
        self.status_pub: Publisher | None = None
        self.joint_state_pub: Publisher | None = None
        self.ackermann_sub: Subscription | None = None
        self.feedback_timer: Timer | None = None
        # Initialised here, not only in on_activate(): _destroy_sub_and_timers
        # reads all three by name, so a node torn down before activation raised
        # AttributeError and never reached _stop_motors_safely() -- on the one
        # path where the motors are least likely to already be stopped.
        self.control_timer: Timer | None = None
        self.watchdog_timer: Timer | None = None
        self.diagnostics_timer: Timer | None = None

        # Current command tracking
        self.current_speed: float = 0.0
        self.target_wheel_rpm: float = 0.0
        self.current_steering_angle: float = 0.0
        self.last_command_time: float = 0.0
        # Elapsed-time tracker for the closed-loop PID's dt -- owned by the
        # node now that ClosedLoopDrive composes a plain DriveDriver + a
        # separately-timed EncoderSensor, rather than a single object timing
        # its own PID internally.
        self._last_pid_time: float | None = None

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
            declare_and_get_str_param(self, "drive_backend", "l298n"),
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

        # H-bridge + encoder pin numbers are NOT ROS2 params any more -- they
        # live on L298nPwmConfig/Bts7960PwmConfig/EncoderConfig
        # (TOML/env-backed), the same way ServoConfig already owns the
        # servo's gpio_pin. See _DriverFactory's docstring.

        # Motor drivers (steering and drive may be one combined object or two)
        self.steering = None
        self.drive = None
        try:
            factory = _DriverFactory(config)
            steering = factory.steering(self.steering_backend)
            raw_drive = factory.drive(self.drive_backend)
            encoder = factory.encoder(self.drive_backend)

            steering.connect()
            if encoder is not None:
                encoder_config = EncoderConfig()
                pid = PIDController(
                    kp=encoder_config.pid_kp,
                    ki=encoder_config.pid_ki,
                    kd=encoder_config.pid_kd,
                    feedforward=1.0 / encoder_config.max_rpm,
                    output_min=-encoder_config.max_duty,
                    output_max=encoder_config.max_duty,
                )
                drive: DriveDriver | ClosedLoopDrive = ClosedLoopDrive(raw_drive, encoder, pid)
                drive.connect()
            else:
                drive = raw_drive
                if drive is not steering:  # combined Build HAT: connect once
                    drive.connect()
            self.get_logger().info("Motor drivers connected")

            # Center steering on startup
            steering.center_steering()
            self.get_logger().info("Steering centered")

            self.steering = steering
            self.drive = drive
            if not isinstance(drive, ClosedLoopDrive):
                # Build HAT has no external encoder, so _drive_control_step's
                # closed-loop RPM control can't run against it -- warn once at
                # startup instead of only discovering it as silent no-ops.
                self.get_logger().warning(
                    f"drive_backend={self.drive_backend.value} has no encoder feedback; "
                    "closed-loop RPM control is disabled for this run",
                )

        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect motor drivers: {e}")
            self.steering = None
            self.drive = None

        self.steering_pos_pub = self.create_lifecycle_publisher(Float32, self._topics.actuators.steering_position, QOS_STREAM)
        self.drive_speed_pub = self.create_lifecycle_publisher(Float32, self._topics.actuators.drive_speed, QOS_STREAM)
        self.status_pub = self.create_lifecycle_publisher(DiagnosticStatus, self._topics.actuators.status, QOS_STREAM)
        self.joint_state_pub = self.create_lifecycle_publisher(JointState, self._topics.actuators.joint_states, QOS_STREAM)

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start accepting Ackermann commands and start the feedback/watchdog timers."""
        self.get_logger().info("Activating Ackermann Motor Node")

        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            self._topics.commands.ackermann_cmd,
            self._ackermann_callback,
            QOS_STREAM,
        )
        self.feedback_timer = self.create_timer(1.0 / PUBLISHER_RATE_HZ, self._publish_feedback)
        self.control_timer = self.create_timer(1.0 / DRIVE_CONTROL_RATE_HZ, self._drive_control_step)
        self.watchdog_timer = self.create_timer(0.5, self._watchdog_check)  # 500ms watchdog
        self.diagnostics_timer = self.create_timer(1.0 / DIAGNOSTICS_RATE_HZ, self._publish_diagnostics)

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
        """Stop the motors, release the drivers, and tear down publishers."""
        self.get_logger().info("Cleaning up Ackermann Motor Node")
        self._stop_motors_safely()
        self._disconnect_drivers()
        self._destroy_sub_and_timers()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info("Shutting down Ackermann Motor Node")
        self._stop_motors_safely()
        self._disconnect_drivers()
        self._destroy_sub_and_timers()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    def _disconnect_drivers(self) -> None:
        """Release the motor drivers' hardware. Never raises.

        Without this, ``on_cleanup``/``on_shutdown`` only zeroed the drive PWM
        duty cycle (via ``_stop_motors_safely``) and dropped this node's
        references -- the sysfs PWM channel stayed exported with ``enable=1``
        and the GPIO direction/encoder lines stayed reserved indefinitely.
        Guards against ``drive is steering`` the same way ``on_configure``
        connects once for a combined driver (e.g. the Build HAT).
        """
        try:
            if self.steering is not None:
                self.steering.disconnect()
            if self.drive is not None and self.drive is not self.steering:
                self.drive.disconnect()
            self.get_logger().info("Motor drivers disconnected")
        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().error(f"Error disconnecting motor drivers: {e}")
        self.steering = None
        self.drive = None

    def _stop_motors_safely(self) -> None:
        """Command the motors to a safe stopped state. Never raises."""
        # Zero the setpoint before stopping so a still-running control timer
        # cannot immediately re-command the motor it was just asked to stop.
        self.target_wheel_rpm = 0.0
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
        for timer_attr in ("feedback_timer", "control_timer", "watchdog_timer", "diagnostics_timer"):
            timer = getattr(self, timer_attr)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                setattr(self, timer_attr, None)

    def _destroy_publishers(self) -> None:
        for pub_attr in ("steering_pos_pub", "drive_speed_pub", "status_pub", "joint_state_pub"):
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
        self._disconnect_drivers()
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

        # The message carries a WHEEL angle (ROS convention, and what the
        # navigator and simulator both reason about). The servo needs a SERVO
        # angle, and the linkage is geared between them -- feeding one straight
        # into the other made the wheels under-turn by ~22%.
        wheel_angle_deg = math.degrees(steering_angle_rad)
        servo_angle_deg = wheel_angle_deg / self.config.steering.linkage_ratio

        # Offset is a servo-side mechanical trim, so it applies after conversion.
        calibrated_steering = servo_angle_deg + self.config.steering.offset

        # max_steering_angle is the SERVO's travel limit, not the wheel's.
        max_angle = self.config.steering.max_steering_angle
        clamped_steering = max(-max_angle, min(max_angle, calibrated_steering))

        if abs(calibrated_steering) > max_angle:
            self.get_logger().warning(
                f"Servo angle {calibrated_steering:.2f}° (wheel {wheel_angle_deg:.2f}°) "
                f"exceeds limit ±{max_angle}°, clamped to {clamped_steering:.2f}°",
            )

        # Convert velocity (true m/s) to a WHEEL rpm setpoint for the closed
        # loop. Open-loop duty was battery-dependent -- the identical command
        # travelled 43 cm on a tired pack and 54 cm on a fresh one -- so a
        # commanded speed only means something if the loop measures it.
        target_wheel_rpm = velocity / (math.pi * RobotSpecs.WHEEL_RADIUS * 2.0) * 60.0

        # Kept only for the diagnostics/clamp below, which still report duty.
        # Deliberately NOT negated for drive.reversed any more -- the driver
        # owns direction now (see the factory), and negating here as well would
        # report a commanded_speed whose sign disagrees with the actual motion.
        motor_speed = int(velocity * self.config.drive.speed_scale)

        # Clamp motor speed to safe limits
        max_speed = self.config.drive.max_speed
        clamped_speed = max(-max_speed, min(max_speed, motor_speed))

        if abs(motor_speed) > max_speed:
            self.get_logger().warning(
                f"Motor speed {motor_speed} exceeds limit ±{max_speed}, clamped to {clamped_speed}",
            )

        # Update tracking variables
        self.current_speed = clamped_speed
        self.target_wheel_rpm = target_wheel_rpm
        self.current_steering_angle = clamped_steering
        self.last_command_time = self.get_clock().now().nanoseconds / 1e9

        # Execute motor commands
        try:
            # Set steering position
            self.steering.move_steering_to(clamped_steering, speed=self.config.steering.turning_speed)

            # The drive is NOT actuated here: the PID needs a fixed 50 Hz step
            # (see DRIVE_CONTROL_RATE_HZ) whereas commands arrive at whatever
            # rate the publisher chooses. _drive_control_step() applies it.
            if target_wheel_rpm == 0.0:
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
            # Get current motor states. The driver reports a SERVO angle;
            # convert back to a WHEEL angle so the published feedback is in the
            # same frame as the commands on /ackermann_cmd. Publishing servo
            # degrees against wheel-degree commands would make any consumer
            # comparing the two -- or closing a loop on them -- silently wrong.
            steering_pos = self.steering.get_steering_position() * self.config.steering.linkage_ratio
            drive_speed = self.drive.get_drive_speed()

            # Publish steering position
            steering_msg = Float32()
            steering_msg.data = steering_pos
            self.steering_pos_pub.publish(steering_msg)

            # Publish drive speed
            speed_msg = Float32()
            speed_msg.data = drive_speed
            self.drive_speed_pub.publish(speed_msg)

            # Publish the same feedback as JointState, which telemetry_bridge_node
            # already subscribes to and which nothing has ever published.
            #
            # It carries what the two Float32 topics cannot: a header timestamp,
            # and an *accumulating* wheel angle rather than an instantaneous
            # speed. Both are what a motion prior needs -- wheel distance is
            # angle x wheel radius, and integrating it between LIDAR scans is
            # only meaningful if you know when each sample was taken.
            #
            # Deliberately alongside the Float32 topics rather than replacing
            # them: drive_speed and steering_position are telemetry payload
            # fields carried through the proto, the OpenAPI contract, generated
            # Go and the frontend dials, so the ROS-side representation can move
            # to SI units here without disturbing any of that.
            if self.joint_state_pub is not None:
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = [DRIVE_JOINT, STEERING_JOINT]
                # SI, unlike the degree-based Float32 topics above.
                joint_msg.position = [
                    math.radians(self.drive.get_drive_position()),
                    math.radians(steering_pos),
                ]
                # Steering velocity is left at zero: the servo reports position
                # only, and a derived rate would be a differentiated command
                # rather than a measurement.
                joint_msg.velocity = [math.radians(drive_speed), 0.0]
                self.joint_state_pub.publish(joint_msg)

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().warning(f"Failed to read motor feedback: {e}")

    def _publish_diagnostics(self) -> None:
        """Publish /motor/status diagnostics.

        Runs on its own slower timer (see DIAGNOSTICS_RATE_HZ) -- separate from
        _publish_feedback's steering/speed/joint-state topics, which telemetry
        and any closed loop actually consume at a real-time rate.
        """
        if (
            self.steering is None
            or self.drive is None
            or self.config is None
            or self.status_pub is None
            or self.steering_backend is None
            or self.drive_backend is None
        ):
            return

        try:
            # Same wheel-frame conversion as _publish_feedback -- see its
            # comment for why the driver's servo-degree reading is converted.
            steering_pos = self.steering.get_steering_position() * self.config.steering.linkage_ratio
            drive_speed = self.drive.get_drive_speed()

            status_msg = DiagnosticStatus()
            status_msg.name = "Ackermann Motors"
            status_msg.level = DiagnosticStatus.OK
            status_msg.message = "Motors operational"
            status_msg.hardware_id = f"{self.steering_backend.value}+{self.drive_backend.value}"

            status_msg.values.append(KeyValue(key="steering_position", value=f"{steering_pos:.2f}"))
            status_msg.values.append(KeyValue(key="drive_speed", value=f"{drive_speed:.2f}"))
            # Raw quadrature counts. drive_speed above is smoothed and rate-derived,
            # so integrating it to recover distance folds in the estimator's
            # smoothing and sampling interval; the counter is exact and is what
            # encoder calibration must be measured against. Only a
            # ClosedLoopDrive (L298N/BTS7960 + encoder) exposes this -- Build
            # HAT is a plain DriveDriver with no encoder counts, so this field
            # is omitted rather than crashing on it.
            if isinstance(self.drive, ClosedLoopDrive):
                status_msg.values.append(KeyValue(key="encoder_counts", value=str(self.drive.get_drive_counts())))
            status_msg.values.append(KeyValue(key="commanded_speed", value=f"{self.current_speed}"))
            status_msg.values.append(KeyValue(key="commanded_steering", value=f"{self.current_steering_angle:.2f}"))
            status_msg.values.append(KeyValue(key="steering_offset", value=f"{self.config.steering.offset:.2f}"))
            status_msg.values.append(KeyValue(key="reverse_drive", value=str(self.config.drive.reversed)))

            self.status_pub.publish(status_msg)

        except (RuntimeError, OSError, ValueError) as e:
            self.get_logger().warning(f"Failed to read motor diagnostics: {e}")

    def _drive_control_step(self) -> None:
        """One closed-loop drive step: PID the duty toward the target wheel rpm.

        Runs on its own fixed-rate timer rather than in the command callback so
        the PID sees the constant dt it assumes. A zero setpoint goes through
        stop_drive() instead of the loop, so the integrator cannot wind up
        against a target the motor is not meant to reach.
        """
        if self.drive is None:
            return
        if not isinstance(self.drive, ClosedLoopDrive):
            # Closed-loop RPM control needs encoder feedback -- Build HAT is a
            # plain DriveDriver with no encoder counts, so this backend simply
            # cannot run this loop. Warned about in on_configure(); nothing
            # more useful to do here each tick than stay silent and no-op.
            return
        try:
            if self.target_wheel_rpm == 0.0:
                # Still sample, so the speed estimate this loop owns stays
                # current for the feedback publisher (which no longer samples
                # for itself) and reads ~0 while stopped rather than freezing
                # at the last moving value.
                self.drive.get_drive_rpm()
                return
            self.drive.run_drive_at_rpm(self.target_wheel_rpm, dt=self._pid_elapsed())
        except Exception as e:  # noqa: BLE001 - any drive fault must stop the motors
            # RcutilsLogger has no exception() (only debug/info/warning/error/
            # fatal) -- calling it here would raise AttributeError instead of
            # stopping the motors, the one thing this handler exists to do.
            self.get_logger().error(f"Closed-loop drive step failed; stopping motors: {type(e).__name__}: {e}")
            self.drive.stop_drive()
            self.target_wheel_rpm = 0.0

    def _pid_elapsed(self) -> float:
        """Seconds since the last closed-loop PID step, seeding it on first use."""
        now = time.monotonic()
        previous = self._last_pid_time
        self._last_pid_time = now
        if previous is None:
            return _NOMINAL_PID_DT_S
        return min(max(now - previous, _MIN_PID_DT_S), _MAX_PID_DT_S)

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
                # Clear the setpoint FIRST: the closed loop re-commands the
                # target every 20 ms, so stopping the motor without zeroing it
                # would just be undone on the next control step.
                self.target_wheel_rpm = 0.0
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

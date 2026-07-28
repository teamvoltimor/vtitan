from pydantic import BaseModel
from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class MotorSteeringConfig(BaseModel):
    """Steering configuration."""

    port: str
    """
    Port for steering motor. This should be the identifier for the motor controller (e.g., serial port, I2C address, etc.) that controls the steering motor.
    """

    offset: float = 0.0
    """Steering center angle offset in degrees for calibration. Positive = bias right, Negative = bias left."""

    left_limit_angle: float
    """
    Left limit for steering position in degrees. This defines the maximum left turn angle for the steering motor.
    """

    right_limit_angle: float
    """
    Right limit for steering position in degrees. This defines the maximum right turn angle for the steering motor.
    """

    center_angle: float
    """Center position for steering in degrees. This defines the angle that corresponds to the centered steering position."""

    max_steering_angle: float = 45.0
    """Maximum steering angle (absolute value) in degrees. Commands beyond ±this angle are clamped for safety."""

    centering_speed: int = 20
    """Speed for centering steering. This can be used to define how quickly the steering motor should move when centering the wheels."""

    turning_speed: int = 30
    """Default speed for turning steering. This can be used as a default speed when moving the steering motor to a specific position, allowing for consistent and predictable steering behavior."""

    reversed: bool = False
    """Whether the steering motor is reversed. This can be used to invert the direction of the steering motor if it is mounted in a way that causes left commands to actually turn the wheels right."""

    linkage_ratio: float = 1.0
    """Road-wheel degrees produced per servo degree.

    Measured 0.78 on this chassis (servo 90 deg -> wheels ~70 deg). Everything
    upstream -- /ackermann_cmd, the navigator, the simulator -- speaks in WHEEL
    angles, per the ROS convention; only the servo speaks servo angles. Without
    this conversion the node fed a wheel angle straight to the servo and the
    wheels under-turned by ~22%, so the robot consistently cornered wider than
    the path it was following.

    1.0 means "servo angle is the wheel angle", i.e. direct-drive steering.
    """


class MotorDriveConfig(BaseModel):
    """Drive configuration."""

    port: str
    """
    Port for drive motor. This should be the identifier for the motor controller (e.g., serial port, I2C address, etc.) that controls the drive motor.
    """

    reversed: bool = False
    """Whether the drive motor is reversed. This can be used to invert the direction of the drive motor if it is mounted in a way that causes forward commands to actually move the robot backward."""

    encoder_reversed: bool = False
    """Whether the encoder counts up when the robot moves backward.

    Independent of ``reversed`` on purpose: the motor leads and the encoder's
    A/B channels are separate connections, so inverting one does not invert the
    other. ``reversed`` also negates the command in software rather than
    rewiring, which leaves the encoder reporting true physical rotation against
    a flipped command frame -- so on this robot both flags are set. Wrong here
    and odometry integrates backwards and a closed speed loop sees inverted
    error, which is a runaway rather than a wrong number.
    """

    min_speed: int
    """Minimum speed for drive motor. This can be used to define the lowest speed at which the drive motor can operate effectively."""

    max_speed: int
    """Maximum speed for drive motor. This can be used to define the highest speed at which the drive motor can operate safely."""

    speed_scale: float = 1.0
    """Scaling factor for drive motor speed. This can be used to adjust the speed commands sent to the drive motor, allowing for fine-tuning of the motor's responsiveness."""

    default_speed: int
    """Default speed for drive motor. This can be used as a fallback speed if no specific speed is provided when running the drive motor."""


class Config(HardwareBaseSettings):
    """Motor configuration."""

    model_config = SettingsConfigDict(
        env_prefix="motor_",
        # "__" (not "_") so nested leaf names containing underscores parse
        # correctly, e.g. MOTOR_DRIVE__MIN_SPEED -> drive.min_speed.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "motors" / "motors.toml",
    )

    # Plain required nested fields (no default_factory): a default_factory
    # would construct the nested BaseModel with zero arguments, bypassing
    # pydantic-settings' env_nested_delimiter resolution entirely and always
    # failing validation regardless of whether MOTOR_STEERING__*/
    # MOTOR_DRIVE__* are set. Leaving them required lets the parent
    # BaseSettings populate them from the nested env vars itself.
    steering: MotorSteeringConfig

    drive: MotorDriveConfig

    test_duration: float
    """
    Duration in seconds for motor test routines. This can be used to specify how long the motors should run during testing.
    """

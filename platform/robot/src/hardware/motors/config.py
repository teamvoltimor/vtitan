from pydantic import BaseModel, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


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


class MotorDriveConfig(BaseModel):
    """Drive configuration."""

    port: str
    """
    Port for drive motor. This should be the identifier for the motor controller (e.g., serial port, I2C address, etc.) that controls the drive motor.
    """

    reversed: bool = False
    """Whether the drive motor is reversed. This can be used to invert the direction of the drive motor if it is mounted in a way that causes forward commands to actually move the robot backward."""

    min_speed: int
    """Minimum speed for drive motor. This can be used to define the lowest speed at which the drive motor can operate effectively."""

    max_speed: int
    """Maximum speed for drive motor. This can be used to define the highest speed at which the drive motor can operate safely."""

    speed_scale: float = 1.0
    """Scaling factor for drive motor speed. This can be used to adjust the speed commands sent to the drive motor, allowing for fine-tuning of the motor's responsiveness."""

    default_speed: int
    """Default speed for drive motor. This can be used as a fallback speed if no specific speed is provided when running the drive motor."""


class Config(BaseSettings):
    """Motor configuration."""

    model_config = SettingsConfigDict(
        env_prefix="motor_",
        # "__" (not "_") so nested leaf names containing underscores parse
        # correctly, e.g. MOTOR_DRIVE__MIN_SPEED -> drive.min_speed.
        env_nested_delimiter="__",
    )

    # MotorSteeringConfig/MotorDriveConfig have no field defaults for the
    # physical parameters (port, angle limits, speeds) -- there is no safe
    # universal default for those, so this factory only succeeds when the
    # nested env vars (MOTOR_STEERING__*/MOTOR_DRIVE__*) are set; mypy can't
    # see that env resolution, hence the ignores.
    steering: MotorSteeringConfig = Field(default_factory=lambda: MotorSteeringConfig())  # type: ignore[call-arg]

    drive: MotorDriveConfig = Field(default_factory=lambda: MotorDriveConfig())  # type: ignore[call-arg]

    test_duration: float
    """
    Duration in seconds for motor test routines. This can be used to specify how long the motors should run during testing.
    """

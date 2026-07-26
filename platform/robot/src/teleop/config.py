from pydantic_settings import BaseSettings, SettingsConfigDict


class Config(BaseSettings):
    """Joystick teleop configuration.

    Bench-testing tool: maps `sensor_msgs/Joy` (published by the stock
    `joy` package's `joy_node`) to `ackermann_msgs/AckermannDriveStamped`
    commands on `/ackermann_cmd`, so `ackermann_motor_node` needs no changes.

    The axis/button indices below are a best guess at a typical Linux
    joydev mapping and are UNVERIFIED for the 8BitDo Ultimate 2 specifically
    -- controllers report different indices depending on Bluetooth vs USB
    and which input mode they're in. After pairing, run `ros2 topic echo
    /joy` and wiggle each stick/button to confirm (or correct) these via
    the matching JOY_TELEOP_* env vars before relying on them.
    """

    model_config = SettingsConfigDict(env_prefix="joy_teleop_")

    steering_axis_index: int = 0
    """Joy `axes` index used for steering (left stick X in a typical mapping)."""

    throttle_axis_index: int = 4
    """Joy `axes` index used for drive speed (right stick Y in a typical mapping)."""

    deadman_button_index: int = 6
    """Joy `buttons` index that must be held (1) for the drive motor to move at all."""

    steering_invert: bool = False
    """Flip the sign of the steering axis reading."""

    throttle_invert: bool = False
    """Flip the sign of the throttle axis reading."""

    max_steering_deg: float = 30.0
    """Steering angle (deg) commanded at full stick deflection. Clamped to this range."""

    max_speed_mps: float = 0.3
    """Drive speed (m/s) commanded at full stick deflection. Kept low for bench safety."""

    publish_rate_hz: float = 20.0
    """Rate at which /ackermann_cmd is republished -- must stay well under
    ackermann_motor_node's 1s command watchdog."""

    joy_timeout_s: float = 0.5
    """If no /joy message arrives within this window (e.g. Bluetooth link
    dropped), the drive command is forced to zero regardless of the
    dead-man button's last known state."""

"""Motor backend selectors.

Typed backends let the motor node choose actuator drivers from configuration
without string comparisons. The Build HAT backend is preserved here even though
the current robot uses the servo + H-bridge pair, so the LEGO stack can be
re-selected for steering and/or drive (independently) without code changes.
"""

from enum import StrEnum


class SteeringBackend(StrEnum):
    """Steering actuator backend."""

    SERVO = "servo"
    BUILD_HAT = "build_hat"


class DriveBackend(StrEnum):
    """Drive H-bridge backend.

    Both L298N and BTS7960 pair with an independently-wired
    ``src.hardware.motors.encoder`` sensor for closed-loop feedback -- see
    ``base.py``'s ``ClosedLoopDrive``. Build HAT needs no external encoder;
    its LEGO motors report their own position/speed.
    """

    L298N = "l298n"
    BTS7960 = "bts7960"
    BUILD_HAT = "build_hat"


# ROS2 joint names
DRIVE_JOINT = "drive_wheel"
"""Joint state name for the drive motor."""

STEERING_JOINT = "steering"
"""Joint state name for the steering servo."""

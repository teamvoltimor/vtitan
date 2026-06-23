"""Motor backend selectors.

Typed backends let the motor node choose actuator drivers from configuration
without string comparisons. The Build HAT backend is preserved here even though
the current robot uses the servo + DC-encoder pair, so the LEGO stack can be
re-selected for steering and/or drive (independently) without code changes.
"""

from enum import Enum


class SteeringBackend(str, Enum):
    """Steering actuator backend."""

    SERVO = "servo"
    BUILD_HAT = "build_hat"


class DriveBackend(str, Enum):
    """Drive actuator backend."""

    DC_ENCODER = "dc_encoder"
    BUILD_HAT = "build_hat"

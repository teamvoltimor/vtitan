"""DC geared motor with quadrature encoder drive backend.

Implements the ``EncodedDriveDriver`` port for a raw brushed DC motor (e.g.
JGB37-520) driven through an external H-bridge with software closed-loop speed
control. The pure control primitives (`control.py`) and the
``SimulatedEncoderDriver`` (`simulated.py`) run anywhere; the hardware
``Driver`` (`driver.py`) lazily imports GPIO libraries so this package still
imports cleanly on dev machines.
"""

from src.hardware.motors.dc_encoder.control import (
    PIDController,
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
    revolutions_to_distance,
)
from src.hardware.motors.dc_encoder.driver import Driver
from src.hardware.motors.dc_encoder.simulated import SimulatedEncoderDriver

__all__ = [
    "Driver",
    "PIDController",
    "SimulatedEncoderDriver",
    "SpeedEstimator",
    "counts_to_distance",
    "counts_to_revolutions",
    "revolutions_to_distance",
]

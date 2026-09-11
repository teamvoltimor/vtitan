"""Quadrature-encoder sensor, independent of any H-bridge drive backend.

The pure control primitives (`control.py`) and the ``SimulatedEncoder``
(`simulated.py`) run anywhere; the hardware ``QuadratureEncoder`` (`driver.py`)
lazily imports GPIO libraries so this package still imports cleanly on dev
machines.
"""

from src.hardware.motors.encoder.config import EncoderConfig
from src.hardware.motors.encoder.control import (
    PIDController,
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
    revolutions_to_distance,
)
from src.hardware.motors.encoder.driver import QuadratureEncoder
from src.hardware.motors.encoder.simulated import SimulatedEncoder

__all__ = [
    "EncoderConfig",
    "PIDController",
    "QuadratureEncoder",
    "SimulatedEncoder",
    "SpeedEstimator",
    "counts_to_distance",
    "counts_to_revolutions",
    "revolutions_to_distance",
]

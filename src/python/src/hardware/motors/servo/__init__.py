"""RC servo steering backend (single-servo parallel/crab steering).

Implements the ``SteeringDriver`` port for a hobby RC servo on a 50 Hz PWM
signal. The hardware ``Driver`` lazily imports gpiozero, so this package still
imports cleanly on dev machines without GPIO libraries.
"""

from src.hardware.motors.servo.config import ServoConfig
from src.hardware.motors.servo.driver import Driver

__all__ = ["Driver", "ServoConfig"]

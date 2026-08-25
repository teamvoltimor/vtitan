"""L298N (or TB6612FNG-class) H-bridge drive backend.

Encoder feedback lives in ``src.hardware.motors.encoder`` -- see that
package's docstring and ``base.py``'s ``ClosedLoopDrive`` for why it is not
part of this driver.
"""

from src.hardware.motors.l298n.config import L298nPwmConfig
from src.hardware.motors.l298n.driver import Driver

__all__ = [
    "Driver",
    "L298nPwmConfig",
]

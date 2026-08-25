"""BTS7960 (IBT-2 module) H-bridge drive backend, through an external PWM demux.

See ``driver.py``'s docstring and ``docs/bts7960-ibt2-wiring.md`` for the
demux this depends on. Encoder feedback lives in ``src.hardware.motors.encoder``
-- see that package's docstring and ``base.py``'s ``ClosedLoopDrive``.
"""

from src.hardware.motors.bts7960.config import Bts7960PwmConfig
from src.hardware.motors.bts7960.driver import Driver

__all__ = [
    "Bts7960PwmConfig",
    "Driver",
]

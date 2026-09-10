"""Quadrature-encoder calibration constants.

Measured on hardware, not derived from the datasheet -- see
``scripts/hardware/calibrate_encoder.py`` if the drivetrain changes.

``counts_per_rev``/``max_rpm`` are NOT constants here -- they are required
fields on ``EncoderConfig`` (``config/hardware/motors/encoder.toml``), since
they are per-motor physical data and a stale Python default would silently
misconfigure a different motor rather than erroring at startup. See that
TOML file for the derivation of the currently shipped value.
"""

from __future__ import annotations

from shared.config.constants import RobotSpecs

# Previously an independent hardcoded 0.056m, drifted from RobotSpecs.WHEEL_RADIUS (a
# placeholder pending hardware bring-up, per the package docstring). Derived from the same
# measured wheel radius the rest of the stack uses instead of a second independent guess.
DEFAULT_WHEEL_DIAMETER_M = RobotSpecs.WHEEL_RADIUS * 2

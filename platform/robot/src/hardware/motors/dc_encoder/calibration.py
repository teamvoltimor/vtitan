"""DC-encoder drive calibration constants.

Measured on hardware, not derived from the datasheet -- see
``scripts/hardware/calibrate_encoder.py`` if the drivetrain changes.
"""

from __future__ import annotations

from shared.config.constants import RobotSpecs

# JGB37-520 1590 RPM variant defaults — confirm the printed gear ratio per unit.
# MEASURED on hardware 2026-07-25, not derived from the datasheet: the previous
# 194.0 (11 PPR x4 quadrature x an assumed ~4.4 gear ratio) made
# counts_to_distance() over-report by ~3.5x. Calibrated from raw quadrature
# counts against tape-measured travel, at two duty levels:
#     1650 counts / 54 cm  -> 672 counts per wheel revolution
#     2447 counts / 79 cm  -> 681 counts per wheel revolution
# The two agree to 1.3%, which is the point: wheel slip only ever inflates the
# count for a given distance, so agreement across speeds means slip is
# negligible and this is the true geometric ratio. Back-predicts both runs to
# within 1%. Implies ~15.4:1 gearing (676/44 counts per motor revolution).
#
# Measure with scripts/hardware/calibrate_encoder.py if the drivetrain changes. Do NOT
# derive it by integrating /motor/drive_speed -- that feedback is exponentially
# smoothed and rate-derived, and doing so gave answers ~2x wrong.
DEFAULT_COUNTS_PER_REV = 676.0

# Previously an independent hardcoded 0.056m, drifted from RobotSpecs.WHEEL_RADIUS (a
# placeholder pending hardware bring-up, per the package docstring). Derived from the same
# measured wheel radius the rest of the stack uses instead of a second independent guess.
DEFAULT_WHEEL_DIAMETER_M = RobotSpecs.WHEEL_RADIUS * 2

DEFAULT_MAX_RPM = 42.5
"""Maximum achievable WHEEL rpm, measured 2026-07-25 (2447 counts / 5.11 s).

Was 1590.0 -- the motor's free-running rpm from the datasheet, which is the
wrong quantity twice over: it is the motor shaft rather than the wheel (~15.4:1
apart), and it is the unloaded figure. Since counts_per_rev counts WHEEL
revolutions, get_drive_rpm() reports wheel rpm, so the PID's feedforward term
(1/max_rpm) was scaled ~37x too small -- it would contribute ~3% duty where
~70% is needed to overcome stiction, leaving the integrator to crawl there
alone.

Corresponds to ~0.156 m/s. Note the achievable maximum sags with battery
charge (0.129 m/s measured on a tired pack), so commanded speeds should stay
below this for the loop to have headroom to correct.
"""

"""Sensor perturbations sized from the real robot, shared across diag scripts.

Simulation defaults to perfect sensing, which is the easy side of every
question that depends on the pose estimate. These numbers come from measured
hardware rather than being invented, so a sweep run with them is asking
whether a result survives the error budget the robot actually has -- the
closest simulation gets to hardware validation for anything except the
~1.42x understeer, which no sensor model reaches.
"""

from __future__ import annotations

from src.simulation.imu_error_model import SensorErrors

REAL_SENSOR_ERRORS = SensorErrors(
    # Post-626a011 the start pose is measured off the scan rather than assumed,
    # and landed within 3.6-4.8 cm on all three real bags -- so this is the
    # residual that survives the measurement, not the old assumption's
    # 0.35-0.80 m.
    start_pos_error_m=0.05,
    yaw_bias_rad=0.03,
    imu_drift_rad_per_s=0.000145,  # BNO085's quoted 0.5 deg/min
    gyro_scale_error=0.005,
    imu_noise_rad=0.005,
)
"""What the real sensors do, for asking whether a sim result survives them."""

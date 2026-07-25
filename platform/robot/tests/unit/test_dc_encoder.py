"""Unit tests for the DC-encoder drive backend (pure, no hardware).

Covers the control primitives (conversions, speed estimation, PID anti-windup)
and the SimulatedEncoderDriver reference implementation of EncodedDriveDriver.
"""

from __future__ import annotations

import math

import pytest

from src.hardware.motors.base import DriveOdometry, EncodedDriveDriver
from src.hardware.motors.dc_encoder.control import (
    PIDController,
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
    revolutions_to_distance,
)
from src.hardware.motors.dc_encoder.driver import SimulatedEncoderDriver


class FakeClock:
    """Deterministic monotonic clock for time-based tests."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class TestConversions:
    def test_counts_to_revolutions(self):
        assert counts_to_revolutions(400, 200) == pytest.approx(2.0)

    def test_counts_to_revolutions_rejects_bad_cpr(self):
        with pytest.raises(ValueError, match="counts_per_rev"):
            counts_to_revolutions(100, 0)

    def test_revolutions_to_distance(self):
        assert revolutions_to_distance(1.0, 0.06) == pytest.approx(math.pi * 0.06)

    def test_counts_to_distance(self):
        assert counts_to_distance(200, 200, 0.06) == pytest.approx(math.pi * 0.06)


class TestSpeedEstimator:
    def test_first_sample_returns_zero(self):
        est = SpeedEstimator(counts_per_rev=200)
        assert est.update(1000, dt=0.1) == 0.0

    def test_steady_rpm(self):
        # 200 cpr, +200 counts in 0.1 s = 1 rev / 0.1 s = 600 RPM.
        est = SpeedEstimator(counts_per_rev=200, smoothing=1.0)
        est.update(0, dt=0.1)
        assert est.update(200, dt=0.1) == pytest.approx(600.0)

    def test_zero_dt_holds_value(self):
        est = SpeedEstimator(counts_per_rev=200)
        assert est.update(500, dt=0.0) == 0.0


class TestPIDController:
    def test_output_clamped(self):
        pid = PIDController(kp=100.0, ki=0.0, output_min=-1.0, output_max=1.0)
        assert pid.update(setpoint=10.0, measurement=0.0, dt=0.1) == 1.0

    def test_positive_error_positive_output(self):
        pid = PIDController(kp=0.01, ki=0.0)
        assert pid.update(setpoint=100.0, measurement=0.0, dt=0.1) > 0

    def test_rejects_nonpositive_dt(self):
        pid = PIDController(kp=1.0, ki=1.0)
        with pytest.raises(ValueError, match="dt"):
            pid.update(1.0, 0.0, dt=0.0)

    def test_anti_windup_recovers_immediately(self):
        # Saturate for many ticks, then remove the error; the output must not
        # stay railed (the integrator never wound up while saturated).
        pid = PIDController(kp=10.0, ki=1.0, output_min=-1.0, output_max=1.0)
        for _ in range(50):
            assert pid.update(setpoint=100.0, measurement=0.0, dt=0.1) == 1.0
        assert pid.update(setpoint=0.0, measurement=0.0, dt=0.1) == pytest.approx(0.0)


class TestSimulatedEncoderDriver:
    def test_is_encoded_drive_driver(self):
        assert isinstance(SimulatedEncoderDriver(), EncodedDriveDriver)

    def test_counts_accumulate_from_commanded_rpm(self):
        clock = FakeClock()
        drv = SimulatedEncoderDriver(counts_per_rev=200, max_rpm=600, time_source=clock)
        drv.connect()
        drv.run_drive_at_rpm(600)  # 10 rev/s
        clock.advance(1.0)
        # 600 RPM = 10 rev/s -> 10 * 200 cpr = 2000 counts in 1 s.
        assert drv.get_drive_counts() == 2000

    def test_odometry_distance(self):
        clock = FakeClock()
        drv = SimulatedEncoderDriver(
            counts_per_rev=200,
            wheel_diameter_m=0.06,
            max_rpm=600,
            time_source=clock,
        )
        drv.connect()
        drv.run_drive_at_rpm(600)
        clock.advance(1.0)
        odo = drv.get_drive_odometry()
        assert isinstance(odo, DriveOdometry)
        assert odo.revolutions == pytest.approx(10.0)
        assert odo.distance_m == pytest.approx(10.0 * math.pi * 0.06)

    def test_reset_zeroes_counts(self):
        clock = FakeClock()
        drv = SimulatedEncoderDriver(counts_per_rev=200, max_rpm=600, time_source=clock)
        drv.connect()
        drv.run_drive_at_rpm(600)
        clock.advance(1.0)
        drv.reset_drive_encoder()
        assert drv.get_drive_counts() == 0

    def test_invert_reverses_sign(self):
        clock = FakeClock()
        drv = SimulatedEncoderDriver(
            counts_per_rev=200,
            max_rpm=600,
            invert=True,
            time_source=clock,
        )
        drv.connect()
        drv.run_drive_at_rpm(600)
        clock.advance(1.0)
        assert drv.get_drive_counts() == -2000

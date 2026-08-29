"""Unit tests for the quadrature-encoder backend (pure, no hardware).

Covers the control primitives (conversions, speed estimation, PID anti-windup)
and the SimulatedEncoder reference implementation of EncoderSensor.
"""

from __future__ import annotations

import math

import pytest

from src.hardware.motors.base import ClosedLoopDrive, DriveOdometry, EncoderSensor
from src.hardware.motors.encoder.control import (
    PIDController,
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
    revolutions_to_distance,
)
from src.hardware.motors.encoder.simulated import SimulatedEncoder


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

    def test_window_holds_last_value_until_min_window_elapsed(self):
        # min_window_s=0.1, three 0.03s ticks (0.09s total) don't reach it.
        est = SpeedEstimator(counts_per_rev=200, smoothing=1.0, min_window_s=0.1)
        est.update(0, dt=0.03)
        assert est.update(1, dt=0.03) == 0.0
        assert est.update(1, dt=0.03) == 0.0

    def test_window_computes_rate_once_min_window_elapsed(self):
        # 86 cpr / ~0.39 counts per 0.02s tick, the bench regime this widened
        # window exists for. First call only seeds prev_counts; five more
        # 0.02s ticks (0.1s) are needed to reach min_window_s, accumulating
        # 2 counts total.
        est = SpeedEstimator(counts_per_rev=86, smoothing=1.0, min_window_s=0.1)
        est.update(0, dt=0.02)  # seed
        est.update(0, dt=0.02)
        est.update(0, dt=0.02)
        est.update(1, dt=0.02)
        est.update(1, dt=0.02)
        # 2 counts / 86 cpr / 0.1s * 60 = 13.95... rpm.
        assert est.update(2, dt=0.02) == pytest.approx(2 / 86 / 0.1 * 60)

    def test_window_resets_after_computing(self):
        est = SpeedEstimator(counts_per_rev=200, smoothing=1.0, min_window_s=0.1)
        est.update(0, dt=0.1)
        first = est.update(200, dt=0.1)  # window completes: 1 rev / 0.1s = 600 rpm
        assert first == pytest.approx(600.0)
        # Immediately after, a single small tick shouldn't reuse stale window state.
        assert est.update(200, dt=0.01) == pytest.approx(600.0)  # held, window not yet full

    def test_rejects_negative_min_window(self):
        with pytest.raises(ValueError, match="min_window_s"):
            SpeedEstimator(counts_per_rev=200, min_window_s=-0.1)


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


class TestSimulatedEncoder:
    def test_is_encoder_sensor(self):
        assert isinstance(SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06), EncoderSensor)

    def test_counts_accumulate_from_target_rpm(self):
        clock = FakeClock()
        enc = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06, time_source=clock)
        enc.connect()
        enc.set_target_rpm(600)  # 10 rev/s
        clock.advance(1.0)
        # 600 RPM = 10 rev/s -> 10 * 200 cpr = 2000 counts in 1 s.
        assert enc.get_counts() == 2000

    def test_odometry_distance(self):
        clock = FakeClock()
        enc = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06, time_source=clock)
        enc.connect()
        enc.set_target_rpm(600)
        clock.advance(1.0)
        odo = enc.get_odometry()
        assert isinstance(odo, DriveOdometry)
        assert odo.revolutions == pytest.approx(10.0)
        assert odo.distance_m == pytest.approx(10.0 * math.pi * 0.06)

    def test_reset_zeroes_counts(self):
        clock = FakeClock()
        enc = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06, time_source=clock)
        enc.connect()
        enc.set_target_rpm(600)
        clock.advance(1.0)
        enc.reset()
        assert enc.get_counts() == 0

    def test_invert_reverses_sign(self):
        clock = FakeClock()
        enc = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06, invert=True, time_source=clock)
        enc.connect()
        enc.set_target_rpm(600)
        clock.advance(1.0)
        assert enc.get_counts() == -2000

    def test_get_last_rpm_does_not_resample(self):
        clock = FakeClock()
        enc = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06, time_source=clock)
        enc.connect()
        enc.set_target_rpm(600)
        assert enc.get_rpm() == pytest.approx(600.0)
        assert enc.get_last_rpm() == pytest.approx(600.0)


class FakeDrive:
    """Records the last open-loop command a ClosedLoopDrive issued to it."""

    def __init__(self) -> None:
        self.connected = False
        self.last_forward: float | None = None
        self.last_reverse: float | None = None
        self.stopped = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def run_drive_forward(self, speed=None) -> None:
        self.last_forward = speed
        self.last_reverse = None

    def run_drive_reverse(self, speed=None) -> None:
        self.last_reverse = speed
        self.last_forward = None

    def stop_drive(self) -> None:
        self.stopped = True

    def get_drive_position(self) -> float:
        return 0.0

    def get_drive_speed(self) -> float:
        return 0.0


class TestClosedLoopDrive:
    def test_connect_connects_both(self):
        drive = FakeDrive()
        encoder = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06)
        loop = ClosedLoopDrive(drive, encoder, PIDController(kp=1.0, ki=0.0))
        loop.connect()
        assert drive.connected

    def test_positive_pid_output_drives_forward(self):
        drive = FakeDrive()
        encoder = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06)
        loop = ClosedLoopDrive(drive, encoder, PIDController(kp=1.0, ki=0.0))
        loop.run_drive_at_rpm(100.0, dt=0.1)
        assert drive.last_forward is not None
        assert drive.last_reverse is None

    def test_negative_pid_output_drives_reverse(self):
        drive = FakeDrive()
        encoder = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06)
        loop = ClosedLoopDrive(drive, encoder, PIDController(kp=1.0, ki=0.0))
        loop.run_drive_at_rpm(-100.0, dt=0.1)
        assert drive.last_reverse is not None
        assert drive.last_forward is None

    def test_stop_drive_stops_and_resets_pid(self):
        drive = FakeDrive()
        encoder = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06)
        pid = PIDController(kp=1.0, ki=1.0)
        loop = ClosedLoopDrive(drive, encoder, pid)
        loop.run_drive_at_rpm(100.0, dt=0.1)
        loop.stop_drive()
        assert drive.stopped

    def test_get_drive_counts_delegates_to_encoder(self):
        drive = FakeDrive()
        clock = FakeClock()
        encoder = SimulatedEncoder(counts_per_rev=200, wheel_diameter_m=0.06, time_source=clock)
        encoder.connect()
        encoder.set_target_rpm(600)
        clock.advance(1.0)
        loop = ClosedLoopDrive(drive, encoder, PIDController(kp=1.0, ki=0.0))
        assert loop.get_drive_counts() == 2000

"""Unit tests for JerkLimiter (HI-01).

Verifies:
- Step input 0 → 0.7 m/s takes ≥ 350 ms to reach 95%.
- Output never overshoots the target.
- reset() returns filter to given state.
- Per-tick delta never exceeds max_accel_mps2 * dt.
"""

from __future__ import annotations

import pytest

from src.navigation.speed_control import JerkLimiter

DT = 0.05  # 20 Hz
TAU = 0.20


class TestJerkLimiter:
    def test_step_to_95pct_takes_at_least_350ms(self):
        limiter = JerkLimiter(dt=DT, tau=TAU, max_accel_mps2=2.0)
        target = 0.7
        threshold = target * 0.95  # 0.665 m/s

        elapsed = 0.0
        reached_at = None
        for step in range(500):
            v = limiter.filter(target)
            elapsed += DT
            if v >= threshold and reached_at is None:
                reached_at = elapsed

        assert reached_at is not None, "Never reached 95% of target"
        assert reached_at >= 0.350, f"Reached 95% too fast: {reached_at:.3f}s < 350ms"

    def test_output_never_overshoots(self):
        limiter = JerkLimiter(dt=DT, tau=TAU, max_accel_mps2=2.0)
        for _ in range(100):
            v = limiter.filter(0.7)
            assert v <= 0.7 + 1e-9

    def test_per_tick_delta_bounded(self):
        limiter = JerkLimiter(dt=DT, tau=TAU, max_accel_mps2=2.0)
        max_allowed_delta = 2.0 * DT + 1e-9
        prev = 0.0
        for _ in range(100):
            v = limiter.filter(0.7)
            assert abs(v - prev) <= max_allowed_delta, f"Delta {abs(v - prev):.4f} > max {max_allowed_delta:.4f}"
            prev = v

    def test_reset_clears_state(self):
        limiter = JerkLimiter(dt=DT, tau=TAU)
        for _ in range(20):
            limiter.filter(0.7)
        limiter.reset(0.0)
        v = limiter.filter(0.0)
        assert v == pytest.approx(0.0, abs=1e-6)

    def test_starts_from_zero(self):
        limiter = JerkLimiter(dt=DT, tau=TAU)
        first = limiter.filter(0.7)
        assert first > 0.0
        assert first < 0.7

    def test_converges_to_target(self):
        limiter = JerkLimiter(dt=DT, tau=TAU, max_accel_mps2=10.0)
        for _ in range(200):
            v = limiter.filter(0.5)
        assert v == pytest.approx(0.5, abs=0.001)

    def test_zero_target_slows_from_nonzero(self):
        limiter = JerkLimiter(dt=DT, tau=TAU)
        for _ in range(40):
            limiter.filter(0.7)
        for _ in range(100):
            v = limiter.filter(0.0)
        assert v == pytest.approx(0.0, abs=0.01)

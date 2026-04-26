"""Unit tests for SpeedControl module.

Tests speed scaling logic for clearance zones and heading errors.
"""

import pytest
from src.navigation.config import NavigationConfig
from src.navigation.speed_control import SpeedScaler


class TestSpeedScaler:
    """Tests for SpeedScaler speed scaling calculations."""

    @pytest.fixture
    def scaler(self):
        config = NavigationConfig.default()
        return SpeedScaler(config)

    def test_scale_for_contact_zone(self, scaler):
        speed = scaler.scale_for_clearance(forward_clearance=0.05)
        assert 0 < speed <= 1.0

    def test_scale_for_slow_zone(self, scaler):
        speed = scaler.scale_for_clearance(forward_clearance=0.20)
        assert 0 < speed <= 1.0

    def test_scale_for_medium_zone(self, scaler):
        speed = scaler.scale_for_clearance(forward_clearance=0.40)
        assert 0 < speed <= 1.0

    def test_scale_for_fast_zone(self, scaler):
        speed = scaler.scale_for_clearance(forward_clearance=0.80)
        assert 0 < speed <= 1.0

    def test_scale_for_full_speed(self, scaler):
        speed = scaler.scale_for_clearance(forward_clearance=1.50)
        assert speed == pytest.approx(1.0)

    def test_scale_increases_with_clearance(self, scaler):
        s_contact = scaler.scale_for_clearance(0.05)
        s_slow = scaler.scale_for_clearance(0.20)
        s_medium = scaler.scale_for_clearance(0.40)
        s_fast = scaler.scale_for_clearance(0.80)
        s_full = scaler.scale_for_clearance(1.50)

        assert s_contact < s_slow < s_medium < s_fast < s_full

    def test_scale_for_crawl_heading_error(self, scaler):
        speed = scaler.scale_for_heading_error(heading_error=1.2)
        assert 0 < speed <= 1.0

    def test_scale_for_slow_heading_error(self, scaler):
        speed = scaler.scale_for_heading_error(heading_error=0.75)
        assert 0 < speed <= 1.0

    def test_scale_for_medium_heading_error(self, scaler):
        speed = scaler.scale_for_heading_error(heading_error=0.50)
        assert 0 < speed <= 1.0

    def test_scale_for_small_heading_error(self, scaler):
        speed = scaler.scale_for_heading_error(heading_error=0.2)
        assert speed == pytest.approx(1.0)

    def test_combined_clearance_is_more_restrictive(self, scaler):
        # Very close obstacle (contact zone) + small heading error
        # → clearance dominates
        combined = scaler.scale_combined(forward_clearance=0.05, heading_error=0.1)
        clearance_only = scaler.scale_for_clearance(0.05)
        assert combined == pytest.approx(clearance_only)

    def test_combined_heading_is_more_restrictive(self, scaler):
        # Clear path + large heading error → heading dominates
        combined = scaler.scale_combined(forward_clearance=2.0, heading_error=1.2)
        heading_only = scaler.scale_for_heading_error(1.2)
        assert combined == pytest.approx(heading_only)

    def test_combined_equal_limits(self, scaler):
        # When both return same fraction, combined == that fraction
        combined = scaler.scale_combined(forward_clearance=0.05, heading_error=1.2)
        assert 0 < combined <= 1.0

    def test_speed_command_positive(self, scaler):
        cmd = scaler.compute_speed_command(
            forward_clearance=0.5, heading_error=0.0, max_linear_speed=0.5
        )
        assert cmd > 0

    def test_speed_command_zero_fraction(self, scaler):
        # A zero max speed gives zero command regardless of zones
        cmd = scaler.compute_speed_command(
            forward_clearance=2.0, heading_error=0.0, max_linear_speed=0.0
        )
        assert cmd == 0.0

    def test_speed_command_scales_linearly(self, scaler):
        cmd_high = scaler.compute_speed_command(
            forward_clearance=2.0, heading_error=0.0, max_linear_speed=1.0
        )
        cmd_low = scaler.compute_speed_command(
            forward_clearance=0.05, heading_error=0.0, max_linear_speed=1.0
        )
        assert cmd_high > cmd_low

    def test_clearance_vs_heading_speed_precedence(self, scaler):
        # Contact zone (low clearance) with small heading error → clearance limits
        result_clearance_limits = scaler.scale_combined(
            forward_clearance=0.05, heading_error=0.1
        )
        # Clear path, large heading error → heading limits
        result_heading_limits = scaler.scale_combined(
            forward_clearance=2.0, heading_error=1.2
        )

        # Both should be less than 1.0 (something is limiting)
        assert result_clearance_limits < 1.0
        assert result_heading_limits < 1.0

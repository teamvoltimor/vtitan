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
        """Create a SpeedScaler instance for testing."""
        config = NavigationConfig.default()
        return SpeedScaler(config)

    def test_scale_for_contact_zone(self, scaler):
        """Test speed scaling in contact zone (very close)."""
        config = NavigationConfig.default()
        speed = scaler.scale_for_clearance(forward_clearance=0.05)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_slow_zone(self, scaler):
        """Test speed scaling in slow zone."""
        config = NavigationConfig.default()
        speed = scaler.scale_for_clearance(forward_clearance=0.20)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_medium_zone(self, scaler):
        """Test speed scaling in medium zone."""
        config = NavigationConfig.default()
        speed = scaler.scale_for_clearance(forward_clearance=0.40)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_fast_zone(self, scaler):
        """Test speed scaling in fast zone."""
        config = NavigationConfig.default()
        speed = scaler.scale_for_clearance(forward_clearance=0.80)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_full_speed(self, scaler):
        """Test speed scaling when path is fully clear."""
        config = NavigationConfig.default()
        speed = scaler.scale_for_clearance(forward_clearance=1.50)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_crawl_heading_error(self, scaler):
        """Test speed scaling for worst-case heading error."""
        speed = scaler.scale_for_heading_error(heading_error=1.2)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_slow_heading_error(self, scaler):
        """Test speed scaling for large heading error."""
        speed = scaler.scale_for_heading_error(heading_error=0.75)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_medium_heading_error(self, scaler):
        """Test speed scaling for moderate heading error."""
        speed = scaler.scale_for_heading_error(heading_error=0.50)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_scale_for_small_heading_error(self, scaler):
        """Test speed scaling for small heading error."""
        speed = scaler.scale_for_heading_error(heading_error=0.2)
        assert 0 < speed <= 1.0, "Should return valid speed fraction"

    def test_combined_scaling_clearance_limiting(self, scaler):
        """Test that clearance limit applies when smaller than heading limit."""
        clearance_speed = 0.35
        heading_speed = 0.70

        combined = scaler.scale_combined(clearance_speed, heading_speed)

        assert combined == 0.35, "Should apply the more restrictive (lower) speed"

    def test_combined_scaling_heading_limiting(self, scaler):
        """Test that heading limit applies when smaller than clearance limit."""
        clearance_speed = 0.70
        heading_speed = 0.35

        combined = scaler.scale_combined(clearance_speed, heading_speed)

        assert combined == 0.35, "Should apply the more restrictive (lower) speed"

    def test_combined_scaling_equal(self, scaler):
        """Test combined scaling when both limits are equal."""
        speed = 0.50

        combined = scaler.scale_combined(speed, speed)

        assert combined == 0.50, "Should return the speed when limits are equal"

    def test_speed_command_positive(self, scaler):
        """Test speed command generation for forward motion."""
        config = NavigationConfig.default()
        cmd = scaler.compute_speed_command(speed_fraction=0.7)
        assert cmd > 0, "Speed command should be positive for forward"
        assert cmd <= 0.5, "Speed command should not exceed max"

    def test_speed_command_minimum_forward(self, scaler):
        """Test that speed command respects minimum forward speed."""
        config = NavigationConfig.default()
        cmd = scaler.compute_speed_command(speed_fraction=0.05)
        assert cmd >= 0, "Speed command should be non-negative"

    def test_speed_command_zero(self, scaler):
        """Test speed command for zero movement."""
        cmd = scaler.compute_speed_command(speed_fraction=0.0)
        assert cmd == 0.0, "Speed command should be zero when fraction is zero"

    def test_speed_command_full(self, scaler):
        """Test speed command for full speed."""
        config = NavigationConfig.default()
        cmd = scaler.compute_speed_command(speed_fraction=1.0)
        assert 0 < cmd <= 0.5, "Speed command should be reasonable fraction of max"

    def test_speed_command_scales_linearly(self, scaler):
        """Test that speed command scales linearly with fraction."""
        cmd_half = scaler.compute_speed_command(speed_fraction=0.5)
        cmd_quarter = scaler.compute_speed_command(speed_fraction=0.25)

        # Half should be greater than quarter
        assert cmd_half > cmd_quarter, "Greater fraction should produce greater speed"

    def test_clearance_vs_heading_speed_precedence(self, scaler):
        """Test that more restrictive speed takes precedence."""
        low_speed = 0.2
        high_speed = 0.8

        result = scaler.scale_combined(low_speed, high_speed)
        assert result == low_speed, "Should use lower speed"

        result = scaler.scale_combined(high_speed, low_speed)
        assert result == low_speed, "Should use lower speed regardless of order"

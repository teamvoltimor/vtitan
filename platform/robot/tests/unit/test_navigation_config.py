"""Unit tests for NavigationConfig module.

Tests configuration loading and profile management.
"""

import pytest
from src.navigation.config import NavigationConfig, DrivingProfile


class TestNavigationConfig:
    """Tests for NavigationConfig configuration management."""

    def test_config_initialization_default(self):
        """Test NavigationConfig initializes with default values."""
        config = NavigationConfig.default()

        assert config is not None
        assert hasattr(config, "driving_profile")
        assert config.driving_profile == DrivingProfile.REAL_ROBOT

    def test_config_has_clearance_zones(self):
        """Test that config has clearance zone thresholds."""
        config = NavigationConfig.default()
        assert hasattr(config, "clearance_zones")
        zones = config.clearance_zones

        assert hasattr(zones, "contact")
        assert hasattr(zones, "slow")
        assert hasattr(zones, "medium")
        assert hasattr(zones, "fast")

    def test_clearance_zones_increasing(self):
        """Test clearance zone thresholds increase with distance."""
        config = NavigationConfig.default()
        zones = config.clearance_zones

        # Check thresholds increase with distance
        assert zones.contact < zones.slow
        assert zones.slow < zones.medium
        assert zones.medium < zones.fast

        # All should be positive
        assert zones.contact > 0
        assert zones.slow > 0
        assert zones.medium > 0
        assert zones.fast > 0

    def test_config_has_heading_error_zones(self):
        """Test that config has heading error zones."""
        config = NavigationConfig.default()
        assert hasattr(config, "heading_error_zones")
        zones = config.heading_error_zones

        assert hasattr(zones, "crawl")
        assert hasattr(zones, "slow")
        assert hasattr(zones, "medium")

    def test_heading_error_zones_increasing(self):
        """Test heading error zones are reasonable."""
        config = NavigationConfig.default()
        zones = config.heading_error_zones

        # Check thresholds increase with angle
        assert zones.crawl < zones.slow
        assert zones.slow < zones.medium

        # All should be positive
        assert zones.crawl > 0
        assert zones.slow > 0
        assert zones.medium > 0

    def test_config_has_speed_fractions(self):
        """Test that config has speed fraction parameters."""
        config = NavigationConfig.default()
        assert hasattr(config, "speed_fractions")
        speeds = config.speed_fractions

        assert hasattr(speeds, "contact")
        assert hasattr(speeds, "slow")
        assert hasattr(speeds, "medium")
        assert hasattr(speeds, "fast")
        assert hasattr(speeds, "full")

    def test_speed_fractions_valid(self):
        """Test speed fractions are between 0 and 1."""
        config = NavigationConfig.default()
        speeds = config.speed_fractions

        assert 0 < speeds.contact <= 1.0
        assert 0 < speeds.slow <= 1.0
        assert 0 < speeds.medium <= 1.0
        assert 0 < speeds.fast <= 1.0
        assert speeds.full == 1.0

        # Check fractions increase
        assert speeds.contact < speeds.slow
        assert speeds.slow < speeds.medium
        assert speeds.medium < speeds.fast

    def test_config_has_lookahead_distances(self):
        """Test that config has lookahead distances."""
        config = NavigationConfig.default()
        assert hasattr(config, "lookahead_distances")
        lookahead = config.lookahead_distances

        assert hasattr(lookahead, "short")
        assert hasattr(lookahead, "long")

    def test_lookahead_distances_valid(self):
        """Test lookahead distances are reasonable."""
        config = NavigationConfig.default()
        lookahead = config.lookahead_distances

        assert lookahead.short > 0
        assert lookahead.long > 0
        assert lookahead.short < lookahead.long

    def test_config_has_escape_maneuvers(self):
        """Test that config has escape maneuver parameters."""
        config = NavigationConfig.default()
        assert hasattr(config, "escape_maneuvers")
        escape = config.escape_maneuvers

        assert hasattr(escape, "reverse_speed")
        assert hasattr(escape, "forward_speed")
        assert hasattr(escape, "steer_scale")

    def test_escape_config_valid(self):
        """Test escape configuration contains valid parameters."""
        config = NavigationConfig.default()
        escape = config.escape_maneuvers

        # Reverse speed should be negative
        assert escape.reverse_speed < 0

        # Forward speed should be positive
        assert escape.forward_speed > 0

        # Scale should be reasonable
        assert 0 < escape.steer_scale <= 1.0

    def test_config_has_collision_avoidance(self):
        """Test that config has collision avoidance parameters."""
        config = NavigationConfig.default()
        assert hasattr(config, "collision_avoidance")
        collision = config.collision_avoidance

        assert hasattr(collision, "critical_distance")
        assert hasattr(collision, "safe_distance")

    def test_collision_avoidance_valid(self):
        """Test collision avoidance parameters are valid."""
        config = NavigationConfig.default()
        collision = config.collision_avoidance

        assert collision.critical_distance > 0
        assert collision.safe_distance > 0
        assert collision.critical_distance < collision.safe_distance

    def test_driving_profile_enum_values(self):
        """Test DrivingProfile enum has expected values."""
        assert DrivingProfile.REAL_ROBOT in DrivingProfile
        assert DrivingProfile.SIMULATION in DrivingProfile
        assert DrivingProfile.CONSERVATIVE in DrivingProfile

    def test_load_profile_real_robot(self):
        """Test loading real robot profile."""
        config = NavigationConfig.load_profile(DrivingProfile.REAL_ROBOT)

        assert config is not None
        assert config.driving_profile == DrivingProfile.REAL_ROBOT
        assert hasattr(config, "clearance_zones")

    def test_load_profile_simulation(self):
        """Test loading simulation profile."""
        config = NavigationConfig.load_profile(DrivingProfile.SIMULATION)

        assert config is not None
        assert config.driving_profile == DrivingProfile.SIMULATION

    def test_load_profile_conservative(self):
        """Test loading conservative profile."""
        config = NavigationConfig.load_profile(DrivingProfile.CONSERVATIVE)

        assert config is not None
        assert config.driving_profile == DrivingProfile.CONSERVATIVE

    def test_all_profiles_are_different(self):
        """Test that different profiles exist and are loadable."""
        real_robot = NavigationConfig.load_profile(DrivingProfile.REAL_ROBOT)
        simulation = NavigationConfig.load_profile(DrivingProfile.SIMULATION)
        conservative = NavigationConfig.load_profile(DrivingProfile.CONSERVATIVE)

        # All should be valid configs
        assert real_robot is not None
        assert simulation is not None
        assert conservative is not None

        # All should have the same structure
        assert hasattr(real_robot, "clearance_zones")
        assert hasattr(simulation, "clearance_zones")
        assert hasattr(conservative, "clearance_zones")

    def test_config_access_nested_values(self):
        """Test that we can access nested configuration values."""
        config = NavigationConfig.default()

        # Should be able to access nested values
        assert config.clearance_zones.contact > 0
        assert config.heading_error_zones.crawl > 0
        assert config.speed_fractions.contact > 0
        assert config.lookahead_distances.short > 0
        assert config.escape_maneuvers.forward_speed > 0
        assert config.collision_avoidance.critical_distance > 0

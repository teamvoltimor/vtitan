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
        assert hasattr(config, "profile")
        assert config.profile == DrivingProfile.REAL_ROBOT

    def test_config_has_clearance_zones(self):
        """Test that config has clearance zone thresholds."""
        config = NavigationConfig.default()
        assert hasattr(config, "clearance")
        zones = config.clearance

        assert hasattr(zones, "contact_dist")
        assert hasattr(zones, "slow_dist")
        assert hasattr(zones, "medium_dist")
        assert hasattr(zones, "fast_dist")

    def test_clearance_zones_increasing(self):
        """Test clearance zone thresholds increase with distance."""
        config = NavigationConfig.default()
        zones = config.clearance

        assert zones.contact_dist < zones.slow_dist
        assert zones.slow_dist < zones.medium_dist
        assert zones.medium_dist < zones.fast_dist

        assert zones.contact_dist > 0
        assert zones.slow_dist > 0
        assert zones.medium_dist > 0
        assert zones.fast_dist > 0

    def test_config_has_heading_error_zones(self):
        """Test that config has heading error zones."""
        config = NavigationConfig.default()
        assert hasattr(config, "heading_error")
        zones = config.heading_error

        assert hasattr(zones, "crawl")
        assert hasattr(zones, "slow")
        assert hasattr(zones, "medium")

    def test_heading_error_zones_increasing(self):
        """Test heading error zones are reasonable."""
        config = NavigationConfig.default()
        zones = config.heading_error

        assert zones.medium < zones.slow
        assert zones.slow < zones.crawl

        assert zones.crawl > 0
        assert zones.slow > 0
        assert zones.medium > 0

    def test_config_has_speed_fractions(self):
        """Test that config has speed fraction parameters."""
        config = NavigationConfig.default()
        assert hasattr(config, "speed")
        speeds = config.speed

        assert hasattr(speeds, "contact")
        assert hasattr(speeds, "slow")
        assert hasattr(speeds, "medium")
        assert hasattr(speeds, "fast")
        assert hasattr(speeds, "full")

    def test_speed_fractions_valid(self):
        """Test speed fractions are between 0 and 1."""
        config = NavigationConfig.default()
        speeds = config.speed

        assert 0 < speeds.contact <= 1.0
        assert 0 < speeds.slow <= 1.0
        assert 0 < speeds.medium <= 1.0
        assert 0 < speeds.fast <= 1.0
        assert speeds.full == 1.0

        assert speeds.contact < speeds.slow
        assert speeds.slow < speeds.medium
        assert speeds.medium < speeds.fast

    def test_config_has_lookahead_distances(self):
        """Test that config has lookahead distances."""
        config = NavigationConfig.default()
        assert hasattr(config, "lookahead")
        lookahead = config.lookahead

        assert hasattr(lookahead, "short")
        assert hasattr(lookahead, "long")

    def test_lookahead_distances_valid(self):
        """Test lookahead distances are reasonable."""
        config = NavigationConfig.default()
        lookahead = config.lookahead

        assert lookahead.short > 0
        assert lookahead.long > 0
        assert lookahead.short < lookahead.long

    def test_config_has_escape_maneuvers(self):
        """Test that config has escape maneuver parameters."""
        config = NavigationConfig.default()
        assert hasattr(config, "escape")
        escape = config.escape

        assert hasattr(escape, "rev_speed")
        assert hasattr(escape, "obs_fwd_speed")
        assert hasattr(escape, "steer_scale")

    def test_escape_config_valid(self):
        """Test escape configuration contains valid parameters."""
        config = NavigationConfig.default()
        escape = config.escape

        assert escape.rev_speed < 0
        assert escape.obs_fwd_speed > 0
        assert 0 < escape.steer_scale <= 1.0

    def test_config_has_collision_avoidance(self):
        """Test that config has collision avoidance parameters."""
        config = NavigationConfig.default()
        assert hasattr(config, "collision")
        collision = config.collision

        assert hasattr(collision, "critical_dist")
        assert hasattr(collision, "caution_dist")

    def test_collision_avoidance_valid(self):
        """Test collision avoidance parameters are valid."""
        config = NavigationConfig.default()
        collision = config.collision

        assert collision.critical_dist > 0
        assert collision.caution_dist > 0
        assert collision.critical_dist < collision.caution_dist

    def test_driving_profile_enum_values(self):
        """Test DrivingProfile enum has expected values."""
        assert DrivingProfile.REAL_ROBOT in DrivingProfile
        assert DrivingProfile.SIMULATION in DrivingProfile
        assert DrivingProfile.CONSERVATIVE in DrivingProfile

    def test_load_profile_real_robot(self):
        """Test loading real robot profile."""
        config = NavigationConfig.load_profile(DrivingProfile.REAL_ROBOT)

        assert config is not None
        assert config.profile == DrivingProfile.REAL_ROBOT
        assert hasattr(config, "clearance")

    def test_load_profile_simulation(self):
        """Test loading simulation profile."""
        config = NavigationConfig.load_profile(DrivingProfile.SIMULATION)

        assert config is not None
        assert config.profile == DrivingProfile.SIMULATION

    def test_load_profile_conservative(self):
        """Test loading conservative profile."""
        config = NavigationConfig.load_profile(DrivingProfile.CONSERVATIVE)

        assert config is not None
        assert config.profile == DrivingProfile.CONSERVATIVE

    def test_all_profiles_are_different(self):
        """Test that different profiles exist and are loadable."""
        real_robot = NavigationConfig.load_profile(DrivingProfile.REAL_ROBOT)
        simulation = NavigationConfig.load_profile(DrivingProfile.SIMULATION)
        conservative = NavigationConfig.load_profile(DrivingProfile.CONSERVATIVE)

        assert real_robot is not None
        assert simulation is not None
        assert conservative is not None

        assert hasattr(real_robot, "clearance")
        assert hasattr(simulation, "clearance")
        assert hasattr(conservative, "clearance")

    def test_config_access_nested_values(self):
        """Test that we can access nested configuration values."""
        config = NavigationConfig.default()

        assert config.clearance.contact_dist > 0
        assert config.heading_error.crawl > 0
        assert config.speed.contact > 0
        assert config.lookahead.short > 0
        assert config.escape.obs_fwd_speed > 0
        assert config.collision.critical_dist > 0

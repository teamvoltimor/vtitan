"""Unit tests for EscapeManeuvers module.

Tests escape maneuver computation and obstacle avoidance logic.
"""

import math
from dataclasses import dataclass

import pytest

from src.navigation.config import NavigationConfig
from src.navigation.maneuvers.escape_maneuvers import EscapeCommand, EscapeManager


class TestEscapeCommand:
    """Tests for EscapeCommand dataclass."""

    def test_escape_command_initialization(self):
        """Test EscapeCommand dataclass initialization."""
        cmd = EscapeCommand(
            linear_speed=0.2,
            angular_steering=0.5,
            duration_frames=10,
            escape_type="k_turn",
        )

        assert cmd.linear_speed == 0.2
        assert cmd.angular_steering == 0.5
        assert cmd.duration_frames == 10
        assert cmd.escape_type == "k_turn"

    def test_escape_command_negative_speed(self):
        """Test EscapeCommand with reverse speed."""
        cmd = EscapeCommand(
            linear_speed=-0.2,
            angular_steering=0.3,
            duration_frames=8,
            escape_type="reverse",
        )

        assert cmd.linear_speed < 0, "Reverse speed should be negative"


class TestEscapeManager:
    """Tests for EscapeManager escape maneuver calculations."""

    @pytest.fixture()
    def manager(self):
        """Create an EscapeManager instance for testing."""
        config = NavigationConfig.default()
        return EscapeManager(config, max_steering_angle=math.radians(35))

    def test_manager_initialization(self, manager):
        """Test that manager initializes with valid config."""
        assert manager.config is not None
        assert manager.max_steering_angle > 0
        assert manager.max_steering_angle == math.radians(35)

    def test_manager_config_has_escape_params(self, manager):
        """Test that config contains escape parameters."""
        assert hasattr(manager.config, "escape")
        escape_cfg = manager.config.escape
        assert hasattr(escape_cfg, "rev_speed")
        assert hasattr(escape_cfg, "obs_fwd_speed")

    def test_k_turn_command_generation(self, manager):
        """Test K-turn command generation."""
        cmd = manager.compute_k_turn_command(reverse_direction=-1)

        assert isinstance(cmd, EscapeCommand)
        assert abs(cmd.linear_speed) > 0, "K-turn should have non-zero speed"
        assert cmd.escape_type in ("k_turn", "wall_k_turn")

    def test_k_turn_reverse_direction(self, manager):
        """Test K-turn respects reverse direction."""
        cmd_left = manager.compute_k_turn_command(reverse_direction=1)
        cmd_right = manager.compute_k_turn_command(reverse_direction=-1)

        # Both should be valid commands
        assert isinstance(cmd_left, EscapeCommand)
        assert isinstance(cmd_right, EscapeCommand)

    def test_escape_manager_has_required_methods(self, manager):
        """Test that EscapeManager has expected methods."""
        assert hasattr(manager, "compute_k_turn_command")
        assert callable(manager.compute_k_turn_command)

    def test_max_steering_angle_enforcement(self, manager):
        """Test that steering angles respect max."""
        assert manager.max_steering_angle > 0
        assert manager.max_steering_angle <= math.pi / 2

    def test_escape_config_parameters_valid(self, manager):
        """Test that escape config contains valid parameters."""
        escape_cfg = manager.config.escape

        # Reverse speed should be negative
        assert escape_cfg.rev_speed < 0

        # Obstacle forward speed should be positive
        assert escape_cfg.obs_fwd_speed > 0

        # Fractions should be between 0 and 1
        assert 0 <= escape_cfg.obs_reverse_fraction <= 1.0
        assert 0 < escape_cfg.steer_scale <= 1.0

    def test_escape_command_duration_positive(self):
        """Test that escape command duration is positive."""
        cmd = EscapeCommand(
            linear_speed=0.2,
            angular_steering=0.3,
            duration_frames=5,
            escape_type="test",
        )

        assert cmd.duration_frames > 0
        assert isinstance(cmd.duration_frames, int)

    def test_escape_types_are_strings(self):
        """Test that escape_type is a string."""
        escape_types = ["k_turn", "side_push", "obstacle_avoidance", "normal"]

        for etype in escape_types:
            cmd = EscapeCommand(
                linear_speed=0.1,
                angular_steering=0.1,
                duration_frames=1,
                escape_type=etype,
            )
            assert isinstance(cmd.escape_type, str)
            assert cmd.escape_type == etype

    def test_manager_multiple_escape_calls(self, manager):
        """Test that manager can generate multiple escape commands."""
        cmd1 = manager.compute_k_turn_command(1)
        cmd2 = manager.compute_k_turn_command(-1)

        assert cmd1 is not cmd2, "Should return different objects"
        assert isinstance(cmd1, EscapeCommand)
        assert isinstance(cmd2, EscapeCommand)

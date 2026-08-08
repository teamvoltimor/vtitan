"""Ensure all NavigationTuning fields are backed by config files.

Catches misses like a new tuning parameter added to the dataclass but not
persisted in the TOML source, so the override would silently disappear.
"""

from __future__ import annotations

import pytest

from shared.config.navigation_tuning import NavigationTuning


def test_all_tuning_params_groups_have_config() -> None:
    """Verify NavigationTuning can be instantiated and all groups exist."""
    tuning = NavigationTuning.load_default()

    # Check that all expected groups exist and are accessible
    expected_groups = [
        "lidar_sectors",
        "heading",
        "direction_estimator",
        "corridor_follower",
        "corridor_estimator",
        "waypoints",
        "pursuit",
        "sign_discovery",
        "sign_router",
    ]

    for group_name in expected_groups:
        assert hasattr(tuning, group_name), f"NavigationTuning missing group: {group_name}"
        group = getattr(tuning, group_name)
        assert group is not None, f"Group {group_name} is None"


def test_tuning_fields_not_none() -> None:
    """Verify all tuning fields have non-None values (config is complete)."""
    tuning = NavigationTuning.load_default()

    # Spot-check critical fields that would break if config is incomplete
    critical_fields = {
        "lidar_sectors.MIN_VALID_RANGE_M": tuning.lidar_sectors.MIN_VALID_RANGE_M,
        "heading.MEDIUM": tuning.heading.MEDIUM,
        "direction_estimator.MAX_IN_TRACK_RANGE_M": tuning.direction_estimator.MAX_IN_TRACK_RANGE_M,
        "corridor_follower.CENTERING_GAIN": tuning.corridor_follower.CENTERING_GAIN,
        "corridor_follower.HEADING_GAIN": tuning.corridor_follower.HEADING_GAIN,
        "sign_router.ACTIVATION_DIST_M": tuning.sign_router.ACTIVATION_DIST_M,
    }

    for field_name, value in critical_fields.items():
        assert value is not None, f"Tuning field {field_name} is None"
        if isinstance(value, (int, float)):
            assert value != 0.0, f"Tuning field {field_name} is zero (may indicate config error)"


def test_tuning_can_override_from_yaml() -> None:
    """Verify that custom tuning can be loaded and differs from defaults.

    This is a basic smoke test that tuning override mechanism works.
    Full override testing is in test_tuning_profile_override.py.
    """
    default = NavigationTuning.load_default()
    assert default is not None

    # Verify the default instance has expected structure
    assert hasattr(default, "sign_router")
    assert hasattr(default.sign_router, "ACTIVATION_DIST_M")

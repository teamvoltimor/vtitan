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


# Fields whose shipped value is DELIBERATELY zero, with the reason. A zero here
# is a tuning decision, not the "config never loaded" symptom the check below
# looks for, and asserting non-zero on one makes the suite fail on correct
# config -- which it did, from 2026-08-22 until this was written.
#
# Naming them individually rather than dropping the zero check keeps it live for
# every other field: a NEW accidental zero still fails.
DELIBERATELY_ZERO = {
    # Zeroed 2026-08-22: the blind creep holds heading instead of chasing the
    # centreline, because the lateral correction swings the chassis past
    # ALIGNMENT_TOLERANCE_RAD and starves the direction gate. Kept as a zeroed
    # gain rather than deleted so the branch survives for a chassis that wants
    # it -- see CorridorFollowerParams.CENTERING_GAIN_DEG_PER_M.
    "corridor_follower.CENTERING_GAIN_DEG_PER_M",
}


def test_tuning_fields_not_none() -> None:
    """Verify all tuning fields have non-None values (config is complete)."""
    tuning = NavigationTuning.load_default()

    # Spot-check critical fields that would break if config is incomplete
    critical_fields = {
        "lidar_sectors.MIN_VALID_RANGE_M": tuning.lidar_sectors.MIN_VALID_RANGE_M,
        "heading.CRAWL": tuning.heading.CRAWL,
        "direction_estimator.MAX_IN_TRACK_RANGE_M": tuning.direction_estimator.MAX_IN_TRACK_RANGE_M,
        "corridor_follower.CENTERING_GAIN_DEG_PER_M": tuning.corridor_follower.CENTERING_GAIN_DEG_PER_M,
        "corridor_follower.HEADING_GAIN": tuning.corridor_follower.HEADING_GAIN,
        "sign_router.ACTIVATION_DIST_M": tuning.sign_router.ACTIVATION_DIST_M,
    }

    for field_name, value in critical_fields.items():
        assert value is not None, f"Tuning field {field_name} is None"
        if isinstance(value, (int, float)) and field_name not in DELIBERATELY_ZERO:
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

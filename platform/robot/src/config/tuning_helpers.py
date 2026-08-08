"""Shared helper for the ``tuning or NavigationTuning.load_default()`` pattern
repeated across every navigation/simulation component that accepts an
optional ``NavigationTuning`` override.
"""

from __future__ import annotations

from shared.config.navigation_tuning import NavigationTuning


def get_tuning(tuning: NavigationTuning | None) -> NavigationTuning:
    """Return ``tuning`` unchanged, or the checked-in default if ``None``."""
    return tuning or NavigationTuning.load_default()

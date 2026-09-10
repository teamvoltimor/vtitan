"""Recursive merge for parsed TOML dicts.

Shared by :mod:`shared.config.robot_constants` and
:mod:`shared.config.navigation_tuning` to apply a hardware-profile overlay
(:mod:`shared.config.hardware_profile`) on top of the checked-in base
config. An overlay only declares the keys it changes, so a plain
``dict.update()`` would drop sibling keys inside any table the overlay also
touches -- e.g. overlaying just ``[steering].max_wheel_angle_deg`` must not
lose ``[steering].servo_max_angle_deg``.
"""

from __future__ import annotations


def deep_merge(base: dict[str, object], override: dict[str, object]) -> dict[str, object]:
    """Merge ``override`` onto ``base``, recursing into nested dicts.

    ``override``'s scalar/list values win outright; dict values recurse so
    only the leaf keys actually present in ``override`` replace their
    ``base`` counterpart.
    """
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)  # type: ignore[arg-type]
        else:
            result[key] = value
    return result

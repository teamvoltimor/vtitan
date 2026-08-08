"""Shared helper used by every tuning-group module in this package."""

from __future__ import annotations

from pydantic import AliasChoices


def _alias(name: str) -> AliasChoices:
    """Accept both SHOUT_CASE and lowercase tuning keys.

    Accepts the SHOUT_CASE field name (YAML/JSON profiles, direct kwargs) and
    its lowercase TOML-file spelling, so nested tuning groups keep their
    existing SHOUT_CASE attribute names everywhere they're read
    (``tuning.pursuit.LOOKAHEAD_SHORT``) while the checked-in per-group TOML
    files under DEFAULT_CONFIG_DIR use lowercase keys.
    """
    return AliasChoices(name, name.lower())

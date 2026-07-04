"""Round-trip tests for shared.config.navigation_tuning.NavigationTuning.

Regression coverage for the ClassVar bug: every tuning field used to be
annotated ``ClassVar``, which made the nested dataclasses accept zero
constructor arguments. A YAML/JSON profile with any override therefore either
raised ``TypeError`` (non-empty section) or was silently ignored (empty
section) — competition-day tuning changes never actually applied.
"""

from __future__ import annotations

import json

import pytest
import yaml
from shared.config.navigation_tuning import (
    ClearanceZones,
    EscapeManeuverParams,
    HeadingErrorZones,
    NavigationTuning,
    PurePursuitParams,
    SensorHealthParams,
    SpeedControlParams,
)

# One overridden value per group, distinct from the default, so a silently
# ignored section is caught by the round-trip assertion.
_OVERRIDES: dict[str, dict[str, float]] = {
    "clearance": {"CONTACT_DIST": 0.05, "SLOW_DIST": 0.20, "MEDIUM_DIST": 0.45, "FAST_DIST": 0.90},
    "heading": {"CRAWL": 1.2, "SLOW": 0.8, "MEDIUM": 0.5, "NORMAL": 0.25},
    "pursuit": {"LOOKAHEAD_SHORT": 0.15, "STEER_KP": 2.0},
    "speed": {"FAST_SPEED": 0.60},
    "escape": {"REV_SPEED": -0.30, "SIDE_CORRECTION_STEER": 0.4},
    "sensor": {"STALE_TIMEOUT_SEC": 0.75},
}


def test_defaults_construct_with_no_args():
    tuning = NavigationTuning()
    assert tuning.clearance.CONTACT_DIST == 0.10
    assert pytest.approx(0.3) == tuning.escape.SIDE_CORRECTION_STEER
    assert pytest.approx(0.5) == tuning.sensor.STALE_TIMEOUT_SEC


@pytest.mark.parametrize(
    ("group", "dataclass_type"),
    [
        ("clearance", ClearanceZones),
        ("heading", HeadingErrorZones),
        ("pursuit", PurePursuitParams),
        ("speed", SpeedControlParams),
        ("escape", EscapeManeuverParams),
        ("sensor", SensorHealthParams),
    ],
)
def test_group_accepts_keyword_overrides(group, dataclass_type):
    """Each nested tuning group must accept its documented field names.

    This is the direct regression check for the ClassVar bug: a ClassVar
    annotation would make the dataclass reject every one of these kwargs.
    """
    instance = dataclass_type(**_OVERRIDES[group])
    for field_name, value in _OVERRIDES[group].items():
        assert getattr(instance, field_name) == pytest.approx(value)


def test_load_from_yaml_round_trip(tmp_path):
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.dump(_OVERRIDES), encoding="utf-8")

    tuning = NavigationTuning.load_from_yaml(path)

    assert pytest.approx(0.05) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(1.2) == tuning.heading.CRAWL
    assert pytest.approx(2.0) == tuning.pursuit.STEER_KP
    assert pytest.approx(0.60) == tuning.speed.FAST_SPEED
    assert pytest.approx(-0.30) == tuning.escape.REV_SPEED
    assert pytest.approx(0.75) == tuning.sensor.STALE_TIMEOUT_SEC


def test_load_from_json_round_trip(tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(_OVERRIDES), encoding="utf-8")

    tuning = NavigationTuning.load_from_json(path)

    assert pytest.approx(0.05) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(0.4) == tuning.escape.SIDE_CORRECTION_STEER


def test_load_from_yaml_partial_profile_keeps_other_defaults(tmp_path):
    path = tmp_path / "partial.yaml"
    path.write_text(yaml.dump({"clearance": {"CONTACT_DIST": 0.08}}), encoding="utf-8")

    tuning = NavigationTuning.load_from_yaml(path)

    assert pytest.approx(0.08) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(0.25) == tuning.clearance.SLOW_DIST  # untouched default
    assert tuning.escape == EscapeManeuverParams()  # untouched group


def test_to_dict_round_trips_through_yaml(tmp_path):
    tuning = NavigationTuning.load_from_yaml(
        _write_yaml(tmp_path, _OVERRIDES),
    )
    exported = tuning.to_dict()

    path = tmp_path / "exported.yaml"
    path.write_text(yaml.dump(exported), encoding="utf-8")
    reloaded = NavigationTuning.load_from_yaml(path)

    assert reloaded == tuning


def _write_yaml(tmp_path, data: dict) -> str:
    path = tmp_path / "input.yaml"
    path.write_text(yaml.dump(data), encoding="utf-8")
    return str(path)

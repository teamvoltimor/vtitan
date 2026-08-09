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
    WaypointParams,
)

# One overridden value per group, distinct from the default, so a silently
# ignored section is caught by the round-trip assertion.
_OVERRIDES: dict[str, dict[str, float]] = {
    "clearance": {"CONTACT_DIST": 0.05, "SLOW_DIST": 0.20, "MEDIUM_DIST": 0.45, "FAST_DIST": 0.90},
    "heading": {"CRAWL": 1.2, "SLOW": 0.8, "MEDIUM": 0.5, "NORMAL": 0.25},
    "pursuit": {"LOOKAHEAD_SHORT": 0.15, "STEER_KP": 2.0},
    "speed": {"FAST_FRAC": 0.60},
    "escape": {"REV_SPEED": -0.30, "SIDE_CORRECTION_STEER": 0.4},
    "sensor": {"STALE_TIMEOUT_SEC": 0.75},
    "waypoints": {"ARC_RADIUS": 0.35},
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
        ("waypoints", WaypointParams),
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
    assert pytest.approx(0.60) == tuning.speed.FAST_FRAC
    assert pytest.approx(-0.30) == tuning.escape.REV_SPEED
    assert pytest.approx(0.75) == tuning.sensor.STALE_TIMEOUT_SEC
    assert pytest.approx(0.35) == tuning.waypoints.ARC_RADIUS


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


def _write_toml_line(value) -> str:
    return repr(value) if isinstance(value, str) else str(value)


def _write_toml_dir(tmp_path, groups: dict[str, dict]) -> str:
    """One <subfolder>/<group>.toml per key, matching load_from_toml_dir's
    expected layout -- the subfolder comes from NavigationTuning._GROUPS
    itself, so this stays correct if a group's subfolder ever changes.
    """
    subfolder_by_group = {key: subfolder for key, _, subfolder in NavigationTuning._GROUPS}
    directory = tmp_path / "navigation"
    directory.mkdir()
    for group, fields in groups.items():
        group_dir = directory / subfolder_by_group[group]
        group_dir.mkdir(exist_ok=True)
        lines = [f"{key} = {_write_toml_line(value)}" for key, value in fields.items()]
        (group_dir / f"{group}.toml").write_text("\n".join(lines), encoding="utf-8")
    return str(directory)


def test_load_from_toml_dir_round_trip(tmp_path):
    directory = _write_toml_dir(tmp_path, _OVERRIDES)

    tuning = NavigationTuning.load_from_toml_dir(directory)

    assert pytest.approx(0.05) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(1.2) == tuning.heading.CRAWL
    assert pytest.approx(2.0) == tuning.pursuit.STEER_KP
    assert pytest.approx(0.60) == tuning.speed.FAST_FRAC
    assert pytest.approx(-0.30) == tuning.escape.REV_SPEED
    assert pytest.approx(0.75) == tuning.sensor.STALE_TIMEOUT_SEC
    assert pytest.approx(0.35) == tuning.waypoints.ARC_RADIUS


def test_load_from_toml_dir_partial_files_keep_other_defaults(tmp_path):
    directory = _write_toml_dir(tmp_path, {"clearance": {"CONTACT_DIST": 0.08}})

    tuning = NavigationTuning.load_from_toml_dir(directory)

    assert pytest.approx(0.08) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(0.25) == tuning.clearance.SLOW_DIST  # untouched field, same group
    assert tuning.escape == EscapeManeuverParams()  # untouched group -- no escape.toml at all


def test_load_from_toml_dir_missing_directory_returns_defaults(tmp_path):
    tuning = NavigationTuning.load_from_toml_dir(tmp_path / "does_not_exist")

    assert tuning == NavigationTuning()


def test_load_default_finds_the_checked_in_config_tree():
    """The actual platform/shared/config/navigation/ tree this repo ships."""
    tuning = NavigationTuning.load_default()

    assert pytest.approx(0.10) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(1.2) == tuning.pursuit.STEER_KP
    assert pytest.approx(1.00) == tuning.clearance.FAST_DIST


class TestConfiguredValuesAreActuallyRead:
    """Every configured field must have a reader, and no module may shadow one.

    Two failure modes, both silent, both found in this codebase on 2026-08-01:

    * ``SignRouterParams.DEFORM_DEPTH_BUFFER_M`` sat in sign_router.toml with no
      reader anywhere while the router used its own literal — editing the config
      file did nothing at all.
    * ``parking.py``, ``sign_discovery.py`` and ``waypoints.py`` each kept a
      private literal beside a live config field, annotated "same concept/value
      as NavigationTuning.X", so the two agreed only as long as a human kept
      them agreeing.

    A config entry that nothing reads is worse than no entry: it advertises
    control it does not have. This walks the declared fields and asserts each is
    referenced somewhere outside its own definition.
    """

    _SEARCH_ROOTS = ("src", "tests", "scripts")
    _KNOWN_UNREAD = {
        # Declared, configurable, and read by nothing. Left failing-visible here
        # rather than silently excluded: each is either dead config to delete or
        # a limit someone believed was in force. Speed limits in particular look
        # like they bound the robot and do not.
        "SLALOM_REVERSE_FRAMES",
        "SLALOM_FORWARD_FRAMES",
        # The rest of HeadingErrorZones: heading.toml ships, loads, MEDIUM is read
        # (src.navigation.utils' _ALIGNMENT_TOLERANCE_RAD, wired 2026-08-02), and
        # CRAWL/SLOW are read now too (CoreNavigator.step's heading-to-speed
        # coupling, wired 2026-08-03 alongside the pure-pursuit steering fix --
        # see docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md).
        # NORMAL is still dead config: the speed ladder only needed three
        # thresholds (mirroring ClearanceZones' three), so it was left unused
        # rather than forcing a fourth rung. Delete it, or wire it to a finer
        # "full speed" cutoff if one turns out to be needed.
        "NORMAL",
    }

    @staticmethod
    def _corpus() -> str:
        import pathlib

        root = pathlib.Path(__file__).resolve().parents[2]
        chunks = []
        for sub in TestConfiguredValuesAreActuallyRead._SEARCH_ROOTS:
            for path in (root / sub).rglob("*.py"):
                # Skip the definition itself, and this module: it names the
                # fields it is checking, so counting it as a reader would make
                # every exclusion look wired up.
                if path.name in {"navigation_tuning.py", "test_navigation_tuning.py"}:
                    continue
                try:
                    chunks.append(path.read_text(encoding="utf-8"))
                except OSError:
                    continue
        return "\n".join(chunks)

    @staticmethod
    def _reader_names(group, field: str) -> list[str]:
        """Names that count as reading ``field``, including via an accessor.

        A field is not always read under its own name. ``SpeedControlParams``
        stores fractions of the drivetrain ceiling and exposes each one through
        a ``*_mps()`` method, because a bare fraction is not a speed and callers
        must never treat it as one. ``SLOW_FRAC`` is therefore read as
        ``slow_mps()`` and a plain name grep cannot see it.

        Resolving the accessor here keeps the check honest in both directions:
        an accessor that nothing calls still fails, and a field whose accessor
        does not exist is not quietly excused.
        """
        names = [field]
        prefix = field.removesuffix("_FRAC")
        if prefix != field and hasattr(group, f"{prefix.lower()}_mps"):
            names.append(f"{prefix.lower()}_mps")
        return names

    def test_every_configured_field_has_a_reader(self):
        import re

        corpus = self._corpus()
        tuning = NavigationTuning()
        unread = []
        for group_name in type(tuning).__dataclass_fields__:
            group = getattr(tuning, group_name)
            for field in getattr(type(group), "model_fields", {}):
                if field in self._KNOWN_UNREAD:
                    continue
                names = self._reader_names(group, field)
                if not any(re.search(rf"\b{re.escape(name)}\b", corpus) for name in names):
                    unread.append(f"{group_name}.{field}")
        assert not unread, (
            f"configured but never read: {unread}. Either wire them up or delete them — "
            f"a config entry nothing reads advertises control it does not have."
        )

    def test_known_unread_list_has_not_grown_stale(self):
        """If one of these gets wired up, drop it from the exclusion list."""
        import re

        corpus = self._corpus()
        now_read = [f for f in self._KNOWN_UNREAD if re.search(rf"\b{re.escape(f)}\b", corpus)]
        assert not now_read, f"now read, remove from _KNOWN_UNREAD: {now_read}"

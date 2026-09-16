"""Round-trip tests for shared.config.navigation_tuning.NavigationTuning.

Regression coverage for the ClassVar bug: every tuning field used to be
annotated ``ClassVar``, which made the nested dataclasses accept zero
constructor arguments. A YAML/JSON profile with any override therefore either
raised ``TypeError`` (non-empty section) or was silently ignored (empty
section) — competition-day tuning changes never actually applied.
"""

from __future__ import annotations

import json
import tomllib
from typing import ClassVar

import pytest
import yaml
from pydantic import ValidationError
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
from shared.domain.enums import ScenarioType

# One overridden value per group, distinct from the default, so a silently
# ignored section is caught by the round-trip assertion.
_OVERRIDES: dict[str, dict[str, float]] = {
    "clearance": {"contact_dist": 0.05, "slow_dist": 0.20, "medium_dist": 0.45, "fast_dist": 0.90},
    "heading": {"crawl": 1.2},
    "pursuit": {"lookahead_short": 0.15, "steer_kp": 2.0},
    "speed": {"fast_mps": 0.140},
    "escape": {"rev_speed": -0.30, "side_correction_steer_deg": 20.0},
    "sensor": {"stale_timeout_sec": 0.75},
    "waypoints": {"arc_radius": 0.35},
}


def _base_profile(**overrides: dict) -> dict[str, dict]:
    """The shipped base values (read from the checked-in TOML), with per-group overrides applied.

    The TOML is the single source now, so a profile must carry every group:
    building one from a bare ``NavigationTuning()`` keeps each test's overrides
    the only thing it varies.
    """
    base = NavigationTuning().to_dict()
    for group, fields in overrides.items():
        base[group] = {**base[group], **fields}
    return base


def test_defaults_construct_with_no_args():
    tuning = NavigationTuning()
    assert tuning.clearance.contact_dist == 0.10
    assert pytest.approx(16.5) == tuning.escape.side_correction_steer_deg
    assert pytest.approx(0.5) == tuning.sensor.stale_timeout_sec


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
    base = getattr(NavigationTuning(), group).model_dump()
    instance = dataclass_type(**{**base, **_OVERRIDES[group]})
    for field_name, value in _OVERRIDES[group].items():
        assert getattr(instance, field_name) == pytest.approx(value)


def test_load_from_yaml_round_trip(tmp_path):
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.dump(_base_profile(**_OVERRIDES)), encoding="utf-8")

    tuning = NavigationTuning.load_from_yaml(path)

    assert pytest.approx(0.05) == tuning.clearance.contact_dist
    assert pytest.approx(1.2) == tuning.heading.crawl
    assert pytest.approx(2.0) == tuning.pursuit.steer_kp
    assert pytest.approx(0.140) == tuning.speed.fast_mps
    assert pytest.approx(-0.30) == tuning.escape.rev_speed
    assert pytest.approx(0.75) == tuning.sensor.stale_timeout_sec
    assert pytest.approx(0.35) == tuning.waypoints.arc_radius


def test_load_from_json_round_trip(tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(_base_profile(**_OVERRIDES)), encoding="utf-8")

    tuning = NavigationTuning.load_from_json(path)

    assert pytest.approx(0.05) == tuning.clearance.contact_dist
    assert pytest.approx(20.0) == tuning.escape.side_correction_steer_deg


def test_load_from_yaml_partial_profile_raises(tmp_path):
    path = tmp_path / "partial.yaml"
    path.write_text(yaml.dump({"clearance": {"contact_dist": 0.08}}), encoding="utf-8")

    # Every group must be present: a partial profile is an error, not a silent
    # fallback to a hardcoded value.
    with pytest.raises(ValidationError):
        NavigationTuning.load_from_yaml(path)


def test_to_dict_round_trips_through_yaml(tmp_path):
    tuning = NavigationTuning.load_from_yaml(
        _write_yaml(tmp_path, _base_profile(**_OVERRIDES)),
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
    if isinstance(value, bool):
        return "true" if value else "false"
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
        lines = [
            f"{key} = {_write_toml_line(value)}" for key, value in fields.items() if value is not None
        ]
        (group_dir / f"{group}.toml").write_text("\n".join(lines), encoding="utf-8")
    return str(directory)


def test_load_from_toml_dir_round_trip(tmp_path):
    directory = _write_toml_dir(tmp_path, _base_profile(**_OVERRIDES))

    tuning = NavigationTuning.load_from_toml_dir(directory)

    assert pytest.approx(0.05) == tuning.clearance.contact_dist
    assert pytest.approx(1.2) == tuning.heading.crawl
    assert pytest.approx(2.0) == tuning.pursuit.steer_kp
    assert pytest.approx(0.140) == tuning.speed.fast_mps
    assert pytest.approx(-0.30) == tuning.escape.rev_speed
    assert pytest.approx(0.75) == tuning.sensor.stale_timeout_sec
    assert pytest.approx(0.35) == tuning.waypoints.arc_radius


def test_load_from_toml_dir_partial_tree_raises(tmp_path):
    directory = _write_toml_dir(tmp_path, {"clearance": {"contact_dist": 0.08}})

    # A partial tree (no escape.toml, and the other groups absent too) is an
    # error, not a silent fallback.
    with pytest.raises(ValidationError):
        NavigationTuning.load_from_toml_dir(directory)


def test_load_from_toml_dir_missing_directory_raises(tmp_path):
    with pytest.raises(ValidationError):
        NavigationTuning.load_from_toml_dir(tmp_path / "does_not_exist")


def test_load_default_finds_the_checked_in_config_tree():
    """The actual src/config/navigation/ tree this repo ships."""
    tuning = NavigationTuning.load_default()

    assert pytest.approx(0.10) == tuning.clearance.contact_dist
    assert pytest.approx(1.2) == tuning.pursuit.steer_kp
    assert pytest.approx(1.00) == tuning.clearance.fast_dist


def test_load_default_open_challenge_is_byte_identical_to_no_challenge():
    """Standing regression guard: Open Challenge tuning must never change as a
    side effect of Obstacles Challenge work. The checked-in
    navigation-challenges/open/ overlay is expected to stay empty forever --
    this assertion is the enforcement, not the overlay directory being empty.
    """
    assert NavigationTuning.load_default(challenge=ScenarioType.OPEN) == NavigationTuning.load_default()


def test_load_default_challenge_none_matches_no_challenge_argument():
    """Omitting ``challenge`` entirely must reproduce pre-existing behaviour."""
    assert NavigationTuning.load_default(challenge=None) == NavigationTuning.load_default()


def test_load_from_toml_dirs_merges_a_challenge_overlay_last(tmp_path):
    """A later directory's values win -- this is what lets a challenge overlay
    retune a key without touching the base config or any other challenge.
    """
    base = _write_toml_dir(tmp_path, _base_profile(waypoints={"arc_radius": 0.45}))
    overlay = tmp_path / "obstacles_overlay"
    overlay.mkdir()
    (overlay / "waypoint").mkdir()
    (overlay / "waypoint" / "waypoints.toml").write_text("arc_radius = 0.30", encoding="utf-8")

    merged = NavigationTuning.load_from_toml_dirs([base, overlay])
    base_only = NavigationTuning.load_from_toml_dirs([base])

    assert pytest.approx(0.30) == merged.waypoints.arc_radius
    assert pytest.approx(0.45) == base_only.waypoints.arc_radius
    # Untouched fields still fall back through the base directory, not the default.
    assert merged.clearance == base_only.clearance


class TestConfiguredValuesAreActuallyRead:
    """Every configured field must have a reader, and no module may shadow one.

    Two failure modes, both silent:

    * A ``SignRouterParams`` field sat in sign_router.toml with no reader
      anywhere while the router used its own literal: editing the config file
      did nothing at all.
    * ``parking.py``, ``sign_discovery.py`` and ``waypoints.py`` each kept a
      private literal beside a live config field, annotated "same concept/value
      as NavigationTuning.X", so the two agreed only as long as a human kept
      them agreeing.

    A config entry that nothing reads is worse than no entry: it advertises
    control it does not have. This walks the declared fields and asserts each is
    referenced somewhere outside its own definition. See
    adr:0069-config-governance.
    """

    _SEARCH_ROOTS = ("src", "tests", "scripts")
    _KNOWN_UNREAD: ClassVar[set[str]] = {
        # Declared, configurable, and read by nothing. Left failing-visible here
        # rather than silently excluded: each is either dead config to delete or
        # a limit someone believed was in force. Speed limits in particular look
        # like they bound the robot and do not.
        "slalom_reverse_s",
        "slalom_forward_s",
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
        stores absolute m/s and exposes each tier through a ``*_mps()`` method
        that clamps it to the drivetrain ceiling, so callers cannot command a
        speed the motor has no way to reach. ``SLOW_MPS`` is therefore read as
        ``slow_mps()`` and a plain name grep cannot see it.

        Resolving the accessor here keeps the check honest in both directions:
        an accessor that nothing calls still fails, and a field whose accessor
        does not exist is not quietly excused.
        """
        names = [field]
        # SLOW_MPS is read as slow_mps(); the accessor is the field lowercased,
        # so resolve by attribute rather than by rewriting a suffix. Written
        # this way after the speed tiers moved from fractions to absolute m/s
        # (see adr:0085-speed-envelope): a suffix rule silently stopped matching
        # and excused every field it could no longer see.
        #
        # A field whose stored unit differs from the unit the actuator takes
        # gets a converting accessor instead, and the name changes with it:
        # rev_steer_deg holds a physical road-wheel angle and is read as
        # rev_steer_norm(), because the normalised command that delivers that
        # angle depends on the servo's reach. Both spellings are resolved by
        # attribute, so an accessor that does not exist still fails the check.
        lower = field.lower()
        candidates = [lower]
        if lower.endswith("_deg"):
            candidates.append(f"{lower[: -len('_deg')]}_norm")
        # Same shape again for durations: an escape length is STORED in
        # seconds and READ as a tick count, because the loop is discrete and
        # a frame count stored directly would silently mean a different
        # duration if CONTROL_HZ ever moved. k_turn_min_s is read as
        # k_turn_min_frames().
        if lower.endswith("_s"):
            candidates.append(f"{lower[: -len('_s')]}_frames")
        # A per-challenge tier override is read through the resolver that
        # applies it, not under its own name: OPEN_FAST_MPS reaches the
        # navigator as for_open_challenge().fast_mps. Same indirection as the
        # *_mps() accessors above, one level further out -- and resolved by
        # attribute for the same reason, so a resolver that stops existing
        # (or that nothing in robot code calls) still fails this check rather
        # than excusing the four fields it would have applied.
        candidates.extend(
            f"for_{prefix.lower()}_challenge"
            for prefix in ("OPEN", "OBSTACLES")
            if lower.startswith(f"{prefix.lower()}_")
        )
        names.extend(
            accessor for accessor in candidates if accessor != field and callable(getattr(group, accessor, None))
        )
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


class TestBareTuningReadsTheBaseToml:
    """A bare ``NavigationTuning()`` is the base TOML, not a second copy in code.

    With the per-group ``_DEFAULTS`` gone, there is no hardcoded fallback to
    drift from the checked-in tree: constructing tuning bare reads the same
    base files ``load_default`` does, so the two agree by construction.
    """

    def test_bare_tuning_matches_base_with_no_profile(self, monkeypatch):
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "")
        assert NavigationTuning() == NavigationTuning.load_default()

    def test_every_group_has_a_base_file(self):
        from shared.config.navigation_tuning import DEFAULT_CONFIG_DIR

        missing = [
            f"{key}: no {subfolder}/{key}.toml in the base tree"
            for key, _dataclass_type, subfolder in NavigationTuning._GROUPS
            if not (DEFAULT_CONFIG_DIR / subfolder / f"{key}.toml").exists()
        ]
        assert not missing, f"base TOML tree incomplete: {missing}"



class TestPerChallengeSpeedTiers:
    """The shared speed ladder and the per-challenge overrides the schema keeps.

    The old per-JOB creep split (``contact_mps`` and friends) and the
    per-challenge ``creep`` tier were dropped from the schema; those accessors
    now track the shared ``creep_mps`` value, and the challenge resolvers apply
    only the slow/medium/fast/max tiers the profiles declare.
    """

    def test_unset_keeps_one_shared_ladder(self):
        speed = NavigationTuning().speed
        assert speed.open_fast_mps is None
        assert speed.obstacles_fast_mps is None
        assert speed.for_open_challenge().fast_mps == speed.fast_mps
        assert speed.for_obstacles_challenge().fast_mps == speed.fast_mps

    def test_each_challenge_reads_its_own_override(self):
        speed = NavigationTuning().speed.model_copy(
            update={"open_fast_mps": 0.15, "obstacles_fast_mps": 0.12}
        )
        assert speed.for_open_challenge().fast_mps == pytest.approx(0.15)
        assert speed.for_obstacles_challenge().fast_mps == pytest.approx(0.12)

    def test_one_override_does_not_move_the_other_challenge(self):
        base = NavigationTuning().speed
        speed = base.model_copy(update={"open_fast_mps": 0.15})
        assert speed.for_open_challenge().fast_mps == pytest.approx(0.15)
        assert speed.for_obstacles_challenge().fast_mps == base.fast_mps

    def test_a_tier_above_its_challenge_cap_is_rejected(self):
        """Inert tuning must fail loudly -- the ladder clamps to max_mps."""
        base = NavigationTuning().speed.model_dump()
        with pytest.raises(ValidationError, match="open_fast_mps"):
            SpeedControlParams.model_validate(base | {"open_fast_mps": 0.90, "open_max_mps": 0.50})


class TestRetiredCreepJobsTrackTheSharedTier:
    """The per-JOB creep split is gone from the schema.

    ``creep_mps`` is still read by five unrelated jobs; each job's accessor is
    kept so call sites do not change, but every one now returns the shared
    ``creep_mps`` value, which is what they resolved to whenever unset.
    """

    JOBS: ClassVar[tuple[str, ...]] = (
        "contact_mps",
        "contact_reverse_mps",
        "sign_evade_mps",
        "escape_nudge_mps",
        "heading_floor_mps",
    )

    def test_every_job_tracks_the_shared_tier(self):
        speed = NavigationTuning().speed
        for job in self.JOBS:
            assert getattr(speed, job)() == speed.creep_mps, job

    def test_moving_the_shared_tier_moves_every_job(self):
        speed = NavigationTuning().speed.model_copy(update={"creep_mps": 0.20})
        for job in self.JOBS:
            assert getattr(speed, job)() == pytest.approx(0.20), job


class TestCornerPreviewIsASingleValue:
    """The wide/narrow preview split is gone from the schema.

    The generated DTO declares only ``corner_preview_distance_m``; the retired
    ``wide_corner_preview_distance_m`` resolver returned the shared value when
    unset, which is what ships, so one value now serves both width classes. See
    adr:0052-pursuit-target-selection.
    """

    def test_one_value_for_both_classes(self):
        pursuit = NavigationTuning.load_default().pursuit
        assert pursuit.corner_preview_distance_m == pytest.approx(0.80)

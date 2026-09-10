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
    "clearance": {"CONTACT_DIST": 0.05, "SLOW_DIST": 0.20, "MEDIUM_DIST": 0.45, "FAST_DIST": 0.90},
    "heading": {"CRAWL": 1.2},
    "pursuit": {"LOOKAHEAD_SHORT": 0.15, "STEER_KP": 2.0},
    "speed": {"FAST_MPS": 0.140},
    "escape": {"REV_SPEED": -0.30, "SIDE_CORRECTION_STEER_DEG": 20.0},
    "sensor": {"STALE_TIMEOUT_SEC": 0.75},
    "waypoints": {"ARC_RADIUS": 0.35},
}


def test_defaults_construct_with_no_args():
    tuning = NavigationTuning()
    assert tuning.clearance.CONTACT_DIST == 0.10
    assert pytest.approx(16.5) == tuning.escape.SIDE_CORRECTION_STEER_DEG
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
    assert pytest.approx(0.140) == tuning.speed.FAST_MPS
    assert pytest.approx(-0.30) == tuning.escape.REV_SPEED
    assert pytest.approx(0.75) == tuning.sensor.STALE_TIMEOUT_SEC
    assert pytest.approx(0.35) == tuning.waypoints.ARC_RADIUS


def test_load_from_json_round_trip(tmp_path):
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(_OVERRIDES), encoding="utf-8")

    tuning = NavigationTuning.load_from_json(path)

    assert pytest.approx(0.05) == tuning.clearance.CONTACT_DIST
    assert pytest.approx(20.0) == tuning.escape.SIDE_CORRECTION_STEER_DEG


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
    assert pytest.approx(0.140) == tuning.speed.FAST_MPS
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
    base = _write_toml_dir(tmp_path, {"waypoints": {"ARC_RADIUS": 0.45}})
    overlay = tmp_path / "obstacles_overlay"
    overlay.mkdir()
    (overlay / "waypoint").mkdir()
    (overlay / "waypoint" / "waypoints.toml").write_text("ARC_RADIUS = 0.30", encoding="utf-8")

    merged = NavigationTuning.load_from_toml_dirs([base, overlay])
    base_only = NavigationTuning.load_from_toml_dirs([base])

    assert pytest.approx(0.30) == merged.waypoints.ARC_RADIUS
    assert pytest.approx(0.45) == base_only.waypoints.ARC_RADIUS
    # Untouched fields still fall back through the base directory, not the default.
    assert merged.clearance == base_only.clearance


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
    _KNOWN_UNREAD: ClassVar[set[str]] = {
        # Declared, configurable, and read by nothing. Left failing-visible here
        # rather than silently excluded: each is either dead config to delete or
        # a limit someone believed was in force. Speed limits in particular look
        # like they bound the robot and do not.
        "SLALOM_REVERSE_S",
        "SLALOM_FORWARD_S",
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
        # this way after the tiers moved from *_FRAC to *_MPS in 2026-08-21 --
        # a suffix rule silently stopped matching and excused every field it
        # could no longer see.
        #
        # A field whose stored unit differs from the unit the actuator takes
        # gets a converting accessor instead, and the name changes with it:
        # REV_STEER_DEG holds a physical road-wheel angle and is read as
        # rev_steer_norm(), because the normalised command that delivers that
        # angle depends on the servo's reach. Both spellings are resolved by
        # attribute, so an accessor that does not exist still fails the check.
        candidates = [field.lower()]
        if field.endswith("_DEG"):
            candidates.append(f"{field[: -len('_DEG')].lower()}_norm")
        # Same shape again for durations: an escape length is STORED in
        # seconds and READ as a tick count, because the loop is discrete and
        # a frame count stored directly would silently mean a different
        # duration if CONTROL_HZ ever moved. K_TURN_MIN_S is read as
        # k_turn_min_frames().
        if field.endswith("_S"):
            candidates.append(f"{field[: -len('_S')].lower()}_frames")
        # A per-challenge tier override is read through the resolver that
        # applies it, not under its own name: OPEN_FAST_MPS reaches the
        # navigator as for_open_challenge().fast_mps(). Same indirection as the
        # *_mps() accessors above, one level further out -- and resolved by
        # attribute for the same reason, so a resolver that stops existing
        # (or that nothing in robot code calls) still fails this check rather
        # than excusing the four fields it would have applied.
        candidates.extend(
            f"for_{prefix.lower()}_challenge"
            for prefix in ("OPEN", "OBSTACLES")
            if field.startswith(f"{prefix}_")
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


class TestFieldDefaultsMatchShippedToml:
    """The pydantic field defaults are a SECOND copy of the shipped TOML values.

    Nothing keeps them in step, and nothing fails when they part: a bare
    ``NavigationTuning()`` simply plans a different car than the checked-in
    config does. ``WIDE_CENTER_BIAS_M`` (then named ``CENTER_BIAS_M``) had
    drifted to 0.05 against the TOML's 0.10 --
    half the commanded offset from the corridor centreline -- and every
    diagnostic that builds tuning bare (``diag_sign_sweep.tuning()`` among them)
    measured at the drifted value without any signal that it had.
    """

    def test_field_defaults_match_shipped_toml(self, monkeypatch):
        # Compared against the BASE tree only, with no hardware profile active.
        # `load_default` composes [DEFAULT_CONFIG_DIR, *profile_dirs()], and
        # navigation tuning has per-profile overlays of its own
        # (profiles/<name>/motion/speed.toml), so "the shipped value" is not one
        # number -- rev-hd-hex-motor-6000rpm ships max_mps 0.50 where the base
        # ships 0.156. A single Field default cannot mirror every profile, and
        # comparing it against whichever one happens to be pinned makes this
        # test fail on correct config: it passed only while the pinned profile
        # was byte-identical to the base, which stopped being true when
        # 35a86a3e moved the suite to the current build.
        #
        # The drift this guards is still guarded. A bare NavigationTuning() has
        # no profile by definition, so the base tree is exactly what its
        # defaults are a second copy of -- WIDE_CENTER_BIAS_M, and the
        # BLIND_WEDGE_* and MAX_STEERING_RATE drift found in 0d4a8b70, were all
        # base-vs-default and all still caught here.
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "")
        bare = NavigationTuning()
        shipped = NavigationTuning.load_default()

        drifted = {}
        for group_name in type(bare).__dataclass_fields__:
            bare_group = getattr(bare, group_name)
            shipped_group = getattr(shipped, group_name)
            if not hasattr(bare_group, "model_dump"):
                continue
            shipped_values = shipped_group.model_dump()
            for field, bare_value in bare_group.model_dump().items():
                # A ``None`` default is an ABSENCE, not a second copy of a
                # value, so it cannot drift in the way this test guards against.
                # Optional per-profile overrides (speed's OPEN_*/OBSTACLES_*
                # tiers) are declared unset precisely so a motor without the
                # headroom inherits the shared ladder; giving them a concrete
                # default to satisfy this check would hand every motor the fast
                # profile's numbers, which is the bug this test exists to catch,
                # inverted. Fields with a real default are still compared.
                if bare_value is None:
                    continue
                if bare_value != shipped_values[field]:
                    drifted[f"{group_name}.{field}"] = (bare_value, shipped_values[field])

        assert not drifted, (
            f"field default(s) out of step with the shipped TOML: {drifted}. "
            f"Each pair is (bare default, shipped). Update the Field(default=...) "
            f"to match the config file -- code that constructs tuning bare is "
            f"otherwise silently running values nobody chose."
        )


class TestShippedTreeIsComplete:
    """The checked-in base TOML tree must name every knob its groups declare.

    Partial loading is DELIBERATE everywhere else and stays untouched: a
    hardware profile ships only the keys it retunes, challenge overlays may be
    empty, and a bare ``NavigationTuning()`` in a sim/test context falls back
    through everything to pydantic defaults. This check pins only the base
    tree -- the file ``load_default`` is documented to be "the normal way to
    construct a NavigationTuning in production code" -- so that a knob with a
    concrete default cannot be half-landed: declared in the model (where the
    shipped value rests) but absent from the file an operator would edit to
    reach it. That failure happened for real, twice: every bay-exit key lived
    only as a Python literal until 2026-09-05, and an earlier zero-lap round
    shipped with ``SLOW_MPS`` in the model while speed.toml named nothing of
    the tier ladder.
    """

    # Concrete defaults that are RESOLVED rather than restated. The Field
    # default is not a second opinion here: it is lifted from another single
    # source at class-definition time, so naming the value in the TOML would
    # re-create exactly the two-names-for-one-number style this repo deletes.
    _RESOLVED_DEFAULTS: ClassVar[set[str]] = {
        # Value is RobotSpecs.MIN_TURN_RADIUS_M, read from robot.toml, the
        # same measurement the dead reckoning and the kinematics floor cite.
        "simulation.MIN_TURN_RADIUS_M",
    }

    def test_every_concrete_default_appears_in_the_base_toml(self):
        from shared.config.navigation_tuning import DEFAULT_CONFIG_DIR

        missing = []
        drifted = {}
        base = NavigationTuning()
        for key, _dataclass_type, subfolder in NavigationTuning._GROUPS:
            toml_path = DEFAULT_CONFIG_DIR / subfolder / f"{key}.toml"
            if not toml_path.exists():
                missing.append(f"{key}: no {subfolder}/{key}.toml in the base tree")
                continue
            data = tomllib.loads(toml_path.read_text(encoding="utf-8"))
            bare = getattr(base, key).model_dump()
            for field_name, field in type(getattr(base, key)).model_fields.items():
                if field.default is None:
                    # A None default is an ABSENCE, not a value owed here --
                    # per-challenge tiers and optional gates are deliberately
                    # unset and TestFieldDefaultsMatchShippedToml treats them
                    # the same way.
                    continue
                if f"{key}.{field_name}" in self._RESOLVED_DEFAULTS:
                    continue
                shipped_key = field_name if field_name in data else field_name.lower()
                if shipped_key not in data:
                    missing.append(f"{key}.{field_name}: unnamed in {subfolder}/{key}.toml")
                elif data.get(shipped_key) != field.default and not isinstance(
                    field.default, bool
                ):
                    # bool(repr) format differences do not exist in TOML; only
                    # float-vs-int spelling can differ (0 vs 0.0), and pydantic
                    # accepts both, so compare with its tolerance.
                    drifted[f"{key}.{field_name}"] = (data.get(shipped_key), field.default)
        assert not drifted, f"shipped values have drifted from the model: {drifted}"
        assert not missing, f"base TOML tree incomplete: {missing}"


class TestPerChallengeCreep:
    """``CREEP_MPS`` splits by challenge because it does two conflicting jobs.

    It is the contact-zone speed, argued in centimetres of lateral margin, and
    it is the heading limiter's floor, which the 2026-09-08 bags put under
    every momentary stop in normal_drive. Obstacles wants it low, Open spends
    44-64% of the round on it. One number cannot serve both.
    """

    def test_unset_keeps_one_shared_tier(self):
        """The fallback is the point, not a degenerate case."""
        speed = NavigationTuning().speed
        assert speed.OPEN_CREEP_MPS is None
        assert speed.OBSTACLES_CREEP_MPS is None
        assert speed.for_open_challenge().creep_mps() == speed.creep_mps()
        assert speed.for_obstacles_challenge().creep_mps() == speed.creep_mps()

    def test_each_challenge_reads_its_own_override(self):
        speed = NavigationTuning().speed.model_copy(
            update={"OPEN_CREEP_MPS": 0.20, "OBSTACLES_CREEP_MPS": 0.12}
        )
        assert speed.for_open_challenge().creep_mps() == pytest.approx(0.20)
        assert speed.for_obstacles_challenge().creep_mps() == pytest.approx(0.12)

    def test_one_override_does_not_move_the_other_challenge(self):
        base = NavigationTuning().speed
        speed = base.model_copy(update={"OPEN_CREEP_MPS": 0.20})
        assert speed.for_open_challenge().creep_mps() == pytest.approx(0.20)
        assert speed.for_obstacles_challenge().creep_mps() == base.creep_mps()

    def test_a_creep_above_its_challenge_cap_is_rejected(self):
        """Inert tuning must fail loudly -- the ladder clamps to max_mps()."""
        base = NavigationTuning().speed.model_dump()
        with pytest.raises(ValidationError, match="OPEN_CREEP_MPS"):
            SpeedControlParams.model_validate(base | {"OPEN_CREEP_MPS": 0.90, "OPEN_MAX_MPS": 0.50})

    def test_a_creep_below_the_friction_floor_is_rejected(self):
        """The envelope clamp would swallow it, and hide any tuning done to it."""
        base = NavigationTuning().speed.model_dump()
        with pytest.raises(ValidationError, match="OPEN_CREEP_MPS"):
            SpeedControlParams.model_validate(base | {"OPEN_CREEP_MPS": 0.001})


class TestHeadingFloorIsSeparableFromCreep:
    """``CREEP_MPS`` is read by five unrelated jobs; the heading one can leave.

    Of 6105 ticks commanded at the creep floor across the 2026-09-08 session,
    97.8% arrived through the heading term alone. The other readers are
    emergencies whose failure is a collision, not a slow lap, so the corner
    floor must be able to move without them.
    """

    def test_unset_is_the_coupled_behaviour(self):
        speed = NavigationTuning().speed
        assert speed.HEADING_FLOOR_MPS is None
        assert speed.heading_floor_mps() == speed.creep_mps()

    def test_set_moves_only_the_heading_floor(self):
        base = NavigationTuning().speed
        speed = base.model_copy(update={"HEADING_FLOOR_MPS": 0.20})
        assert speed.heading_floor_mps() == pytest.approx(0.20)
        assert speed.creep_mps() == base.creep_mps()

    def test_it_composes_with_the_per_challenge_creep(self):
        """One splits the tier by JOB, the other by CHALLENGE."""
        speed = NavigationTuning().speed.model_copy(
            update={"HEADING_FLOOR_MPS": 0.20, "OPEN_CREEP_MPS": 0.18}
        )
        assert speed.for_open_challenge().creep_mps() == pytest.approx(0.18)
        assert speed.for_open_challenge().heading_floor_mps() == pytest.approx(0.20)


class TestEveryCreepJobIsNameable:
    """All five readers of the creep tier have their own optional field.

    Only the heading one is earned by evidence today (97.8% of the ticks that
    reach the floor). The other four exist so that moving one job cannot move
    another by accident -- the contact jobs fail as COLLISIONS and the heading
    job fails as a SLOW LAP, so they have to be able to disagree.
    """

    JOBS: ClassVar[tuple[str, ...]] = (
        "contact_mps",
        "contact_reverse_mps",
        "sign_evade_mps",
        "escape_nudge_mps",
        "heading_floor_mps",
    )

    def test_unset_every_job_tracks_the_shared_tier(self):
        """The fallback is the shipped behaviour, bit for bit."""
        speed = NavigationTuning().speed
        for job in self.JOBS:
            assert getattr(speed, job)() == speed.creep_mps(), job

    def test_moving_one_job_moves_only_that_job(self):
        base = NavigationTuning().speed
        for job in self.JOBS:
            field = job.upper()
            speed = base.model_copy(update={field: 0.20})
            assert getattr(speed, job)() == pytest.approx(0.20), job
            others = [other for other in self.JOBS if other != job]
            for other in others:
                assert getattr(speed, other)() == base.creep_mps(), f"{job} moved {other}"

    def test_the_contact_reverse_distance_uses_the_reverse_speed(self):
        """The predicted distance and the commanded speed must not disagree.

        CONTACT_REVERSE_TICKS is turned into metres with this speed, so a
        manoeuvre reading one value and reporting the other would travel a
        different distance from the one it claims.
        """
        speed = NavigationTuning().speed.model_copy(update={"CONTACT_REVERSE_MPS": 0.20})
        assert speed.contact_reverse_mps() == pytest.approx(0.20)
        assert speed.contact_mps() == speed.creep_mps()

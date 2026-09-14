"""Navigation tuning parameters for WRO 2026 track following.

This package centralizes all robot navigation tuning constants to enable:
- Runtime configuration without code edits
- Multiple tuning profiles for A/B testing
- Consistent parameter sharing between robot and simulation
- Easy parameter experimentation during competition

All parameters have been extracted from robot/src/navigation/ modules
and consolidated into a single source of truth. Split across this package's
modules by theme (motion control, escape maneuvers, waypoints, sensors,
blind navigation, sign handling, parking, simulation) so no single file grows
past a few hundred lines; every tuning-group class is re-exported here so
existing imports (``from shared.config.navigation_tuning import X``) are
unaffected by the split.

Example usage:
    from shared.config.navigation_tuning import NavigationTuning

    # Use defaults
    tuning = NavigationTuning()
    print(tuning.pursuit.lookahead_short)  # 0.16

    # Load custom profile from YAML
    tuning = NavigationTuning.load_from_yaml("tuning_profiles/aggressive.yaml")
"""

from __future__ import annotations

import functools
import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar, cast

from pydantic import BaseModel

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is an optional extra
    yaml = None

from shared.config._merge import deep_merge
from shared.config.hardware_profile import profile_dirs
from shared.config.navigation_tuning.blind_nav import (
    CorridorEstimatorParams,
    CorridorFollowerParams,
    DirectionEstimatorParams,
    LocalizationParams,
    StateEstimatorParams,
)
from shared.config.navigation_tuning.escape import EscapeManeuverParams
from shared.config.navigation_tuning.motion import (
    ClearanceZones,
    ControlLoopParams,
    HeadingErrorZones,
    PurePursuitParams,
    SpeedControlParams,
)
from shared.config.navigation_tuning.parking_params import ParkingParams
from shared.config.navigation_tuning.sensors import (
    LidarSectorParams,
    SensorHealthParams,
    StartMeasurementParams,
    WallHeadingParams,
)
from shared.config.navigation_tuning.signs import SignDiscoveryParams, SignRouterParams
from shared.config.navigation_tuning.simulation_params import SimulationParams
from shared.config.navigation_tuning.waypoint import WaypointParams
from shared.config.paths import SHARED_CONFIG_ROOT, load_toml_merged
from shared.domain.enums import ScenarioType

__all__ = [
    "DEFAULT_CONFIG_DIR",
    "ClearanceZones",
    "ControlLoopParams",
    "CorridorEstimatorParams",
    "CorridorFollowerParams",
    "DirectionEstimatorParams",
    "EscapeManeuverParams",
    "HeadingErrorZones",
    "LidarSectorParams",
    "LocalizationParams",
    "NavigationTuning",
    "ParkingParams",
    "PurePursuitParams",
    "SensorHealthParams",
    "SignDiscoveryParams",
    "SignRouterParams",
    "SimulationParams",
    "SpeedControlParams",
    "StartMeasurementParams",
    "StateEstimatorParams",
    "WallHeadingParams",
    "WaypointParams",
    "shipped_group",
]

DEFAULT_CONFIG_DIR: Path = SHARED_CONFIG_ROOT / "navigation"
"""src/config/navigation -- the checked-in per-group TOML tree.

Resolved via shared.config.paths (anchored from hardware_profile's fixed depth)
rather than a fragile ``parents[N]`` relative to this file, so a module
relocation can't change the resolved root. This package is the one that actually
knows where its own config lives -- callers (e.g. CoreNavigator) shouldn't have
to know or assume the two are siblings under the same platform/ root."""

CHALLENGES_ROOT: Path = SHARED_CONFIG_ROOT / "navigation-challenges"
"""src/config/navigation-challenges -- per-challenge overlay tree.

One ``<challenge>/<subfolder>/<group>.toml`` directory per :class:`ScenarioType`
value (``open``, ``obstacles``), same per-group layout as ``DEFAULT_CONFIG_DIR``
and the hardware-profile tree (:mod:`shared.config.hardware_profile`). Kept as
a sibling of ``navigation/`` rather than nested inside it, mirroring how
hardware profiles stay external to the checked-in base tree -- an overlay
should never be ambiguous with the default it overlays. A challenge overlay
only needs a file for the specific keys it retunes; everything else falls
back through ``DEFAULT_CONFIG_DIR``. See :meth:`NavigationTuning.load_default`."""


# (group key, dataclass, TOML subfolder) triples - the single source of truth
# for which sections load_from_yaml/load_from_json/to_dict/load_from_toml_dir
# and _shipped_group handle, so adding a new tuning group never requires
# touching more than this tuple. The subfolder mirrors this package's own
# module grouping (motion.py, blind_nav.py, etc.) under src/config/navigation/,
# so a TOML file's location and its Python group's home module always agree.
_GROUP_SPECS: tuple[tuple[str, type, str], ...] = (
    ("clearance", ClearanceZones, "motion"),
    ("heading", HeadingErrorZones, "motion"),
    ("pursuit", PurePursuitParams, "motion"),
    ("speed", SpeedControlParams, "motion"),
    ("escape", EscapeManeuverParams, "escape"),
    ("sensor", SensorHealthParams, "sensors"),
    ("waypoints", WaypointParams, "waypoint"),
    ("lidar_sectors", LidarSectorParams, "sensors"),
    ("start_measurement", StartMeasurementParams, "sensors"),
    ("corridor_estimator", CorridorEstimatorParams, "blind_nav"),
    ("corridor_follower", CorridorFollowerParams, "blind_nav"),
    ("direction_estimator", DirectionEstimatorParams, "blind_nav"),
    ("wall_heading", WallHeadingParams, "sensors"),
    ("control", ControlLoopParams, "motion"),
    ("sign_router", SignRouterParams, "signs"),
    ("sign_discovery", SignDiscoveryParams, "signs"),
    ("parking", ParkingParams, "parking"),
    ("localization", LocalizationParams, "blind_nav"),
    ("state_estimator", StateEstimatorParams, "blind_nav"),
    ("simulation", SimulationParams, "simulation"),
)


@functools.cache
def _shipped_group_cached(key: str) -> object:
    """Parse and validate one group's checked-in base TOML file, memoized.

    The group models are frozen, so the cached instance is safe to share.
    """
    for group_key, group_cls, subfolder in _GROUP_SPECS:
        if group_key == key:
            path = DEFAULT_CONFIG_DIR / subfolder / f"{key}.toml"
            return group_cls.model_validate(load_toml_merged(path))
    raise KeyError(key)


def _shipped_group(key: str) -> object:
    """Load one tuning group from its checked-in base TOML file.

    The TOML is the single source of values: a bare ``NavigationTuning()`` (no
    hardware profile, no challenge overlay) reads the same base files
    ``load_default`` does, so there is no parallel copy of the shipped values in
    code to drift. A missing or partial file raises rather than falling back to
    a hardcoded value, so an incomplete tree is loud.
    """
    return _shipped_group_cached(key)


def shipped_group[ModelT: BaseModel](model_cls: type[ModelT]) -> ModelT:
    """Load one tuning group as a standalone value object, from the base TOML.

    For callers that need a single group without building the whole
    :class:`NavigationTuning`. The value comes from the checked-in TOML, never
    from a hardcoded fallback, so a type it does not recognise is a loud error.
    """
    for key, group_cls, _subfolder in _GROUP_SPECS:
        if group_cls is model_cls:
            return cast("ModelT", _shipped_group_cached(key))
    msg = f"{model_cls.__name__} is not a navigation tuning group"
    raise TypeError(msg)


@dataclass(frozen=True, slots=True)
class NavigationTuning:
    """Complete navigation tuning configuration.

    Aggregates all tuning parameters into a single frozen dataclass
    for immutability and type safety.

    Can be instantiated with defaults or loaded from YAML files
    to support multiple tuning profiles.

    Example:
        # Use hardcoded defaults
        tuning = NavigationTuning()

        # Load from YAML for custom tuning
        tuning = NavigationTuning.load_from_yaml("aggressive.yaml")

        # Access parameters
        print(tuning.pursuit.lookahead_short)
        print(tuning.clearance.slow_dist)
    """

    clearance: ClearanceZones = field(default_factory=lambda: _shipped_group("clearance"))
    heading: HeadingErrorZones = field(default_factory=lambda: _shipped_group("heading"))
    pursuit: PurePursuitParams = field(default_factory=lambda: _shipped_group("pursuit"))
    speed: SpeedControlParams = field(default_factory=lambda: _shipped_group("speed"))
    escape: EscapeManeuverParams = field(default_factory=lambda: _shipped_group("escape"))
    sensor: SensorHealthParams = field(default_factory=lambda: _shipped_group("sensor"))
    waypoints: WaypointParams = field(default_factory=lambda: _shipped_group("waypoints"))
    lidar_sectors: LidarSectorParams = field(default_factory=lambda: _shipped_group("lidar_sectors"))
    start_measurement: StartMeasurementParams = field(
        default_factory=lambda: _shipped_group("start_measurement")
    )
    corridor_estimator: CorridorEstimatorParams = field(
        default_factory=lambda: _shipped_group("corridor_estimator")
    )
    corridor_follower: CorridorFollowerParams = field(
        default_factory=lambda: _shipped_group("corridor_follower")
    )
    direction_estimator: DirectionEstimatorParams = field(
        default_factory=lambda: _shipped_group("direction_estimator")
    )
    wall_heading: WallHeadingParams = field(default_factory=lambda: _shipped_group("wall_heading"))
    control: ControlLoopParams = field(default_factory=lambda: _shipped_group("control"))
    sign_router: SignRouterParams = field(default_factory=lambda: _shipped_group("sign_router"))
    sign_discovery: SignDiscoveryParams = field(default_factory=lambda: _shipped_group("sign_discovery"))
    parking: ParkingParams = field(default_factory=lambda: _shipped_group("parking"))
    localization: LocalizationParams = field(default_factory=lambda: _shipped_group("localization"))
    state_estimator: StateEstimatorParams = field(default_factory=lambda: _shipped_group("state_estimator"))
    simulation: SimulationParams = field(default_factory=lambda: _shipped_group("simulation"))

    def __post_init__(self) -> None:
        """Check invariants that span two tuning groups.

        This is a frozen dataclass aggregating pydantic groups, so per-group
        rules live on the groups as validators and only cross-group ones belong
        here. A ``@model_validator`` would be inert on a dataclass -- it is
        silently ignored, which is worse than no check at all.

        The turn must begin strictly after the direction window opens.

        Turning swings the heading past the direction estimator's alignment
        gate, which then refuses every reading. If the robot began its turn at
        the same clearance that makes the left/right comparison decisive, it
        would rotate straight through the only window in which it can read
        which side is open, and come out the far side with a wall on both
        sides and nothing learned. Measured with both at 0.75 m: three fixtures
        never settled at all and two settled wrong after 20-plus seconds.

        The two values live in different groups and so in different TOML files,
        which is exactly why this belongs here -- editing one file cannot see
        the other.
        """
        turn = self.corridor_follower.turn_clearance_m
        corner = self.direction_estimator.corner_clearance_m
        if turn >= corner:
            msg = (
                f"corridor_follower.turn_clearance_m ({turn}) must be strictly below "
                f"direction_estimator.corner_clearance_m ({corner}); the gap is the window "
                "in which the robot is still square to the corridor and can read which side is open"
            )
            raise ValueError(msg)

        # Same reasoning, narrow-corridor variant: narrow_turn_clearance_m only
        # helps if it actually moves the turn-commit point earlier than
        # turn_clearance_m would. Equal or later reproduces the zero-window bug
        # this field exists to fix -- see
        # open_challenge_narrow_corridor_root_cause_2026_08_15.
        narrow_turn = self.corridor_follower.narrow_turn_clearance_m
        if narrow_turn >= turn:
            msg = (
                f"corridor_follower.narrow_turn_clearance_m ({narrow_turn}) must be strictly below "
                f"corridor_follower.turn_clearance_m ({turn}); otherwise a narrow corridor gets no more "
                "of a direction-settling window than a wide one does"
            )
            raise ValueError(msg)

    # Module-level _GROUP_SPECS, exposed as a ClassVar so tests and callers can
    # read the (group key, dataclass, TOML subfolder) mapping from the class.
    _GROUPS: ClassVar[tuple[tuple[str, type, str], ...]] = _GROUP_SPECS


    # Per-challenge tuning lives in CHALLENGES_ROOT (load_default(challenge=...)),
    # not as a hardcoded for_obstacles()-style classmethod. An earlier
    # for_obstacles() profile (lookahead 0.12/0.24 + FAST_SPEED 0.30) was
    # removed after re-measurement against the corrected four-wheel-steer
    # kinematics (8eb3c38) and the closed drive loop (668e40a) showed both
    # halves were inert -- the speed cap cannot do anything since
    # AckermannKinematics clamps to the measured 0.156 m/s drivetrain ceiling
    # regardless of the configured cap, and the lookahead change genuinely
    # improved path-tracking accuracy (p90 cross-track 12.9->5.1 cm) without
    # moving the sign-collision rate at all (16/16 at every value 0.10-0.40).
    # See ``src/docs/sign-avoidance-investigation.md``. That
    # history still applies to whatever gets written under CHALLENGES_ROOT:
    # populate an overlay file only with a measurement that survives the
    # current model, not a guess.

    @classmethod
    def _from_mapping(cls, data: dict[str, object]) -> NavigationTuning:
        """Reconstruct nested tuning dataclasses from a parsed mapping.

        Shared by :meth:`load_from_yaml` and :meth:`load_from_json` so both
        formats stay in lockstep with ``_GROUPS`` instead of duplicating the
        per-group reconstruction. Every group must be present in the mapping;
        a missing one raises rather than falling back to a hardcoded value.
        """
        return cls(**{key: dataclass_type(**data.get(key, {})) for key, dataclass_type, _ in cls._GROUPS})

    @classmethod
    def load_from_yaml(cls, path: Path | str) -> NavigationTuning:
        """Load tuning configuration from YAML file.

        Enables runtime configuration without code recompilation.
        Supports multiple tuning profiles for experimentation.

        Requires the ``yaml`` extra: ``uv add "vtitan-shared[yaml]"``.

        Args:
            path: Path to YAML file with tuning parameters

        Returns:
            NavigationTuning instance with loaded parameters

        Raises:
            FileNotFoundError: If YAML file not found
            yaml.YAMLError: If YAML parsing fails
            TypeError: If YAML structure invalid

        Example YAML structure:
            clearance:
              contact_dist: 0.05
              slow_dist: 0.20
              medium_dist: 0.45
              fast_dist: 0.90
            pursuit:
              lookahead_short: 0.15
              steer_kp: 2.0
            # ... etc
        """
        if yaml is None:
            msg = "PyYAML required for loading YAML tuning files"
            raise ImportError(msg)

        path = Path(path)
        if not path.exists():
            msg = f"Tuning file not found: {path}"
            raise FileNotFoundError(msg)

        with path.open(encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            msg = "YAML must contain a mapping (dict)"
            raise TypeError(msg)

        return cls._from_mapping(data)

    @classmethod
    def load_from_toml_dir(cls, directory: Path | str) -> NavigationTuning:
        """Load tuning configuration from a directory of per-group TOML files.

        One ``<subfolder>/<group>.toml`` per ``_GROUPS`` entry (e.g.
        ``motion/clearance.toml``, ``motion/pursuit.toml``) -- that file's own
        top-level fields ARE the group, no wrapper table needed since the
        filename already disambiguates which group it is. The subfolder is
        ``_GROUPS``'s own third element, so it always matches this package's
        module grouping (see the comment on ``_GROUPS``). A missing file is an
        error: the checked-in tree is complete, so an absent group means a
        partial or wrong directory rather than a value to invent. A missing
        directory is likewise an error, not a silent all-defaults result.

        Args:
            directory: Directory containing the per-group TOML tree.

        Returns:
            NavigationTuning instance with loaded parameters.
        """
        return cls.load_from_toml_dirs([directory])

    @classmethod
    def load_from_toml_dirs(cls, directories: Sequence[Path | str]) -> NavigationTuning:
        """Load and merge per-group TOML from multiple directories, in order.

        Each later directory's files are deep-merged onto the accumulated
        result of the earlier ones -- see :meth:`load_from_toml_dir` for the
        per-file layout each directory shares. This is how
        :meth:`load_default` layers a hardware profile's overlay
        (:mod:`shared.config.hardware_profile`) on top of the checked-in
        ``DEFAULT_CONFIG_DIR`` tree: a profile only needs a
        ``<subfolder>/<group>.toml`` for the specific keys it retunes.

        Args:
            directories: Directories to merge, in increasing priority (a
                later directory's values win on any key both set).

        Returns:
            NavigationTuning instance with loaded parameters.
        """
        data: dict[str, object] = {}
        for directory in directories:
            directory = Path(directory)
            if not directory.is_dir():
                continue
            for key, _, subfolder in cls._GROUPS:
                toml_path = directory / subfolder / f"{key}.toml"
                if toml_path.exists():
                    group_data = load_toml_merged(toml_path)
                    data[key] = deep_merge(data[key], group_data) if key in data else group_data

        return cls._from_mapping(data)

    @classmethod
    def load_default(cls, challenge: ScenarioType | None = None) -> NavigationTuning:
        """Load the checked-in DEFAULT_CONFIG_DIR TOML tree, with any active hardware profile
        and (if given) a challenge-scoped overlay from CHALLENGES_ROOT layered on top.

        The normal way to construct a NavigationTuning in production code. The
        checked-in DEFAULT_CONFIG_DIR tree is complete, so every group loads;
        a missing file or directory raises rather than falling back to a
        hardcoded value, which keeps the shrunk TOML tree the only source of
        truth and makes an incomplete checkout loud.

        ``challenge`` is optional and additive: omitting it (the default)
        reproduces the pre-existing behaviour exactly. When given, the
        challenge overlay merges last -- after the hardware profile -- since
        challenge-scoped navigation tuning is more specific to the immediate
        run than a hardware profile's own navigation defaults.

        ``profile_dirs()`` is re-resolved (from ``VTITAN_HARDWARE_PROFILE``)
        on every call -- deliberately not itself cached here -- because it's
        cheap (an env var read plus one ``is_dir()`` per active profile
        name, normally 1-2) and because several tests exercise different
        profile values within one process via ``monkeypatch.setenv``
        (see ``tests/unit/test_hardware_profile.py``); caching this method
        by ``challenge`` alone would silently return a stale profile's
        tuning to a later test. The actual expensive work is delegated to
        ``_load_from_toml_dirs_cached``, keyed on the exact resolved
        directory tuple, so it only recomputes when the effective
        directories actually change.
        """
        dirs: list[Path] = [DEFAULT_CONFIG_DIR, *profile_dirs()]
        if challenge is not None:
            dirs.append(CHALLENGES_ROOT / challenge.value)
        return cls._load_from_toml_dirs_cached(tuple(dirs))

    @classmethod
    @functools.lru_cache(maxsize=32)
    def _load_from_toml_dirs_cached(cls, directories: tuple[Path, ...]) -> NavigationTuning:
        """Memoized core of :meth:`load_default`, keyed on the resolved directory tuple.

        ``paths._read_toml`` already memoized each individual TOML
        file's parse for this reason (see its docstring: "the single
        largest cost" in an earlier profiling pass), but that fix was
        incomplete -- a fresh profile of a 437-step Obstacles scenario
        (2026-08-29) found ``load_from_toml_dirs``'s surrounding directory
        walk (``Path.is_dir()``/``Path.exists()`` per group, per directory,
        on every one of 649 calls) still cost ~7.5s of 12.4s total runtime,
        almost entirely filesystem stat calls rather than TOML parsing.
        Confirmed after this fix: 3x wall-clock speedup on the same
        scenario, byte-identical simulation result.

        Safe to cache the returned value directly, not just a deep copy of
        it: ``NavigationTuning`` is ``@dataclass(frozen=True, slots=True)``
        aggregating exclusively ``ConfigDict(frozen=True, ...)`` pydantic
        groups -- fully immutable end to end, so no caller can mutate a
        cached instance and corrupt another caller's config the way a
        cached mutable dict could. ``maxsize=32`` covers every
        (hardware-profile-set, challenge) combination any real process or
        test session actually exercises with headroom to spare.
        """
        return cls.load_from_toml_dirs(directories)

    @classmethod
    def load_from_json(cls, path: Path | str) -> NavigationTuning:
        """Load tuning configuration from JSON file.

        Similar to load_from_yaml but uses JSON format instead.

        Args:
            path: Path to JSON file with tuning parameters

        Returns:
            NavigationTuning instance with loaded parameters

        Raises:
            FileNotFoundError: If JSON file not found
            json.JSONDecodeError: If JSON parsing fails
            TypeError: If JSON structure invalid
        """
        path = Path(path)
        if not path.exists():
            msg = f"Tuning file not found: {path}"
            raise FileNotFoundError(msg)

        with path.open(encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, dict):
            msg = "JSON must contain a mapping (dict)"
            raise TypeError(msg)

        return cls._from_mapping(data)

    def to_dict(self) -> dict[str, object]:
        """Export configuration as nested dictionary.

        Useful for serialization or debugging.

        Dumped in pydantic's ``json`` mode so enum-valued fields (e.g.
        ``waypoints.WIDE_CENTER_BIAS_SIDE``) come out as their plain string value
        rather than as the enum member. Python mode emits the member, which
        ``yaml.dump`` then writes as a ``python/object/apply:`` tag that
        ``load_from_yaml``'s ``safe_load`` refuses -- so an exported profile
        could not be read back, and ``--tuning`` is the robot's only profile
        mechanism. The string round-trips because each group revalidates its
        own fields on load.

        Returns:
            Dictionary representation of all parameters
        """
        return {key: getattr(self, key).model_dump(mode="json") for key, _, _sub in self._GROUPS}

    def to_json(self) -> str:
        """Export configuration as JSON string.

        Returns:
            JSON string representation of all parameters
        """
        return json.dumps(self.to_dict(), indent=2)

    def to_yaml(self) -> str:
        """Export configuration as YAML string.

        Requires the ``yaml`` extra: ``uv add "vtitan-shared[yaml]"``.

        Returns:
            YAML string representation of all parameters

        Raises:
            ImportError: If PyYAML not available
        """
        if yaml is None:
            msg = "PyYAML required for YAML export"
            raise ImportError(msg)

        return yaml.dump(self.to_dict(), default_flow_style=False)

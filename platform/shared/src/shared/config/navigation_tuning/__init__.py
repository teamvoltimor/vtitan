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
    print(tuning.pursuit.LOOKAHEAD_SHORT)  # 0.20

    # Load custom profile from YAML
    tuning = NavigationTuning.load_from_yaml("tuning_profiles/aggressive.yaml")
"""

from __future__ import annotations

import json
import tomllib
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is an optional extra
    yaml = None

from shared.config._merge import deep_merge
from shared.config.hardware_profile import profile_dirs
from shared.domain.enums import ScenarioType
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
]

DEFAULT_CONFIG_DIR: Path = Path(__file__).resolve().parents[4] / "config" / "navigation"
"""platform/shared/config/navigation -- the checked-in per-group TOML tree.

Resolved relative to this module's own location (platform/shared/src/shared/
config/navigation_tuning/__init__.py) rather than the caller's, since this
package is the one that actually knows where its own config lives -- callers
(e.g. CoreNavigator) shouldn't have to know or assume the two are siblings
under the same platform/ root."""

CHALLENGES_ROOT: Path = Path(__file__).resolve().parents[4] / "config" / "navigation-challenges"
"""platform/shared/config/navigation-challenges -- per-challenge overlay tree.

One ``<challenge>/<subfolder>/<group>.toml`` directory per :class:`ScenarioType`
value (``open``, ``obstacles``), same per-group layout as ``DEFAULT_CONFIG_DIR``
and the hardware-profile tree (:mod:`shared.config.hardware_profile`). Kept as
a sibling of ``navigation/`` rather than nested inside it, mirroring how
hardware profiles stay external to the checked-in base tree -- an overlay
should never be ambiguous with the default it overlays. A challenge overlay
only needs a file for the specific keys it retunes; everything else falls
back through ``DEFAULT_CONFIG_DIR``. See :meth:`NavigationTuning.load_default`."""


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
        print(tuning.pursuit.LOOKAHEAD_SHORT)
        print(tuning.clearance.SLOW_DIST)
    """

    clearance: ClearanceZones = field(default_factory=ClearanceZones)
    heading: HeadingErrorZones = field(default_factory=HeadingErrorZones)
    pursuit: PurePursuitParams = field(default_factory=PurePursuitParams)
    speed: SpeedControlParams = field(default_factory=SpeedControlParams)
    escape: EscapeManeuverParams = field(default_factory=EscapeManeuverParams)
    sensor: SensorHealthParams = field(default_factory=SensorHealthParams)
    waypoints: WaypointParams = field(default_factory=WaypointParams)
    lidar_sectors: LidarSectorParams = field(default_factory=LidarSectorParams)
    start_measurement: StartMeasurementParams = field(default_factory=StartMeasurementParams)
    corridor_estimator: CorridorEstimatorParams = field(default_factory=CorridorEstimatorParams)
    corridor_follower: CorridorFollowerParams = field(default_factory=CorridorFollowerParams)
    direction_estimator: DirectionEstimatorParams = field(default_factory=DirectionEstimatorParams)
    wall_heading: WallHeadingParams = field(default_factory=WallHeadingParams)
    control: ControlLoopParams = field(default_factory=ControlLoopParams)
    sign_router: SignRouterParams = field(default_factory=SignRouterParams)
    sign_discovery: SignDiscoveryParams = field(default_factory=SignDiscoveryParams)
    parking: ParkingParams = field(default_factory=ParkingParams)
    localization: LocalizationParams = field(default_factory=LocalizationParams)
    state_estimator: StateEstimatorParams = field(default_factory=StateEstimatorParams)
    simulation: SimulationParams = field(default_factory=SimulationParams)

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
        turn = self.corridor_follower.TURN_CLEARANCE_M
        corner = self.direction_estimator.CORNER_CLEARANCE_M
        if turn >= corner:
            msg = (
                f"corridor_follower.TURN_CLEARANCE_M ({turn}) must be strictly below "
                f"direction_estimator.CORNER_CLEARANCE_M ({corner}); the gap is the window "
                "in which the robot is still square to the corridor and can read which side is open"
            )
            raise ValueError(msg)

    # (group key, dataclass, TOML subfolder) triples — the single source of
    # truth for which sections load_from_yaml/load_from_json/to_dict/
    # load_from_toml_dir/load_from_toml_dirs handle, so adding a new tuning group never requires
    # touching more than this tuple. The subfolder mirrors this package's own
    # module grouping (motion.py, blind_nav.py, etc.) under
    # platform/shared/config/navigation/, so a TOML file's location and its
    # Python group's home module always agree.
    _GROUPS: ClassVar[tuple[tuple[str, type, str], ...]] = (
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
    # See ``platform/robot/docs/sign-avoidance-investigation.md``. That
    # history still applies to whatever gets written under CHALLENGES_ROOT:
    # populate an overlay file only with a measurement that survives the
    # current model, not a guess.

    @classmethod
    def _from_mapping(cls, data: dict[str, object]) -> NavigationTuning:
        """Reconstruct nested tuning dataclasses from a parsed mapping.

        Shared by :meth:`load_from_yaml` and :meth:`load_from_json` so both
        formats stay in lockstep with ``_GROUPS`` instead of duplicating the
        per-group reconstruction. Missing groups fall back to their defaults,
        allowing partial config files.
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
              CONTACT_DIST: 0.05
              SLOW_DIST: 0.20
              MEDIUM_DIST: 0.45
              FAST_DIST: 0.90
            pursuit:
              LOOKAHEAD_SHORT: 0.15
              STEER_KP: 2.0
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
        module grouping (see the comment on ``_GROUPS``). A missing file
        falls back to that group's defaults, same as a missing key in
        ``load_from_yaml``/``load_from_json``'s single-file mapping. A
        missing directory returns all-defaults outright, so constructing a
        navigator in a test/sim context with no config tree on disk still
        works.

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
                    with toml_path.open("rb") as f:
                        group_data = tomllib.load(f)
                    data[key] = deep_merge(data[key], group_data) if key in data else group_data

        return cls._from_mapping(data)

    @classmethod
    def load_default(cls, challenge: ScenarioType | None = None) -> NavigationTuning:
        """Load the checked-in DEFAULT_CONFIG_DIR TOML tree, with any active hardware profile
        and (if given) a challenge-scoped overlay from CHALLENGES_ROOT layered on top.

        The normal way to construct a NavigationTuning in production code --
        falls back to hardcoded per-group defaults for any file (or the
        whole directory) that isn't present, so it's also safe to call from
        a test/sim context that doesn't have the full repo checked out.

        ``challenge`` is optional and additive: omitting it (the default)
        reproduces the pre-existing behaviour exactly. When given, the
        challenge overlay merges last -- after the hardware profile -- since
        challenge-scoped navigation tuning is more specific to the immediate
        run than a hardware profile's own navigation defaults.
        """
        dirs = [DEFAULT_CONFIG_DIR, *profile_dirs()]
        if challenge is not None:
            dirs.append(CHALLENGES_ROOT / challenge.value)
        return cls.load_from_toml_dirs(dirs)

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
        ``waypoints.CENTER_BIAS_SIDE``) come out as their plain string value
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

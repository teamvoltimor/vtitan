"""Mat geometry constants for the WRO 2026 Future Engineers track.

Loads ``platform/shared/config/track.toml`` directly at runtime -- the single
source of truth also consumed by the Go ``simconfig`` package (regenerated via
``task gen:track-constants``). Python used to read a checked-in generated
module (``track_constants_gen.py``) instead, which duplicated the TOML into a
second, driftable Python file; this reads the TOML itself, the same way
:class:`~shared.config.navigation_tuning.NavigationTuning` reads its own TOML
tree.
"""

from __future__ import annotations

import tomllib
from decimal import Decimal
from pathlib import Path

from pydantic import BaseModel, ConfigDict, model_validator

DEFAULT_CONFIG_PATH: Path = Path(__file__).resolve().parents[3] / "config" / "track.toml"
"""platform/shared/config/track.toml -- resolved relative to this module's own
location rather than the caller's, same rationale as NavigationTuning's
DEFAULT_CONFIG_DIR."""


def _dec(x: float) -> Decimal:
    """Convert a TOML-parsed float to an exact Decimal via its repr.

    ``Decimal(0.6) - Decimal(0.4)`` still goes through float64 first and
    reproduces the same ``0.19999999999999996`` error this exists to avoid;
    routing through ``str()`` reparses the shortest decimal string that
    round-trips to that float, which for every value this TOML actually holds
    is the value as written. Mirrors the Go generator's own use of
    ``decimal.Decimal`` for this arithmetic -- see track_constants.gen.go's
    package doc for why float64 alone is not closed over the TOML's values.
    """
    return Decimal(str(x))


class Track(BaseModel):
    """Mat and driveable-track extents and the corner region."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mat_size: float
    size: float
    min_coord: float
    max_coord: float
    corner_min: float
    corner_max: float

    @property
    def center_coord(self) -> float:
        """Middle of the track on both axes."""
        return float((_dec(self.min_coord) + _dec(self.max_coord)) / 2)

    @property
    def corner_size(self) -> float:
        """Side length of one corner region."""
        return float(_dec(self.corner_max) - _dec(self.corner_min))


class Wall(BaseModel):
    """Exterior and interior wall dimensions."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    height: float
    thickness: float
    collision_thickness: float
    exterior_offset: float
    interior_offset: float
    color: tuple[float, float, float]


class Corridor(BaseModel):
    """Legal corridor widths and the division lines that cut every corridor lengthwise."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    narrow: float
    wide: float
    obstacles: float
    min_width: float
    max_width: float
    division_lines: tuple[float, ...]

    @model_validator(mode="after")
    def _check_division_lines_increase(self) -> Corridor:
        """Reject division lines that don't strictly increase within the corridor.

        Mirrors the Go generator's own ``Config.Validate`` -- Python now reads
        track.toml directly instead of a checked-in generated file, so a bad
        edit here used to be caught by Go codegen failing before the stale
        (still-valid) generated file was ever overwritten. Reading the TOML
        live bypasses that gate, so the same check has to live here too.
        """
        prev = 0.0
        for i, line in enumerate(self.division_lines):
            if line <= prev:
                msg = f"corridor.division_lines[{i}] = {line} does not increase past {prev}"
                raise ValueError(msg)
            prev = line
        if prev >= self.wide:
            msg = f"corridor.division_lines end at {prev}, outside the {self.wide} m wide corridor"
            raise ValueError(msg)
        return self

    @property
    def band_widths(self) -> tuple[float, ...]:
        """Division lines converted into the widths of the bands they delimit.

        Measured out from the outer wall across a full-width corridor: lines
        at 0.40 and 0.60 in a 1.0 m corridor give (0.40, 0.20, 0.40).
        """
        bands = []
        prev = Decimal(0)
        for line in self.division_lines:
            bands.append(_dec(line) - prev)
            prev = _dec(line)
        bands.append(_dec(self.wide) - prev)
        return tuple(float(b) for b in bands)

    @property
    def division_width(self) -> float:
        """Width of the middle band -- the two division lines' own gap."""
        return self.band_widths[1]


class Sign(BaseModel):
    """Traffic pillar dimensions, grid rows and colours."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    width: float
    depth: float
    height: float
    z_position: float
    grid_depth_near: float
    grid_depth_middle: float
    grid_depth_far: float
    placement_circle_diameter: float
    min_count: int
    max_count: int
    red_color: tuple[float, float, float]
    green_color: tuple[float, float, float]
    red_std: tuple[float, float, float]
    green_std: tuple[float, float, float]


class Parking(BaseModel):
    """Magenta parking block dimensions and bay sizing."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    length: float
    width: float
    height: float
    z_position: float
    wall_offset: float
    spacing_factor: float
    color: tuple[float, float, float]


class StartingZone(BaseModel):
    """Starting square's cell size, appearance and per-band spawn alignment."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    default_length: float
    thickness: float
    obstacles_size_factor: float
    indicator_radius: float
    color: tuple[float, float, float]
    clockwise_color: tuple[float, float, float]
    counterclockwise_color: tuple[float, float, float]
    spawn_alignment: tuple[str, ...]


class TrackConstants(BaseModel):
    """Mat geometry constants, loaded from track.toml."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    track: Track
    wall: Wall
    corridor: Corridor
    sign: Sign
    parking: Parking
    starting_zone: StartingZone

    @property
    def cell_centers_along(self) -> tuple[float, float]:
        """Along-corridor midpoints of the two starting cells in each band.

        The starting square occupies the middle metre of a side, so the cells
        sit half a cell either side of the track centre.
        """
        half = _dec(self.starting_zone.default_length) / 2
        center = _dec(self.track.center_coord)
        return float(center - half), float(center + half)

    def spawn_offsets(self, chassis_width: float) -> tuple[float, ...]:
        """Where the robot is placed inside each band, measured out from the outer wall.

        Derived by pushing the chassis flush against the band edge named in
        ``starting_zone.spawn_alignment``, never centred: one band edge is a
        painted line the robot can sit against harmlessly, the other may be
        the inner block, so splitting the slack evenly would spend half the
        margin on the line. Deriving rather than declaring is what keeps the
        placement correct across a chassis re-measurement.

        Args:
            chassis_width: Robot chassis width (m) -- ``RobotConstants``'s
                ``chassis.width``, passed in rather than read here so this
                model has no dependency on ``RobotConstants``.

        Returns:
            One offset per band, in the same order as ``spawn_alignment``.
        """
        bands = self.corridor.band_widths
        alignment = self.starting_zone.spawn_alignment
        if len(alignment) != len(bands):
            msg = (
                f"starting_zone.spawn_alignment has {len(alignment)} entries, "
                f"want {len(bands)} (one per band)"
            )
            raise ValueError(msg)

        half = _dec(chassis_width) / 2
        offsets = []
        edge = Decimal(0)
        for i, band in enumerate(bands):
            band_dec = _dec(band)
            lo, hi = edge, edge + band_dec
            if band < chassis_width:
                msg = (
                    f"band {i} is {band} m wide, narrower than the {chassis_width} m "
                    "chassis, so no start fits in it"
                )
                raise ValueError(msg)
            match alignment[i]:
                case "outer":
                    offsets.append(lo + half)
                case "inner":
                    offsets.append(hi - half)
                case other:
                    msg = f'starting_zone.spawn_alignment[{i}] = "{other}", want "outer" or "inner"'
                    raise ValueError(msg)
            edge = hi
        return tuple(float(o) for o in offsets)

    @classmethod
    def load_default(cls) -> TrackConstants:
        """Load from the checked-in ``platform/shared/config/track.toml``."""
        with DEFAULT_CONFIG_PATH.open("rb") as f:
            data: dict[str, object] = tomllib.load(f)
        return cls.model_validate(data)

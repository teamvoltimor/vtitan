"""Hand-written wrapper over the generated ``TrackConfig`` DTO.

The DTO (``shared.config.generated.track_config``) is generated from
``src/config/schemas/track.schema.json`` and holds only the TOML's fields and
their descriptions. Everything with behavior lives here: the exact-decimal
derivations (band widths, corner size, spawn offsets), the ``division_lines``
invariant, and the TOML loader.

Kept behind the same public names the codebase already imports, so the split
between generated DTO and hand-written wrapper is invisible to callers.
"""

from __future__ import annotations

from decimal import Decimal
from typing import TYPE_CHECKING, ClassVar

from pydantic import model_validator

from shared.config.generated.track_schema import (
    Corridor as _CorridorDTO,
    Markings,
    Parking,
    Sign,
    SpawnAlignment,
    StartingZone,
    Track as _TrackDTO,
    TrackConfig as _TrackConfigDTO,
    Wall,
)
from shared.config.paths import SHARED_CONFIG_ROOT, load_toml_model

if TYPE_CHECKING:
    from pathlib import Path

__all__ = [
    "Corridor",
    "Markings",
    "Parking",
    "Sign",
    "StartingZone",
    "Track",
    "TrackConstants",
    "Wall",
]

DEFAULT_CONFIG_PATH: Path = SHARED_CONFIG_ROOT / "track.toml"
"""src/config/track.toml -- resolved via shared.config.paths rather than a
fragile ``parents[N]`` relative to this file."""


def _dec(x: float) -> Decimal:
    """Convert a TOML-parsed float to an exact Decimal via its repr.

    ``Decimal(0.6) - Decimal(0.4)`` still goes through float64 first and
    reproduces the same ``0.19999999999999996`` error this exists to avoid;
    routing through ``str()`` reparses the shortest decimal string that
    round-trips to that float, which for every value the TOML holds is the
    value as written. Mirrors the Go generator's own decimal arithmetic.
    """
    return Decimal(str(x))


class Track(_TrackDTO):
    """Mat and driveable-track extents, plus the derived center and corner size."""

    @property
    def center_coord(self) -> float:
        """Middle of the track on both axes."""
        return float((_dec(self.min_coord) + _dec(self.max_coord)) / 2)

    @property
    def corner_size(self) -> float:
        """Side length of one corner region."""
        return float(_dec(self.corner_max) - _dec(self.corner_min))


class Corridor(_CorridorDTO):
    """Legal corridor widths, the division lines, and the bands they delimit."""

    @model_validator(mode="after")
    def _check_division_lines_increase(self) -> Corridor:
        """Reject division lines that don't strictly increase within the corridor.

        Reading the TOML live bypasses the Go generator's own ``Config.Validate``,
        so the same check has to live here too.
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

        Measured out from the outer wall across a full-width corridor: lines at
        0.40 and 0.60 in a 1.0 m corridor give (0.40, 0.20, 0.40). Exact-decimal
        so the middle band is 0.2, not 0.19999999999999996.
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


class TrackConstants(_TrackConfigDTO):
    """Mat geometry constants, loaded from track.toml.

    Subclasses the generated DTO purely to attach the derived values and the
    loader; the field declarations are all generated.
    """

    track: Track
    corridor: Corridor

    default_config_path: ClassVar[Path] = DEFAULT_CONFIG_PATH

    @classmethod
    def load_default(cls) -> TrackConstants:
        """Load and validate from track.toml."""
        return load_toml_model(cls, DEFAULT_CONFIG_PATH)

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
        painted line the robot can sit against harmlessly, the other may be the
        inner block, so splitting the slack evenly would spend half the margin
        on the line.

        Args:
            chassis_width: Robot chassis width (m), passed in rather than read
                here so this model has no dependency on RobotConstants.

        Returns:
            One offset per band, in the same order as ``spawn_alignment``.
        """
        bands = self.corridor.band_widths
        alignment = self.starting_zone.spawn_alignment
        if len(alignment) != len(bands):
            msg = f"starting_zone.spawn_alignment has {len(alignment)} entries, want {len(bands)} (one per band)"
            raise ValueError(msg)

        half = _dec(chassis_width) / 2
        offsets = []
        edge = Decimal(0)
        for i, band in enumerate(bands):
            band_dec = _dec(band)
            lo, hi = edge, edge + band_dec
            if band < chassis_width:
                msg = f"band {i} is {band} m wide, narrower than the {chassis_width} m chassis, so no start fits in it"
                raise ValueError(msg)
            match alignment[i]:
                case SpawnAlignment.outer:
                    offsets.append(lo + half)
                case SpawnAlignment.inner:
                    offsets.append(hi - half)
                case other:
                    msg = f'starting_zone.spawn_alignment[{i}] = "{other}", want "outer" or "inner"'
                    raise ValueError(msg)
            edge = hi
        return tuple(float(o) for o in offsets)

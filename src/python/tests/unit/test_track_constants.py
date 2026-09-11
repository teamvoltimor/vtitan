"""The mat geometry moved from hand-written constants to a generated module.

These tests pin the values against the literals that were in
``shared/config/constants.py`` before the move, so the extraction to
``track.toml`` is provably value-preserving rather than merely plausible. They
are not a restatement of the config: if a value here has to change, the change
is a deliberate re-measurement of the mat and should be argued for in the diff,
not absorbed silently by a generator.
"""

from __future__ import annotations

import pytest
from shared.config.constants import (
    CorridorDimensions,
    GridSections,
    ParkingLotSpecs,
    RobotSpecs,
    StartingZoneSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
)
from shared.config.starting_zone import STARTING_ZONE_LAYOUT, StartingZoneLayout


class TestValuesSurvivedTheMove:
    """Every generated constant equals the literal it replaced."""

    @pytest.mark.parametrize(
        ("actual", "expected"),
        [
            (TrackDimensions.MAT_SIZE, 3.2),
            (TrackDimensions.TRACK_SIZE, 3.0),
            (TrackDimensions.MIN_COORD, 0.0),
            (TrackDimensions.MAX_COORD, 3.0),
            (TrackDimensions.CENTER_COORD, 1.5),
            (TrackDimensions.CORNER_MIN, 1.0),
            (TrackDimensions.CORNER_MAX, 2.0),
            (TrackDimensions.CORNER_SIZE, 1.0),
            (WallSpecs.HEIGHT, 0.1),
            (WallSpecs.THICKNESS, 0.1),
            # 0.10, not the 0.18 this pinned before. The 40 mm per-side
            # collision margin was removed deliberately (see track.toml): it
            # made every corridor behave 8 cm narrower than spec and made
            # parking arithmetically impossible -- a 0.194 m chassis in a
            # 0.20 m bay needs its centre within 0.10 m of the wall while
            # collision stopped it at 0.14 m, which was the whole 0/16 park
            # rate. Collision now matches the visual wall.
            (WallSpecs.COLLISION_THICKNESS, 0.10),
            (WallSpecs.EXTERIOR_OFFSET, 0.05),
            (WallSpecs.INTERIOR_OFFSET, 0.05),
            (CorridorDimensions.NARROW, 0.6),
            (CorridorDimensions.WIDE, 1.0),
            (CorridorDimensions.OBSTACLES_WIDTH, 1.0),
            (CorridorDimensions.MIN_WIDTH, 0.5),
            (CorridorDimensions.MAX_WIDTH, 1.5),
            (CorridorDimensions.DIVISION_OUTER, 0.4),
            (CorridorDimensions.DIVISION_INNER, 0.6),
            (CorridorDimensions.DIVISION_WIDTH, 0.2),
            (TrafficSignSpecs.WIDTH, 0.05),
            (TrafficSignSpecs.DEPTH, 0.05),
            (TrafficSignSpecs.HEIGHT, 0.10),
            (TrafficSignSpecs.Z_POSITION, 0.05),
            (TrafficSignSpecs.GRID_DEPTH_NEAR, 1.0),
            (TrafficSignSpecs.GRID_DEPTH_MIDDLE, 1.5),
            (TrafficSignSpecs.GRID_DEPTH_FAR, 2.0),
            (TrafficSignSpecs.GRID_WIDTH_OUTER, 0.4),
            (TrafficSignSpecs.GRID_WIDTH_INNER, 0.6),
            (TrafficSignSpecs.MIN_SIGNS, 6),
            (TrafficSignSpecs.MAX_SIGNS, 14),
            (ParkingLotSpecs.LENGTH, 0.20),
            (ParkingLotSpecs.WIDTH, 0.02),
            (ParkingLotSpecs.HEIGHT, 0.10),
            (ParkingLotSpecs.Z_POSITION, 0.05),
            (ParkingLotSpecs.WALL_OFFSET, 0.1),
            (ParkingLotSpecs.BLOCK_SPACING_FACTOR, 1.5),
            (StartingZoneSpecs.DEFAULT_LENGTH, 0.5),
            (StartingZoneSpecs.WIDTH, 0.2),
            (StartingZoneSpecs.THICKNESS, 0.001),
            (StartingZoneSpecs.OBSTACLES_SIZE_FACTOR, 0.9),
            (StartingZoneSpecs.INDICATOR_RADIUS, 0.035),
            (GridSections.LENGTH_SECTION_LEFT, 1.25),
            (GridSections.LENGTH_SECTION_RIGHT, 1.75),
        ],
    )
    def test_scalar(self, actual: float, expected: float) -> None:
        # Exact equality, not approx: a generator that only round-trips values
        # approximately is the failure mode this whole extraction exists to
        # avoid (0.60 - 0.40 must be 0.2, not 0.19999999999999996).
        assert actual == expected

    @pytest.mark.parametrize(
        ("actual", "expected"),
        [
            (WallSpecs.COLOR, (0.0, 0.0, 0.0)),
            (TrafficSignSpecs.RED_COLOR, (0.933, 0.153, 0.216)),
            (TrafficSignSpecs.GREEN_COLOR, (0.267, 0.839, 0.173)),
            (TrafficSignSpecs.RED_STD, (0.05, 0.02, 0.02)),
            (TrafficSignSpecs.GREEN_STD, (0.02, 0.05, 0.02)),
            (ParkingLotSpecs.COLOR, (1.0, 0.0, 1.0)),
            (StartingZoneSpecs.COLOR, (0.5, 0.5, 0.5)),
            (StartingZoneSpecs.CLOCKWISE_COLOR, (0.2, 0.4, 1.0)),
            (StartingZoneSpecs.COUNTERCLOCKWISE_COLOR, (0.2, 1.0, 0.4)),
        ],
    )
    def test_color(self, actual: tuple[float, ...], expected: tuple[float, ...]) -> None:
        assert actual == expected

    def test_whole_numbers_are_floats(self) -> None:
        """A whole-number length must not come out an int.

        The generator emits `3.0`, not `3`; `Final[float] = 3` would bind an
        int and give a length a different type from its neighbours.
        """
        for value in (TrackDimensions.TRACK_SIZE, TrackDimensions.MIN_COORD, CorridorDimensions.WIDE):
            assert isinstance(value, float)


class TestStartingZoneLayout:
    """The layout model's invariants, which the Go generator also enforces."""

    def test_derived_from_division_lines(self) -> None:
        """Bands are the widths the corridor's division lines delimit.

        The middle band is compared with a tolerance while the constant itself
        is exact, and the asymmetry is the point: 0.6 - 0.4 evaluates to
        0.20000000000000007 in float. Deriving the band widths here at runtime
        would put that error into the geometry, which is why the subtraction
        happens once, in exact rational arithmetic, inside the generator.
        """
        assert STARTING_ZONE_LAYOUT.band_widths == (0.4, 0.2, 0.4)
        lines = (CorridorDimensions.DIVISION_OUTER, CorridorDimensions.DIVISION_INNER)
        assert lines[0] == STARTING_ZONE_LAYOUT.band_widths[0]
        assert lines[1] - lines[0] == pytest.approx(STARTING_ZONE_LAYOUT.band_widths[1])
        assert lines[1] - lines[0] != STARTING_ZONE_LAYOUT.band_widths[1]

    def test_bands_within_corridor(self) -> None:
        """0.40 + 0.20 fills a narrow corridor exactly; a wide one holds all three."""
        assert STARTING_ZONE_LAYOUT.bands_within(CorridorDimensions.NARROW) == 2
        assert STARTING_ZONE_LAYOUT.bands_within(CorridorDimensions.WIDE) == 3

    def test_offsets_sit_inside_their_bands(self) -> None:
        """No spawn straddles a band boundary for the real chassis."""
        half = RobotSpecs.WIDTH / 2
        edge = 0.0
        for band, offset in zip(STARTING_ZONE_LAYOUT.band_widths, STARTING_ZONE_LAYOUT.spawn_offsets, strict=True):
            assert offset - half >= edge - 1e-9
            assert offset + half <= edge + band + 1e-9
            edge += band

    def test_rejects_offset_outside_its_band(self) -> None:
        """The model refuses a layout the mat would not allow."""
        with pytest.raises(ValueError, match="outside band"):
            StartingZoneLayout(
                band_widths=(0.40, 0.20, 0.40),
                # 0.80 is the third band's centre, but a 0.20 m chassis there
                # spans 0.70-0.90 and the band starts at 0.60 -- legal. 0.95
                # spans 0.85-1.05, past the corridor entirely.
                spawn_offsets=(0.30, 0.50, 0.95),
                cell_length=0.5,
                cell_centers_along=(1.25, 1.75),
            )

    def test_rejects_offset_count_mismatch(self) -> None:
        """One offset per band, or the pairing is ambiguous."""
        with pytest.raises(ValueError, match="one offset per band"):
            StartingZoneLayout(
                band_widths=(0.40, 0.20, 0.40),
                spawn_offsets=(0.30, 0.50),
                cell_length=0.5,
                cell_centers_along=(1.25, 1.75),
            )

    def test_cell_centers_bracket_the_track_centre(self) -> None:
        """The square occupies the middle metre, so its cells straddle centre."""
        left, right = STARTING_ZONE_LAYOUT.cell_centers_along
        assert left == TrackDimensions.CENTER_COORD - STARTING_ZONE_LAYOUT.cell_length / 2
        assert right == TrackDimensions.CENTER_COORD + STARTING_ZONE_LAYOUT.cell_length / 2

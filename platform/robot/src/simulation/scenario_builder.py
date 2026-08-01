"""Build valid Open Challenge scenario metadata (same schema as simgen).

Shared by the headless test battery (``tests/unit/test_open_challenge_sim.py``)
and by ``find_recovery_envelope.py``/``test_deviation_recovery.py`` so both
drive the navigator from the exact same start-pose math. The RViz visualizer
and Obstacles Challenge scenarios instead load Go-generated fixtures — see
``scenario_catalog.py``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions, TrackDimensions
from shared.config.enums import Section
from shared.config.starting_zone import STARTING_ZONE_LAYOUT
from shared.domain.models import (
    CorridorWidthEntry,
    CorridorWidths,
    Position2D,
    ScenarioMetadata,
    StartingConditions,
)

# Re-exported: the start-pose geometry moved to the navigation layer so the
# deployed node can derive a starting pose without a scenario file. Kept here
# because the test battery and the recovery-envelope scripts import it from
# this module.
from src.navigation.start_conditions import start_pose

if TYPE_CHECKING:
    from shared.config.enums import Direction

_NARROW_MM = int(CorridorDimensions.NARROW * 1000)

__all__ = ["build_open_metadata", "start_cells", "start_pose", "uniform_widths"]


def start_cells(section: Section, widths_m: dict[str, float]) -> list[tuple[float, float]]:
    """Every legal starting-zone spawn pose for this side, as (x, y).

    Ordered outer wall inward, and within each band along the travel axis, so
    index 0 is always the cell hard against the outer wall.

    The pose is the band's chosen spawn offset, not the band centre -- the cell
    is where the robot must legally be, the offset is where within it we choose
    to put it. Both come from
    :data:`shared.config.starting_zone.STARTING_ZONE_LAYOUT`, which is
    generated from ``track.toml`` and shared with the Go scenario generator.

    ``start_pose`` spawns on the corridor centreline instead, which is not one
    of these cells and is the only start the simulator has ever exercised. A
    blind robot's first readings depend on where across the corridor it begins
    -- that is the input to corridor width estimation and to the side ranges
    the direction estimator votes on -- so the cells are not cosmetic.

    Args:
        section: Which side of the mat the starting square is on.
        widths_m: Corridor width per side name, in metres.

    Returns:
        Spawn poses, four for a narrow corridor and six for a wide one.
    """
    width = widths_m[section.value.lower()]
    track_max = TrackDimensions.MAX_COORD
    layout = STARTING_ZONE_LAYOUT

    # The square occupies the middle metre of the side, leaving a metre of
    # corner region either end; the two cells sit either side of the midpoint.
    alongs = layout.cell_centers_along

    cells: list[tuple[float, float]] = []
    # Bands are only reachable while they lie inside the corridor; past its
    # inner edge the band is under the centre square.
    for across in layout.spawn_offsets[: layout.bands_within(width)]:
        for along in alongs:
            if section is Section.SOUTH:
                cells.append((along, across))
            elif section is Section.NORTH:
                cells.append((along, track_max - across))
            elif section is Section.WEST:
                cells.append((across, along))
            else:
                cells.append((track_max - across, along))
    return cells


def build_open_metadata(
    widths_mm: dict[str, int],
    section: Section,
    direction: Direction,
    scenario_id: int = 0,
    start_cell: int | None = None,
) -> ScenarioMetadata:
    """Construct a valid Open Challenge metadata (same schema as simgen).

    Args:
        widths_mm: Corridor width per side name, in millimetres.
        section: Which side the robot starts on.
        direction: Travel direction for the round.
        scenario_id: Identifier carried into the metadata.
        start_cell: Index into :func:`start_cells` to start from. ``None``
            keeps the corridor-centreline spawn, which is what every existing
            caller expects and what the fixtures use -- but which is not one of
            the legal cells, so pass an index to exercise a real start.
    """
    widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
    sx, sy, yaw = start_pose(section, direction, widths_m)
    if start_cell is not None:
        cells = start_cells(section, widths_m)
        sx, sy = cells[start_cell % len(cells)]
    return ScenarioMetadata(
        scenario_id=scenario_id,
        challenge_type="open",
        num_signs=0,
        has_parking_lot=False,
        parking_lot=None,
        sign_positions=[],
        corridor_widths=CorridorWidths(
            **{
                side: CorridorWidthEntry(
                    type="narrow" if widths_mm[side] == _NARROW_MM else "wide",
                    width_mm=widths_mm[side],
                )
                for side in ("north", "south", "east", "west")
            },
        ),
        starting_conditions=StartingConditions(
            direction=str(direction),
            section=section.capitalized,
            position=Position2D(x=sx, y=sy),
            yaw=yaw,
        ),
    )


def uniform_widths(mm: int) -> dict[str, int]:
    """All four corridor sides at the same width (millimetres)."""
    return dict.fromkeys(("north", "south", "east", "west"), mm)

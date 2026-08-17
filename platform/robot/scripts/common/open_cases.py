"""Open Challenge scenario-space case enumeration.

Shared by the wall-collision-margin probe, the narrow-middle-band harness
probe, and the full-space exhaustive/parallel/AB sweeps -- all three walk the
same (layout, section, direction, start cell) space, differing only in which
slice of it they need.
"""

from __future__ import annotations

import itertools
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions
from shared.domain.enums import Direction, Section

from src.simulation.scenario_builder import start_cells

if TYPE_CHECKING:
    from collections.abc import Sequence

SIDES = ("south", "north", "east", "west")
NARROW_MM = int(CorridorDimensions.NARROW * 1000)
WIDE_MM = int(CorridorDimensions.WIDE * 1000)

MIDDLE_BAND_CELLS = (2, 3)
"""Cell indices of the middle band: two cells per band, ordered outer wall
inward, so band 1 is indices 2 and 3."""


def case_space(
    *, narrow_only: bool = False, cells: Sequence[int] | None = None
) -> list[tuple[tuple[int, ...], Section, Direction, int]]:
    """Every (layout, section, direction, start cell) combo the track can present.

    ``narrow_only`` restricts to sections whose own starting corridor is
    narrow -- the wall-collision-margin/flush-start failing set. ``cells``
    restricts to specific starting cell indices (e.g. the chassis-width
    middle band); omit it for every legal cell in that section.
    """
    cases: list[tuple[tuple[int, ...], Section, Direction, int]] = []
    for widths in itertools.product((NARROW_MM, WIDE_MM), repeat=len(SIDES)):
        widths_m = {k: v / 1000.0 for k, v in zip(SIDES, widths, strict=True)}
        for section in Section:
            if narrow_only and widths_m[section.value.lower()] != CorridorDimensions.NARROW:
                continue
            n_cells = len(start_cells(section, widths_m))
            candidate_cells = cells if cells is not None else range(n_cells)
            for direction in Direction:
                cases.extend((widths, section, direction, cell) for cell in candidate_cells if cell < n_cells)
    return cases

"""Open Challenge scenario-space case enumeration.

Shared by the wall-collision-margin probe, the narrow-middle-band harness
probe, and the full-space exhaustive/parallel/AB sweeps -- all three walk the
same (layout, section, direction, start cell) space, differing only in which
slice of it they need.
"""

from __future__ import annotations

import itertools
import random
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
        widths_m = {Section(k): v / 1000.0 for k, v in zip(SIDES, widths, strict=True)}
        for section in Section:
            if narrow_only and widths_m[section] != CorridorDimensions.NARROW:
                continue
            n_cells = len(start_cells(section, widths_m))
            candidate_cells = cells if cells is not None else range(n_cells)
            for direction in Direction:
                cases.extend((widths, section, direction, cell) for cell in candidate_cells if cell < n_cells)
    return cases


BALANCED_128_SIZE = 128
"""16 layouts x 4 sections x 2 directions. The cell is what varies within it."""


def balanced_128_cases(*, seed: int = 0) -> list[tuple[tuple[int, ...], Section, Direction, int]]:
    """128 cases covering every layout/section/direction, with the start cell VARIED.

    The historical ``open128`` corpus is this same 128-combo grid with
    ``start_cell`` pinned to **0**, which ``start_cells`` documents as "always
    the cell hard against the outer wall" -- one fixed spawn, never varied. That
    is the wrong constant to freeze: a blind robot's opening readings depend on
    where across the corridor it begins, and those readings are the input to
    corridor-width estimation and to the side ranges the direction estimator
    votes on. Every Open pass rate in this project's history (96 -> 125 -> 126)
    was measured against that single spawn.

    The full space is 640 because each combo admits **six** legal cells in a wide
    corridor and **four** in a narrow one. Sampling 128 uniformly from those 640
    (what ``diag_open_ab`` did) does cover cells, but leaves layout/section
    coverage to chance, so two runs at different seeds are not comparable
    scenario-for-scenario.

    This keeps the grid complete AND varies the cell: one case per combo, so the
    count stays 128 and every layout/section/direction still appears exactly
    once, while the cell is assigned round-robin per cell-count class. The combo
    order is shuffled before assignment purely to decorrelate the cell from
    section/direction/layout -- without it, cell would track enumeration order
    and reintroduce a systematic bias of a different shape.

    Deterministic for a given ``seed``: the same seed always yields the same 128
    scenarios, so arms remain comparable. Varying the seed draws an independent
    balanced corpus, which is the honest way to check a result is not an artefact
    of one spawn assignment.
    """
    combos: list[tuple[tuple[int, ...], Section, Direction, int]] = []
    for widths in itertools.product((NARROW_MM, WIDE_MM), repeat=len(SIDES)):
        widths_m = {Section(k): v / 1000.0 for k, v in zip(SIDES, widths, strict=True)}
        for section in Section:
            n_cells = len(start_cells(section, widths_m))
            for direction in Direction:
                combos.append((widths, section, direction, n_cells))

    order = list(range(len(combos)))
    random.Random(seed).shuffle(order)  # noqa: S311 - reproducible corpus selection, not crypto

    # Round-robin per cell-count class, so the 4-cell and 6-cell combos each
    # spread evenly over their own range rather than one class starving the
    # other's high indices.
    taken: dict[int, int] = {}
    assigned: dict[int, int] = {}
    for position in order:
        n_cells = combos[position][3]
        nth = taken.get(n_cells, 0)
        assigned[position] = nth % n_cells
        taken[n_cells] = nth + 1

    return [
        (widths, section, direction, assigned[i]) for i, (widths, section, direction, _) in enumerate(combos)
    ]

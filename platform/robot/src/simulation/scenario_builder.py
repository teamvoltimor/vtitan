"""Build valid Open Challenge scenario metadata (same schema as simgen).

Shared by the headless test battery (``tests/unit/test_open_challenge_sim.py``)
and by ``find_recovery_envelope.py``/``test_deviation_recovery.py`` so both
drive the navigator from the exact same start-pose math. The RViz visualizer
and Obstacles Challenge scenarios instead load Go-generated fixtures — see
``scenario_catalog.py``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions
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
    from shared.config.enums import Direction, Section

_NARROW_MM = int(CorridorDimensions.NARROW * 1000)

__all__ = ["build_open_metadata", "start_pose", "uniform_widths"]


def build_open_metadata(
    widths_mm: dict[str, int],
    section: Section,
    direction: Direction,
    scenario_id: int = 0,
) -> ScenarioMetadata:
    """Construct a valid Open Challenge metadata (same schema as simgen)."""
    widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
    sx, sy, yaw = start_pose(section, direction, widths_m)
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

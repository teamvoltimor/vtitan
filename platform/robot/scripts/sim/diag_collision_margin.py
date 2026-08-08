"""Does the wall collision margin make a legal start unreachable?

``track_model`` inflates every wall inward by ``WALL_COLLISION_THICKNESS/2 -
WALL_THICKNESS/2`` = 40 mm, mirroring the fatter collision mesh the Go
generator emits for Gazebo. That mesh exists so the LIDAR link, which sits
30 mm ahead of the chassis front, cannot end up inside a wall in the physics
engine. The headless simulator has no physics body for the LIDAR -- it
raycasts from the chassis centre against the *visual* faces -- so here the
inflation only makes the chassis 8 cm wider and 8 cm longer than it is.

In a 0.6 m corridor that moves the effective inner boundary to 0.560, while
the starting square's middle band spans 0.400-0.600 and its only legal
placement puts the chassis edge at 0.594. The start is 34 mm inside the
collision box before the robot has moved, and INNER_WALL is non-terminal for
the Open Challenge, so the chassis is *held* rather than the run ended.

This runs the affected starts under both margins and reports the difference.
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions
from shared.domain.enums import Direction, Section

from src.simulation import track_model
from src.simulation.scenario_builder import build_open_metadata, start_cells
from src.simulation.scenario_simulator import ScenarioSimulator

_SIDES = ("south", "north", "east", "west")
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
_MIDDLE_BAND_CELLS = (2, 3)

_DEFAULT_LAPS = 3
_DEFAULT_LIMIT = 8


def _cases(limit: int) -> list[tuple[tuple[int, ...], Section, Direction, int]]:
    """Narrow-corridor middle-band starts, which is the whole failing set."""
    out = []
    for widths in itertools.product((_NARROW_MM, _WIDE_MM), repeat=len(_SIDES)):
        widths_m = {k: v / 1000.0 for k, v in zip(_SIDES, widths, strict=True)}
        for section in Section:
            if widths_m[section.value.lower()] != CorridorDimensions.NARROW:
                continue
            n = len(start_cells(section, widths_m))
            for direction in Direction:
                out.extend((widths, section, direction, c) for c in _MIDDLE_BAND_CELLS if c < n)
    return out[:limit] if limit else out


def main() -> None:
    """Run each affected start with the margin on and off."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=_DEFAULT_LAPS)
    parser.add_argument("--limit", type=int, default=_DEFAULT_LIMIT)
    args = parser.parse_args()

    original = track_model._COLLISION_MARGIN  # noqa: SLF001
    cases = _cases(args.limit)
    print(f"{len(cases)} arranques de banda media en pasillo estrecho")
    print(f"margen actual = {original * 1000:.0f} mm\n")

    with_margin = without = 0
    for i, (widths, section, direction, cell) in enumerate(cases):
        widths_mm = dict(zip(_SIDES, widths, strict=True))
        meta = build_open_metadata(widths_mm, section, direction, scenario_id=i, start_cell=cell)

        results = {}
        for label, margin in (("con", original), ("sin", 0.0)):
            track_model._COLLISION_MARGIN = margin  # noqa: SLF001
            sim = ScenarioSimulator(meta, num_laps=args.laps, seed=i, blind=True)
            results[label] = sim.run()
        track_model._COLLISION_MARGIN = original  # noqa: SLF001

        with_margin += bool(results["con"].success)
        without += bool(results["sin"].success)
        label = "-".join(str(w) for w in widths)
        print(
            f"[{i:2d}] {label} {section.value}/{direction} c{cell}  "
            f"con={'ok ' if results['con'].success else 'FAIL'} "
            f"d={results['con'].distance_m:5.2f}m  "
            f"sin={'ok ' if results['sin'].success else 'FAIL'} "
            f"d={results['sin'].distance_m:5.2f}m laps={results['sin'].laps_completed}/{args.laps}",
            flush=True,
        )

    print(f"\ncon margen: {with_margin}/{len(cases)} ok")
    print(f"sin margen: {without}/{len(cases)} ok")


if __name__ == "__main__":
    main()

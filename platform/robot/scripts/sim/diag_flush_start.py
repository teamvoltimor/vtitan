"""Does the narrow-corridor middle-band start fail on its own, or on harness policy?

``diag_open_exhaustive.py`` runs the strict contact policy, under which a wall
touch ends the run on the first tick outside the opening grace window. In a
0.6 m corridor the middle band is exactly chassis-width, so the robot begins
flush against the inner block -- and that whole cell fails, 0 of 23.

The simulator's own docstring says the reversing escape "is observable at all"
only under ``contact_grace_s`` with ``solid_walls``, which is what
``visualize_scenario --recover`` turns on. So the failure could be the robot
being unable to free itself, or it could be the harness ending the run before
the escape has a chance. This runs every narrow middle-band case both ways and
prints the two pass rates side by side.
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions
from shared.domain.enums import Direction, Section

from src.simulation.scenario_builder import build_open_metadata, start_cells
from src.simulation.scenario_simulator import ScenarioSimulator

_SIDES = ("south", "north", "east", "west")
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)

# Cell indices of the middle band: two cells per band, ordered outer wall
# inward, so band 1 is indices 2 and 3.
_MIDDLE_BAND_CELLS = (2, 3)

_DEFAULT_LAPS = 3
_DEFAULT_GRACE_S = 5.0
_DEFAULT_LIMIT = 0


def _middle_band_cases() -> list[tuple[tuple[int, ...], Section, Direction, int]]:
    """Every layout whose starting corridor is narrow, starting in band 1."""
    cases = []
    for widths in itertools.product((_NARROW_MM, _WIDE_MM), repeat=len(_SIDES)):
        widths_m = {k: v / 1000.0 for k, v in zip(_SIDES, widths, strict=True)}
        for section in Section:
            if widths_m[section.value.lower()] != CorridorDimensions.NARROW:
                continue
            n_cells = len(start_cells(section, widths_m))
            for direction in Direction:
                cases.extend(
                    (widths, section, direction, cell) for cell in _MIDDLE_BAND_CELLS if cell < n_cells
                )
    return cases


def _run(meta: object, laps: int, seed: int, *, recover: bool, grace_s: float) -> object:
    sim = ScenarioSimulator(meta, num_laps=laps, seed=seed, blind=True, solid_walls=recover)
    return sim.run(contact_grace_s=grace_s if recover else None)


def main() -> None:
    """Run each narrow middle-band case under both contact policies."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=_DEFAULT_LAPS)
    parser.add_argument("--grace", type=float, default=_DEFAULT_GRACE_S, help="Seconds pinned before failure.")
    parser.add_argument("--limit", type=int, default=_DEFAULT_LIMIT, help="Cap the case count (0 = all).")
    args = parser.parse_args()

    cases = _middle_band_cases()
    if args.limit:
        cases = cases[: args.limit]
    print(f"{len(cases)} casos de banda media en pasillo estrecho\n", flush=True)

    strict_ok = recover_ok = 0
    for i, (widths, section, direction, cell) in enumerate(cases):
        widths_mm = dict(zip(_SIDES, widths, strict=True))
        meta = build_open_metadata(widths_mm, section, direction, scenario_id=i, start_cell=cell)

        strict = _run(meta, args.laps, i, recover=False, grace_s=args.grace)
        rec = _run(meta, args.laps, i, recover=True, grace_s=args.grace)
        strict_ok += bool(strict.success)
        recover_ok += bool(rec.success)

        label = "-".join(str(w) for w in widths)
        print(
            f"[{i:3d}] {label} {section.value}/{direction} c{cell}  "
            f"estricto={'ok ' if strict.success else 'FAIL'} laps={strict.laps_completed}/{args.laps}  "
            f"recover={'ok ' if rec.success else 'FAIL'} laps={rec.laps_completed}/{args.laps} "
            f"contactos={rec.contact_count}",
            flush=True,
        )

    total = len(cases)
    print(f"\nestricto: {strict_ok}/{total} ok")
    print(f"recover:  {recover_ok}/{total} ok")


if __name__ == "__main__":
    main()

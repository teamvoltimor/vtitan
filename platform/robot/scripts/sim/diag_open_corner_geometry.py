"""Where does the PLANNED path run at each corner type, wall to wall?

The blind Open Challenge fails in exactly one place: a wide->narrow first
corner. Measured over the 128-scenario sweep, on both hardware builds and at
two different centring authorities, every failure sits there -- 7/7 wedges on
the slower build, 3/3 collisions on the faster one, 8/8 wedges at the lower
centring cap -- against zero failures at W->W (47 runs), N->W (24) and N->N
(21). This asks whether the PLAN is already wrong there, before any control or
sensor error is involved.

The suspicion is ``corner_arc_radius``:

    r = min(ARC_RADIUS, max(W_entry, W_exit) / 2 - center_bias_m)

``max`` is deliberate and correct for reaching the wider corridor's centreline
(``min`` was measured to leave 0.05 m on a mixed corner). But a corner joining
a 1.0 m corridor to a 0.6 m one has its two centrelines at DIFFERENT distances
from the inner block -- 0.45 m and 0.25 m with the shipped 0.05 m bias -- and a
single circle cannot be tangent to both. Sized to the wide side, the arc must
run wide of the narrow corridor's centreline on exit, toward its OUTER wall.
In the Open Challenge the outer wall is the run-ending surface.

Why this has not been ruled out already: the refuted corner work
(corner-widen, corner-blend, SIGN_LANE_CORNER_ENTRY_M) was all measured on the
OBSTACLES challenge, whose corridors are all 1.0 m by rule -- a mixed-width
corner does not exist there, so none of it tested this.

NO SIMULATION. This reads the planner's own output and the track's own
geometry, so it separates "the plan is wrong" from "the plan is fine and
tracking loses it" -- a distinction three refuted Obstacles hypotheses were
measured on the wrong side of. Clearance is taken at closest approach along the
POLYLINE, not at waypoints: a corner arc's waypoints are its coarsest samples,
and the tightest point of an arc generally falls between two of them.

Usage (from ``platform/robot``, with PYTHONPATH=".")::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        python scripts/sim/diag_open_corner_geometry.py
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, CorridorDimensions, RobotSpecs, TrackDimensions
from shared.domain.enums import Direction, Section

from scripts.common.diag_base import add_tuning_arg, load_tuning
from scripts.common.open_cases import SIDES
from scripts.common.tables import print_table
from src.navigation.planning.waypoints import _build_corridor_order, _rotate_to_start, calculate_waypoints
from src.navigation.track_geometry import corridor_geometry_from_widths
from src.simulation.scenario_builder import build_open_metadata

_SAMPLES_PER_SEGMENT = 200
"""Points interpolated along each polyline segment.

The path is a polyline, so its true closest approach to a wall lies between
waypoints. Sampling densely is the cheap way to find it without reconstructing
the arc analytically -- and reconstructing it would measure the arc this script
is trying to audit rather than the points the controller is actually given."""


def _corner_label(entry_w: float, exit_w: float) -> str:
    """``W->N`` style label for the two corridors a corner joins."""
    wide = (CorridorDimensions.NARROW + CorridorDimensions.WIDE) / 2

    def side(width: float) -> str:
        return "W" if width > wide else "N"

    return f"{side(entry_w)}->{side(exit_w)}"


def _outer_wall_distance(x: float, y: float) -> float:
    """Distance from a point to the nearest outer wall."""
    return min(x - TrackDimensions.MIN_COORD, TrackDimensions.MAX_COORD - x,
               y - TrackDimensions.MIN_COORD, TrackDimensions.MAX_COORD - y)


def _inner_block_distance(x: float, y: float, block) -> float:  # noqa: ANN001 - InnerBlock, imported indirectly
    """Distance from a point to the inner block rectangle (0 if inside)."""
    dx = max(block.x_min - x, 0.0, x - block.x_max)
    dy = max(block.y_min - y, 0.0, y - block.y_max)
    return math.hypot(dx, dy)


def _corner_centre(entry: Section, exit_: Section, block) -> tuple[float, float]:  # noqa: ANN001
    """The inner-block corner where these two corridors meet.

    Used only to attribute a sampled point to a corner, never to measure -- the
    measurement is taken against the walls themselves.
    """
    x = block.x_min if Section.WEST in (entry, exit_) else block.x_max
    y = block.y_min if Section.SOUTH in (entry, exit_) else block.y_max
    return x, y


def _sample_polyline(waypoints: list) -> list[tuple[float, float]]:
    """Densely interpolate the planned polyline the controller will chase."""
    points: list[tuple[float, float]] = []
    for a, b in itertools.pairwise(waypoints):
        for i in range(_SAMPLES_PER_SEGMENT):
            t = i / _SAMPLES_PER_SEGMENT
            points.append((a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t))
    if waypoints:
        points.append((waypoints[-1].x, waypoints[-1].y))
    return points


def _measure(widths_mm: dict[str, int], direction: Direction, tuning) -> list[dict[str, object]]:  # noqa: ANN001
    """Clearances at each of the four corners of one track layout."""
    section = Section.SOUTH
    meta = build_open_metadata(widths_mm, section, direction, scenario_id=0, start_cell=0)
    waypoints = calculate_waypoints(meta, num_laps=1, tuning=tuning)
    widths_m = {s: widths_mm[s.value.lower()] / 1000.0 for s in Section}
    geometry = corridor_geometry_from_widths(widths_m)
    block = geometry.inner_block

    points = _sample_polyline(waypoints)
    order = _rotate_to_start(_build_corridor_order(direction), section)

    rows: list[dict[str, object]] = []
    for index, entry in enumerate(order):
        exit_ = order[(index + 1) % len(order)]
        cx, cy = _corner_centre(entry, exit_, block)
        # Attribute a sample to this corner by proximity to the inner-block
        # corner the two corridors share. The arc is tangent to both
        # centrelines, so everything on it is nearer this corner than any
        # other, and the radius bounds how far that reaches.
        near = [(x, y) for x, y in points if math.hypot(x - cx, y - cy) <= tuning.waypoints.ARC_RADIUS * 2.0]
        if not near:
            continue
        rows.append(
            {
                "corner": _corner_label(widths_m[entry], widths_m[exit_]),
                "entry": entry.value,
                "exit": exit_.value,
                "outer": min(_outer_wall_distance(x, y) for x, y in near),
                "inner": min(_inner_block_distance(x, y, block) for x, y in near),
            }
        )
    return rows


def main() -> None:
    """Report planned-path clearance by corner type, over every width layout."""
    parser = argparse.ArgumentParser(description=__doc__)
    add_tuning_arg(parser)
    args = parser.parse_args()
    tuning = load_tuning(args.tuning)

    half_width = RobotSpecs.WIDTH / 2.0
    print(
        f"chassis half-width {half_width:.3f} m, ARC_RADIUS {tuning.waypoints.ARC_RADIUS}, "
        f"WIDE_CENTER_BIAS_M {tuning.waypoints.WIDE_CENTER_BIAS_M} / "
        f"NARROW_CENTER_BIAS_M {tuning.waypoints.NARROW_CENTER_BIAS_M} "
        f"below {tuning.waypoints.NARROW_WIDTH_THRESHOLD_M} m "
        f"({tuning.waypoints.CENTER_BIAS_SIDE})\n",
        flush=True,
    )

    narrow_mm, wide_mm = int(CorridorDimensions.NARROW * 1000), int(CorridorDimensions.WIDE * 1000)
    buckets: dict[str, list[dict[str, object]]] = {}
    for widths in itertools.product((narrow_mm, wide_mm), repeat=len(SIDES)):
        widths_mm = dict(zip(SIDES, widths, strict=True))
        for direction in Direction:
            for row in _measure(widths_mm, direction, tuning):
                buckets.setdefault(str(row["corner"]), []).append(row)

    table = []
    for corner in ("W->W", "W->N", "N->W", "N->N"):
        rows = buckets.get(corner, [])
        if not rows:
            continue
        outer = [float(r["outer"]) for r in rows]
        inner = [float(r["inner"]) for r in rows]
        table.append(
            [
                corner,
                len(rows),
                f"{min(outer):.3f}",
                f"{min(outer) - half_width:+.3f}",
                f"{min(inner):.3f}",
                f"{min(inner) - half_width:+.3f}",
            ]
        )
    print("planned-path clearance at closest approach, by corner type:", flush=True)
    print_table(
        table,
        ["corner", "corners", "outer wall", "margin", "inner block", "margin"],
    )
    print(
        "\nMargin subtracts the chassis HALF-WIDTH only. A real chassis sweeps its\n"
        "half-diagonal through a turn, so these are upper bounds -- a negative\n"
        "margin here means the planned line alone puts the robot into the wall,\n"
        "with no tracking error spent yet.",
        flush=True,
    )


if __name__ == "__main__":
    main()

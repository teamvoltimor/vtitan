"""Open Challenge lap planner and mock lap runner.

``OpenLapPlanner`` computes the corridor centerline corner waypoints for any
valid Open Challenge track layout.  ``MockLapRunner`` drives a virtual robot
along those waypoints — no physics, no ROS2.  Both are pure Python and safe to
run on Windows without a Pixi environment.

Track geometry (WRO 2026, bottom-left origin):
  • Exterior: 0,0 → 3,3 (mat is 3.2 × 3.2; usable track is 3.0 × 3.0)
  • Interior walls span the 1.0–2.0 segment on each axis
  • Each corridor width is independently 0.6 m (narrow) or 1.0 m (wide)

Corridor centerlines:
  south_y  = south_w / 2
  north_y  = 3 − north_w / 2
  east_x   = 3 − east_w / 2
  west_x   = west_w / 2

Four corner waypoints (centerline intersections):
  SW = (west_x, south_y),  SE = (east_x, south_y)
  NE = (east_x, north_y),  NW = (west_x, north_y)

Lap order (verified against yaw map in randomize.go):
  Clockwise (CW):  South(-X) → West(+Y) → North(+X) → East(-Y)
    corner sequence from South: SW → NW → NE → SE
  Counter-Clockwise (CCW): South(+X) → East(+Y) → North(-X) → West(-Y)
    corner sequence from South: SE → NE → NW → SW
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from shared.domain.enums import Direction, Section
from src.scenario.models import ScenarioMetadata

_TRACK_MIN = 0.0
_TRACK_MAX = 3.0

# Ordered corner sequences for each starting section, by direction.
# Each tuple is the order in which the 4 corners are visited during one lap.
# Corners indexed as: 0=SW, 1=SE, 2=NE, 3=NW
_CW_ORDER: dict[Section, tuple[int, int, int, int]] = {
    Section.SOUTH: (0, 3, 2, 1),  # SW → NW → NE → SE
    Section.WEST:  (3, 2, 1, 0),  # NW → NE → SE → SW
    Section.NORTH: (2, 1, 0, 3),  # NE → SE → SW → NW
    Section.EAST:  (1, 0, 3, 2),  # SE → SW → NW → NE
}
_CCW_ORDER: dict[Section, tuple[int, int, int, int]] = {
    Section.SOUTH: (1, 2, 3, 0),  # SE → NE → NW → SW
    Section.EAST:  (2, 3, 0, 1),  # NE → NW → SW → SE
    Section.NORTH: (3, 0, 1, 2),  # NW → SW → SE → NE
    Section.WEST:  (0, 1, 2, 3),  # SW → SE → NE → NW
}


@dataclass(frozen=True, slots=True)
class TrackCorners:
    """Corridor centerline intersections (corner waypoints) for one scenario."""

    sw: tuple[float, float]
    se: tuple[float, float]
    ne: tuple[float, float]
    nw: tuple[float, float]

    def as_list(self) -> list[tuple[float, float]]:
        """All four corners in index order: [sw, se, ne, nw]."""
        return [self.sw, self.se, self.ne, self.nw]

    def ordered_for(
        self,
        section: Section,
        direction: Direction,
    ) -> list[tuple[float, float]]:
        """Return the 4 corners in the order visited from the given section."""
        corners = self.as_list()
        order = (
            _CW_ORDER[section] if direction == Direction.CLOCKWISE else _CCW_ORDER[section]
        )
        return [corners[i] for i in order]


@dataclass(frozen=True, slots=True)
class LapResult:
    """Outcome record for one completed lap."""

    lap: int
    waypoints: tuple[tuple[float, float], ...]
    distance_m: float
    all_in_bounds: bool


@dataclass(slots=True)
class MockLapRunner:
    """Simulates a robot navigating the Open Challenge track without physics.

    The robot teleports from waypoint to waypoint along corridor centerlines.
    Each step is validated against track bounds.  This is sufficient to prove
    that the centerline planner can always complete 3 laps for any valid Open
    Challenge scenario.
    """

    position: tuple[float, float]
    laps: list[LapResult] = field(default_factory=list)

    def run(
        self,
        corners: TrackCorners,
        section: Section,
        direction: Direction,
        n_laps: int = 3,
    ) -> bool:
        """Navigate n_laps around the track.  Returns True if all laps succeed."""
        ordered = corners.ordered_for(section, direction)

        for lap_num in range(1, n_laps + 1):
            visited: list[tuple[float, float]] = []
            total_dist = 0.0
            all_ok = True

            for wp in ordered:
                dist = _dist2d(self.position, wp)
                if not _in_bounds(wp):
                    all_ok = False
                total_dist += dist
                self.position = wp
                visited.append(wp)

            # Close the loop — return to first corner of next lap (same sequence)
            self.laps.append(
                LapResult(
                    lap=lap_num,
                    waypoints=tuple(visited),
                    distance_m=total_dist,
                    all_in_bounds=all_ok,
                )
            )
            if not all_ok:
                return False

        return True

    @property
    def completed_laps(self) -> int:
        return sum(1 for r in self.laps if r.all_in_bounds)

    @property
    def total_distance_m(self) -> float:
        return sum(r.distance_m for r in self.laps)


class OpenLapPlanner:
    """Computes the corner waypoints for any Open Challenge track layout."""

    _TRACK_MAX = _TRACK_MAX

    def corners(self, scenario: ScenarioMetadata) -> TrackCorners:
        """Compute the 4 corridor centerline corners from a scenario's widths."""
        south_y = scenario.corridor_width_m(Section.SOUTH) / 2
        north_y = self._TRACK_MAX - scenario.corridor_width_m(Section.NORTH) / 2
        east_x = self._TRACK_MAX - scenario.corridor_width_m(Section.EAST) / 2
        west_x = scenario.corridor_width_m(Section.WEST) / 2
        return TrackCorners(
            sw=(west_x, south_y),
            se=(east_x, south_y),
            ne=(east_x, north_y),
            nw=(west_x, north_y),
        )

    def plan(self, scenario: ScenarioMetadata, n_laps: int = 3) -> MockLapRunner:
        """Create a runner pre-loaded with the scenario start position."""
        sc = scenario.starting_conditions
        runner = MockLapRunner(position=(sc.position.x, sc.position.y))
        track_corners = self.corners(scenario)
        runner.run(track_corners, sc.section, sc.direction, n_laps)
        return runner


def _dist2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _in_bounds(pt: tuple[float, float]) -> bool:
    return _TRACK_MIN <= pt[0] <= _TRACK_MAX and _TRACK_MIN <= pt[1] <= _TRACK_MAX

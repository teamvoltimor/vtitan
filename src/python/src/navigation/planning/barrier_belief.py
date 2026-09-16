"""Where the robot has repeatedly seen the MAGENTA parking barrier.

The camera labels the parking-lot barrier magenta far more often than it
mislabels it red, and every magenta detection was discarded:
:func:`~src.navigation.planning.sign_discovery.detection_to_observation`
returns ``None`` for any colour that is not RED or GREEN. Meanwhile the same
physical object, seen red under motion blur, was seeded into the sign map as a
pillar and routed around -- which asks the planner for a pass around a WALL.

This keeps the discarded evidence instead. It is deliberately a belief about a
PLACE rather than a classifier: a red box is not re-examined, it is simply not
believed if it stands where the barrier has repeatedly been seen. That is the
one discriminator available here, because shape cannot do it -- the sign and the
barrier are both exactly 0.10 m tall (``track.toml``), so with an intact bbox
height their aspect ratios are the only difference, and frame clipping is what
destroys it.

Measured rationale: ``adr:0058-sign-discovery-range-and-barrier-belief``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from shared.config.constants import ParkingLotSpecs, RobotSpecs, TrackDimensions

__all__ = ["BarrierBelief", "BarrierSighting"]


_LOT_HALF_SPAN_M = ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH / 2.0
"""Half the rulebook distance between the lot's two fins, 0.225 m.

The same expression ``parking_lot_from_in_bay_start`` uses to place BLOCK1 and
BLOCK2: the factor multiplies the CHASSIS length, not the fin's.
"""


def _lot_runs_along_x(x_m: float, y_m: float) -> bool:
    """Does the lot's span lie along X, i.e. is it on the north or south wall?

    The lot stands against an OUTER wall, so the nearest one names the axis.
    Mirrors ``parking_lot_from_in_bay_start``'s ``section in (SOUTH, NORTH)``
    without needing the section, which the belief does not carry.
    """
    to_x_wall = min(abs(x_m - TrackDimensions.MIN_COORD), abs(x_m - TrackDimensions.MAX_COORD))
    to_y_wall = min(abs(y_m - TrackDimensions.MIN_COORD), abs(y_m - TrackDimensions.MAX_COORD))
    return to_y_wall <= to_x_wall


@dataclass(slots=True)
class BarrierSighting:
    """One believed barrier location and how much evidence stands behind it."""

    x_m: float
    y_m: float
    sightings: int = 1

    def absorb(self, x_m: float, y_m: float) -> None:
        """Fold another sighting in, moving the centroid by its share."""
        self.sightings += 1
        weight = 1.0 / self.sightings
        self.x_m += (x_m - self.x_m) * weight
        self.y_m += (y_m - self.y_m) * weight


@dataclass(slots=True)
class BarrierBelief:
    """Accumulates magenta sightings and suppresses reds that land on them.

    Args:
        min_sightings: Sightings that must agree before a location is believed.
            ``0`` disables the belief: nothing is ever suppressed.
        merge_radius_m: Sightings closer than this are the same barrier.
        span_along_wall: Treat the believed lot as the 0.45 m SPAN the rulebook
            gives it rather than a point, extending the test along the wall
            only. False reproduces the point behaviour exactly.
        suppression_radius_m: A red detection within this distance of a
            BELIEVED barrier is not a pillar.
    """

    min_sightings: int
    merge_radius_m: float
    suppression_radius_m: float
    span_along_wall: bool = False
    _sightings: list[BarrierSighting] = field(default_factory=list)

    @property
    def enabled(self) -> bool:
        """Whether this belief can ever suppress anything."""
        return self.min_sightings > 0

    def observe(self, x_m: float, y_m: float) -> None:
        """Record a magenta sighting at a world position."""
        if not self.enabled:
            return
        nearest = self._nearest(x_m, y_m, self.merge_radius_m)
        if nearest is None:
            self._sightings.append(BarrierSighting(x_m, y_m))
        else:
            nearest.absorb(x_m, y_m)

    def suppresses(self, x_m: float, y_m: float) -> bool:
        """Is this position close enough to a believed barrier to be one?

        Only sightings that have reached ``min_sightings`` count. A single
        stray magenta detection must not be able to blank out a real pillar --
        the cost of wrongly suppressing a pillar is a missed route, while the
        cost of wrongly believing one is the wall-pass that ends a round, so
        the threshold is the place to trade the two.

        ``span_along_wall`` changes the SHAPE, not the size, of that test. The
        lot is not a point: the rulebook puts two fins
        ``ParkingLotSpecs.BLOCK_SPACING_FACTOR`` chassis lengths apart along the
        wall, which ``parking_lot_from_in_bay_start`` already builds as BLOCK1
        and BLOCK2. A centroid with a round bubble therefore has to cover that
        span, and extending ALONG the wall and not across it is the point:
        widening the radius uniformly is what costs real pillars, and the extra
        coverage is only wanted where the lot physically extends. See
        ``adr:0058-sign-discovery-range-and-barrier-belief`` for the measured
        geometry.
        """
        if not self.enabled:
            return False
        for lot in self.believed():
            dx, dy = abs(lot.x_m - x_m), abs(lot.y_m - y_m)
            if self.span_along_wall:
                if _lot_runs_along_x(lot.x_m, lot.y_m):
                    dx = max(0.0, dx - _LOT_HALF_SPAN_M)
                else:
                    dy = max(0.0, dy - _LOT_HALF_SPAN_M)
            if math.hypot(dx, dy) <= self.suppression_radius_m:
                return True
        return False

    def believed(self) -> list[BarrierSighting]:
        """The single best-supported location, if any has cleared the threshold.

        THERE IS ONE PARKING LOT. Taking every cluster over the threshold
        instead is far too expensive: the belief settles on several locations
        per run and their bubbles refuse real pillars -- a worse failure than
        the one being fixed.

        The scatter is not noise to be averaged away either; it is the pinhole
        range under-reading (see ``_detection_to_world``: ``RANGE_SCALE`` ships
        at 1.0 deliberately, and the estimate is short by roughly 2x). So the
        clusters are real disagreements about depth along one bearing, and the
        rulebook is the only thing that can arbitrate them: keep the one with
        the most evidence behind it. See
        ``adr:0058-sign-discovery-range-and-barrier-belief``.
        """
        if not self.enabled:
            return []
        qualified = [s for s in self._sightings if s.sightings >= self.min_sightings]
        if not qualified:
            return []
        return [max(qualified, key=lambda s: s.sightings)]

    def _nearest(
        self,
        x_m: float,
        y_m: float,
        radius_m: float,
        *,
        min_sightings: int = 0,
    ) -> BarrierSighting | None:
        best: BarrierSighting | None = None
        best_d = radius_m
        for sighting in self._sightings:
            if sighting.sightings < min_sightings:
                continue
            d = math.hypot(sighting.x_m - x_m, sighting.y_m - y_m)
            if d <= best_d:
                best = sighting
                best_d = d
        return best

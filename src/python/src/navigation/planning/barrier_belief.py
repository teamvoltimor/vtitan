"""Where the robot has repeatedly seen the MAGENTA parking barrier.

The camera labels the parking-lot barrier magenta far more often than it
mislabels it red, and until now every magenta detection was discarded:
:func:`~src.navigation.planning.sign_discovery.detection_to_observation`
returns ``None`` for any colour that is not RED or GREEN. Meanwhile the same
physical object, seen red under motion blur at p50 confidence 0.79, was seeded
into the sign map as a pillar and routed around -- which asks the planner for a
pass around a WALL.

MEASURED over the four 2026-09-14 rounds: 1,921 magenta detections thrown away,
against 748 wall-shaped reds (94% of them) admitted to the map. On
run_20260914_214824 that ended the round -- the chassis pendulumed for 66.4 s,
54% of the run, at (0.75, 0.25) beside the west parking corridor, travelling
10.336 m of wheel to net 0.596 m.

This keeps the discarded evidence instead. It is deliberately a belief about a
PLACE rather than a classifier: a red box is not re-examined, it is simply not
believed if it stands where the barrier has repeatedly been seen. That is the
one discriminator available here, because shape cannot do it -- the sign and the
barrier are both exactly 0.10 m tall (``track.toml``), so with an intact bbox
height their aspect ratios are the only difference, and frame clipping is what
destroys it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = ["BarrierBelief", "BarrierSighting"]


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
        suppression_radius_m: A red detection within this distance of a
            BELIEVED barrier is not a pillar.
    """

    min_sightings: int
    merge_radius_m: float
    suppression_radius_m: float
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
        """
        if not self.enabled:
            return False
        return any(
            math.hypot(lot.x_m - x_m, lot.y_m - y_m) <= self.suppression_radius_m for lot in self.believed()
        )

    def believed(self) -> list[BarrierSighting]:
        """The single best-supported location, if any has cleared the threshold.

        THERE IS ONE PARKING LOT. Taking every cluster over the threshold
        instead was measured on the four 2026-09-14 rounds and is far too
        expensive: the belief settled on SIX locations per counter-clockwise
        run, and six 0.30 m bubbles refused 42.2% of real pillars on
        run_20260914_214824 -- a worse failure than the one being fixed.

        The scatter is not noise to be averaged away either; it is the pinhole
        range under-reading (see ``_detection_to_world``: RANGE_SCALE ships at
        1.0 deliberately, and the estimate is short by roughly 2x). So the
        clusters are real disagreements about depth along one bearing, and the
        rulebook is the only thing that can arbitrate them: keep the one with
        the most evidence behind it.
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

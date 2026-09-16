"""The magenta barrier belief: what it suppresses, and what it must not."""

from __future__ import annotations

import pytest

from src.navigation.planning.barrier_belief import BarrierBelief


def _belief(min_sightings: int = 3) -> BarrierBelief:
    return BarrierBelief(min_sightings=min_sightings, merge_radius_m=0.35, suppression_radius_m=0.30)


class TestEvidenceThreshold:
    def test_one_sighting_suppresses_nothing(self):
        """A single stray magenta box must not blank out a real pillar.

        The asymmetry is deliberate: wrongly suppressing a pillar costs a
        missed route, wrongly believing one costs the wall-pass that ends a
        round -- but a lone detection is not evidence of either.
        """
        belief = _belief()
        belief.observe(1.0, 1.0)

        assert not belief.suppresses(1.0, 1.0)

    def test_suppresses_once_the_threshold_is_reached(self):
        belief = _belief(min_sightings=3)
        for _ in range(3):
            belief.observe(1.0, 1.0)

        assert belief.suppresses(1.0, 1.0)

    def test_zero_min_sightings_disables_the_belief(self):
        """The documented off switch: nothing is ever suppressed."""
        belief = BarrierBelief(min_sightings=0, merge_radius_m=0.35, suppression_radius_m=0.30)
        for _ in range(50):
            belief.observe(1.0, 1.0)

        assert not belief.enabled
        assert not belief.suppresses(1.0, 1.0)
        assert belief.believed() == []


class TestItIsABeliefAboutAPlace:
    def test_a_pillar_well_clear_of_the_lot_survives(self):
        belief = _belief()
        for _ in range(10):
            belief.observe(1.0, 1.0)

        assert not belief.suppresses(2.0, 1.0)

    @pytest.mark.parametrize("offset", [0.0, 0.1, 0.29])
    def test_reds_inside_the_suppression_radius_are_refused(self, offset: float):
        belief = _belief()
        for _ in range(5):
            belief.observe(1.0, 1.0)

        assert belief.suppresses(1.0 + offset, 1.0)

    def test_just_outside_the_radius_survives(self):
        belief = _belief()
        for _ in range(5):
            belief.observe(1.0, 1.0)

        assert not belief.suppresses(1.0 + 0.31, 1.0)


class TestSightingsMerge:
    def test_nearby_sightings_are_one_barrier_not_many(self):
        belief = _belief()
        for dx in (0.0, 0.05, -0.05, 0.1):
            belief.observe(1.0 + dx, 1.0)

        believed = belief.believed()
        assert len(believed) == 1
        assert believed[0].sightings == 4

    def test_the_centroid_tracks_the_sightings(self):
        belief = _belief(min_sightings=1)
        belief.observe(1.0, 1.0)
        belief.observe(1.2, 1.0)

        believed = belief.believed()
        assert len(believed) == 1
        assert believed[0].x_m == pytest.approx(1.1)

    def test_only_the_best_supported_lot_is_believed(self):
        """THERE IS ONE PARKING LOT, and the evidence count arbitrates.

        Distant sightings are tracked apart -- the pinhole range under-reads by
        roughly 2x, so clusters along one bearing are a systematic disagreement
        about depth rather than noise to average -- but only the one with the
        most evidence is acted on. Believing every cluster over the threshold
        settled on SIX lots per counter-clockwise round on 2026-09-14 and
        refused 42.2% of real pillars.
        """
        belief = _belief(min_sightings=1)
        for _ in range(5):
            belief.observe(1.0, 1.0)
        belief.observe(2.5, 1.0)

        believed = belief.believed()
        assert len(believed) == 1
        assert believed[0].x_m == pytest.approx(1.0)
        assert not belief.suppresses(2.5, 1.0)


class TestTheLotIsASpanNotAPoint:
    """The rulebook lot is two fins 0.45 m apart, not a dot with a bubble.

    Measured on two hardware wedges hours apart at the same place: the true
    fins sit at x = 0.94 (14,403 LIDAR returns) and 1.42, a 0.48 m span centred
    on 1.18, while the belief placed its centroid at 1.30. The WEST fin then
    lands 0.36 m from that centroid -- outside the 0.30 m radius -- so reds on
    it were never suppressed and the chassis spent 70 s and 172 s fighting it
    while the planner routed around a phantom 0.45 m away.

    Wall-shaped reds on that fin: suppressed 1/14 and 2/19 by the point model,
    10/14 and 13/19 by the span.
    """

    def _belief(self, *, span: bool) -> BarrierBelief:
        b = BarrierBelief(
            min_sightings=1, merge_radius_m=0.35, suppression_radius_m=0.30, span_along_wall=span
        )
        b.observe(1.30, 0.14)  # the centroid the belief actually settled on
        return b

    def test_off_is_bit_identical(self):
        """The control: the flag off must reproduce the point model exactly."""
        assert self._belief(span=False).suppresses(1.30, 0.14) is True
        assert self._belief(span=False).suppresses(0.94, 0.16) is False

    def test_the_west_fin_is_reached(self):
        """REACHABILITY, and it is the exact hardware failure."""
        assert self._belief(span=True).suppresses(0.94, 0.16) is True

    def test_it_does_not_widen_across_the_wall(self):
        """The whole point of extending along the wall and not around it.

        A legal pillar stands on a division line 0.4-0.6 m out; widening the
        radius uniformly to reach the fin would refuse those, which is the cost
        this shape exists to avoid.
        """
        assert self._belief(span=True).suppresses(1.30, 0.55) is False

    def test_a_west_wall_lot_spans_the_other_axis(self):
        """The span follows the nearest wall, mirroring parking_lot_from_in_bay_start."""
        b = BarrierBelief(
            min_sightings=1, merge_radius_m=0.35, suppression_radius_m=0.30, span_along_wall=True
        )
        b.observe(0.16, 1.30)

        assert b.suppresses(0.16, 0.94) is True, "along the west wall, the span runs in Y"
        assert b.suppresses(0.55, 1.30) is False, "and still does not widen across it"

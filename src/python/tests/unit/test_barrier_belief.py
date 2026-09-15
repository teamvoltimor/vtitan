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

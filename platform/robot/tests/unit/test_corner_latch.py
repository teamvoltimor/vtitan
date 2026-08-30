"""The corner preview must survive its own decay through the arc."""

from __future__ import annotations

import math

import pytest

from src.navigation.core_navigator.corner_latch import CornerLatch

_THRESHOLD = 0.35
"""Matches pursuit.CORNER_TURN_THRESHOLD_RAD; the caller supplies it."""


class TestPassthrough:
    """With no corner in play the latch must be invisible."""

    def test_a_straight_returns_the_raw_reading(self) -> None:
        latch = CornerLatch()
        for _ in range(20):
            assert latch.update(0.0, 0.0, _THRESHOLD) == 0.0
        assert not latch.is_latched

    def test_below_threshold_never_arms(self) -> None:
        """0.197 rad is what a NARROW corridor's corner actually previewed.

        Measured on run_20260830_013702. It is below the shipped threshold, so
        the latch must not arm -- holding a signal that never armed would turn
        a threshold question into a latch question and hide the real issue.
        """
        latch = CornerLatch()
        for _ in range(10):
            assert latch.update(0.197, 0.0, _THRESHOLD) == pytest.approx(0.197)
        assert not latch.is_latched


class TestHoldsThroughTheArc:
    def test_the_preview_survives_decaying_to_zero(self) -> None:
        """The failure this exists for: armed on approach, decayed mid-corner.

        Sequence from run_20260830_014612's first corner (west -> south, 6.1 s),
        where the raw signal ran 1.373 -> 0.980 -> 0.590 -> 0.197 -> 0.000 and
        the lookahead went long two seconds BEFORE the corner.
        """
        latch = CornerLatch()
        assert latch.update(1.373, 0.0, _THRESHOLD) == pytest.approx(1.373)
        assert latch.update(0.980, 0.05, _THRESHOLD) == pytest.approx(0.980)
        assert latch.update(0.590, 0.10, _THRESHOLD) == pytest.approx(0.590)
        # Below threshold now, but the chassis has barely turned.
        assert latch.update(0.197, 0.15, _THRESHOLD) == pytest.approx(1.373)
        assert latch.update(0.000, 0.30, _THRESHOLD) == pytest.approx(1.373)
        assert latch.is_latched

    def test_releases_once_the_turn_has_been_driven(self) -> None:
        latch = CornerLatch()
        latch.update(1.0, 0.0, _THRESHOLD)
        assert latch.update(0.0, 0.5, _THRESHOLD) == pytest.approx(1.0), "released half way round"
        # 0.8 of the previewed 1.0 rad is the completion test.
        assert latch.update(0.0, 0.85, _THRESHOLD) == pytest.approx(1.0), "release tick still reports held"
        assert not latch.is_latched
        assert latch.update(0.0, 0.90, _THRESHOLD) == 0.0, "still held after the turn was driven"

    def test_holds_the_largest_preview_not_the_last(self) -> None:
        """A preview still growing when it arms must not be pinned low."""
        latch = CornerLatch()
        latch.update(0.40, 0.0, _THRESHOLD)
        latch.update(1.20, 0.02, _THRESHOLD)
        latch.update(0.50, 0.04, _THRESHOLD)
        # Turned 0.5 rad: past 0.8*0.5 but nowhere near 0.8*1.2.
        assert latch.update(0.0, 0.50, _THRESHOLD) == pytest.approx(1.20)

    def test_wrapping_past_pi_still_measures_the_turn(self) -> None:
        """Yaw is wrapped, so a corner straddling +-pi must not read as zero."""
        latch = CornerLatch()
        latch.update(1.0, 3.0, _THRESHOLD)
        held = latch.update(0.0, -3.0, _THRESHOLD)
        turned = abs(math.atan2(math.sin(-3.0 - 3.0), math.cos(-3.0 - 3.0)))
        assert turned == pytest.approx(0.2832, abs=1e-3), "sanity: this is a small turn, not 6 rad"
        assert held == pytest.approx(1.0), "released on a wrap artefact"


class TestSafety:
    def test_a_spinning_robot_releases_within_one_revolution(self) -> None:
        """A failed escape can spin; |yaw - yaw_at_arm| alone is not monotone.

        Without the accumulated-yaw backstop a spin re-satisfies the release
        test only periodically, so the latch could hold for a long time.
        """
        latch = CornerLatch()
        latch.update(6.0, 0.0, _THRESHOLD)  # previewed turn larger than any real corner
        yaw = 0.0
        for _ in range(200):
            yaw = math.atan2(math.sin(yaw + 0.1), math.cos(yaw + 0.1))
            latch.update(0.0, yaw, _THRESHOLD)
            if not latch.is_latched:
                break
        assert not latch.is_latched, "held past a full revolution"

    def test_reset_clears_a_corner_in_progress(self) -> None:
        latch = CornerLatch()
        latch.update(1.0, 0.0, _THRESHOLD)
        assert latch.is_latched
        latch.reset()
        assert not latch.is_latched
        assert latch.update(0.0, 0.0, _THRESHOLD) == 0.0

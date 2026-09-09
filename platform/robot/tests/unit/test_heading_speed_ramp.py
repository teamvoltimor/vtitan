"""The heading limiter's shape: a step by default, a ramp when asked.

``heading_speed`` was two-valued -- ``fast_mps()`` below ``heading.CRAWL`` and
``creep_mps()`` at or above it -- a 0.348 m/s change across ONE degree of
heading error. Nothing in the physics it models is discontinuous: the servo's
slew time grows smoothly with the angle it has to cover.

``CRAWL_RAMP_START`` interpolates between the two. It ships at 0, which must
reproduce the step EXACTLY, because the evidence for changing the shape is an
open-loop replay of recorded heading errors and cannot see that driving faster
through a corner changes the error the law then reads. The mechanism ships
inert; the band is chosen on the track.
"""

from __future__ import annotations

import math

import pytest

from src.config.tuning_helpers import tuning_with_overrides


def _heading_speed(abs_error: float, tuning) -> float:
    """The limiter, as ``CoreNavigator._select_speed`` computes it."""
    crawl = tuning.heading.CRAWL
    start = tuning.heading.CRAWL_RAMP_START
    fast, creep = tuning.speed.fast_mps(), tuning.speed.creep_mps()
    if abs_error >= crawl:
        return creep
    if 0.0 < start < crawl and abs_error > start:
        return fast + (abs_error - start) / (crawl - start) * (creep - fast)
    return fast


def test_the_shipped_default_is_still_a_step() -> None:
    """0 must be bit-identical to the two-valued cut, or this is a silent change."""
    tuning = tuning_with_overrides({})
    assert tuning.heading.CRAWL_RAMP_START == 0.0
    fast, creep = tuning.speed.fast_mps(), tuning.speed.creep_mps()
    crawl = tuning.heading.CRAWL
    for err in (0.0, crawl * 0.5, crawl - 1e-9):
        assert _heading_speed(err, tuning) == fast
    for err in (crawl, crawl + 0.5, math.pi):
        assert _heading_speed(err, tuning) == creep


def test_a_ramp_removes_the_cliff_without_moving_its_endpoints() -> None:
    """The ramp must be continuous, monotone, and agree with the step at both ends."""
    tuning = tuning_with_overrides({"CRAWL": 1.0, "CRAWL_RAMP_START": 0.5}, group="heading")
    fast, creep = tuning.speed.fast_mps(), tuning.speed.creep_mps()

    assert _heading_speed(0.4, tuning) == fast, "below the ramp start nothing is taxed"
    assert _heading_speed(1.0, tuning) == creep, "at CRAWL the floor is unchanged"

    samples = [_heading_speed(e / 200.0, tuning) for e in range(0, 241)]
    assert all(b <= a + 1e-12 for a, b in zip(samples, samples[1:])), "speed must not rise with error"
    worst = max(abs(a - b) for a, b in zip(samples, samples[1:]))
    step_cliff = fast - creep
    assert worst < step_cliff / 10.0, f"largest jump {worst:.4f} is still a cliff"


def test_the_midpoint_of_the_ramp_is_the_midpoint_of_the_speeds() -> None:
    """Linear, not eased -- so a band chosen from the replay table means what it says."""
    tuning = tuning_with_overrides({"CRAWL": 1.0, "CRAWL_RAMP_START": 0.4}, group="heading")
    fast, creep = tuning.speed.fast_mps(), tuning.speed.creep_mps()
    assert _heading_speed(0.7, tuning) == pytest.approx((fast + creep) / 2.0)


def test_a_ramp_start_at_or_above_crawl_falls_back_to_the_step() -> None:
    """A misconfigured band must degrade to the shipped behaviour, not divide by zero."""
    tuning = tuning_with_overrides({"CRAWL": 1.0, "CRAWL_RAMP_START": 1.0}, group="heading")
    assert _heading_speed(0.99, tuning) == tuning.speed.fast_mps()
    assert _heading_speed(1.0, tuning) == tuning.speed.creep_mps()

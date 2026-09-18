"""The bay-exit hand-over must not lay a plan around a pose that explains nothing.

The hand-over out of the pocket re-anchors the plan to the pose of that tick
(adr:0060-bay-exit-clearance-guard). That fixed a measured failure -- two rounds
whose steer target sat 121-165 degrees BEHIND the chassis both turned around and
died inside 30 s -- but it trusts the pose at the one moment in a round when
trust is least deserved, since the pocket is where the localizer is worst.

MEASURED on run_20260915_215743, the first session to carry the re-anchor: fit
cost 0.0454 at hand-over against 0.0120-0.0127 on the four sibling rounds, the
plan anchored at waypoint 41 instead of 5, and the chassis drove the wrong way
round. `relocalization_count` stayed 0 all round, because Obstacles fixes all
four corridors at 1000 mm and `relocalize_min_width_spread_m` refuses the global
search that would have rescued it.

Over all 38 recorded in-bay rounds the gate fires on 6, of which 3 drove the
wrong way, against 2 of the 32 below threshold (odds 15.0, Fisher p = 0.0207).
Predictive, not decisive -- see `pose_is_worth_anchoring` for both error
directions.

These tests cover the decision only. It is deliberately a module-level pure
function so that it does not need a ROS node to test, the same reason
`pose_at_time` is one.
"""

from __future__ import annotations

import pytest
from shared.config.navigation_tuning import LocalizationParams, shipped_group

from src.ros2.navigation.track_navigator_node import pose_is_worth_anchoring

_SHIPPED = shipped_group(LocalizationParams).relocalize_cost_threshold


class TestPoseIsWorthAnchoring:
    def test_the_measured_healthy_costs_are_all_accepted(self) -> None:
        """The four sibling rounds of 215743, by their real fit costs."""
        for fit in (0.0122, 0.0120, 0.0127, 0.0123):
            assert pose_is_worth_anchoring(fit, _SHIPPED), f"{fit} is a healthy round"

    def test_the_measured_bad_cost_is_refused(self) -> None:
        """run_20260915_215743, the round that drove the wrong way round."""
        assert not pose_is_worth_anchoring(0.0454, _SHIPPED)

    def test_the_shipped_threshold_sits_between_the_two_populations(self) -> None:
        """The threshold is inherited, not fitted, and this pins that.

        If a future change moves ``relocalize_cost_threshold`` for its own
        reasons, it must not silently walk past either measured population; the
        two assertions above would then be arbitrary.
        """
        assert 0.0127 < _SHIPPED < 0.0454

    def test_a_missing_fit_cost_is_trusted(self) -> None:
        """Before the first scan there is nothing to refuse it with.

        Refusing on ``None`` would silently disable the re-anchor on every round
        whose hand-over lands on a tick with no fresh fix, which is the opposite
        of what this gate is for.
        """
        assert pose_is_worth_anchoring(None, _SHIPPED)

    def test_the_boundary_is_inclusive(self) -> None:
        """Equal to the threshold counts as explaining the scan.

        Stated as a test because the two orderings differ only at the boundary
        and nothing else in the call chain would reveal which was chosen.
        """
        assert pose_is_worth_anchoring(_SHIPPED, _SHIPPED)
        assert not pose_is_worth_anchoring(_SHIPPED + 1e-9, _SHIPPED)

    @pytest.mark.parametrize("threshold", [0.0, -1.0])
    def test_a_zero_or_negative_threshold_refuses_every_real_cost(self, threshold: float) -> None:
        """No hidden "0 disables the gate" convention.

        Several knobs in this repo use 0.0 to mean inert, so the absence of that
        convention here is worth pinning: the fit cost is a squared residual and
        is never negative, so a 0.0 threshold refuses everything a scan can
        produce. A caller wanting the gate off must not reach for 0.0.
        """
        assert not pose_is_worth_anchoring(0.0454, threshold)
        assert not pose_is_worth_anchoring(0.0122, threshold)
        # ... but a missing cost still passes, since that branch never compares.
        assert pose_is_worth_anchoring(None, threshold)

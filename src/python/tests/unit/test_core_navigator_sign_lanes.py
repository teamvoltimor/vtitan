"""The sign-lane planner must rewrite the Obstacles path and no other.

``sign_lane`` itself is pinned by ``test_sign_lane.py``. What is pinned HERE is
the wiring: that the rebuild fires for an Obstacles run, that it cannot fire
for an Open Challenge one (which builds no ``SignRouter`` at all -- see
``track_navigator_node._build_sign_router``), and that the rebuild never
re-lanes an already-laned path, which would compound the offset every time
blind discovery refined a sign by a centimetre.
"""

from __future__ import annotations

import math
from dataclasses import replace

from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section
from shared.domain.models import Pose, SignColor, Waypoint

from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.sign_router import SignRouter, SignSpec
from src.navigation.race_tracker import LapDetector
from tests.fixtures import FakeGateway

# SOUTH corridor centreline, spanning the straight the lane may rewrite.
_WAYPOINTS = [Waypoint(1.0 + 0.1 * i, 0.5) for i in range(11)]
_ROBOT_POSE = Pose(x=1.0, y=0.5, yaw=0.0)
_SIGN = SignSpec(x=1.5, y=0.5, color=SignColor.RED)


def _tuning(*, planner: bool) -> NavigationTuning:
    base = NavigationTuning.load_default()
    return replace(base, sign_router=base.sign_router.model_copy(update={"SIGN_LANE_PLANNER": planner}))


def _navigator(tuning: NavigationTuning, *, with_sign_router: bool) -> CoreNavigator:
    gateway = FakeGateway(_ROBOT_POSE)
    router = (
        SignRouter([_SIGN], direction=Direction.COUNTERCLOCKWISE, tuning=tuning) if with_sign_router else None
    )
    nav = CoreNavigator(
        gateway=gateway,
        waypoints=_WAYPOINTS,
        num_laps=3,
        tuning=tuning,
        sign_router=router,
        lap_detector=LapDetector(
            start_pos=_WAYPOINTS[0],
            start_section=Section.SOUTH,
            direction=Direction.COUNTERCLOCKWISE,
        ),
    )
    nav._current_corridor = Section.SOUTH
    return nav


def _max_lateral_shift(nav: CoreNavigator) -> float:
    return max(abs(a.y - b.y) for a, b in zip(nav._waypoints, _WAYPOINTS, strict=True))


class TestSignLaneWiring:
    def test_obstacles_run_relanes_the_path(self) -> None:
        nav = _navigator(_tuning(planner=True), with_sign_router=True)

        nav.step()

        # Red in a SOUTH corridor passes OUTWARD, i.e. toward lower y.
        assert _max_lateral_shift(nav) > 0.0
        assert min(wp.y for wp in nav._waypoints) < _SIGN.y

    def test_open_challenge_path_is_untouched_even_with_the_flag_on(self) -> None:
        """No ``SignRouter`` means no lane, regardless of tuning.

        The guarantee this test exists for is structural, not numeric: Open
        Challenge shares this navigator, and its path must come out of a tick
        identical to the one that went in.
        """
        nav = _navigator(_tuning(planner=True), with_sign_router=False)

        nav.step()

        assert nav._waypoints == _WAYPOINTS

    def test_flag_off_leaves_the_path_alone(self) -> None:
        """Regression guard for the toggle: without it the ON case proves nothing."""
        nav = _navigator(_tuning(planner=False), with_sign_router=True)

        nav.step()

        assert nav._waypoints == _WAYPOINTS

    def test_repeated_ticks_do_not_compound_the_offset(self) -> None:
        """Lanes are recomputed from the planned centreline, never from the last lane.

        Blind discovery refines sign positions continuously, so this path is
        re-laned many times in a real run. Re-laning an already-laned path
        would add an offset per rebuild and walk the route into a wall.
        """
        nav = _navigator(_tuning(planner=True), with_sign_router=True)

        nav.step()
        after_first = list(nav._waypoints)
        # Force the rebuild to run again rather than short-circuit on the
        # unchanged fingerprint -- the compounding this guards against would
        # only ever show up on a second rebuild.
        nav._lane_fingerprint = None
        nav.step()

        assert nav._waypoints == after_first

    def test_lane_shift_never_exceeds_the_commanded_offset(self) -> None:
        """A sanity bound: the lane is one offset from the sign, not two."""
        nav = _navigator(_tuning(planner=True), with_sign_router=True)

        nav.step()

        assert _max_lateral_shift(nav) < 0.5
        assert not any(math.isnan(wp.y) for wp in nav._waypoints)

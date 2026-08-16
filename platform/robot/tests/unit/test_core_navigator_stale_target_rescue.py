"""A sharply corner-cut waypoint index must not freeze when neither the
"next is closer" test nor the reached-radius test ever fires.

Root-caused under the `wideonly` (85 deg steering) hardware profile,
go_obstacles_0042 on subset64: the chassis cut a corner sharply enough that
its actual heading swung past what the waypoint polyline (spaced for 55 deg
turning) assumed, leaving BOTH the current and next waypoint reading as
farther away every tick. The pass-by rescue in core_navigator.py only
advances on "next is closer than current", so it never fired; the index froze,
select_target_point's own forward search kept returning the same distant
"ahead" point from that stale start, and the chassis clipped the wall still
chasing it.

Obstacles-only by construction: gated on sign_router presence (always None
for Open Challenge) as well as its own toggle, so this must never fire
without both.
"""

from __future__ import annotations

import math
from dataclasses import replace

from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section
from shared.domain.models import Pose, Waypoint

from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.sign_router import SignRouter
from src.navigation.race_tracker import LapDetector
from tests.fixtures import FakeGateway

# Mirrors the traced go_obstacles_0042 geometry: waypoint 0 and 1 both sit
# well behind a chassis that has cut hard across the corner.
_WAYPOINTS = [
    Waypoint(0.847, 0.630),
    Waypoint(0.717, 0.717),
    Waypoint(0.600, 0.900),
    Waypoint(0.500, 1.100),
]
_ROBOT_POSE = Pose(x=2.000, y=0.790, yaw=math.radians(73.0))


def _navigator(tuning: NavigationTuning, *, with_sign_router: bool) -> tuple[CoreNavigator, FakeGateway]:
    gateway = FakeGateway(_ROBOT_POSE)
    sign_router = SignRouter([], direction=Direction.CLOCKWISE, tuning=tuning) if with_sign_router else None
    nav = CoreNavigator(
        gateway=gateway,
        waypoints=_WAYPOINTS,
        num_laps=3,
        tuning=tuning,
        sign_router=sign_router,
        lap_detector=LapDetector(
            start_pos=_WAYPOINTS[0],
            start_section=Section.EAST,
            direction=Direction.CLOCKWISE,
        ),
    )
    nav._current_corridor = Section.EAST
    return nav, gateway


class TestStaleTargetRescue:
    def test_rescue_on_advances_past_a_behind_waypoint_neither_distance_test_catches(self):
        tuning = replace(
            NavigationTuning.load_default(),
            sign_router=NavigationTuning.load_default().sign_router.model_copy(
                update={"STALE_TARGET_RESCUE": True},
            ),
        )
        nav, _ = _navigator(tuning, with_sign_router=True)

        # Confirm the premise: neither waypoint 0 nor 1 is closer than the
        # other from the chassis's position -- the "next is closer" test
        # alone would never advance past this.
        d0 = math.dist((_WAYPOINTS[0].x, _WAYPOINTS[0].y), (_ROBOT_POSE.x, _ROBOT_POSE.y))
        d1 = math.dist((_WAYPOINTS[1].x, _WAYPOINTS[1].y), (_ROBOT_POSE.x, _ROBOT_POSE.y))
        assert d1 >= d0, "fixture must reproduce the neither-test-fires premise"

        nav.step()

        assert nav._waypoint_index > 0, "must advance past a waypoint reading as behind the chassis"

    def test_rescue_off_leaves_the_index_frozen(self):
        """Regression guard for the toggle itself: STALE_TARGET_RESCUE=False
        must reproduce the original freeze, or the ON case above is not
        measuring what it claims to."""
        tuning = NavigationTuning.load_default()
        nav, _ = _navigator(tuning, with_sign_router=True)

        nav.step()

        assert nav._waypoint_index == 0

    def test_rescue_never_fires_without_a_sign_router_even_if_the_toggle_is_on(self):
        """Open Challenge never builds a sign_router (see
        track_navigator_node._build_sign_router). The rescue must stay
        inert in that configuration regardless of the tuning flag, so an
        Obstacles-only feature can never leak into Open Challenge's
        waypoint-advance behaviour."""
        tuning = replace(
            NavigationTuning.load_default(),
            sign_router=NavigationTuning.load_default().sign_router.model_copy(
                update={"STALE_TARGET_RESCUE": True},
            ),
        )
        nav, _ = _navigator(tuning, with_sign_router=False)

        nav.step()

        assert nav._waypoint_index == 0

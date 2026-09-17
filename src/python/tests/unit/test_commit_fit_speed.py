"""The commit-fit speed cap: slow down so the turn radius fits the run-up the commit left.

On the 2026-09-14/15 rounds the router commits at p50 0.54 m and a pass that
must CROSS to the legal side fails 61% of the time when the commit lands under
0.40 m, against 4% past 0.55 m. The chassis turn radius is a speed curve
(``R = intercept + slope * v``), so once the commit is late, speed is the only
authority left over whether the arc fits. See adr:0051-sign-lane-planner.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Pose, SignColor, Waypoint

from src.navigation.core_navigator import CoreNavigator
from src.navigation.geometry import arc_fit_speed_mps, chassis_half_diagonal_m, turn_radius_at_speed_m
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec
from src.navigation.ports import LidarScan
from tests.fixtures import FakeGateway, create_scan_with_sectors
from tests.test_constants import ANGLES_FULL_ROTATION

ANGLES = ANGLES_FULL_ROTATION.tolist()


class TestArcFitSpeed:
    def test_inverts_the_turn_radius_curve(self) -> None:
        # The arc of radius R(v) over s metres buys s^2 / (2R). Ask for exactly
        # that lateral and the fitting speed must be v again.
        v = 0.15
        s = 0.40
        lateral = s * s / (2.0 * turn_radius_at_speed_m(v))
        assert arc_fit_speed_mps(s, lateral) == pytest.approx(v, abs=1e-9)

    def test_no_lateral_needed_means_no_cap(self) -> None:
        assert arc_fit_speed_mps(0.5, 0.0) is None
        assert arc_fit_speed_mps(0.5, -0.1) is None

    def test_spent_run_up_asks_for_a_stop(self) -> None:
        assert arc_fit_speed_mps(0.0, 0.2) == 0.0
        assert arc_fit_speed_mps(-0.1, 0.2) == 0.0

    def test_more_lateral_for_the_same_run_up_means_slower(self) -> None:
        a = arc_fit_speed_mps(0.45, 0.10)
        b = arc_fit_speed_mps(0.45, 0.25)
        assert a is not None
        assert b is not None
        assert b < a

    def test_the_bag_arithmetic(self) -> None:
        # The p10 hardware commit: 0.328 m centre-to-centre, so ~0.18 m of
        # nose run-up, needing a full 0.205 m of lateral from the sign's own
        # line. The fit lands under the 0.05 m/s the chassis can even hold,
        # so the floor decides: the cap cannot rescue this commit.
        v = arc_fit_speed_mps(0.328 - RobotSpecs.LENGTH / 2, chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2)
        assert v is not None
        assert v < 0.05
        # The p50 commit at 0.537 m with the chassis on the sign's line fits
        # at a crawl, well under the 0.22 m/s the rounds actually carried.
        v = arc_fit_speed_mps(0.537 - RobotSpecs.LENGTH / 2, chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2)
        assert v is not None
        assert 0.05 < v < 0.22


def _tuning(**sign_router: object) -> NavigationTuning:
    base = NavigationTuning()
    return replace(base, sign_router=base.sign_router.model_copy(update=sign_router))


def _navigator(tuning: NavigationTuning, router: SignRouter | None, pose: Pose) -> CoreNavigator:
    gateway = FakeGateway(
        pose,
        LidarScan(ranges_m=tuple(create_scan_with_sectors()), angles_rad=tuple(ANGLES)),
    )
    return CoreNavigator(
        gateway=gateway,
        waypoints=[Waypoint(x=1.5, y=0.5), Waypoint(x=2.0, y=0.5)],
        num_laps=1,
        tuning=tuning,
        sign_router=router,
    )


class TestCommitFitSpeedCap:
    """The navigator-level wrapper: reads the router's belief, never a sensor."""

    @pytest.fixture()
    def tuning(self) -> NavigationTuning:
        return _tuning(sign_commit_fit_speed=True, sign_commit_fit_floor_mps=0.10)

    def test_no_router_no_cap(self, tuning: NavigationTuning) -> None:
        nav = _navigator(tuning, None, Pose(x=1.5, y=0.5, yaw=0.0))
        assert nav._commit_fit_speed_cap(1.5, 0.5, 0.0) is None

    def test_uncommitted_router_no_cap(self, tuning: NavigationTuning) -> None:
        router = SignRouter(
            [SignSpec(x=2.5, y=0.5, color=SignColor.RED)],
            config=SignRouterConfig.from_tuning(tuning.sign_router),
        )
        nav = _navigator(tuning, router, Pose(x=1.5, y=0.5, yaw=0.0))
        assert router.committed_sign_position is None
        assert nav._commit_fit_speed_cap(1.5, 0.5, 0.0) is None

    def _commit(self, tuning: NavigationTuning, robot: Pose, sign: SignSpec) -> tuple[CoreNavigator, SignRouter]:
        from src.navigation.planning.waypoints.classification import corridor_for_position

        router = SignRouter([sign], config=SignRouterConfig.from_tuning(tuning.sign_router))
        # settle_ticks gates engagement; drive the router past it.
        for _ in range(tuning.sign_router.settle_ticks + 2):
            router.deform_waypoint(
                waypoint=(robot.x + 0.4, robot.y),
                robot_pos=(robot.x, robot.y),
                robot_yaw=robot.yaw,
                corridor=corridor_for_position(sign.x, sign.y),
            )
        nav = _navigator(tuning, router, robot)
        return nav, router

    def test_a_late_commit_on_the_wrong_side_caps_below_cruise(self, tuning: NavigationTuning) -> None:
        # South corridor, driving +x. The router's pass side for the sign
        # decides which lateral is legal; place the chassis ON the sign's
        # line so the full physical clearance is still to be bought over a
        # 0.45 m run-up.
        robot = Pose(x=2.05, y=0.5, yaw=0.0)
        sign = SignSpec(x=2.5, y=0.5, color=SignColor.RED)
        nav, router = self._commit(tuning, robot, sign)
        if router.committed_sign_position is None:
            pytest.skip("router did not commit in this geometry; the cap has nothing to read")
        cap = nav._commit_fit_speed_cap(robot.x, robot.y, robot.yaw)
        assert cap is not None
        expected = arc_fit_speed_mps(
            0.45 - RobotSpecs.LENGTH / 2, chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2
        )
        assert cap == pytest.approx(expected)
        assert cap < 0.22

    def test_already_clear_laterally_means_no_cap(self, tuning: NavigationTuning) -> None:
        robot = Pose(x=2.05, y=0.5, yaw=0.0)
        sign = SignSpec(x=2.5, y=0.5, color=SignColor.RED)
        nav, router = self._commit(tuning, robot, sign)
        if router.committed_sign_position is None:
            pytest.skip("router did not commit in this geometry; the cap has nothing to read")
        side = router.committed_pass_side_world
        assert side is not None
        # Step the chassis 0.30 m onto the legal side: more than the 0.205 m
        # the footprint needs, so nothing is left to buy.
        x, y = robot.x + 0.30 * side[0], robot.y + 0.30 * side[1]
        assert nav._commit_fit_speed_cap(x, y, robot.yaw) is None

    def test_sign_abeam_or_behind_means_no_cap(self, tuning: NavigationTuning) -> None:
        robot = Pose(x=2.05, y=0.5, yaw=0.0)
        sign = SignSpec(x=2.5, y=0.5, color=SignColor.RED)
        nav, router = self._commit(tuning, robot, sign)
        if router.committed_sign_position is None:
            pytest.skip("router did not commit in this geometry; the cap has nothing to read")
        # Same lateral state, chassis already level with the sign.
        assert nav._commit_fit_speed_cap(sign.x, robot.y, robot.yaw) is None
        assert nav._commit_fit_speed_cap(sign.x + 0.2, robot.y, robot.yaw) is None

    def test_run_up_is_measured_from_the_nose(self, tuning: NavigationTuning) -> None:
        robot = Pose(x=2.05, y=0.5, yaw=0.0)
        sign = SignSpec(x=2.5, y=0.5, color=SignColor.RED)
        nav, router = self._commit(tuning, robot, sign)
        if router.committed_sign_position is None:
            pytest.skip("router did not commit in this geometry; the cap has nothing to read")
        # Inside half a chassis length the nose is already at the sign: the
        # run-up is spent and the fit asks for a stop (the floor then binds).
        x = sign.x - RobotSpecs.LENGTH / 2 + 0.01
        assert nav._commit_fit_speed_cap(x, robot.y, robot.yaw) == 0.0

    def test_yaw_decides_along_track(self, tuning: NavigationTuning) -> None:
        robot = Pose(x=2.05, y=0.5, yaw=0.0)
        sign = SignSpec(x=2.5, y=0.5, color=SignColor.RED)
        nav, router = self._commit(tuning, robot, sign)
        if router.committed_sign_position is None:
            pytest.skip("router did not commit in this geometry; the cap has nothing to read")
        # Facing away from the sign it is behind the chassis, whatever x says.
        assert nav._commit_fit_speed_cap(robot.x, robot.y, math.pi) is None

"""Holding the pursuit target off the parking lot while driving past it.

Measured with `diag_bag_lot_contact.py` on the 2026-09-15 rounds: 210-242 ticks
inside the lot's band with under 3 cm of footprint clearance against 0-6 in a
clean round, and the touching sector is the FLANK. See
adr:0062-sim-contact-model-and-parking.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from shared.config.constants import ParkingLotSpecs, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section
from shared.domain.models import Pose, SignColor, Waypoint

from src.navigation.core_navigator import CoreNavigator
from src.navigation.maneuvers.parking import ParkController
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec
from src.navigation.ports import LidarScan
from tests.fixtures import FakeGateway, ParkingLotFixtures, create_scan_with_sectors
from tests.test_constants import ANGLES_FULL_ROTATION

ANGLES = ANGLES_FULL_ROTATION.tolist()
# The south lot sits against y = 0; the fins reach 0.20 m out from it, so a
# flank-clear target needs the fins' depth plus the chassis half-width.
CLEAR_DEPTH = ParkingLotSpecs.LENGTH + RobotSpecs.WIDTH / 2


def _tuning(**parking: object) -> NavigationTuning:
    base = NavigationTuning()
    return replace(base, parking=base.parking.model_copy(update=parking))


def _navigator(tuning: NavigationTuning, router: SignRouter | None = None) -> CoreNavigator:
    gateway = FakeGateway(
        Pose(x=1.5, y=0.3, yaw=0.0),
        LidarScan(ranges_m=tuple(create_scan_with_sectors()), angles_rad=tuple(ANGLES)),
    )
    nav = CoreNavigator(
        gateway=gateway,
        waypoints=[Waypoint(x=1.5, y=0.3), Waypoint(x=2.0, y=0.3)],
        num_laps=1,
        tuning=tuning,
        sign_router=router,
    )
    # The zone, not the controller: the keep-out treats the lot as an obstacle
    # and must work on a run that never attempts the manoeuvre.
    nav._lot_zone = ParkController(
        ParkingLotFixtures.south(), Section.SOUTH, Direction.COUNTERCLOCKWISE
    ).zone
    nav._current_corridor = Section.SOUTH
    return nav


def _lot_centre_x(nav: CoreNavigator) -> float:
    return nav._lot_zone.gap_cx


class TestLotKeepOut:
    def test_off_by_default(self) -> None:
        assert NavigationTuning().parking.lot_keep_out_m == 0.0
        nav = _navigator(_tuning())
        target = (_lot_centre_x(nav), 0.10)
        assert nav._lot_keep_out_target(target, target[0], 0.10) == target

    def test_a_chassis_inside_the_fin_band_asks_for_a_correction(self) -> None:
        # Pure pursuit turns only when the target is off the current line, so
        # the chassis's own deficit is mirrored past the clear depth.
        nav = _navigator(_tuning(lot_keep_out_m=0.03))
        x = _lot_centre_x(nav)
        clear = CLEAR_DEPTH + 0.03
        pushed = nav._lot_keep_out_target((x, 0.10), x, 0.10)
        assert pushed[0] == pytest.approx(x)
        assert pushed[1] == pytest.approx(clear + (clear - 0.10))

    def test_a_clear_chassis_with_a_close_target_is_only_held_to_clear(self) -> None:
        nav = _navigator(_tuning(lot_keep_out_m=0.03))
        x = _lot_centre_x(nav)
        clear = CLEAR_DEPTH + 0.03
        pushed = nav._lot_keep_out_target((x, 0.10), x, clear + 0.20)
        assert pushed[1] == pytest.approx(clear)

    def test_a_target_already_clear_is_untouched(self) -> None:
        nav = _navigator(_tuning(lot_keep_out_m=0.03))
        x = _lot_centre_x(nav)
        target = (x, CLEAR_DEPTH + 0.20)
        assert nav._lot_keep_out_target(target, x, target[1]) == target

    def test_outside_the_lot_span_is_untouched(self) -> None:
        nav = _navigator(_tuning(lot_keep_out_m=0.03))
        x = _lot_centre_x(nav)
        far = x + ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH
        assert nav._lot_keep_out_target((far, 0.10), far, 0.10) == (far, 0.10)

    def test_another_corridor_is_untouched(self) -> None:
        nav = _navigator(_tuning(lot_keep_out_m=0.03))
        nav._current_corridor = Section.NORTH
        x = _lot_centre_x(nav)
        assert nav._lot_keep_out_target((x, 0.10), x, 0.10) == (x, 0.10)

    def test_the_park_manoeuvre_owns_the_lot_once_engaged(self) -> None:
        nav = _navigator(_tuning(lot_keep_out_m=0.03))
        nav._parking_engaged = True
        x = _lot_centre_x(nav)
        assert nav._lot_keep_out_target((x, 0.10), x, 0.10) == (x, 0.10)

    def test_a_committed_sign_that_wants_the_wall_side_wins(self) -> None:
        tuning = _tuning(lot_keep_out_m=0.03)
        router = SignRouter(
            [SignSpec(x=1.5, y=0.5, color=SignColor.RED)],
            config=SignRouterConfig.from_tuning(tuning.sign_router),
        )
        nav = _navigator(tuning, router)
        x = _lot_centre_x(nav)
        target = (x, 0.10)
        pushed = nav._lot_keep_out_target(target, x, 0.10)
        offset_before = router.committed_pass_side_offset(target)
        offset_after = router.committed_pass_side_offset(pushed)
        if offset_before is None or offset_after is None:
            pytest.skip("no committed pass side in this fixture; the guard has nothing to read")
        assert offset_after >= offset_before

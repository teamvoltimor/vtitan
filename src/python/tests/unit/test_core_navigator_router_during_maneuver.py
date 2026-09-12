"""``TICK_ROUTER_DURING_MANEUVER``: keep the sign map fed while an escape runs.

``CoreNavigator.step()`` returns early whenever an escape manoeuvre is latched,
so the sign-router call is skipped for the manoeuvre's whole duration. That
call is not only a steering computation -- it owns the blind discovery ingest,
engage/pass bookkeeping and ``routed_sign_positions``, the list the escape mask
itself reads.

Measured 2026-09-12 on three hardware rounds: 183 manoeuvre episodes covering
22.3% of all ticks, 179 of 183 (97.8%) holding a CONSTANT steering value, up to
44 ticks. For a quarter of a race the wheel runs open-loop AND the map takes in
nothing.

The flag ships OFF, so the first test pins the CURRENT behaviour and would fail
if the default ever flipped silently. The third is the safety property that
makes the flag safe to try at all: feeding the router must not move the wheel.
"""

from __future__ import annotations

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Pose, SignColor, Waypoint

from src.config.tuning_helpers import tuning_with_overrides
from src.navigation.control.controllers import EscapeManeuver, ManeuverType
from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec
from tests.fixtures import FakeGateway
from tests.test_constants import ANGLES_FULL_ROTATION, LIDAR_DEFAULT_FAR, NUM_RAYS

from src.navigation.ports import LidarScan

LATCHED_STEERING = 0.42
"""An arbitrary value no planner would produce, so an assertion that the
command still carries it cannot pass by coincidence."""

LATCHED_SPEED = -0.2
"""Reverse, matching the hardware regime: 99.4% of side_correction ticks."""


def _scan() -> LidarScan:
    return LidarScan(
        ranges_m=tuple([LIDAR_DEFAULT_FAR] * NUM_RAYS),
        angles_rad=tuple(ANGLES_FULL_ROTATION.tolist()),
    )


def _navigator(*, flag: bool) -> tuple[CoreNavigator, FakeGateway, list[int]]:
    """A navigator mid-manoeuvre, with the router's ingest call counted.

    The spy wraps ``deform_waypoint`` rather than inspecting the sign map,
    because the fake gateway publishes no detections: what is under test is
    whether the router is CALLED at all during a manoeuvre, not what it would
    have learned.
    """
    tuning = NavigationTuning.load_default()
    if flag:
        tuning = tuning_with_overrides({"TICK_ROUTER_DURING_MANEUVER": True}, group="escape", base=tuning)

    router = SignRouter(
        [SignSpec(x=2.0, y=0.5, color=SignColor.RED)],
        config=SignRouterConfig.from_tuning(tuning.sign_router),
    )
    calls: list[int] = []
    original = router.deform_waypoint

    def counting(*args, **kwargs):  # noqa: ANN002, ANN003, ANN202
        calls.append(1)
        return original(*args, **kwargs)

    router.deform_waypoint = counting  # type: ignore[method-assign]

    gateway = FakeGateway(Pose(x=1.0, y=0.5, yaw=0.0), _scan())
    nav = CoreNavigator(
        gateway=gateway,
        waypoints=[Waypoint(5.0, 0.5), Waypoint(10.0, 0.5)],
        num_laps=1,
        tuning=tuning,
        sign_router=router,
    )
    # Latch a manoeuvre with enough frames left that this step cannot end it,
    # so the early return is exercised rather than the tick that clears it.
    nav._active_maneuver = EscapeManeuver(
        maneuver_type=ManeuverType.SIDE_CORRECTION,
        steering=LATCHED_STEERING,
        speed=LATCHED_SPEED,
        duration_frames=20,
        priority=1,
    )
    nav._maneuver_frames_left = 20
    return nav, gateway, calls


class TestRouterTickDuringManeuver:
    def test_shipped_default_skips_the_router(self) -> None:
        """OFF (as shipped): a latched manoeuvre starves the router entirely."""
        nav, _gateway, calls = _navigator(flag=False)
        nav.step()
        assert calls == [], "the shipped path must not call the router mid-manoeuvre"

    def test_flag_keeps_feeding_the_router(self) -> None:
        """ON: the router is ticked even though the manoeuvre owns the chassis."""
        nav, _gateway, calls = _navigator(flag=True)
        nav.step()
        assert len(calls) == 1, "the flag must tick the router exactly once per step"

    def test_flag_does_not_move_the_wheel(self) -> None:
        """The safety property: feeding the router must not change the command.

        This is what separates the flag from ``SIDE_CORRECTION_BLENDS``. The
        deformed waypoint is discarded, so the published command must be the
        manoeuvre's own, byte-for-byte identical to the flag-off arm.
        """
        off_nav, off_gw, _ = _navigator(flag=False)
        on_nav, on_gw, _ = _navigator(flag=True)
        off_nav.step()
        on_nav.step()

        assert len(on_gw.commands) == len(off_gw.commands) == 1
        assert on_gw.commands[0] == off_gw.commands[0]
        assert on_gw.commands[0].steering_norm == pytest.approx(LATCHED_STEERING)
        assert on_gw.commands[0].speed_mps == pytest.approx(LATCHED_SPEED)

    def test_flag_still_reports_the_maneuver_phase(self) -> None:
        """Ticking the router must not reclassify the tick as normal driving."""
        nav, _gateway, _calls = _navigator(flag=True)
        nav.step()
        assert nav._debug is not None
        assert nav._debug.active_maneuver_type is ManeuverType.SIDE_CORRECTION

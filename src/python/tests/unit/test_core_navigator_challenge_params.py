"""The per-challenge parameter sets must follow a mid-process challenge switch.

``CoreNavigator`` resolves three things off one discriminator -- is there a
sign router -- and every one of them is an Open/Obstacles choice: the speed
ladder, the clearance zones and the escape params.

``track_navigator_node`` builds the navigator before BOOT_CHECK has published a
challenge and FALLS BACK TO OPEN, then calls ``replace_sign_router`` once the
jumper resolves. Resolving the three only in ``__init__`` therefore left the
first Obstacles race of every fresh process driving the Open configuration --
observed on all nine Obstacles rounds of 2026-09-13/14, where 13,852 driving
ticks commanded an Open tier and none commanded an Obstacles one.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Pose, SignColor, Waypoint

from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec
from src.navigation.ports import LidarScan
from tests.fixtures import FakeGateway, create_scan_with_sectors
from tests.test_constants import ANGLES_FULL_ROTATION

ANGLES = ANGLES_FULL_ROTATION.tolist()

# The shipped base profile declares no per-challenge tiers, so both resolvers
# return the same ladder and the assertions below would hold vacuously. These
# stand in for the drivetrain profiles that do declare them (rev-hd-hex ships
# open_slow_mps 0.26 against a base slow_mps of 0.22).
_OPEN_SLOW = 0.15
_OBSTACLES_SLOW = 0.11


@pytest.fixture
def tuning() -> NavigationTuning:
    base = NavigationTuning()
    # NavigationTuning is a frozen DATACLASS of pydantic groups: the group is
    # revalidated with model_copy, the aggregate replaced with dataclasses
    # .replace. Mixing the two silently returns the base unchanged, which makes
    # every assertion here pass against one shared ladder.
    speed = base.speed.model_copy(
        update={"open_slow_mps": _OPEN_SLOW, "obstacles_slow_mps": _OBSTACLES_SLOW}
    )
    # Pinned here rather than read from the shipped TOML so the test states the
    # split it is asserting: shared off, Obstacles on.
    escape = base.escape.model_copy(
        update={
            "escape_side_follows_committed_sign": False,
            "obstacles_escape_side_follows_committed_sign": True,
        }
    )
    return replace(base, speed=speed, escape=escape)


def _navigator(tuning: NavigationTuning, *, sign_router: SignRouter | None) -> CoreNavigator:
    gateway = FakeGateway(
        Pose(x=1.5, y=0.5, yaw=0.0),
        LidarScan(ranges_m=tuple(create_scan_with_sectors()), angles_rad=tuple(ANGLES)),
    )
    return CoreNavigator(
        gateway=gateway,
        waypoints=[Waypoint(x=1.5, y=0.5), Waypoint(x=2.0, y=0.5)],
        num_laps=1,
        tuning=tuning,
        sign_router=sign_router,
    )


def _router(tuning: NavigationTuning) -> SignRouter:
    return SignRouter(
        [SignSpec(x=2.5, y=0.5, color=SignColor.RED)],
        config=SignRouterConfig.from_tuning(tuning.sign_router),
    )


class TestChallengeParamsFollowTheRouter:
    def test_open_build_then_obstacles_switch_adopts_the_obstacles_ladder(self, tuning):
        """The bug: this used to keep the Open ladder for the whole process."""
        nav = _navigator(tuning, sign_router=None)
        assert nav._speed.slow_mps == pytest.approx(_OPEN_SLOW)

        nav.replace_sign_router(_router(tuning))

        assert nav._speed.slow_mps == pytest.approx(_OBSTACLES_SLOW)

    def test_obstacles_build_then_open_switch_adopts_the_open_ladder(self, tuning):
        nav = _navigator(tuning, sign_router=_router(tuning))
        assert nav._speed.slow_mps == pytest.approx(_OBSTACLES_SLOW)

        nav.replace_sign_router(None)

        assert nav._speed.slow_mps == pytest.approx(_OPEN_SLOW)

    def test_the_collision_controller_is_rebuilt_with_the_new_params(self, tuning):
        """The controllers COPY the params, so re-resolving alone is not enough.

        `escape_side_follows_committed_sign` is the flag that decides which
        side of a pillar the robot comes out of a K-turn on, and the collision
        controller reads its own copy, taken at construction. Re-resolving
        `self._escape` without rebuilding the controller leaves the previous
        challenge's value live on the only path that uses it.
        """
        nav = _navigator(tuning, sign_router=None)
        assert nav._collision_controller.escape_side_follows_committed_sign is False

        nav.replace_sign_router(_router(tuning))

        assert nav._collision_controller.escape_side_follows_committed_sign is True
        assert nav._collision_controller.contact_dist == pytest.approx(
            tuning.clearance.for_obstacles_challenge().contact_dist
        )

    def test_clearance_and_escape_switch_with_the_ladder(self, tuning):
        """All three key on the same discriminator, so all three must move.

        Asserted as identity against the resolvers rather than on a specific
        field, so a future OBSTACLES_* override is covered without editing this.
        """
        nav = _navigator(tuning, sign_router=None)
        nav.replace_sign_router(_router(tuning))

        assert nav._clearance == tuning.clearance.for_obstacles_challenge()
        assert nav._escape == tuning.escape.for_obstacles_challenge()

        nav.replace_sign_router(None)

        assert nav._clearance == tuning.clearance
        assert nav._escape == tuning.escape

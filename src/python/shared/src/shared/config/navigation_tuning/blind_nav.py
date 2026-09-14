"""Blind-navigation tuning groups.

Covers corridor width estimation, corridor following, direction inference,
LIDAR pose search, and odometry/IMU fusion.

Fields are inherited from the generated DTOs under
:mod:`shared.config.generated.navigation.blind_nav`; every field value comes
from the checked-in ``blind_nav/*.toml`` (see ``NavigationTuning``). These
classes add only derived behaviour. The generated field descriptions carry the
measurement history that used to live here.
"""

from __future__ import annotations

from shared.config.generated.navigation.blind_nav.corridor_estimator_schema import (
    NavigationBlindNavCorridorEstimator,
)
from shared.config.generated.navigation.blind_nav.corridor_follower_schema import (
    NavigationBlindNavCorridorFollower,
)
from shared.config.generated.navigation.blind_nav.direction_estimator_schema import (
    NavigationBlindNavDirectionEstimator,
)
from shared.config.generated.navigation.blind_nav.localization_schema import (
    NavigationBlindNavLocalization,
)
from shared.config.generated.navigation.blind_nav.state_estimator_schema import (
    NavigationBlindNavStateEstimator,
)

__all__ = [
    "CorridorEstimatorParams",
    "CorridorFollowerParams",
    "DirectionEstimatorParams",
    "LocalizationParams",
    "StateEstimatorParams",
]


class CorridorEstimatorParams(NavigationBlindNavCorridorEstimator):
    """Blind corridor-width estimation parameters."""


class CorridorFollowerParams(NavigationBlindNavCorridorFollower):
    """Blind corridor-following, corner-turn, and parking-bay-exit parameters."""


class DirectionEstimatorParams(NavigationBlindNavDirectionEstimator):
    """Blind travel-direction inference parameters."""


class LocalizationParams(NavigationBlindNavLocalization):
    """LIDAR-based pose search (LidarLocalizer) parameters."""


class StateEstimatorParams(NavigationBlindNavStateEstimator):
    """Odometry/IMU fusion parameters."""


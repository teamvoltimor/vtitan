"""Traffic-sign routing and blind sign-discovery tuning groups.

Fields are inherited from the generated DTOs under
:mod:`shared.config.generated.navigation.signs`. These classes adds the derived accessors (degrees to a
normalised steering command) that belong to tuning rather than the schema. The
generated field descriptions carry the measurement history that used to live
here.
"""

from __future__ import annotations

import math
from dataclasses import fields
from typing import Any

from shared.config.constants import RobotSpecs
from shared.config.generated.navigation.signs.sign_discovery_schema import (
    NavigationSignsSignDiscovery,
)
from shared.config.generated.navigation.signs.sign_router_schema import (
    NavigationSignsSignRouter,
)
from shared.domain.steering import angle_rad_to_steering_norm

__all__ = ["SignDiscoveryParams", "SignRouterParams"]


class SignRouterParams(NavigationSignsSignRouter):
    """Traffic-sign avoidance routing parameters."""

    def resolve_unset(self, target: Any, *, prefix: str = "") -> None:
        """Fill every ``None`` field of a tuning-mirror dataclass from this group.

        Convention-driven so the mapping itself has no copy to keep in step: a
        target field ``activation_dist`` reads ``activation_dist_m`` and one
        named ``depth_pin`` reads ``depth_pin``; mirrors nested under a tuning
        sub-prefix pass ``prefix`` (e.g. ``"SIGN_LANE_"`` for
        ``SignLaneParams``). A ``None`` field matching neither spelling raises
        instead of being skipped, mirroring the getattr failure that made a
        renamed tunable loud rather than letting a default quietly stop tracking
        the TOML.
        """
        for field in fields(type(target)):
            if getattr(target, field.name) is not None:
                continue
            name = (prefix + field.name).lower()
            for attr in (name, f"{name}_m"):
                if hasattr(self, attr):
                    object.__setattr__(target, field.name, getattr(self, attr))
                    break
            else:
                msg = f"{type(target).__name__}.{field.name} resolved no default from {type(self).__name__}"
                raise AttributeError(msg)

    def sign_contact_steer_norm(self) -> float:
        """Sign-evade steering as the normalised command the actuator takes.

        Stored as a physical road-wheel angle, so a wider servo yields a
        SMALLER normalised command for the same 19.25 degrees rather than the
        same command meaning a wider swerve. Same rationale as
        :meth:`~shared.config.navigation_tuning.escape.EscapeManeuverParams.rev_steer_norm`.
        """
        return angle_rad_to_steering_norm(math.radians(self.sign_contact_steer_deg), RobotSpecs.MAX_STEERING_ANGLE)

    def retrace_steer_gain_norm(self, lateral_over_distance: float) -> float:
        """Reverse-pure-pursuit steering for a target ``lateral/distance`` off-axis.

        The caller passes the dimensionless bearing ratio; the gain turns it
        into a road-wheel angle, and only then does the servo's reach enter.
        Clamping happens in normalised space, exactly as the previous inline
        ``max(-1.0, min(1.0, ...))`` did.
        """
        return angle_rad_to_steering_norm(
            math.radians(self.retrace_steer_gain_deg) * lateral_over_distance, RobotSpecs.MAX_STEERING_ANGLE
        )


class SignDiscoveryParams(NavigationSignsSignDiscovery):
    """Blind sign-discovery (ObservedSignMap) parameters."""


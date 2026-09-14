"""Speed/steering control tuning groups.

Covers clearance zones, heading zones, pure pursuit, speed control, and the
control loop rate they all run at.

Fields are inherited from the generated DTOs under
:mod:`shared.config.generated.navigation.motion`. These classes adds the derived behaviour (challenge
resolution, ceiling clamping, degrees-to-normalised steering) that belongs to
tuning rather than the schema. The generated field descriptions carry the
measurement history that used to live here.
"""

from __future__ import annotations

from typing import ClassVar

from pydantic import model_validator

from shared.config.constants import RobotSpecs
from shared.config.generated.navigation.motion.clearance_schema import (
    NavigationMotionClearance,
)
from shared.config.generated.navigation.motion.control_schema import (
    NavigationMotionControl,
)
from shared.config.generated.navigation.motion.heading_schema import (
    NavigationMotionHeading,
)
from shared.config.generated.navigation.motion.pursuit_schema import (
    NavigationMotionPursuit,
)
from shared.config.generated.navigation.motion.speed_schema import (
    NavigationMotionSpeed,
)

__all__ = [
    "ClearanceZones",
    "ControlLoopParams",
    "HeadingErrorZones",
    "PurePursuitParams",
    "SpeedControlParams",
]


class ClearanceZones(NavigationMotionClearance):
    """LIDAR clearance thresholds for speed control, in metres."""

    def for_obstacles_challenge(self) -> ClearanceZones:
        """These zones as the Obstacles Challenge should run them."""
        return self.model_copy(update={"contact_dist": self.obstacles_contact_dist})

    @model_validator(mode="after")
    def _contact_below_slow(self) -> ClearanceZones:
        """The contact zone must stay below the slow zone, override included.

        The ladder in ``core_navigator`` is an if/elif chain ordered
        contact < slow < medium, so a contact threshold at or above ``slow_dist``
        does not widen the contact zone -- it makes the slow rung unreachable
        and silently deletes a tier.
        """
        contact = self.obstacles_contact_dist
        if contact >= self.slow_dist:
            msg = (
                f"clearance.obstacles_contact_dist ({contact}) must stay below "
                f"clearance.slow_dist ({self.slow_dist}); the ladder is an ordered "
                "if/elif chain, so an equal or larger contact zone makes the slow "
                "tier unreachable instead of widening the contact one."
            )
            raise ValueError(msg)
        return self


class HeadingErrorZones(NavigationMotionHeading):
    """The heading error above which speed is cut, in radians.

    One threshold, not a ladder. ``crawl`` is the one that describes something
    real: past ~57 deg the steering servo's fixed slew rate cannot track the
    demand, so speed has to come down. The middle rungs of the retired ladder
    taxed every corner rather than catching a dangerous case.
    """


class PurePursuitParams(NavigationMotionPursuit):
    """Pure pursuit controller parameters for waypoint following.

    ``corner_preview_distance_m`` is read directly (the per-width-class
    ``wide_corner_preview_distance_m`` override is no longer part of the schema,
    so an unknown corridor and a wide one both keep the shipped value).
    """

    def for_open_challenge(self) -> PurePursuitParams:
        """These parameters as the Open Challenge should run them."""
        return self.model_copy(update={"lookahead_long": self.open_lookahead_long})

    def for_obstacles_challenge(self) -> PurePursuitParams:
        """These parameters as the Obstacles Challenge should run them."""
        return self.model_copy(
            update={"yaw_gain_compensation": self.obstacles_yaw_gain_compensation}
        )


class SpeedControlParams(NavigationMotionSpeed):
    """Speed control parameters for different zones, in ABSOLUTE m/s.

    Every tier is a real speed. The drivetrain ceiling
    (``RobotSpecs.MAX_SPEED_MPS``) is applied as a CLAMP by the model's
    after-validator, never as a multiplier -- so a tier the motor cannot reach
    is inert rather than fiction, and ``mps_ceiling()`` reports the bound.

    The old per-JOB split of the creep tier (``contact_mps`` and friends) is no
    longer a schema field; each such accessor now tracks the shared ``creep_mps``
    tier, which is what they resolved to whenever they were left unset.
    """

    _CHALLENGE_PREFIXES: ClassVar[tuple[str, ...]] = ("open", "obstacles")

    @model_validator(mode="after")
    def _clamp_tiers_to_ceiling(self) -> SpeedControlParams:
        """Clamp every tier to the drivetrain ceiling, at load.

        This replaces the per-accessor ``min(value, RobotSpecs.MAX_SPEED_MPS)``
        clamp the old ``*_mps()`` methods applied at read time, so a consumer
        reading the field directly gets exactly what the method used to return.
        ``None`` means "no override" and is left alone.
        """
        ceiling = RobotSpecs.MAX_SPEED_MPS
        clamped: dict[str, float] = {}
        for name in (
            "min_mps",
            "max_mps",
            "creep_mps",
            "slow_mps",
            "medium_mps",
            "fast_mps",
            "corner_mps",
        ):
            value = getattr(self, name)
            if value is not None and value > ceiling:
                clamped[name] = ceiling
        for prefix in self._CHALLENGE_PREFIXES:
            for tier in ("max", "slow", "medium", "fast"):
                name = f"{prefix}_{tier}_mps"
                value = getattr(self, name)
                if value is not None and value > ceiling:
                    clamped[name] = ceiling
        return self.model_copy(update=clamped) if clamped else self

    @model_validator(mode="after")
    def _challenge_tiers_below_challenge_cap(self) -> SpeedControlParams:
        """A per-challenge tier above that challenge's cap is inert -- reject it."""
        for prefix in self._CHALLENGE_PREFIXES:
            override_cap = getattr(self, f"{prefix}_max_mps")
            cap = override_cap if override_cap is not None else self.max_mps
            for tier in ("slow", "medium", "fast"):
                name = f"{prefix}_{tier}_mps"
                value = getattr(self, name)
                if value is not None and value > cap:
                    msg = (
                        f"speed.{name} ({value}) exceeds the {prefix.title()}-Challenge cap "
                        f"({cap}); core_navigator clamps every tier to max_mps(), so this "
                        f"tier would be silently inert. Raise {prefix}_max_mps or lower the tier."
                    )
                    raise ValueError(msg)
        return self

    def _for_challenge_prefix(self, prefix: str) -> SpeedControlParams:
        """Apply one challenge's ``<prefix>_*`` overrides onto the base ladder.

        Returns ``self`` unchanged when that challenge defines no overrides, so
        a drivetrain with no headroom to spare shares one ladder across both
        challenges without any caller needing to know which case it is in.
        """
        overrides = {
            tier: value
            for tier in ("max_mps", "slow_mps", "medium_mps", "fast_mps")
            if (value := getattr(self, f"{prefix}_{tier}")) is not None
        }
        if not overrides:
            return self
        return type(self).model_validate(self.model_dump() | overrides)

    def for_open_challenge(self) -> SpeedControlParams:
        """This ladder as the Open Challenge should run it."""
        return self._for_challenge_prefix("open")

    def for_obstacles_challenge(self) -> SpeedControlParams:
        """This ladder as the Obstacles Challenge should run it.

        Not to be confused with the removed ``NavigationTuning.for_obstacles()``
        classmethod, which baked a whole tuning profile in code. This applies
        only the speed tiers a motor profile declares for itself.
        """
        return self._for_challenge_prefix("obstacles")

    def mps_ceiling(self) -> float:
        """The drivetrain ceiling every tier is clamped to."""
        return RobotSpecs.MAX_SPEED_MPS

    def contact_mps(self) -> float:
        """Forward speed inside the contact zone, in m/s."""
        return self.creep_mps

    def contact_reverse_mps(self) -> float:
        """Reverse speed out of the contact zone, in m/s."""
        return self.creep_mps

    def sign_evade_mps(self) -> float:
        """Speed cap while steering out of a sign contact, in m/s."""
        return self.creep_mps

    def escape_nudge_mps(self) -> float:
        """Forward speed of the stuck-escape nudge, in m/s."""
        return self.creep_mps

    def heading_floor_mps(self) -> float:
        """Speed the heading limiter drops to, in m/s."""
        return self.creep_mps


class ControlLoopParams(NavigationMotionControl):
    """The rate the navigation control loop runs at.

    One number, previously written twice: the waypoint controller carried a
    0.05 s literal and the simulation gateway carried 20 Hz. They agreed only
    because someone kept them agreeing. The controller uses ``dt`` to
    rate-limit steering, so a divergence would silently tune the real robot
    against a cadence the simulator never ran at.
    """


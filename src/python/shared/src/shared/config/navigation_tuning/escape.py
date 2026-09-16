"""Collision-recovery (K-turn, slalom, stuck-detection) tuning group.

Fields are inherited from the generated DTO
(:mod:`shared.config.generated.navigation.escape.escape_schema`). This module
adds the derived accessors (seconds to
control ticks, road-wheel degrees to a normalised steering command) that belong
to tuning rather than the schema.
"""

from __future__ import annotations

import math

from shared.config.constants import RobotSpecs
from shared.config.generated.navigation.escape.escape_schema import (
    NavigationEscapeEscape,
)
from shared.domain.steering import angle_rad_to_steering_norm


class EscapeManeuverParams(NavigationEscapeEscape):
    """Escape maneuver parameters for collision recovery.

    When collision risk is detected, the robot executes escape maneuvers
    (K-turn, slalom) to clear obstacles and resume navigation.

    Every duration is stored in SECONDS and converted to control ticks by the
    ``*_frames`` accessors, because a frame count silently means a different
    duration at a different loop rate. ``rev_steer_deg`` and
    ``side_correction_steer_deg`` are physical road-wheel angles, converted to
    a normalised actuator command by ``rev_steer_norm`` /
    ``side_correction_steer_norm``, so a wider servo produces a smaller
    normalised command for the same physical angle.
    """

    @staticmethod
    def frames(seconds: float, control_hz: float) -> int:
        """Convert a duration into control ticks at ``control_hz``.

        Every escape duration is stored in SECONDS and converted here, rather
        than stored as a frame count. A frame count silently means a different
        thing at a different loop rate: at the shipped 20 Hz a
        ``stuck_timeout`` of 40 frames is 2 s, and at 50 Hz the same 40 frames
        is 0.8 s.

        Rounds rather than truncates, and floors at one tick: a duration
        shorter than a single tick is still a maneuver the caller asked for,
        and zero frames would skip it entirely.
        """
        return max(1, round(seconds * control_hz))

    def k_turn_min_frames(self, control_hz: float) -> int:
        """Minimum K-turn duration in ticks (OBSTACLE risk)."""
        return self.frames(self.k_turn_min_s, control_hz)

    def k_turn_max_frames(self, control_hz: float) -> int:
        """Maximum K-turn duration in ticks (CRITICAL risk)."""
        return self.frames(self.k_turn_max_s, control_hz)

    def slalom_reverse_frames(self, control_hz: float) -> int:
        """Ticks spent reversing during a slalom."""
        return self.frames(self.slalom_reverse_s, control_hz)

    def slalom_forward_frames(self, control_hz: float) -> int:
        """Ticks spent forward-turning during a slalom."""
        return self.frames(self.slalom_forward_s, control_hz)

    def stuck_timeout_frames(self, control_hz: float) -> int:
        """Ticks without movement before declaring the robot stuck."""
        return self.frames(self.stuck_timeout_s, control_hz)

    def side_correction_frames(self, control_hz: float) -> int:
        """Duration of a side-threat correction, in ticks."""
        return self.frames(self.side_correction_s, control_hz)

    def max_escape_frames(self, control_hz: float) -> int:
        """Hard cap on any single escalated escape, in ticks."""
        return self.frames(self.max_escape_s, control_hz)

    def stuck_escalation_per_attempt_frames(self, control_hz: float) -> int:
        """Ticks added to a stuck-reverse per repeated attempt."""
        return self.frames(self.stuck_escalation_per_attempt_s, control_hz)

    def stuck_history_floor_frames(self, control_hz: float) -> int:
        """Minimum pose history the StuckDetector keeps, in ticks."""
        return self.frames(self.stuck_history_floor_s, control_hz)

    def rev_steer_norm(self) -> float:
        """Reverse-escape steering as a normalised command for the actuator.

        The stored value is a physical road-wheel angle, so this is the one
        place the servo's reach enters: a wider servo produces a SMALLER
        normalised command for the same 44 degrees, rather than the same
        command meaning a wider angle. Mirrors the ``*_mps()`` accessors on
        :class:`~shared.config.navigation_tuning.motion.SpeedControlParams`.
        """
        return angle_rad_to_steering_norm(math.radians(self.rev_steer_deg), RobotSpecs.MAX_STEERING_ANGLE)

    def for_obstacles_challenge(self) -> EscapeManeuverParams:
        """These parameters as the Obstacles Challenge should run them.

        Returns ``self`` unchanged when no Obstacles override is set, so the
        Open path and the un-overridden Obstacles path stay byte-identical.
        """
        overrides = {
            name: obstacle_value
            for name, obstacle_value in (
                ("k_turn_fit_rear_gap", self.obstacles_k_turn_fit_rear_gap),
                ("escape_mirrors_reverse", self.obstacles_escape_mirrors_reverse),
                (
                    "escape_side_follows_committed_sign",
                    self.obstacles_escape_side_follows_committed_sign,
                ),
                (
                    "side_correction_follows_committed_sign",
                    self.obstacles_side_correction_follows_committed_sign,
                ),
            )
            if obstacle_value is not None and obstacle_value != getattr(self, name)
        }
        # An override is applied only when it actually DISAGREES with the shared
        # field, not merely when it is set. The generated DTOs type these as plain
        # ``bool`` with a concrete default, so "unset" is no longer representable
        # as ``None`` in the shipped tree; comparing against the shared value
        # recovers the identity promise above -- with every override set to its
        # shared value this returns ``self``, so the Open path and an
        # un-overridden Obstacles path stay byte-identical. A caller can still
        # clear one with ``model_copy(update={...: None})``, which does not
        # revalidate, and the None is dropped here.
        if not overrides:
            return self
        return self.model_copy(update=overrides)

    def side_correction_steer_norm(self) -> float:
        """Side-threat correction steering as a normalised actuator command."""
        return angle_rad_to_steering_norm(math.radians(self.side_correction_steer_deg), RobotSpecs.MAX_STEERING_ANGLE)

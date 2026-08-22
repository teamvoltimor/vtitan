"""Collision-recovery (K-turn, slalom, stuck-detection) tuning group."""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning._shared import _alias
from shared.domain.steering import angle_rad_to_steering_norm


class EscapeManeuverParams(BaseModel):
    """Escape maneuver parameters for collision recovery.

    When collision risk is detected, the robot executes escape maneuvers
    (K-turn, slalom) to clear obstacles and resume navigation.

    Attributes:
        POSE_TRAIL_MIN_STEP_M: Spacing between recorded breadcrumbs on the
            pose trail a retrace-reverse follows. Thins a stationary or
            creeping robot's trail, which would otherwise fill the buffer with
            one position, without dropping resolution on a moving one. Note
            this is a distance-per-tick threshold and so interacts with speed:
            at 0.156 m/s and 20 Hz the chassis advances ~0.008 m per tick and
            the thinning bites, while at 0.234 m/s it advances ~0.012 m and
            nothing is thinned at all. A module constant until 2026-08-22,
            which meant the interaction was invisible and the value could not
            be moved with the profile that changed the speed
        POSE_TRAIL_LEN: Breadcrumbs kept, oldest evicted first. ~1.3 m of
            travel at the default spacing, comfortably more than any retrace
            distance worth driving
        REV_SPEED: Reverse speed during escapes (m/s, negative)
        REV_STEER_DEG: Road-wheel steering angle held while reversing out.
            Named ``REV_STEERING_SCALE`` until 2026-08-21, which was doubly
            wrong: it is not a scale (every call site uses it as
            ``steering = value * side_sign``, a magnitude) and it was
            normalised, so it meant 44 deg of road wheel on the bench-measured
            55 deg chassis and would have silently become 68 deg on a 270 deg
            servo
        K_TURN_MIN_FRAMES: Minimum frames for K-turn maneuver (OBSTACLE risk)
        K_TURN_MAX_FRAMES: Maximum frames for K-turn maneuver (CRITICAL risk)
        SLALOM_REVERSE_FRAMES: Frames spent reversing during slalom
        SLALOM_FORWARD_FRAMES: Frames spent forward turning during slalom
        STUCK_MOVE_THRESHOLD: Distance threshold to detect stuck (m)
        STUCK_TIMEOUT_FRAMES: Frames without movement before stuck (20Hz)
        SIDE_CORRECTION_STEER_DEG: Road-wheel steering angle for a side-threat
            correction. Physical degrees for the same reason as REV_STEER_DEG
        SIDE_CORRECTION_SPEED: Forward speed during a side-threat correction
        SIDE_CORRECTION_FRAMES: Duration of a side-threat correction (frames)
        ESCALATE_AFTER_ATTEMPTS: Consecutive escapes before escalating (longer
            duration, opposite side) instead of repeating an identical pulse
        ESCAPE_SIDE_COMMIT_ATTEMPTS: Consecutive escape attempts made toward one
            side before switching to the other. Escapes used to flip side on
            every attempt, so successive attempts rotated the chassis opposite
            ways and cancelled out -- measured on real hardware as 40 s of
            rocking in place with zero net translation. Committing to a side for
            more than one attempt is what lets a wedged robot actually walk out
        MAX_ESCAPE_FRAMES: Hard cap on any single escalated escape duration
        STUCK_CONFIRMATION_CHECKS: Consecutive below-threshold stuck checks
            required before StuckDetector declares the robot stuck
        STUCK_ESCALATION_FRAMES_PER_ATTEMPT: Frames added to a stuck-reverse
            maneuver's duration per repeated stuck-escape attempt
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    POSE_TRAIL_MIN_STEP_M: float = Field(
        default=0.01, gt=0.0, validation_alias=_alias("POSE_TRAIL_MIN_STEP_M")
    )
    POSE_TRAIL_LEN: int = Field(default=128, gt=0, validation_alias=_alias("POSE_TRAIL_LEN"))
    REV_SPEED: float = Field(default=-0.20, validation_alias=_alias("REV_SPEED"))  # Reverse speed
    # 44.0 deg is what the previous normalised 0.8 meant at the bench-measured
    # 55 deg road-wheel limit, so this conversion changed no behaviour.
    REV_STEER_DEG: float = Field(
        default=44.0, validation_alias=_alias("REV_STEER_DEG")
    )  # Road-wheel angle while reversing
    K_TURN_MIN_FRAMES: int = Field(default=6, validation_alias=_alias("K_TURN_MIN_FRAMES"))  # Minimum K-turn
    K_TURN_MAX_FRAMES: int = Field(default=12, validation_alias=_alias("K_TURN_MAX_FRAMES"))  # Maximum K-turn
    SLALOM_REVERSE_FRAMES: int = Field(
        default=8, validation_alias=_alias("SLALOM_REVERSE_FRAMES")
    )  # Reverse duration in slalom
    SLALOM_FORWARD_FRAMES: int = Field(
        default=10, validation_alias=_alias("SLALOM_FORWARD_FRAMES")
    )  # Forward turn duration
    STUCK_MOVE_THRESHOLD: float = Field(
        default=0.03, validation_alias=_alias("STUCK_MOVE_THRESHOLD")
    )  # 3cm movement threshold
    STUCK_TIMEOUT_FRAMES: int = Field(
        default=40, validation_alias=_alias("STUCK_TIMEOUT_FRAMES")
    )  # ~2 seconds at 20Hz
    # 16.5 deg == the previous normalised 0.3 at the 55 deg road-wheel limit.
    SIDE_CORRECTION_STEER_DEG: float = Field(default=16.5, validation_alias=_alias("SIDE_CORRECTION_STEER_DEG"))
    SIDE_CORRECTION_SPEED: float = Field(default=0.1, validation_alias=_alias("SIDE_CORRECTION_SPEED"))
    SIDE_CORRECTION_FRAMES: int = Field(default=4, validation_alias=_alias("SIDE_CORRECTION_FRAMES"))
    ESCALATE_AFTER_ATTEMPTS: int = Field(default=3, validation_alias=_alias("ESCALATE_AFTER_ATTEMPTS"))
    ESCAPE_SIDE_COMMIT_ATTEMPTS: int = Field(
        default=2, ge=1, validation_alias=_alias("ESCAPE_SIDE_COMMIT_ATTEMPTS")
    )
    MAX_ESCAPE_FRAMES: int = Field(default=20, validation_alias=_alias("MAX_ESCAPE_FRAMES"))
    STUCK_CONFIRMATION_CHECKS: int = Field(default=3, validation_alias=_alias("STUCK_CONFIRMATION_CHECKS"))
    STUCK_ESCALATION_FRAMES_PER_ATTEMPT: int = Field(
        default=2, validation_alias=_alias("STUCK_ESCALATION_FRAMES_PER_ATTEMPT")
    )
    MIN_HISTORY_FOR_DISTANCE: int = Field(
        default=2, validation_alias=_alias("MIN_HISTORY_FOR_DISTANCE")
    )  # Poses needed before StuckDetector can measure distance travelled
    STUCK_HISTORY_FLOOR: int = Field(
        default=60, validation_alias=_alias("STUCK_HISTORY_FLOOR")
    )  # Minimum position history (frames) the StuckDetector keeps, even when
    # STUCK_TIMEOUT_FRAMES * 2 would be smaller

    def rev_steer_norm(self) -> float:
        """Reverse-escape steering as a normalised command for the actuator.

        The stored value is a physical road-wheel angle, so this is the one
        place the servo's reach enters: a wider servo produces a SMALLER
        normalised command for the same 44 degrees, rather than the same
        command meaning a wider angle. Mirrors the ``*_mps()`` accessors on
        :class:`~shared.config.navigation_tuning.motion.SpeedControlParams`,
        for the same reason -- the conversion belongs next to the value, not
        repeated at four call sites that could each drift.
        """
        return angle_rad_to_steering_norm(math.radians(self.REV_STEER_DEG), RobotSpecs.MAX_STEERING_ANGLE)

    def side_correction_steer_norm(self) -> float:
        """Side-threat correction steering as a normalised actuator command."""
        return angle_rad_to_steering_norm(
            math.radians(self.SIDE_CORRECTION_STEER_DEG), RobotSpecs.MAX_STEERING_ANGLE
        )

"""Collision-recovery (K-turn, slalom, stuck-detection) tuning group."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias


class EscapeManeuverParams(BaseModel):
    """Escape maneuver parameters for collision recovery.

    When collision risk is detected, the robot executes escape maneuvers
    (K-turn, slalom) to clear obstacles and resume navigation.

    Attributes:
        REV_SPEED: Reverse speed during escapes
        REV_STEERING_SCALE: Steering aggressiveness while reversing
        K_TURN_MIN_FRAMES: Minimum frames for K-turn maneuver (OBSTACLE risk)
        K_TURN_MAX_FRAMES: Maximum frames for K-turn maneuver (CRITICAL risk)
        SLALOM_REVERSE_FRAMES: Frames spent reversing during slalom
        SLALOM_FORWARD_FRAMES: Frames spent forward turning during slalom
        STUCK_MOVE_THRESHOLD: Distance threshold to detect stuck (m)
        STUCK_TIMEOUT_FRAMES: Frames without movement before stuck (20Hz)
        SIDE_CORRECTION_STEER: Steering magnitude for a side-threat correction
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

    REV_SPEED: float = Field(default=-0.20, validation_alias=_alias("REV_SPEED"))  # Reverse speed
    REV_STEERING_SCALE: float = Field(
        default=0.8, validation_alias=_alias("REV_STEERING_SCALE")
    )  # Steering while reversing
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
    SIDE_CORRECTION_STEER: float = Field(default=0.3, validation_alias=_alias("SIDE_CORRECTION_STEER"))
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

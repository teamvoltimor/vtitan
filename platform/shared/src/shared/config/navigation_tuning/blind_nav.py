"""Blind-navigation tuning groups.

Covers corridor width estimation, corridor following, direction inference,
LIDAR pose search, and odometry/IMU fusion.
"""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias


class CorridorEstimatorParams(BaseModel):
    """Blind corridor-width estimation parameters.

    Attributes:
        MIN_SAMPLES: Width readings a corridor must accumulate before its
            estimate is trusted. Readings are attributed to a corridor by
            heading, so a handful taken while the chassis is still swinging
            through a corner can land in the wrong one.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_SAMPLES: int = Field(default=12, validation_alias=_alias("MIN_SAMPLES"))


class CorridorFollowerParams(BaseModel):
    """Blind corridor-following and corner-turn parameters.

    Attributes:
        TURN_CLEARANCE_M: Forward clearance (m) at which the corner turn
            begins. Must stay strictly below
            DirectionEstimatorParams.CORNER_CLEARANCE_M -- see the
            cross-group check on NavigationTuning.
        CENTERING_GAIN: Steering per metre of lateral offset from the
            corridor centreline.
        HEADING_GAIN: Steering per radian of heading error against the
            corridor axis. Centring on offset alone is undamped -- in a
            steered vehicle heading is the integral of steering and
            position the integral of heading, so the two are 90 degrees
            out of phase and proportional-on-position is an oscillator.
            Measured on real hardware 2026-08-07: 112 steering sign flips
            in 177 s, a 3.2 s limit cycle, 45% of ticks pinned at
            MAX_CENTERING_STEER, heading 30 deg off axis at the median.
            That is what starves the direction gate, which needs the
            chassis square to a corridor at the moment one side opens.
            Raising CENTERING_GAIN cannot fix it and makes it worse (see
            MAX_CENTERING_STEER); the missing term is this one.
        MAX_CENTERING_STEER: Hard cap on the steering that gain may
            produce. Both are deliberately timid: the counter-phase
            four-wheel chassis responds violently, and oscillation swings
            the heading past the direction estimator's alignment gate,
            which then refuses every reading.
        CORNER_SPEED_SCALE: Fraction of creep speed while turning a corner
            blind, which is committed on one comparison rather than a plan.
        REVERSE_SPEED_SCALE: Fraction of creep speed while backing off.
        TURN_ARC_HALF_FOV_DEG: Half-width (deg) of the arc searched for a way
            through before committing to a corner turn. Wider than
            LidarSectorParams.DIRECTION_ARC_HALF_FOV_DEG on purpose -- that
            8 deg cone cannot tell a corridor that has ended from a chassis
            pointed obliquely at the wall beside it.
        TURN_OPEN_RANGE_M: If any bearing within that arc has at least this
            much room, the corridor has not ended and the turn is refused.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    TURN_CLEARANCE_M: float = Field(default=0.60, validation_alias=_alias("TURN_CLEARANCE_M"))
    CENTERING_GAIN: float = Field(default=0.8, validation_alias=_alias("CENTERING_GAIN"))
    HEADING_GAIN: float = Field(default=0.8, validation_alias=_alias("HEADING_GAIN"))
    MAX_CENTERING_STEER: float = Field(default=0.25, validation_alias=_alias("MAX_CENTERING_STEER"))
    CORNER_SPEED_SCALE: float = Field(default=0.6, validation_alias=_alias("CORNER_SPEED_SCALE"))
    REVERSE_SPEED_SCALE: float = Field(default=0.6, validation_alias=_alias("REVERSE_SPEED_SCALE"))
    TURN_ARC_HALF_FOV_DEG: float = Field(default=15.0, validation_alias=_alias("TURN_ARC_HALF_FOV_DEG"))
    TURN_OPEN_RANGE_M: float = Field(default=1.00, validation_alias=_alias("TURN_OPEN_RANGE_M"))


class DirectionEstimatorParams(BaseModel):
    """Blind travel-direction inference parameters.

    Attributes:
        ALIGNMENT_TOLERANCE_RAD: Maximum heading error against the nearest
            track axis (radians) for a side-ray reading to be trusted. Off
            axis the side rays cut a diagonal and read long for no good
            reason. Shared with
            :func:`~src.navigation.corridor_estimator.measure_corridor_width`,
            which gates the same side rays on the same geometry -- the two
            must agree or a scan can be trusted for width and rejected for
            direction. Deliberately not one of the ``heading`` zones: those
            modulate speed, and retuning speed must not move this gate.
        CORNER_CLEARANCE_M: Forward clearance (m) below which the corridor
            counts as ending, opening the window in which the robot reads
            which side is open. Deliberately larger than
            CorridorFollowerParams.TURN_CLEARANCE_M: turning swings the
            heading past the alignment gate, so a robot that begins its
            turn the instant the comparison becomes decisive rotates
            straight through its only measurement window.
        MAX_IN_TRACK_RANGE_M: Side rays longer than this (m) cannot be a
            wall of this track and are rejected. A LIDAR dropout is
            reported as max range, which reads as "this side is open" --
            exactly the signal the estimator looks for.
        MIN_ASYMMETRY_M: Minimum left/right difference (m) for a sweep to
            count as evidence rather than noise.
        PLAUSIBLE_SPAN_THRESHOLD_M: Maximum sum of left + right ranges (m)
            that still represents one corridor. Exceeding this sum means one
            ray ran off-track into an adjacent corridor rather than both
            reading walls of the current one. The margin absorbs scanning
            slightly off-axis.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    ALIGNMENT_TOLERANCE_RAD: float = Field(
        default=math.radians(25.0), validation_alias=_alias("ALIGNMENT_TOLERANCE_RAD")
    )
    CORNER_CLEARANCE_M: float = Field(default=1.00, validation_alias=_alias("CORNER_CLEARANCE_M"))
    MAX_IN_TRACK_RANGE_M: float = Field(default=4.5, validation_alias=_alias("MAX_IN_TRACK_RANGE_M"))
    MIN_ASYMMETRY_M: float = Field(default=0.20, validation_alias=_alias("MIN_ASYMMETRY_M"))
    PLAUSIBLE_SPAN_THRESHOLD_M: float = Field(default=1.25, validation_alias=_alias("PLAUSIBLE_SPAN_THRESHOLD_M"))


class LocalizationParams(BaseModel):
    """LIDAR-based pose search (LidarLocalizer) parameters.

    Attributes:
        SEARCH_RADIUS_M: Half-width (m) of the initial pose search window.
        PASSES: Number of coarse-to-fine grid-search passes.
        GRID_POINTS: Candidates per axis per search pass.
        RESIDUAL_CLIP_M: Per-ray residual clipping distance (m) for the cost
            function (outlier rejection).
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SEARCH_RADIUS_M: float = Field(default=0.15, validation_alias=_alias("SEARCH_RADIUS_M"))
    PASSES: int = Field(default=4, validation_alias=_alias("PASSES"))
    GRID_POINTS: int = Field(default=5, validation_alias=_alias("GRID_POINTS"))
    RESIDUAL_CLIP_M: float = Field(default=0.25, validation_alias=_alias("RESIDUAL_CLIP_M"))


class StateEstimatorParams(BaseModel):
    """Odometry/IMU fusion parameters.

    Attributes:
        YAW_CORRECTION_GAIN: Fraction of the observed yaw discrepancy folded
            into the estimate per update. Distinct from LocalizationParams,
            which tunes the LIDAR pose *search*; this is the dead-reckoning
            blend that runs whether or not localization is enabled.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    YAW_CORRECTION_GAIN: float = Field(default=0.05, validation_alias=_alias("YAW_CORRECTION_GAIN"))

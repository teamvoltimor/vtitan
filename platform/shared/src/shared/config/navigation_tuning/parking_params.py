"""Parallel-parking maneuver tuning group."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias


class ParkingParams(BaseModel):
    """Parallel-parking maneuver parameters.

    Attributes:
        PARALLEL_TOLERANCE_M: WRO rule max allowed wheel-to-wall distance
            difference (m) for "parallel" parking.
        POS_REACH_DIST_M: Distance (m) threshold for "reached staging
            position."
        DEFAULT_MAX_FRAMES: Max control ticks before the parking maneuver
            gives up (20s @ 20Hz by default).
        SATURATED_STEER_THRESHOLD: Normalized steering magnitude counted as
            "at physical lock."
        SATURATION_STUCK_TICKS: Consecutive ticks of saturated steering
            before triggering a reverse-reorient.
        SPEED: Constant driving speed (m/s) during the parking maneuver.
        MIN_LOOKAHEAD_DIST_M: Floor distance (m) to avoid near-zero-distance
            curvature blow-up in pure pursuit.
        WALL_STANDOFF_M: Closest the chassis footprint may approach the
            field wall backing the parking lot.
        MARKER_STANDOFF_M: Closest the chassis footprint may approach a
            parking-bay marker fin.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    PARALLEL_TOLERANCE_M: float = Field(default=0.02, validation_alias=_alias("PARALLEL_TOLERANCE_M"))
    POS_REACH_DIST_M: float = Field(default=0.04, validation_alias=_alias("POS_REACH_DIST_M"))
    DEFAULT_MAX_FRAMES: int = Field(default=400, validation_alias=_alias("DEFAULT_MAX_FRAMES"))
    SATURATED_STEER_THRESHOLD: float = Field(
        default=0.999, validation_alias=_alias("SATURATED_STEER_THRESHOLD")
    )
    SATURATION_STUCK_TICKS: int = Field(default=20, validation_alias=_alias("SATURATION_STUCK_TICKS"))
    SPEED: float = Field(default=0.12, validation_alias=_alias("SPEED"))
    MIN_LOOKAHEAD_DIST_M: float = Field(default=0.02, validation_alias=_alias("MIN_LOOKAHEAD_DIST_M"))
    WALL_STANDOFF_M: float = Field(default=0.05, validation_alias=_alias("WALL_STANDOFF_M"))
    MARKER_STANDOFF_M: float = Field(default=0.01, validation_alias=_alias("MARKER_STANDOFF_M"))

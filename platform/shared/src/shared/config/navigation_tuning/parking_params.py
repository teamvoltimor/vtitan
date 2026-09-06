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
        ATTEMPT_AFTER_FINAL_LAP: Whether to pursue the parking bay once the
            final lap is counted. ``False`` stops in the finish section
            instead.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    PARALLEL_TOLERANCE_M: float = Field(default=0.02, validation_alias=_alias("PARALLEL_TOLERANCE_M"))
    POS_REACH_DIST_M: float = Field(default=0.04, validation_alias=_alias("POS_REACH_DIST_M"))
    DEFAULT_MAX_FRAMES: int = Field(default=400, validation_alias=_alias("DEFAULT_MAX_FRAMES"))
    SATURATED_STEER_THRESHOLD: float = Field(default=0.999, validation_alias=_alias("SATURATED_STEER_THRESHOLD"))
    SATURATION_STUCK_TICKS: int = Field(default=20, validation_alias=_alias("SATURATION_STUCK_TICKS"))
    SPEED: float = Field(default=0.12, validation_alias=_alias("SPEED"))
    MIN_LOOKAHEAD_DIST_M: float = Field(default=0.02, validation_alias=_alias("MIN_LOOKAHEAD_DIST_M"))
    WALL_STANDOFF_M: float = Field(default=0.05, validation_alias=_alias("WALL_STANDOFF_M"))
    MARKER_STANDOFF_M: float = Field(default=0.01, validation_alias=_alias("MARKER_STANDOFF_M"))

    ATTEMPT_AFTER_FINAL_LAP: bool = Field(
        default=False, validation_alias=_alias("ATTEMPT_AFTER_FINAL_LAP")
    )
    """Pursue the parking bay after the final lap, or stop in the finish section.

    ``False`` (shipped) ends the round the way rule 1.3 pays for: the lap
    counter increments at the along-track CENTRE of the 1 m start straight, so
    the robot is already inside the finish section at that instant, and
    ``WaypointParams.FINISH_APPROACH_M`` has capped the approach to
    ``slow_mps`` so the ~0.09 m coast stays inside the 0.50 m of section left
    ahead of it.

    ``True`` restores the pursuit: keep lapping until the robot reaches the
    parking corridor and the staging point, then hand off to ParkController.

    Shipped False because the pursuit is a large net LOSS while the bay is
    geometrically unreachable (0.194 m chassis into a 0.20 m bay). Measured
    blind on the 256 corpus, parking OFF against ON, one invocation:
    ``in-time`` 158 vs 62, collisions 4 vs 51 (wall 2 vs 30, park 0 vs 19),
    timeouts 61 vs 110 -- while ``laps>=3`` is IDENTICAL at 159. The driving is
    the same round either way; everything that separates the two columns is
    clock and contact spent after the laps were already banked. Rule 1.3 pays 3
    points for the finish-section stop, against 7-15 for a park that lands
    2/256 partial and 0/256 full.

    Flip to ``True`` when the parking geometry is solved; the handoff path it
    gates is unchanged and still covered by its own tests.
    """

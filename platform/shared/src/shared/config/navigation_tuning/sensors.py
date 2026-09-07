"""Sensor health, LIDAR sector, and wall-heading estimation tuning groups."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias


class SensorHealthParams(BaseModel):
    """Sensor dropout / staleness detection for the hardware gateway.

    Attributes:
        STALE_TIMEOUT_SEC: A cached sensor reading older than this is treated as
            a dropout — the gateway reports it as unavailable so the navigator
            degrades safely instead of acting on frozen data. Derived as 5x the
            LIDAR scan period (RobotSpecs.LIDAR_UPDATE_RATE), the slowest sensor
            feed the control loop depends on.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    STALE_TIMEOUT_SEC: float = Field(default=0.5, validation_alias=_alias("STALE_TIMEOUT_SEC"))


class LidarSectorParams(BaseModel):
    """LIDAR angular-sector definitions shared by collision avoidance and the OLED.

    Attributes:
        FRONT_HALF_FOV_DEG: Half-width (deg) of the forward clearance cone,
            used by CollisionAvoidanceController.compute_forward_clearance.
        THREAT_HALF_FOV_DEG: Half-width (deg) of the threat-detection sectors
            (front/left/right/back), used by detect_threat_direction and
            related methods -- a narrower cone than the forward clearance
            cone above; both are min-based (see compute_forward_clearance's
            docstring for why it switched from mean 2026-08-04).
        SELF_DETECTION_THRESHOLD_M: Rays no farther than this are discarded
            as chassis/cable self-reflection when a sector filters for it.
        MIN_VALID_RANGE_M: LIDAR ranges at or below this are treated as
            invalid (no-return) readings. Matches RobotSpecs.LIDAR_MIN_RANGE
            (the C1's real rated minimum, 0.05m) -- was 0.01m, five times
            below what the sensor can physically report.
        THREAT_NO_DETECTION_RANGE_M: A sector's nearest reading beyond this
            distance doesn't count as a threat at all -- used by
            detect_threat_direction to return ThreatDirection.NONE instead
            of the nearest-but-still-far sector.
        BLIND_WEDGE_LEFT_MIN_DEG / BLIND_WEDGE_LEFT_MAX_DEG: Bearing range
            (deg, 0 = forward, +90 = left) where the rear-left mount
            structurally occludes the LIDAR -- rays here self-collide
            regardless of range, so they're excluded by angle rather than by
            a distance threshold. Measured 2026-08-04 against a real bag: a
            5deg-resolution sweep found this arc self-colliding on the
            majority of rays at every distance, while the same rays a few
            degrees either side read a clean, consistent open-track range.
            A range-based filter can't tell those two cases apart -- a real
            close object at the same distance in a reliable bearing would be
            discarded too -- so this must be masked by angle, not distance.
        BLIND_WEDGE_RIGHT_MIN_DEG / BLIND_WEDGE_RIGHT_MAX_DEG: Mirror of the
            above for the rear-right mount. Not symmetric with the left
            wedge (measured wider) -- the mount occlusion itself isn't
            symmetric.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    FRONT_HALF_FOV_DEG: float = Field(default=30.0, validation_alias=_alias("FRONT_HALF_FOV_DEG"))
    THREAT_HALF_FOV_DEG: float = Field(default=45.0, validation_alias=_alias("THREAT_HALF_FOV_DEG"))
    SELF_DETECTION_THRESHOLD_M: float = Field(default=0.08, validation_alias=_alias("SELF_DETECTION_THRESHOLD_M"))

    REAR_SELF_DETECTION_FROM_CHASSIS: bool = Field(
        default=True, validation_alias=_alias("REAR_SELF_DETECTION_FROM_CHASSIS")
    )
    """Gate the REAR sector's self-detection by chassis geometry, per bearing.

    ``SELF_DETECTION_THRESHOLD_M`` is one scalar (0.08 m) and the rear cannot be
    described by one: over the rear +/-45 deg sector the chassis boundary runs
    from **0.137 m** at the sector edges to **0.2722 m** straight back. 0.08 sits
    far inside the body everywhere in that sector, so the robot's own structure
    survives the filter.

    Measured on run_20260906_192424: the rear minimum came from -157 deg at
    0.125 m (76% of scans) and -172 deg at 0.187 m (18%), and lay INSIDE the
    chassis footprint on **100%** of them. ``back_m`` therefore read ~0.127 m for
    the whole run, ``most_constrained_side`` was BACK on **84%** of driving ticks
    and on ALL FIVE contact episodes, and ``compute_escape_maneuver`` has no BACK
    branch -- so it returned ``None`` every time. The robot held
    ``escape_risk = critical`` for 212 ticks, across exactly the five moments it
    hit a pillar, and never manoeuvred once. Only the speed band responded.

    With this on the threshold is the ray-vs-rectangle exit distance at each
    bearing (``chassis_exit_range_m``) -- a geometric fact rather than a fitted
    number, since nothing outside the robot can return closer.

    Widening the blind wedges was the alternative and is worse: they stop at
    -155/+160 deg and the structure continues past them, so covering it closes
    the ~25 deg rear slot entirely and ``rear_sector.measured`` goes permanently
    false, removing reverse authorisation rather than fixing it.

    False restores the scalar threshold, the arm every pre-2026-09-06
    measurement was taken on.
    """
    MIN_VALID_RANGE_M: float = Field(default=0.05, validation_alias=_alias("MIN_VALID_RANGE_M"))
    # These four track sensors/lidar_sectors.toml, and the gap between them
    # mattered: the old bare defaults (-180..-115 and 115..180) blinded the
    # rear ARC ENTIRELY, which is the pre-2026-08-31 belief that this chassis
    # has no rear slot at all. The shipped values leave the ~40 deg slot at
    # +-160..180 that was re-measured on 08-31, and the reverse guards key off
    # exactly that: `rear_sector().measured` is False for a fully-blind arc, so
    # bare-constructed tuning refused every reverse escape while the same code
    # loaded from TOML allowed it.
    BLIND_WEDGE_LEFT_MIN_DEG: float = Field(default=-155.0, validation_alias=_alias("BLIND_WEDGE_LEFT_MIN_DEG"))
    BLIND_WEDGE_LEFT_MAX_DEG: float = Field(default=-120.0, validation_alias=_alias("BLIND_WEDGE_LEFT_MAX_DEG"))
    BLIND_WEDGE_RIGHT_MIN_DEG: float = Field(default=120.0, validation_alias=_alias("BLIND_WEDGE_RIGHT_MIN_DEG"))
    BLIND_WEDGE_RIGHT_MAX_DEG: float = Field(default=160.0, validation_alias=_alias("BLIND_WEDGE_RIGHT_MAX_DEG"))
    THREAT_NO_DETECTION_RANGE_M: float = Field(default=1.0, validation_alias=_alias("THREAT_NO_DETECTION_RANGE_M"))
    NO_DATA_RANGE_M: float = Field(default=10.0, validation_alias=_alias("NO_DATA_RANGE_M"))
    """Fallback range (m) when no valid LIDAR readings are available.

    Used as sentinel value in sector computations when all rays are invalid.
    Conservative estimate between min (0.05m) and max (12m) sensor range."""

    DIRECTION_ARC_HALF_FOV_DEG: float = Field(default=8.0, validation_alias=_alias("DIRECTION_ARC_HALF_FOV_DEG"))
    """Half-width (deg) of the narrow forward cone src.navigation.utils'
    _forward_clearance uses, consumed by corridor_follower's turn-start gate
    and direction_estimator's corner-detection gate.

    Deliberately its own field, not FRONT_HALF_FOV_DEG: the two look
    interchangeable (both "how wide is forward") but are not -- FRONT_HALF_FOV_DEG
    (30 deg) is collision-avoidance's braking cone, sized to catch an obstacle
    with margin. This one gates *when a corridor counts as ending* for corner
    detection, where direction_estimator's own docstring warns the gap between
    this and TURN_CLEARANCE_M is a fragile, measured window: three fixtures
    never settled and two settled wrong when it was mistuned by less than this
    field's difference from FRONT_HALF_FOV_DEG alone. A prior refactor pointed
    _forward_clearance at FRONT_HALF_FOV_DEG to remove a hardcoded 8.0, on the
    reasonable-looking assumption that one "forward cone" tuning number should
    serve both -- widening the corner-detection cone by 3.75x broke the
    timing outright, reproduced as every clockwise narrow-corridor run timing
    out mid-lap while every counterclockwise one passed clean (an unrelated
    starting section makes the two directions meet their first corner at a
    different point in this timing window, so the same corruption did not
    fail identically)."""


class StartMeasurementParams(BaseModel):
    """LIDAR-based start-pose measurement (measure_start_pose) parameters.

    Attributes:
        RAY_HALF_WIDTH_DEG: Half-width (deg) of the wedge each cardinal
            distance (forward/back/left/right) is taken over. Wide enough to
            average out per-ray noise, narrow enough that the wedge still
            sees one wall -- at 2.5 m a 4 degree half-angle spans 17 cm of
            wall, well inside a one metre corridor. Stored in degrees, like
            LidarSectorParams above, and converted with math.radians() at
            point of use since TOML has no math functions.
        CLOSING_TOLERANCE_M: How far ``forward + back`` may fall short of the
            mat before the reading is rejected. Opposite rays along a
            corridor must span the mat, so their sum is a free validity
            check -- see measure_start_pose's own docstring for how this was
            sized from real scans.
        RETRY_WINDOW_S: How long after the direction commit the node keeps
            re-attempting a refused measurement. A refusal is transient: on
            both 2026-08-08 rounds that refused, an operator was standing in
            the rearward ray at 0.10-0.19 m, and the ray cleared 0.6 s and
            1.7 s after the commit respectively -- once the robot had driven
            out from under them. Sized to cover that with margin while still
            expiring well inside the first lap, since a measurement taken
            later is read in the starting section's frame and the robot is no
            longer in it.
        RETRY_ALIGN_TOLERANCE_DEG: How far the chassis may sit off the start
            corridor's travel bearing for a retried measurement to be
            believed. The closing check cannot stand in for this: a chassis
            turned through 180 degrees still spans the mat, so ``forward`` and
            ``back`` simply swap and the along-corridor coordinate comes out
            mirrored about the mat's centre. At the commit itself the heading
            is sound by construction (direction was just inferred from the
            same scan), so this gates only the retries.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    RAY_HALF_WIDTH_DEG: float = Field(default=4.0, validation_alias=_alias("RAY_HALF_WIDTH_DEG"))
    CLOSING_TOLERANCE_M: float = Field(default=0.15, validation_alias=_alias("CLOSING_TOLERANCE_M"))
    RETRY_WINDOW_S: float = Field(default=8.0, validation_alias=_alias("RETRY_WINDOW_S"))
    RETRY_ALIGN_TOLERANCE_DEG: float = Field(default=25.0, validation_alias=_alias("RETRY_ALIGN_TOLERANCE_DEG"))


class WallHeadingParams(BaseModel):
    """LIDAR wall-direction estimation for the blind heading reference.

    Fits short segments across the LIDAR returns and takes their common
    direction as the corridor's. Every threshold here decides whether a pair of
    returns describes one flat surface or two different things.

    Attributes:
        MIN_CONCENTRATION: How aligned the segment directions must be before
            the estimate is trusted at all. Low concentration means the
            returns disagree about where the wall runs, which is what a
            corner, a sign or a doorway looks like.
        BASELINE_RAYS: How far apart (in rays) the two returns forming one
            segment are taken. Wider is less noise-sensitive but blurs
            genuine corners.
        MAX_SEGMENT_JUMP_M: Range step above which two returns are treated as
            different surfaces rather than one wall.
        MIN_SEGMENT_M: Segments shorter than this are dominated by range
            noise rather than wall direction.
        NEAR_MAX_RANGE_M: Returns at or beyond this are no-return rays
            sanitised to max range, not real surfaces.
        MIN_RETURNS: Fewer usable returns than this cannot form a segment.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_CONCENTRATION: float = Field(default=0.55, validation_alias=_alias("MIN_CONCENTRATION"))
    BASELINE_RAYS: int = Field(default=15, validation_alias=_alias("BASELINE_RAYS"))
    MAX_SEGMENT_JUMP_M: float = Field(default=0.30, validation_alias=_alias("MAX_SEGMENT_JUMP_M"))
    MIN_SEGMENT_M: float = Field(default=0.02, validation_alias=_alias("MIN_SEGMENT_M"))
    NEAR_MAX_RANGE_M: float = Field(default=11.0, validation_alias=_alias("NEAR_MAX_RANGE_M"))
    MIN_RETURNS: int = Field(default=3, validation_alias=_alias("MIN_RETURNS"))

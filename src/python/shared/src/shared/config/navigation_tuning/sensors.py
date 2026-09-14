"""Sensor health, LIDAR sector, and wall-heading estimation tuning groups.

Fields are inherited from the generated DTOs under
:mod:`shared.config.generated.navigation.sensors`. These classes add only the
tuning layer's shipped fallbacks, so a bare instance still matches the
checked-in ``sensors/*.toml`` without re-declaring a field.
"""

from __future__ import annotations

from typing import Any, ClassVar

from shared.config.constants import RobotSpecs
from shared.config.generated.navigation.sensors.lidar_sectors_schema import (
    NavigationSensorsLidarSectors,
)
from shared.config.generated.navigation.sensors.sensor_schema import (
    NavigationSensorsSensor,
)
from shared.config.generated.navigation.sensors.start_measurement_schema import (
    NavigationSensorsStartMeasurement,
)
from shared.config.generated.navigation.sensors.wall_heading_schema import (
    NavigationSensorsWallHeading,
)
from shared.config.navigation_tuning._shared import TuningModel

STALE_TIMEOUT_SCAN_PERIODS: float = 5.0
"""How many LIDAR scan periods a cached reading may outlive.

The timeout is derived, not restated: 5 / RobotSpecs.LIDAR_UPDATE_RATE. A scan
period is the slowest feed the control loop depends on, so five of them is the
widest the staleness gate can be while still reacting to a sensor that died.
Change the model's update rate in robot.toml and this moves with it; the
shipped sensors/sensor.toml overrides only to restate, and its value and this
derivation are held equal by test_navigation_tuning's shipped-TOML contract.
"""


class SensorHealthParams(TuningModel, NavigationSensorsSensor):
    """Sensor dropout / staleness detection for the hardware gateway.

    ``stale_timeout_sec``: a cached sensor reading older than this is treated as
    a dropout -- the gateway reports it as unavailable so the navigator
    degrades safely instead of acting on frozen data. Derived as
    STALE_TIMEOUT_SCAN_PERIODS of the LIDAR scan period
    (RobotSpecs.LIDAR_UPDATE_RATE), the slowest sensor feed the control loop
    depends on.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "stale_timeout_sec": STALE_TIMEOUT_SCAN_PERIODS / RobotSpecs.LIDAR_UPDATE_RATE,
    }


class LidarSectorParams(TuningModel, NavigationSensorsLidarSectors):
    """LIDAR angular-sector definitions shared by collision avoidance and the OLED.

    ``front_half_fov_deg``: half-width (deg) of the forward clearance cone,
    used by CollisionAvoidanceController.compute_forward_clearance.
    ``threat_half_fov_deg``: half-width (deg) of the threat-detection sectors
    (front/left/right/back), used by detect_threat_direction and related
    methods -- a narrower cone than the forward clearance cone above; both are
    min-based.
    ``self_detection_threshold_m``: rays no farther than this are discarded as
    chassis/cable self-reflection when a sector filters for it.
    ``min_valid_range_m``: LIDAR ranges at or below this are treated as invalid
    (no-return) readings. Must sit strictly below RobotSpecs.LIDAR_MIN_RANGE
    (the C1's real rated minimum), because a floor reading is discarded as
    invalid otherwise.
    ``threat_no_detection_range_m``: a sector's nearest reading beyond this
    distance doesn't count as a threat at all.
    ``blind_wedge_*_deg``: bearing ranges where the rear mount structurally
    occludes the LIDAR -- rays here self-collide regardless of range, so they
    are masked by angle rather than distance. The mount is asymmetric.
    ``no_data_range_m``: fallback range (m) when no valid LIDAR readings are
    available; a sentinel between min and max sensor range.
    ``direction_arc_half_fov_deg``: half-width (deg) of the narrow forward cone
    src.navigation.utils' _forward_clearance uses, consumed by
    corridor_follower's turn-start gate and direction_estimator's
    corner-detection gate. Deliberately separate from front_half_fov_deg --
    see the generated field description for why widening it broke the timing.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "front_half_fov_deg": 30.0,
        "threat_half_fov_deg": 45.0,
        "self_detection_threshold_m": 0.08,
        "rear_self_detection_from_chassis": True,
        "min_valid_range_m": 0.044,
        "blind_wedge_left_min_deg": -155.0,
        "blind_wedge_left_max_deg": -120.0,
        "blind_wedge_right_min_deg": 120.0,
        "blind_wedge_right_max_deg": 160.0,
        "threat_no_detection_range_m": 1.0,
        "no_data_range_m": 10.0,
        "direction_arc_half_fov_deg": 8.0,
    }


class StartMeasurementParams(TuningModel, NavigationSensorsStartMeasurement):
    """LIDAR-based start-pose measurement (measure_start_pose) parameters.

    ``ray_half_width_deg``: half-width (deg) of the wedge each cardinal
    distance (forward/back/left/right) is taken over. Stored in degrees, like
    LidarSectorParams, and converted with math.radians() at point of use.
    ``closing_tolerance_m``: how far ``forward + back`` may fall short of the
    mat before the reading is rejected.
    ``retry_window_s``: how long after the direction commit the node keeps
    re-attempting a refused measurement.
    ``retry_align_tolerance_deg``: how far the chassis may sit off the start
    corridor's travel bearing for a retried measurement to be believed.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "ray_half_width_deg": 4.0,
        "closing_tolerance_m": 0.15,
        "retry_window_s": 8.0,
        "retry_align_tolerance_deg": 25.0,
    }


class WallHeadingParams(TuningModel, NavigationSensorsWallHeading):
    """LIDAR wall-direction estimation for the blind heading reference.

    Fits short segments across the LIDAR returns and takes their common
    direction as the corridor's. Every threshold here decides whether a pair of
    returns describes one flat surface or two different things.

    ``min_concentration``: how aligned the segment directions must be before
    the estimate is trusted at all.
    ``baseline_rays``: how far apart (in rays) the two returns forming one
    segment are taken.
    ``max_segment_jump_m``: range step above which two returns are treated as
    different surfaces rather than one wall.
    ``min_segment_m``: segments shorter than this are dominated by range noise
    rather than wall direction.
    ``near_max_range_m``: returns at or beyond this are no-return rays
    sanitised to max range, not real surfaces.
    ``min_returns``: fewer usable returns than this cannot form a segment.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "min_concentration": 0.55,
        "baseline_rays": 15,
        "max_segment_jump_m": 0.30,
        "min_segment_m": 0.02,
        "near_max_range_m": 11.0,
        "min_returns": 3,
    }

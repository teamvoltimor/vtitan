"""Sensor-range to bumper-gap conversions for collision avoidance.

Clearance thresholds are statements about the chassis ("do not come within 10
cm of something"), but the LIDAR reports distance from its own mount. These
helpers convert a raw sensor range into the gap from the relevant bumper so the
policy comparison stays anchored to the chassis regardless of mount position.
"""

from __future__ import annotations

from shared.config.constants import RobotSpecs


def bumper_gap_ahead(range_m: float) -> float:
    """Convert a FORWARD sensor range into the gap from the front bumper.

    Clearance thresholds are policy statements about the CHASSIS ("do not get
    within 10 cm of something"), but the LIDAR reports distance from itself, and
    the sensor is not at the chassis centre. Comparing a threshold against a raw
    range therefore measures it from wherever the sensor is mounted, so moving
    the mount silently redefines every threshold.

    Converting here keeps the sector helpers' documented contract (they return
    raw sensor ranges) and puts the frame change in the policy comparison, where
    it belongs. See ``adr:0056-raw-and-masked-scan``.

    Approximate off-axis: the offset is exact straight ahead and shortens with
    bearing, so within the +/-30 deg forward cone this is conservative by at
    most a few millimetres. Good enough for a threshold quoted to a centimetre.
    """
    return range_m - RobotSpecs.LIDAR_TO_FRONT_BUMPER


def bumper_gap_behind(range_m: float) -> float:
    """Convert a REAR sensor range into the gap from the rear bumper.

    Subtracts ~0.272 m rather than the front's ~0.028 m, because the sensor sits
    at the front. Without this a rear threshold is unreachable: an obstacle
    touching the rear bumper reports 0.272 m, so the shipped contact distance
    could never fire and the reverse guard could not stop a reverse before
    impact. See ``adr:0055-escape-maneuver-selection``.
    """
    return range_m - RobotSpecs.LIDAR_TO_REAR_BUMPER

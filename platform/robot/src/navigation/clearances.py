"""Build a :class:`LidarClearances` from a robot-frame :class:`LidarScan`.

Single construction point so every consumer (the OLED telemetry bridge, the
navigator's collision gates, tests) derives the four directional clearances the
same way instead of each re-spelling the sector math against
``CollisionAvoidanceController``.
"""

from __future__ import annotations

import enum
import math
from typing import TYPE_CHECKING, Protocol

import numpy as np
from shared.domain.enums import ThreatDirection
from shared.domain.models import LidarClearances

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan


class ClearanceAggregate(enum.Enum):
    """How a sector's valid rays collapse to a single clearance number.

    ``MEAN`` is for display (a single noisy return shouldn't dominate the
    readout); ``MIN`` is for threat detection (the worst case in the cone is the
    one that matters for collision avoidance).
    """

    MEAN = "mean"
    MIN = "min"


def clearances_from_scan(
    scan: LidarScan,
    controller: CollisionAvoidanceControllerProtocol,
    front_half_fov_rad: float,
    aggregate: ClearanceAggregate = ClearanceAggregate.MEAN,
) -> LidarClearances:
    """Return four-sided LIDAR clearances (metres) for ``scan``.

    Front/left/right are the aggregated reading across each sector. ``MEAN``
    (default) matches the OLED display's existing behaviour -- far less
    sensitive to a single noisy return than min-of-window. ``MIN`` matches
    ``detect_threat_direction``'s threat sectors, for use in collision logic.
    The back sector always uses the controller's own min-based rear-cone logic.

    Args:
        scan: Robot-frame sweep (0 rad = forward, +pi/2 = left).
        controller: A collision-avoidance controller exposing ``sector_ranges``
            and ``compute_rear_clearance``.
        front_half_fov_rad: Half-width of the forward/side sectors (radians).
        aggregate: ``MEAN`` for display, ``MIN`` for threat detection.
    """
    if not scan.ranges_m:
        return LidarClearances(front_m=0.0, left_m=0.0, right_m=0.0, back_m=0.0)

    ranges = tuple(scan.ranges_m)
    angles = tuple(scan.angles_rad)

    front = controller.sector_ranges(ranges, angles, 0.0, front_half_fov_rad)
    left = controller.sector_ranges(
        ranges, angles, math.pi / 2, front_half_fov_rad, filter_self_detection=True,
    )
    right = controller.sector_ranges(
        ranges, angles, -math.pi / 2, front_half_fov_rad, filter_self_detection=True,
    )

    reducer = np.min if aggregate is ClearanceAggregate.MIN else np.mean
    front_m = float(reducer(front)) if front.size else 0.0
    left_m = float(reducer(left)) if left.size else 0.0
    right_m = float(reducer(right)) if right.size else 0.0
    back_m = controller.compute_rear_clearance(ranges, angles)

    return LidarClearances(front_m=front_m, left_m=left_m, right_m=right_m, back_m=back_m)


def threat_direction(clearances: LidarClearances, no_detection_range_m: float) -> ThreatDirection:
    """Map ``clearances`` to a :class:`ThreatDirection` for escape logic.

    Returns the most-constrained side when anything is within
    ``no_detection_range_m``, else ``NONE`` -- the same gate
    ``CollisionAvoidanceController.detect_threat_direction`` applies, but driven
    off a shared :class:`LidarClearances` so the navigator builds the scan's
    directional picture exactly once.
    """
    if clearances.any_blocked(no_detection_range_m):
        return clearances.most_constrained_side
    return ThreatDirection.NONE


class CollisionAvoidanceControllerProtocol(Protocol):
    """Structural type for the controller surface :func:`clearances_from_scan` needs."""

    def sector_ranges(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
        center_rad: float,
        half_fov_rad: float,
        filter_self_detection: bool = False,
    ) -> np.ndarray:
        """Return valid ranges within ``center ± half_fov``."""
        ...

    def compute_rear_clearance(
        self, lidar_ranges: np.ndarray | tuple[float, ...], lidar_angles: np.ndarray | tuple[float, ...] | None
    ) -> float:
        """Return minimum rear-sector clearance (m), or ``no_data_range_m`` if unseen."""
        ...

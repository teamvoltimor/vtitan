"""Build a :class:`LidarClearances` from a robot-frame :class:`LidarScan`.

Single construction point so every consumer (the OLED telemetry bridge, the
navigator's collision gates, tests) derives the four directional clearances the
same way instead of each re-spelling the sector-mean math against
``CollisionAvoidanceController``.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from shared.domain.models import LidarClearances

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan


def clearances_from_scan(
    scan: LidarScan,
    controller: CollisionAvoidanceControllerProtocol,
    front_half_fov_rad: float,
) -> LidarClearances:
    """Return four-sided LIDAR clearances (meters) for ``scan``.

    Front/left/right are the mean reading across each sector (matching the OLED
    display's existing behaviour -- mean-over-sector is far less sensitive to a
    single noisy return than min-of-window). The back sector uses the
    controller's own rear-cone logic; when the rear saw nothing it reads as
    ``0.0`` (open road) so the field stays populated for ``any_blocked`` /
    ``most_constrained_side`` without authorising a reverse on its own.

    Args:
        scan: Robot-frame sweep (0 rad = forward, +pi/2 = left).
        controller: A collision-avoidance controller exposing
            ``sector_ranges`` and ``compute_rear_clearance``.
        front_half_fov_rad: Half-width of the forward/side sectors (radians),
            sourced from ``NavigationTuning.lidar_sectors.FRONT_HALF_FOV_DEG``.
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

    front_m = float(np.mean(front)) if front.size else 0.0
    left_m = float(np.mean(left)) if left.size else 0.0
    right_m = float(np.mean(right)) if right.size else 0.0
    back_m = controller.compute_rear_clearance(ranges, angles)

    return LidarClearances(front_m=front_m, left_m=left_m, right_m=right_m, back_m=back_m)


class CollisionAvoidanceControllerProtocol:
    """Structural type for the controller surface :func:`clearances_from_scan` needs."""

    def sector_ranges(
        self,
        lidar_ranges: object,
        lidar_angles: object,
        center_rad: float,
        half_fov_rad: float,
        filter_self_detection: bool = False,
    ) -> np.ndarray:
        """Return valid ranges within ``center ± half_fov``."""

    def compute_rear_clearance(self, lidar_ranges: object, lidar_angles: object) -> float:
        """Return minimum rear-sector clearance (m), or ``no_data_range_m`` if unseen."""

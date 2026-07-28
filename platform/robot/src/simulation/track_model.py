"""Track geometry model for the headless Open Challenge simulation.

Rebuilds the exact wall geometry the Go scenario generator emits
(``platform/gazebo/generator``) from a scenario's corridor widths, then
exposes the two queries the simulated car needs:

* :meth:`TrackModel.raycast_scan` — a simulated Slamtec-C1 LIDAR sweep.
* :meth:`TrackModel.footprint_collides` — chassis-vs-wall collision.

Two wall representations are kept, mirroring the generator's split between
*visual* meshes (0.10 m thick) and *collision* meshes (0.18 m thick):

* **Visual faces** — the outer track boundary at 0/3 m and the inner block at
  the corridor-width lines. The LIDAR rangefinder hits these surfaces.
* **Collision faces** — the same walls inflated inward by half the difference
  between collision and visual thickness (``0.09 - 0.05 = 0.04 m``). The
  chassis footprint must never overlap these. This is what makes the
  ``0.18 m`` collision mesh slightly "fatter" than what the LIDAR sees.

Geometry recap (WRO 2026, bottom-left origin, 3.0 x 3.0 m track):

    south face of inner block  y = south_width
    north face of inner block  y = 3 - north_width
    west  face of inner block  x = west_width
    east  face of inner block  x = 3 - east_width
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import (
    DictKeys,
    ParkingLotSpecs,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
)

from shared.domain.models import CorridorGeometry
from src.navigation.track_geometry import TrackWalls

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.enums import Section

# |cos(yaw)| below this counts as a quarter-turn, so a block's extents are
# swapped rather than treated as axis-aligned.
_AXIS_ALIGN_TOLERANCE = 1e-6

# Wall thickness halves (metres) — straight from the generator's constants:
# WallThickness = 0.10 (visual), WallCollisionThickness = 0.18 (collision).
_WALL_VISUAL_HALF = 0.05
_WALL_COLLISION_HALF = 0.09
# How much further the collision mesh protrudes past the visual face.
_COLLISION_MARGIN = _WALL_COLLISION_HALF - _WALL_VISUAL_HALF  # 0.04 m


class ContactSurface(StrEnum):
    """What the chassis is touching.

    Kept distinct because the challenges forbid different walls — see
    :meth:`TrackModel.contact_surface`.
    """

    NONE = "none"
    OUTER_WALL = "outer_wall"
    INNER_WALL = "inner_wall"
    OBSTACLE = "obstacle"
    """A traffic sign or parking block, which belongs to neither wall."""

_TRACK_MIN = 0.0
_TRACK_MAX = TrackDimensions.MAX_COORD  # 3.0


@dataclass(frozen=True, slots=True)
class _Box:
    """An axis-aligned keep-out box (for footprint collision)."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def corners(self) -> list[tuple[float, float]]:
        """Return the four box corners (CCW from bottom-left)."""
        return [
            (self.x_min, self.y_min),
            (self.x_max, self.y_min),
            (self.x_max, self.y_max),
            (self.x_min, self.y_max),
        ]


@dataclass(frozen=True, slots=True)
class ObstacleBox:
    """A ground obstacle — a traffic sign or a parking block.

    Both are short boxes standing on the mat, so they are modelled the same
    way: an axis-aligned footprint the chassis can hit and the LIDAR can see.
    ``yaw`` is only ever 0 or +-90 degrees for the objects the WRO generator
    emits, so a rotated block is represented by swapping its extents rather
    than carrying a general oriented box through the raycast maths.
    """

    cx: float
    cy: float
    size_x: float
    size_y: float

    @classmethod
    def from_pose(cls, cx: float, cy: float, length: float, width: float, yaw: float = 0.0) -> ObstacleBox:
        """Build a box from a centre pose, swapping extents for a quarter-turn ``yaw``."""
        quarter_turned = abs(math.cos(yaw)) < _AXIS_ALIGN_TOLERANCE
        size_x, size_y = (width, length) if quarter_turned else (length, width)
        return cls(cx=cx, cy=cy, size_x=size_x, size_y=size_y)

    def to_box(self, margin: float = 0.0) -> _Box:
        """Return the axis-aligned bounds, optionally grown by ``margin``."""
        half_x = self.size_x / 2.0 + margin
        half_y = self.size_y / 2.0 + margin
        return _Box(self.cx - half_x, self.cy - half_y, self.cx + half_x, self.cy + half_y)


def obstacles_from_metadata(metadata: dict) -> list[ObstacleBox]:
    """Collect every physical obstacle in a scenario: traffic signs and parking blocks.

    Open Challenge metadata has neither, so this returns an empty list and the
    resulting :class:`TrackModel` behaves exactly as before.
    """
    boxes = [
        ObstacleBox.from_pose(
            cx=float(sign[DictKeys.X]),
            cy=float(sign[DictKeys.Y]),
            length=TrafficSignSpecs.WIDTH,
            width=TrafficSignSpecs.DEPTH,
        )
        for sign in metadata.get(DictKeys.SIGN_POSITIONS, [])
    ]

    parking = metadata.get(DictKeys.PARKING_LOT)
    if parking:
        for pos_key, yaw_key in (("block1_position", "block1_yaw"), ("block2_position", "block2_yaw")):
            block = parking[pos_key]
            boxes.append(
                ObstacleBox.from_pose(
                    cx=float(block[DictKeys.X]),
                    cy=float(block[DictKeys.Y]),
                    length=ParkingLotSpecs.LENGTH,
                    width=ParkingLotSpecs.WIDTH,
                    yaw=float(parking.get(yaw_key, 0.0)),
                ),
            )
    return boxes


class TrackModel:
    """Wall geometry + sensor/collision queries for one Open Challenge layout."""

    def __init__(
        self,
        geometry: CorridorGeometry,
        obstacles: Sequence[ObstacleBox] | None = None,
        lidar_sees_obstacles: bool = True,
    ) -> None:
        """Build the track from corridor geometry.

        Args:
            geometry: Complete corridor layout including widths and inner block.
            obstacles: Traffic signs and parking blocks standing on the mat.
                Empty for the Open Challenge, which has neither.
            lidar_sees_obstacles: Whether obstacles occlude LIDAR rays. Both
                signs and parking blocks are 0.10 m tall — exactly the chassis
                height — so a deck-mounted C1 scans right at their top edge and
                real-world detection is marginal. Defaults to modelling them as
                visible; set False to simulate a LIDAR mounted above them, in
                which case the camera (mounted higher and pitched down) is the
                only sensor that perceives them.
        """
        self._walls = TrackWalls(geometry)
        self._obstacles = list(obstacles or [])
        self._lidar_sees_obstacles = lidar_sees_obstacles
        # Signs and parking blocks are small, rigid and modelled at their true
        # size — unlike the walls there is no separate fatter collision mesh,
        # so visual and collision bounds are the same box.
        self._obstacle_boxes = [ob.to_box() for ob in self._obstacles]

        inner = self._walls.inner_block
        self._inner_visual = _Box(inner.x_min, inner.y_min, inner.x_max, inner.y_max)
        self._inner_collision = _Box(
            inner.x_min - _COLLISION_MARGIN,
            inner.y_min - _COLLISION_MARGIN,
            inner.x_max + _COLLISION_MARGIN,
            inner.y_max + _COLLISION_MARGIN,
        )
        # Footprint must stay within this outer collision boundary.
        self._outer_collision = _Box(
            _TRACK_MIN + _COLLISION_MARGIN,
            _TRACK_MIN + _COLLISION_MARGIN,
            _TRACK_MAX - _COLLISION_MARGIN,
            _TRACK_MAX - _COLLISION_MARGIN,
        )

    @property
    def walls(self) -> TrackWalls:
        """The wall geometry, for code that needs to predict scans from a pose.

        Exposed for :class:`~src.navigation.localization.LidarLocalizer`, which
        matches a real sweep against a predicted one — the same object the ROS2
        node builds for itself from scenario metadata.
        """
        return self._walls

    # Simulated LIDAR

    def raycast_scan(
        self,
        x: float,
        y: float,
        yaw: float,
        angles_robot: np.ndarray,
        max_range: float = RobotSpecs.LIDAR_MAX_RANGE,
    ) -> np.ndarray:
        """Cast a fan of rays and return the nearest wall range per ray.

        Args:
            x: Sensor world X (metres).
            y: Sensor world Y (metres).
            yaw: Robot heading (radians).
            angles_robot: Ray bearings in the robot frame (radians, 0 = forward).
            max_range: Sensor ceiling; rays that hit nothing return this.

        Returns:
            Range (metres) for each bearing, clamped to ``[LIDAR_MIN_RANGE, max_range]``.
        """
        ranges = self._walls.raycast(x, y, yaw, angles_robot, max_range)
        if not (self._lidar_sees_obstacles and self._obstacle_boxes):
            return ranges

        # An obstacle only shortens a ray — never lengthens it — so fold each
        # box in with an elementwise minimum against the wall ranges.
        world_ang = yaw + angles_robot
        dx = np.cos(world_ang)
        dy = np.sin(world_ang)
        for box in self._obstacle_boxes:
            hits = _raycast_box(x, y, dx, dy, box, max_range)
            np.minimum(ranges, hits, out=ranges)
        return np.clip(ranges, RobotSpecs.LIDAR_MIN_RANGE, max_range)

    # Collision

    def footprint_collides(
        self,
        x: float,
        y: float,
        yaw: float,
        length: float = RobotSpecs.LENGTH,
        width: float = RobotSpecs.WIDTH,
    ) -> bool:
        """Return ``True`` if the oriented chassis rectangle hits a wall or an obstacle."""
        return self.contact_surface(x, y, yaw, length, width) is not ContactSurface.NONE

    def contact_surface(
        self,
        x: float,
        y: float,
        yaw: float,
        length: float = RobotSpecs.LENGTH,
        width: float = RobotSpecs.WIDTH,
    ) -> ContactSurface:
        """Which surface the oriented chassis rectangle is touching, if any.

        The caller needs the distinction because the two challenges forbid
        different walls: the Open Challenge is scored on not touching the
        *outer* wall, the Obstacles Challenge on not touching the *inner* one.
        Collapsing all three surfaces into one boolean makes both rules
        unrepresentable.

        Checked outer first, then inner, then obstacles. The order only decides
        what a simultaneous multi-surface contact reports, which is a wedged
        robot either way.
        """
        corners = _rect_corners(x, y, yaw, length, width)

        # Outer boundary: every corner must stay inside the collision box.
        ob = self._outer_collision
        for cx, cy in corners:
            if cx < ob.x_min or cx > ob.x_max or cy < ob.y_min or cy > ob.y_max:
                return ContactSurface.OUTER_WALL

        # Inner block: oriented footprint must not overlap the keep-out box.
        if _convex_overlap(corners, self._inner_collision.corners(), yaw):
            return ContactSurface.INNER_WALL

        if any(_convex_overlap(corners, box.corners(), yaw) for box in self._obstacle_boxes):
            return ContactSurface.OBSTACLE
        return ContactSurface.NONE

    # Geometry helpers exposed for tests / planners

    def point_in_free_space(self, x: float, y: float, clearance: float = 0.0) -> bool:
        """Return ``True`` if (x, y) is in the navigable ring with ``clearance`` margin.

        Used to validate that planned waypoints sit inside the corridor with at
        least ``clearance`` metres to the nearest *visual* wall.
        """
        if not (
            _TRACK_MIN + clearance <= x <= _TRACK_MAX - clearance
            and _TRACK_MIN + clearance <= y <= _TRACK_MAX - clearance
        ):
            return False
        iv = self._inner_visual
        # Inside the inner block (with clearance shrinking the safe corridor) -> blocked.
        return not (iv.x_min + clearance < x < iv.x_max - clearance and iv.y_min + clearance < y < iv.y_max - clearance)

    @property
    def inner_block_visual(self) -> tuple[float, float, float, float]:
        """Inner-block visual bounds ``(x_min, y_min, x_max, y_max)`` in metres."""
        iv = self._inner_visual
        return (iv.x_min, iv.y_min, iv.x_max, iv.y_max)


def _raycast_box(
    x: float,
    y: float,
    dx: np.ndarray,
    dy: np.ndarray,
    box: _Box,
    max_range: float,
) -> np.ndarray:
    """Distance from ``(x, y)`` to an axis-aligned box, per ray, vectorised.

    Standard slab method: intersect the ray against the box's x- and y-bounded
    strips and keep the overlap. Rays that miss (or that only hit behind the
    sensor) return ``max_range`` so the caller's elementwise minimum leaves
    them untouched.
    """
    # errstate: rays exactly parallel to an axis divide by zero here, which is
    # well-defined for the slab method (+-inf correctly means "never leaves
    # this strip") — only the warning is unwanted.
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1 = (box.x_min - x) / dx
        tx2 = (box.x_max - x) / dx
        ty1 = (box.y_min - y) / dy
        ty2 = (box.y_max - y) / dy

    t_near = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    t_far = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))

    # A hit needs the slabs to overlap and the exit point to be in front of the
    # sensor. Starting inside the box yields t_near < 0, reported as range 0.
    hit = np.isfinite(t_near) & (t_far >= np.maximum(t_near, 0.0))
    distance = np.maximum(t_near, 0.0)
    return np.where(hit, distance, max_range)


def _rect_corners(
    cx: float,
    cy: float,
    yaw: float,
    length: float,
    width: float,
) -> list[tuple[float, float]]:
    """Four corners of an oriented rectangle centred at (cx, cy)."""
    hl, hw = length / 2.0, width / 2.0
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    local = ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
    return [(cx + lx * cos_y - ly * sin_y, cy + lx * sin_y + ly * cos_y) for lx, ly in local]


def _convex_overlap(
    poly_a: list[tuple[float, float]],
    poly_b: list[tuple[float, float]],
    yaw: float,
) -> bool:
    """Separating-axis test between two convex polygons (rect vs AABB).

    Axes tested: the oriented rectangle's two edge normals plus the world X/Y
    axes (the AABB's normals). Exact for rectangle-vs-box overlap.
    """
    axes = (
        (np.cos(yaw), np.sin(yaw)),
        (-np.sin(yaw), np.cos(yaw)),
        (1.0, 0.0),
        (0.0, 1.0),
    )
    for ax, ay in axes:
        a_proj = [px * ax + py * ay for px, py in poly_a]
        b_proj = [px * ax + py * ay for px, py in poly_b]
        if max(a_proj) < min(b_proj) or max(b_proj) < min(a_proj):
            return False  # found a separating axis -> no overlap
    return True

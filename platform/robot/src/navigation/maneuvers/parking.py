"""Parallel-park controller for WRO 2026 obstacles challenge.

Drives the robot into the gap between the two magenta parking blocks after
completing 3 laps.  Uses a two-phase pure-pursuit strategy:

  Phase 1 — STAGE : drive to a staging position directly in front of the gap
                    opening (perpendicular approach from the track side).
  Phase 2 — ENTER : drive straight into the gap, correcting heading toward
                    the gap centre.

Stop condition: chassis centre inside the parking-zone bounding box AND
yaw within ±10° (0.1745 rad) of the expected parking angle.

Pure Python — no ROS2 dependencies. Unit testable.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import StrEnum

from shared.config.constants import ParkingLotSpecs
from shared.config.enums import Section

logger = logging.getLogger(__name__)

_YAW_TOLERANCE = math.radians(10.0)  # ±10° stop condition
_APPROACH_CLEARANCE = 0.25  # metres: staging position above gap opening
_POS_REACH_DIST = 0.04  # metres: "reached staging" threshold
_INSIDE_TOLERANCE = 0.02  # metres: zone wall clearance


@dataclass(frozen=True)
class ParkZone:
    """Bounding box and parking angle for the zone."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    target_yaw: float  # expected robot yaw when parked (radians)
    gap_cx: float  # lateral centre of gap (world x or y)
    gap_cy: float  # depth centre of gap


@dataclass
class ParkCommand:
    """Motor command from the park controller."""

    linear: float  # m/s
    steering: float  # normalised [-1, 1]
    done: bool = False
    phase: str = "stage"


class ParkPhase(StrEnum):
    """Parking maneuver phases."""

    STAGE = "stage"
    ENTER = "enter"
    DONE = "done"


class ParkController:
    """Two-phase park controller.

    Phase 1 — STAGE: pure-pursuit toward the staging position directly in
        front of the gap opening (on the track side).
    Phase 2 — ENTER: pure-pursuit toward the gap centre; switches to DONE
        when both position and yaw are within tolerance.

    Args:
        parking_config: Dict with 'block1_pos' and 'block2_pos' keys.
        start_section: Corridor that contains the parking lot.
        speed: Constant driving speed (m/s).
        steer_kp: Proportional heading gain.
    """

    def __init__(
        self,
        parking_config: dict,
        start_section: Section,
        speed: float = 0.12,
        steer_kp: float = 2.5,
    ) -> None:
        self._section = start_section
        self._speed = speed
        self._steer_kp = steer_kp
        self._phase = ParkPhase.STAGE

        b1 = parking_config["block1_pos"]
        b2 = parking_config["block2_pos"]
        self._zone = _build_zone(b1, b2, start_section)
        self._staging = _staging_pos(self._zone, start_section)

        logger.info(
            "ParkController: zone=%s staging=%s section=%s",
            self._zone,
            self._staging,
            start_section,
        )

    @property
    def is_done(self) -> bool:
        """Whether the parking maneuver is complete."""
        return self._phase is ParkPhase.DONE

    @property
    def section(self) -> Section:
        """Corridor that contains the parking lot."""
        return self._section

    @property
    def staging(self) -> tuple[float, float]:
        """Staging position in front of the gap opening (world x, y)."""
        return self._staging

    def update(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        """Compute next motor command.

        Args:
            robot_pos: Current (x, y) world position of robot centre.
            robot_yaw: Current heading (radians, 0=east, π/2=north).

        Returns:
            ParkCommand with speed, normalised steering, and done flag.
        """
        if self._phase is ParkPhase.DONE:
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        # Early exit: if already inside the zone at any phase, we're done.
        pos_inside, yaw_ok = _inside_zone(robot_pos[0], robot_pos[1], robot_yaw, self._zone)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — already inside zone")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        if self._phase is ParkPhase.STAGE:
            return self._handle_stage(robot_pos, robot_yaw)
        return self._handle_enter(robot_pos, robot_yaw)

    # ── Phase handlers ────────────────────────────────────────────────────────

    def _handle_stage(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        tx, ty = self._staging
        rx, ry = robot_pos
        dist = math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2)

        if dist < _POS_REACH_DIST:
            logger.debug("ParkController: STAGE → ENTER")
            self._phase = ParkPhase.ENTER
            return self._handle_enter(robot_pos, robot_yaw)

        steer = _pursuit_steer(robot_pos, robot_yaw, (tx, ty), self._steer_kp)
        return ParkCommand(linear=self._speed, steering=steer, phase="stage")

    def _handle_enter(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        z = self._zone
        rx, ry = robot_pos

        pos_inside, yaw_ok = _inside_zone(rx, ry, robot_yaw, z)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — inside zone, yaw ok")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        steer = _pursuit_steer(
            robot_pos,
            robot_yaw,
            (z.gap_cx, z.gap_cy),
            self._steer_kp,
        )
        return ParkCommand(linear=self._speed, steering=steer, phase="enter")


# ── Pure helpers ──────────────────────────────────────────────────────────────


def _build_zone(
    b1: tuple[float, float],
    b2: tuple[float, float],
    section: Section,
) -> ParkZone:
    """Compute parking bounding box from block positions."""
    hw = ParkingLotSpecs.WIDTH / 2  # half block width (10 mm)
    hl = ParkingLotSpecs.LENGTH / 2  # half block length (100 mm)

    if section in (Section.SOUTH, Section.NORTH):
        x1, x2 = sorted([b1[0], b2[0]])
        gap_cx = (b1[0] + b2[0]) / 2
        gap_cy = (b1[1] + b2[1]) / 2
        x_min = x1 + hw
        x_max = x2 - hw
        if section is Section.SOUTH:
            y_min = 0.0
            y_max = gap_cy + hl + _INSIDE_TOLERANCE
            target_yaw = -math.pi / 2
        else:
            y_min = gap_cy - hl - _INSIDE_TOLERANCE
            y_max = 3.0
            target_yaw = math.pi / 2
    else:
        y1, y2 = sorted([b1[1], b2[1]])
        gap_cx = (b1[0] + b2[0]) / 2
        gap_cy = (b1[1] + b2[1]) / 2
        y_min = y1 + hw
        y_max = y2 - hw
        if section is Section.EAST:
            x_min = gap_cx - hl - _INSIDE_TOLERANCE
            x_max = 3.0
            target_yaw = 0.0
        else:
            x_min = 0.0
            x_max = gap_cx + hl + _INSIDE_TOLERANCE
            target_yaw = math.pi

    return ParkZone(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        target_yaw=target_yaw,
        gap_cx=gap_cx,
        gap_cy=gap_cy,
    )


def _staging_pos(zone: ParkZone, section: Section) -> tuple[float, float]:
    """Position directly in front of the gap opening, on the track side."""
    if section is Section.SOUTH:
        return zone.gap_cx, zone.y_max + _APPROACH_CLEARANCE
    if section is Section.NORTH:
        return zone.gap_cx, zone.y_min - _APPROACH_CLEARANCE
    if section is Section.EAST:
        return zone.x_min - _APPROACH_CLEARANCE, zone.gap_cy
    return zone.x_max + _APPROACH_CLEARANCE, zone.gap_cy  # WEST


def _pursuit_steer(
    robot_pos: tuple[float, float],
    robot_yaw: float,
    target: tuple[float, float],
    kp: float,
) -> float:
    """Proportional heading controller toward target point."""
    dx = target[0] - robot_pos[0]
    dy = target[1] - robot_pos[1]
    desired_yaw = math.atan2(dy, dx)
    err = _normalise_angle(desired_yaw - robot_yaw)
    return _clamp(kp * err, -1.0, 1.0)


def _inside_zone(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> tuple[bool, bool]:
    """Return (position_inside, yaw_ok)."""
    pos_inside = zone.x_min <= rx <= zone.x_max and zone.y_min <= ry <= zone.y_max
    yaw_err = abs(_normalise_angle(robot_yaw - zone.target_yaw))
    return pos_inside, yaw_err <= _YAW_TOLERANCE


def _normalise_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def park_controller_from_metadata(
    metadata: dict,
    start_section: Section,
) -> ParkController | None:
    """Construct a ParkController from scenario metadata. None for open challenge."""
    parking = metadata.get("parking_lot")
    if parking is None:
        return None
    b1 = (parking["block1_position"]["x"], parking["block1_position"]["y"])
    b2 = (parking["block2_position"]["x"], parking["block2_position"]["y"])
    return ParkController(
        parking_config={"block1_pos": b1, "block2_pos": b2},
        start_section=start_section,
    )

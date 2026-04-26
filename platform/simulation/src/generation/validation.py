"""Scenario geometry validation for WRO 2026 simulation.

Validates generated scenarios before writing to disk to prevent invalid
world files from entering the training pipeline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from shared.config.constants import (
    ParkingLotSpecs,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
)
from shared.config.types import ParkingLotConfig, StartingConditions


@dataclass(frozen=True)
class Violation:
    """A single geometry constraint violation."""

    rule: str
    message: str


@dataclass(frozen=True)
class WorldContext:
    """All resolved scenario parameters needed for geometry validation."""

    corridor_widths: dict  # dict[Section, dict[str, Any]]
    sign_positions: list[tuple[float, float]]
    sign_colors: list[tuple[str, list[float]]]
    parking_config: ParkingLotConfig | None
    starting_conditions: StartingConditions


# Minimum separation between two traffic sign centres (m).
_SIGN_MIN_SPACING = TrafficSignSpecs.WIDTH * 3.0

# Minimum clearance between a sign centre and a parking block centre (m).
_SIGN_PARKING_MIN_DIST = ParkingLotSpecs.LENGTH / 2 + TrafficSignSpecs.DEPTH / 2 + 0.05

# Minimum clearance between robot spawn centre and a sign centre (m).
_SPAWN_SIGN_MIN_DIST = RobotSpecs.LENGTH / 2 + TrafficSignSpecs.DEPTH / 2 + 0.05

# Minimum clearance between robot spawn centre and a parking block centre (m).
_SPAWN_PARKING_MIN_DIST = RobotSpecs.LENGTH / 2 + ParkingLotSpecs.LENGTH / 2 + 0.05

# Signs must stay this far inside the track boundary.
_SIGN_BOUNDARY_MARGIN = TrafficSignSpecs.WIDTH / 2


def validate_scenario(ctx: WorldContext) -> list[Violation]:
    """Validate scenario geometry.

    Args:
        ctx: Resolved scenario parameters.

    Returns:
        List of Violations. Empty list means scenario is valid.
    """
    violations: list[Violation] = []
    violations.extend(_check_sign_bounds(ctx.sign_positions))
    violations.extend(_check_sign_overlap(ctx.sign_positions))
    if ctx.parking_config is not None:
        violations.extend(_check_parking_bounds(ctx.parking_config))
        violations.extend(_check_sign_parking_clearance(ctx.sign_positions, ctx.parking_config))
    violations.extend(_check_robot_spawn_clearance(ctx.starting_conditions, ctx.sign_positions, ctx.parking_config))
    return violations


def _check_sign_bounds(sign_positions: list[tuple[float, float]]) -> list[Violation]:
    violations: list[Violation] = []
    lo = TrackDimensions.MIN_COORD + _SIGN_BOUNDARY_MARGIN
    hi = TrackDimensions.MAX_COORD - _SIGN_BOUNDARY_MARGIN
    for i, (x, y) in enumerate(sign_positions):
        if not (lo <= x <= hi and lo <= y <= hi):
            violations.append(Violation(
                rule="sign_bounds",
                message=f"Sign {i} at ({x:.3f}, {y:.3f}) outside track bounds [{lo:.3f}, {hi:.3f}]",
            ))
    return violations


def _check_sign_overlap(sign_positions: list[tuple[float, float]]) -> list[Violation]:
    violations: list[Violation] = []
    for i in range(len(sign_positions)):
        for j in range(i + 1, len(sign_positions)):
            dist = _dist2d(sign_positions[i], sign_positions[j])
            if dist < _SIGN_MIN_SPACING:
                violations.append(Violation(
                    rule="sign_overlap",
                    message=(
                        f"Signs {i} and {j} overlap: "
                        f"dist={dist:.3f} m < min={_SIGN_MIN_SPACING:.3f} m"
                    ),
                ))
    return violations


def _check_parking_bounds(parking_config: ParkingLotConfig) -> list[Violation]:
    violations: list[Violation] = []
    half = ParkingLotSpecs.LENGTH / 2
    lo = TrackDimensions.MIN_COORD + half
    hi = TrackDimensions.MAX_COORD - half
    for label, pos_key in (("block1", "block1_pos"), ("block2", "block2_pos")):
        x, y = parking_config[pos_key]  # type: ignore[literal-required]
        if not (lo <= x <= hi and lo <= y <= hi):
            violations.append(Violation(
                rule="parking_bounds",
                message=f"Parking {label} at ({x:.3f}, {y:.3f}) outside bounds [{lo:.3f}, {hi:.3f}]",
            ))
    return violations


def _check_sign_parking_clearance(
    sign_positions: list[tuple[float, float]],
    parking_config: ParkingLotConfig,
) -> list[Violation]:
    violations: list[Violation] = []
    block_positions = [
        parking_config["block1_pos"],
        parking_config["block2_pos"],
    ]
    for si, sign_pos in enumerate(sign_positions):
        for bi, block_pos in enumerate(block_positions):
            dist = _dist2d(sign_pos, block_pos)
            if dist < _SIGN_PARKING_MIN_DIST:
                violations.append(Violation(
                    rule="sign_parking_clearance",
                    message=(
                        f"Sign {si} too close to parking block{bi + 1}: "
                        f"dist={dist:.3f} m < min={_SIGN_PARKING_MIN_DIST:.3f} m"
                    ),
                ))
    return violations


def _check_robot_spawn_clearance(
    starting_conditions: StartingConditions,
    sign_positions: list[tuple[float, float]],
    parking_config: ParkingLotConfig | None,
) -> list[Violation]:
    violations: list[Violation] = []
    spawn = starting_conditions["position"]

    for i, sign_pos in enumerate(sign_positions):
        dist = _dist2d(spawn, sign_pos)
        if dist < _SPAWN_SIGN_MIN_DIST:
            violations.append(Violation(
                rule="spawn_sign_clearance",
                message=(
                    f"Robot spawn too close to sign {i}: "
                    f"dist={dist:.3f} m < min={_SPAWN_SIGN_MIN_DIST:.3f} m"
                ),
            ))

    if parking_config is not None:
        for label, pos_key in (("block1", "block1_pos"), ("block2", "block2_pos")):
            block_pos = parking_config[pos_key]  # type: ignore[literal-required]
            dist = _dist2d(spawn, block_pos)
            if dist < _SPAWN_PARKING_MIN_DIST:
                violations.append(Violation(
                    rule="spawn_parking_clearance",
                    message=(
                        f"Robot spawn too close to parking {label}: "
                        f"dist={dist:.3f} m < min={_SPAWN_PARKING_MIN_DIST:.3f} m"
                    ),
                ))

    return violations


def _dist2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

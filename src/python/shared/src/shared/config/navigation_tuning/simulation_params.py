"""Headless-simulator-only tuning group.

Fields are inherited from the generated DTO
(:mod:`shared.config.generated.navigation.simulation.simulation_schema`). This
module adds only the tuning layer's shipped fallbacks, so a bare
``SimulationParams()`` still matches the checked-in
``simulation/simulation.toml`` without re-declaring a field.

These knobs do not describe the robot or the mat, so they belong neither in
robot.toml nor track.toml -- but they decide what a simulated run scores, which
makes them exactly the kind of value that must not be a literal buried in a
module. The contact policy in particular governs whether a legal start already
touching a wall is a failure or a recoverable state.

The chassis turn-radius floor is NOT here: ``min_turn_radius_m`` is a physical
property and lives in robot.toml, read directly through
``RobotSpecs.MIN_TURN_RADIUS_M``. ``min_turn_radius_tracks_speed`` only decides
whether the simulator floors the curvature with that constant or with the
speed-dependent curve beside it.
"""

from __future__ import annotations

from typing import Any, ClassVar

from shared.config.generated.navigation.simulation.simulation_schema import (
    NavigationSimulationSimulation,
)
from shared.config.navigation_tuning._shared import TuningModel


class SimulationParams(TuningModel, NavigationSimulationSimulation):
    """Knobs that exist only in the headless simulator."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "start_collision_window_s": 2.0,
        "start_collision_grace_s": 15.0,
        "obstacles_inner_wall_terminal": True,
        "lidar_invalid_ray_rate": 0.01,
        "detection_confidence": 0.9,
        "collision_margin_m": 0.0,
        "axis_align_tolerance": 1e-6,
        "no_progress_window_s": 30.0,
        "no_progress_displacement_m": 0.08,
        "vision_through_pinhole": False,
        "vision_range_model": True,
        "min_turn_radius_tracks_speed": False,
        "vision_detect_r50_m": 1.10,
        "vision_detect_falloff_m": 0.15,
    }

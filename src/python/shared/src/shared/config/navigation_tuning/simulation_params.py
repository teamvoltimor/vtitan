"""Headless-simulator-only tuning group.

Fields are inherited from the generated DTO
(:mod:`shared.config.generated.navigation.simulation.simulation_schema`). This
module adds only derived behaviour; every field value comes from the checked-in TOML.

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

from shared.config.generated.navigation.simulation.simulation_schema import (
    NavigationSimulationSimulation,
)


class SimulationParams(NavigationSimulationSimulation):
    """Knobs that exist only in the headless simulator."""


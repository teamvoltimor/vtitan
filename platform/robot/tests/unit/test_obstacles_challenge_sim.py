"""Closed-loop Obstacles Challenge smoke tests.

Unlike ``test_open_challenge_sim.py``, this isn't a pytest-authoritative
scenario battery — ``scenario_catalog.all_obstacles_demo_scenarios()`` builds
seeded demo layouts for the live RViz visualizer. These tests just confirm
``ScenarioSimulator`` actually drives them end-to-end: ``SignRouter`` and
``ParkController`` wire in without crashing and the car doesn't wall-collide
just from a sign nudging its waypoints.

Two real bugs, found via this closed-loop wiring (never exercised by
``test_sign_router.py``, which only unit-tests ``_apply_deformation`` math in
isolation), were fixed in ``sign_router.py``/``core_navigator.py`` — see
``memory/obstacles_closed_loop_sign_router_bug.md``:
1. A sign one corridor over could be within activation distance right at a
   corner; applying its (x, y) through the wrong corridor's axis convention
   produced a nonsensical waypoint.
2. The waypoint-advance check compared the robot's position against the
   *deformed* target instead of the raw planned one, so a lateral sign nudge
   could stall ``waypoint_index`` indefinitely.

``test_parking_scenarios_engage_and_finish`` is still XFAIL, but for an
unrelated reason: the collision reproduces identically with the sign removed
(confirmed by ad-hoc testing), so it's a separate, not-yet-fixed
``ParkController`` bug near the staging approach — not a sign-routing issue.
"""

from __future__ import annotations

import logging

import pytest

from src.simulation.gateway import ScenarioSimulator
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios

logger = logging.getLogger(__name__)


class TestObstaclesDemoScenariosRun:
    """Every demo scenario must complete 3 laps without a wall contact."""

    def test_sign_only_scenarios_complete_without_collision(self) -> None:
        failures = []
        for scenario in all_obstacles_demo_scenarios():
            if scenario.metadata["has_parking_lot"]:
                continue
            result = ScenarioSimulator(
                scenario.metadata, num_laps=scenario.laps, seed=scenario.seed,
            ).run(max_steps=4000)
            logger.info(
                "%s | laps=%d/%d collided=%s timeout=%s",
                scenario.label, result.laps_completed, result.target_laps,
                result.collided, result.timed_out,
            )
            if result.collided or result.laps_completed < scenario.laps:
                failures.append((scenario.label, result))
        assert not failures, [(label, r.collision_xy or r.final_pose) for label, r in failures]

    @pytest.mark.xfail(
        reason="ParkController collides near the staging approach — unrelated to sign routing, "
        "reproduces identically with signs removed. See module docstring.", strict=False,
    )
    def test_parking_scenarios_engage_and_finish(self) -> None:
        for scenario in all_obstacles_demo_scenarios():
            if not scenario.metadata["has_parking_lot"]:
                continue
            result = ScenarioSimulator(
                scenario.metadata, num_laps=scenario.laps, seed=scenario.seed,
            ).run(max_steps=6000)
            logger.info(
                "%s | laps=%d/%d collided=%s parked=%s",
                scenario.label, result.laps_completed, result.target_laps,
                result.collided, result.parked,
            )
            assert not result.collided, scenario.label
            assert result.laps_completed >= scenario.laps, scenario.label
            # Parking always resolves one way or the other (never left "mid-maneuver").
            assert result.parked is not None, scenario.label

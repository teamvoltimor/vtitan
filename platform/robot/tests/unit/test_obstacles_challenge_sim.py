"""Closed-loop Obstacles Challenge smoke tests.

Unlike ``test_open_challenge_sim.py``, this isn't a pytest-authoritative
scenario battery — ``scenario_catalog.all_obstacles_demo_scenarios()`` builds
seeded demo layouts for the live RViz visualizer. These tests just confirm
``ScenarioSimulator`` actually drives them end-to-end: ``SignRouter`` and
``ParkController`` wire in without crashing and the car doesn't wall-collide
just from a sign nudging its waypoints.

Multiple real bugs, found via this closed-loop wiring (never exercised by
``test_sign_router.py``, which only unit-tests ``_apply_deformation`` math in
isolation) plus direct user feedback watching it live in RViz, were fixed in
``sign_router.py``/``core_navigator.py``/``scenario_catalog.py`` — see
``memory/obstacles_closed_loop_sign_router_bug.md`` for the full history.
Most recently: ``SignRouter._passed`` was never cleared between laps, so a
sign avoided once (lap 1) was silently ignored on laps 2-3 of every 3-lap
scenario — fixed with ``SignRouter.reset_for_new_lap()``, called from
``core_navigator.py`` at both lap-completion sites. ``core_navigator.py``
applies sign deformation to whichever waypoint the pure-pursuit lookahead
search actually returns (after the search, not before) — deforming a raw-path
candidate before the search picked from it let the search itself decide
whether the nudge ever reached steering.

``test_parking_scenarios_engage_and_finish`` is still XFAIL, but for an
unrelated reason: the collision reproduces identically with the sign removed
(confirmed by ad-hoc testing), so it's a separate, not-yet-fixed
``ParkController`` bug near the staging approach — not a sign-routing issue.

``obstacles_demo[East/counterclockwise]`` previously needed excluding from
these checks: making avoidance magnitude real exposed what looked like a
corner-negotiation fragility (a red sign pushed the car toward the outer wall
just before a corner, and it failed to turn afterward). That resolved on its
own once ``_ROUTING_TABLE`` was fixed to make red/green outward/inward
absolute (not travel-direction-relative, see the pass-side rule fix below) —
the counterclockwise row for that corridor had been pushing the sign's
avoidance the wrong way, which was the actual cause of the instability, not a
distinct corner-turning bug. No exclusion needed anymore.

Also fixed: ``_ROUTING_TABLE`` pinned red/green to the robot's OWN left/right
(travel-relative), which flips outward vs inward between CW and CCW. The
actual WRO rule is absolute: red is always avoided OUTWARD, green always
INWARD, regardless of which direction the round is driven. The COUNTERCLOCKWISE
rows were wrong under the correct interpretation; ``TestPassSideRule`` and
``TestDeformationDirections`` in ``test_sign_router.py`` were rewritten to pin
the absolute rule instead of the travel-relative one.
"""

from __future__ import annotations

import logging

import pytest

import src.navigation.planning.sign_router as sign_router_module
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

    def test_signs_actually_deform_the_path(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """Every sign-only scenario must trigger at least one real deformation.

        A scenario that "runs clean" only because its signs never actually
        engage (e.g. misplaced into a corridor the robot doesn't drive near)
        would pass ``test_sign_only_scenarios_complete_without_collision`` for
        the wrong reason — this catches that silently-disabled-routing failure
        mode directly.
        """
        deform_counts: dict[str, int] = {}
        current_label = [""]
        orig = sign_router_module.SignRouter.deform_waypoint

        def counting_deform_waypoint(self, waypoint, robot_pos, robot_yaw, corridor, detections=None):
            result = orig(self, waypoint, robot_pos, robot_yaw, corridor, detections)
            if result != waypoint:
                deform_counts[current_label[0]] = deform_counts.get(current_label[0], 0) + 1
            return result

        monkeypatch.setattr(
            sign_router_module.SignRouter, "deform_waypoint", counting_deform_waypoint,
        )

        never_engaged = []
        for scenario in all_obstacles_demo_scenarios():
            if scenario.metadata["has_parking_lot"]:
                continue
            current_label[0] = scenario.label
            ScenarioSimulator(
                scenario.metadata, num_laps=scenario.laps, seed=scenario.seed,
            ).run(max_steps=4000)
            count = deform_counts.get(scenario.label, 0)
            logger.info("%s | deformations=%d", scenario.label, count)
            if count == 0:
                never_engaged.append(scenario.label)

        assert not never_engaged, f"signs never engaged in: {never_engaged}"

    def test_signs_engage_on_every_lap(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """A sign avoided on lap 1 must be avoided again on laps 2 and 3.

        Regression guard for a real bug: ``SignRouter._passed`` never cleared
        between laps, so once a sign was marked passed on lap 1 it was
        silently skipped for the rest of a 3-lap scenario.
        """
        orig = sign_router_module.SignRouter.deform_waypoint
        laps_seen_per_sign: dict[int, set[int]] = {}
        navigator_ref: list = [None]

        def recording_deform_waypoint(self, waypoint, robot_pos, robot_yaw, corridor, detections=None):
            nearest_idx, _ = self._nearest_active_sign(robot_pos, corridor)
            result = orig(self, waypoint, robot_pos, robot_yaw, corridor, detections)
            if result != waypoint and nearest_idx >= 0:
                laps_seen_per_sign.setdefault(nearest_idx, set()).add(
                    navigator_ref[0].laps_completed,
                )
            return result

        monkeypatch.setattr(
            sign_router_module.SignRouter, "deform_waypoint", recording_deform_waypoint,
        )

        under_engaged = []
        for scenario in all_obstacles_demo_scenarios():
            if scenario.metadata["has_parking_lot"]:
                continue
            laps_seen_per_sign.clear()
            sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
            navigator_ref[0] = sim._navigator
            sim.run(max_steps=4000)
            for i in range(len(scenario.metadata["sign_positions"])):
                laps = laps_seen_per_sign.get(i, set())
                if len(laps) < scenario.laps:
                    under_engaged.append((scenario.label, i, sorted(laps)))

        assert not under_engaged, f"sign engaged on fewer than {scenario.laps} laps: {under_engaged}"

    @pytest.mark.xfail(
        reason="ParkController collides near the staging approach — unrelated to sign routing, "
        "reproduces identically with signs removed. See module docstring.", strict=True,
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

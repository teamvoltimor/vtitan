"""Closed-loop recovery tests: can the navigator re-center after a mid-run pose kick?

Real runs never track the planned path perfectly — a wheel slip, an uneven mat
seam, or an imperfect start can knock the robot off-line. These tests inject a
single pose disturbance partway through an otherwise-solvable Open Challenge
lap and check the real ``CoreNavigator`` steers back onto the path (cross-track
error decays back under a small threshold within a bounded step budget) and
still completes the round, rather than only ever testing "flies perfectly" runs
like ``test_open_challenge_sim.py`` does.

For "how much deviation can it actually take" as a reference number (not a
pass/fail regression), see ``python -m src.simulation.find_recovery_envelope``.
"""

from __future__ import annotations

import logging
from itertools import product
from typing import TYPE_CHECKING, Any

import pytest
from shared.config.constants import CompetitionSpecs, CorridorDimensions
from shared.config.enums import Direction, Section

from src.navigation.track_geometry import cross_track_error
from src.simulation import PoseDisturbance, ScenarioSimulator
from src.simulation.scenario_builder import build_open_metadata, uniform_widths

if TYPE_CHECKING:
    from src.simulation.kinematics import AckermannState

logger = logging.getLogger(__name__)

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)

_DISTURB_AT_STEP = 150
"""Well after start-up settling (7.5 s at 20 Hz), mid-corridor on every layout below."""

_RECOVERY_WINDOW_STEPS = 150
"""Budget (7.5 s) to get back under ``_RECOVERY_THRESHOLD_M`` after the kick."""

_RECOVERY_THRESHOLD_M = 0.05

_LATERAL_KICK_M = 0.08
"""Conservative vs. the measured envelope (narrow/clockwise is worst-case at ~0.115m —
see ``python -m src.simulation.find_recovery_envelope``)."""

_HEADING_KICK_RAD = 0.35
"""~20 degrees, vs. a measured worst-case envelope of ~35 degrees (narrow/clockwise)."""


def _run_with_disturbance(
    meta: dict[str, Any],
    disturbance: PoseDisturbance,
) -> tuple[Any, list[float]]:
    sim = ScenarioSimulator(meta, num_laps=_N_LAPS)
    trace: list[float] = []

    def on_step(state: AckermannState, _scan: object) -> None:
        trace.append(cross_track_error(sim.waypoints, state.x, state.y))

    result = sim.run(on_step=on_step, disturb_at_step=_DISTURB_AT_STEP, disturbance=disturbance)
    return result, trace


def _steps_to_recover(trace: list[float]) -> int | None:
    """Steps from the kick until cross-track error first drops back under threshold."""
    for i in range(_DISTURB_AT_STEP - 1, len(trace)):
        if trace[i] <= _RECOVERY_THRESHOLD_M:
            return i - (_DISTURB_AT_STEP - 1)
    return None


def _assert_recovers(meta: dict[str, Any], disturbance: PoseDisturbance, label: str) -> None:
    result, trace = _run_with_disturbance(meta, disturbance)
    recovered_in = _steps_to_recover(trace)
    logger.info(
        "%s | recovered_in=%s steps success=%s collided=%s laps=%d/%d",
        label,
        recovered_in,
        result.success,
        result.collided,
        result.laps_completed,
        _N_LAPS,
    )
    assert not result.collided, f"{label}: collided after disturbance at {result.collision_xy}"
    assert result.success, f"{label}: did not complete the round after disturbance"
    assert recovered_in is not None, f"{label}: cross-track error never recovered under {_RECOVERY_THRESHOLD_M}m"
    assert recovered_in <= _RECOVERY_WINDOW_STEPS, (
        f"{label}: took {recovered_in} steps to recover, budget is {_RECOVERY_WINDOW_STEPS}"
    )


class TestRecoversFromLateralKick:
    """A sideways pose kick mid-lap must not cause a collision and must re-center."""

    @pytest.mark.parametrize(("section", "direction"), list(product(list(Section), list(Direction))))
    def test_wide_corridor(self, section: Section, direction: Direction) -> None:
        meta = build_open_metadata(uniform_widths(_WIDE_MM), section, direction)
        _assert_recovers(
            meta,
            PoseDisturbance(lateral_m=_LATERAL_KICK_M),
            f"WIDE {section.capitalized}/{direction} lateral",
        )

    @pytest.mark.parametrize(("section", "direction"), list(product(list(Section), list(Direction))))
    def test_narrow_corridor(self, section: Section, direction: Direction) -> None:
        meta = build_open_metadata(uniform_widths(_NARROW_MM), section, direction)
        _assert_recovers(
            meta,
            PoseDisturbance(lateral_m=_LATERAL_KICK_M),
            f"NARROW {section.capitalized}/{direction} lateral",
        )


class TestRecoversFromHeadingKick:
    """A heading-only pose kick mid-lap must not cause a collision and must re-center."""

    @pytest.mark.parametrize(("section", "direction"), list(product(list(Section), list(Direction))))
    def test_wide_corridor(self, section: Section, direction: Direction) -> None:
        meta = build_open_metadata(uniform_widths(_WIDE_MM), section, direction)
        _assert_recovers(
            meta,
            PoseDisturbance(lateral_m=0.0, heading_rad=_HEADING_KICK_RAD),
            f"WIDE {section.capitalized}/{direction} heading",
        )

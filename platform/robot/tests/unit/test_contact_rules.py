"""Each challenge forbids a different wall, so contact cannot be one boolean.

WRO scores the Open Challenge on not touching the *outer* wall and the
Obstacles Challenge on not touching the *inner* one. The collision model
already tested outer boundary, inner block and obstacles separately; it just
collapsed them into a single bool, which makes both rules unrepresentable.
These pin the distinction and the per-challenge mapping built on it.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import TrackDimensions
from shared.config.enums import Direction, ScenarioType, Section

from src.navigation.ports import DriveCommand
from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.gateway import TERMINAL_SURFACES, SimulatedHardwareGateway
from src.simulation.kinematics import AckermannState
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.track_model import ContactSurface, TrackModel

_WIDE_MM = 1000
_MAX = TrackDimensions.MAX_COORD


def _track() -> TrackModel:
    metadata = build_open_metadata(uniform_widths(_WIDE_MM), Section.SOUTH, Direction.CLOCKWISE)
    # ``build_open_metadata`` returns a model while the geometry helpers are
    # still dict-based, so the boundary conversion is the caller's job.
    return TrackModel(corridor_widths_from_metadata(metadata.model_dump()))


class TestContactSurface:
    """The three surfaces are reported distinctly."""

    def test_free_space_touches_nothing(self) -> None:
        track = _track()
        # Mid-corridor on the south side: clear of both the outer wall and the block.
        assert track.contact_surface(_MAX / 2, _WIDE_MM / 1000 / 2, 0.0) is ContactSurface.NONE

    def test_outside_the_track_is_the_outer_wall(self) -> None:
        track = _track()
        assert track.contact_surface(-1.0, _MAX / 2, 0.0) is ContactSurface.OUTER_WALL

    def test_centre_of_the_track_is_the_inner_wall(self) -> None:
        """The inner block occupies the middle of the mat."""
        track = _track()
        assert track.contact_surface(_MAX / 2, _MAX / 2, 0.0) is ContactSurface.INNER_WALL

    def test_footprint_collides_still_agrees_with_the_surface(self) -> None:
        """The bool is now derived, so the two can never disagree."""
        track = _track()
        for x, y in [(-1.0, 1.5), (1.5, 1.5), (1.5, 0.25)]:
            expected = track.contact_surface(x, y, 0.0) is not ContactSurface.NONE
            assert track.footprint_collides(x, y, 0.0) is expected


class TestTerminalSurfaces:
    """The per-challenge mapping of which contact ends a run."""

    def test_open_challenge_ends_on_the_outer_wall(self) -> None:
        assert ContactSurface.OUTER_WALL in TERMINAL_SURFACES[ScenarioType.OPEN]

    def test_open_challenge_survives_the_inner_wall(self) -> None:
        assert ContactSurface.INNER_WALL not in TERMINAL_SURFACES[ScenarioType.OPEN]

    def test_obstacles_challenge_ends_on_the_inner_wall(self) -> None:
        assert ContactSurface.INNER_WALL in TERMINAL_SURFACES[ScenarioType.OBSTACLES]

    def test_obstacles_challenge_survives_the_outer_wall(self) -> None:
        assert ContactSurface.OUTER_WALL not in TERMINAL_SURFACES[ScenarioType.OBSTACLES]

    def test_knocking_an_obstacle_over_ends_an_obstacles_run(self) -> None:
        """Assumption, not a quoted rule — see TERMINAL_SURFACES' docstring."""
        assert ContactSurface.OBSTACLE in TERMINAL_SURFACES[ScenarioType.OBSTACLES]

    def test_no_challenge_forbids_everything(self) -> None:
        """If both walls were terminal the distinction would be decorative."""
        for surfaces in TERMINAL_SURFACES.values():
            assert ContactSurface.NONE not in surfaces
            assert not {ContactSurface.OUTER_WALL, ContactSurface.INNER_WALL} <= surfaces


class TestSolidWalls:
    """A refused move is the contact signal once walls stop being passable."""

    @staticmethod
    def _gateway(*, solid: bool, x: float = 0.30, yaw: float = math.pi) -> SimulatedHardwareGateway:
        """A gateway with the chassis placed facing the west outer wall."""
        return SimulatedHardwareGateway(
            track=_track(),
            initial_state=AckermannState(x=x, y=_MAX / 2, yaw=yaw),
            solid_walls=solid,
        )

    @staticmethod
    def _drive(gw: SimulatedHardwareGateway, speed: float, ticks: int) -> None:
        gw.publish_drive(DriveCommand(speed_mps=speed, steering_norm=0.0))
        for _ in range(ticks):
            gw.advance(0.05)

    def test_ghost_walls_let_the_chassis_pass_through(self) -> None:
        """The default model only *detects* contact, which is why recovery needed opting in."""
        gw = self._gateway(solid=False)
        self._drive(gw, 0.5, 60)
        assert gw.blocked is False
        # Drove clean through the west wall rather than stopping at it.
        assert gw.state.x < 0.0

    def test_solid_walls_refuse_the_move_and_hold_position(self) -> None:
        gw = self._gateway(solid=True)
        self._drive(gw, 0.5, 60)
        assert gw.blocked is True
        assert gw.contact_surface is ContactSurface.OUTER_WALL
        # Held on the track side of the wall rather than driven through it.
        assert gw.state.x > 0.0

    def test_reversing_frees_a_blocked_chassis(self) -> None:
        """The behaviour the whole recovery model exists to make observable."""
        gw = self._gateway(solid=True)
        self._drive(gw, 0.5, 60)
        pinned_x = gw.state.x
        assert gw.blocked is True

        self._drive(gw, -0.3, 20)
        assert gw.blocked is False
        assert gw.state.x > pinned_x


class TestPermittedSurfacesStillBlock:
    """A surface that stops ending the run must start stopping the chassis.

    This is the hazard the per-challenge rule introduces. While every contact
    was terminal, the run ended on the tick it happened, so it never mattered
    that walls were passable. Making the inner block survivable in the Open
    Challenge would otherwise let the robot drive straight through the middle
    of the mat and go on counting laps — a pass rate built on cutting the
    course.
    """

    def test_the_inner_block_stops_an_open_challenge_robot(self) -> None:
        from src.simulation.gateway import ScenarioSimulator

        metadata = build_open_metadata(uniform_widths(_WIDE_MM), Section.SOUTH, Direction.CLOCKWISE)
        sim = ScenarioSimulator(metadata, num_laps=1, seed=0)
        gw = sim.gateway
        assert ContactSurface.INNER_WALL not in TERMINAL_SURFACES[ScenarioType.OPEN]

        # The start pose faces along the south corridor; turn to face the inner
        # block (+y) so driving forward runs straight into it.
        gw.apply_disturbance(0.0, heading_rad=math.pi / 2 - gw.state.yaw)
        start = gw.state
        gw.publish_drive(DriveCommand(speed_mps=0.5, steering_norm=0.0))
        for _ in range(200):
            gw.advance(0.05)
            if gw.blocked:
                break

        assert gw.blocked is True, "inner block let the chassis through"
        assert gw.contact_surface is ContactSurface.INNER_WALL
        assert gw.state.y < _MAX / 2, "chassis reached the far side of the inner block"
        assert gw.state.y > start.y, "chassis never moved toward the block"


@pytest.mark.parametrize("challenge", [ScenarioType.OPEN, ScenarioType.OBSTACLES])
def test_every_challenge_has_a_rule(challenge: ScenarioType) -> None:
    assert TERMINAL_SURFACES[challenge]

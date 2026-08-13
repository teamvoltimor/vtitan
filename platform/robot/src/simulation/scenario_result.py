"""Outcome types and per-run accumulators for :class:`~src.simulation.scenario_simulator.ScenarioSimulator`.

Separated from the run loop itself: these are passive record-keeping (the
result shape, per-tick metric accumulation, and the wall-contact
terminate/recover policy), not scenario setup or control-loop stepping.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from shared.config.constants import CompetitionSpecs
from shared.domain.enums import ScenarioType

from src.simulation.track_model import ContactSurface

if TYPE_CHECKING:
    from src.simulation.simulated_hardware_gateway import SimulatedHardwareGateway

TERMINAL_SURFACES: dict[ScenarioType, frozenset[ContactSurface]] = {
    ScenarioType.OPEN: frozenset({ContactSurface.OUTER_WALL}),
    ScenarioType.OBSTACLES: frozenset({ContactSurface.INNER_WALL, ContactSurface.OBSTACLE}),
}
"""Which contacts end a run, per challenge.

Each challenge forbids one wall: the Open Challenge the *outer* one, the
Obstacles Challenge the *inner* one. Contact with the other wall is still
recorded in ``SimResult.contact_count`` but does not end the run, so a scrape
the robot drives out of no longer scores the same as failing to complete.

Obstacles are grouped with the inner wall rather than given a rule of their
own: knocking a traffic sign or parking block over is a scored failure and only
exists in the Obstacles Challenge. The user's rule covers walls, so this half
is an assumption -- if a sign is meant to be a survivable penalty instead, this
is the one line to change.
"""


@dataclass(slots=True)
class SimResult:
    """Outcome of one closed-loop scenario run."""

    target_laps: int
    laps_completed: int
    collided: bool
    timed_out: bool
    steps: int
    sim_time_s: float
    distance_m: float
    max_speed_mps: float
    avg_speed_mps: float
    min_lidar_range_m: float
    collision_xy: tuple[float, float] | None
    final_pose: tuple[float, float, float]
    contact_count: int = 0
    """Distinct wall-contact episodes, whether or not any was terminal.

    Always recorded, so a run that recovers from contact is not scored as if
    it never touched anything — under ``contact_grace_s`` this is the number
    that stands in for a penalty."""

    contact_time_s: float = 0.0
    """Total time spent in contact with a run-ending surface."""

    terminal_surface: ContactSurface = ContactSurface.NONE
    """Which surface ended the run, or ``NONE`` if contact did not end it."""

    lap_step_indices: list[int] = field(default_factory=list)
    parked: bool | None = None
    """``None`` when the scenario has no parking lot; else whether parking finished cleanly
    (as opposed to giving up on its frame budget — see ``ParkController.is_timed_out``)."""

    @property
    def over_time(self) -> bool:
        """Exceeded the official 3-minute round limit.

        Derived from ``sim_time_s`` rather than stored, so it cannot drift from
        the time actually simulated. Distinct from ``timed_out``, which only
        says the run hit the harness's ``max_steps`` budget -- that budget is
        200 s, more generous than the rule, so a run could finish its laps at
        196 s and be scored a clean pass for something the judges would not
        have let finish.
        """
        return self.sim_time_s > CompetitionSpecs.ROUND_TIME_LIMIT_S

    @property
    def success(self) -> bool:
        """Completed all target laps in time, without a wall contact (and parked cleanly, if required)."""
        return (
            self.laps_completed >= self.target_laps
            and not self.collided
            and not self.over_time
            and self.parked is not False
        )


@dataclass(frozen=True, slots=True)
class PoseDisturbance:
    """A one-time pose kick applied mid-run, e.g. to test recovery from drift.

    ``lateral_m`` offsets perpendicular to the current heading (positive =
    left of travel direction); ``heading_rad`` adds to yaw.
    """

    lateral_m: float
    heading_rad: float = 0.0


@dataclass(slots=True)
class RunMetrics:
    """Per-tick accumulators for the run's distance, speed and clearance.

    One object rather than four locals because the control loop accounts for a
    tick in two places -- once for a creep tick taken before the travel
    direction is known, once for a normal driving tick -- and those had drifted
    apart. The creep branch fed distance and the step count but not speed, so
    ``avg_speed`` (which divides by the full step count) was understated on
    every blind run, and a run that collided before the direction settled
    reported ``vmax=vavg=0.00`` alongside a non-zero distance for the same
    ticks. Both branches now call :meth:`observe`, so a metric cannot be added
    to one and forgotten in the other.
    """

    distance: float = 0.0
    max_speed: float = 0.0
    speed_sum: float = 0.0
    min_range: float = math.inf

    def observe(self, gateway: SimulatedHardwareGateway, distance_increment: float) -> None:
        """Fold one tick of motion into the accumulators."""
        self.distance += distance_increment
        speed = abs(gateway.state.v)
        self.max_speed = max(self.max_speed, speed)
        self.speed_sum += speed
        self.min_range = min(self.min_range, gateway.last_min_range)

    def avg_speed(self, steps: int) -> float:
        """Mean speed over every tick of the run, creep included."""
        return (self.speed_sum / steps) if steps else 0.0

    def min_range_or_zero(self) -> float:
        """Closest LIDAR return seen, or 0.0 if no scan ever reported one."""
        return self.min_range if math.isfinite(self.min_range) else 0.0


class ContactTracker:
    """Decides when a wall-contact streak stops being survivable and ends the run.

    Two policies. By default contact is terminal on the first tick, except
    within the opening seconds, where a legal starting position may already sit
    against a wall and the robot is allowed a grace period to steer clear.
    Setting ``grace_s`` switches to treating contact as recoverable everywhere:
    a streak ends the run only if the robot cannot free itself in time, which
    is the only policy under which the navigator's reversing escape is
    observable at all.
    """

    def __init__(
        self,
        dt: float,
        start_window_s: float,
        start_grace_s: float,
        grace_s: float | None,
        forbidden: frozenset[ContactSurface],
    ) -> None:
        self._dt = dt
        self._start_window_s = start_window_s
        self._start_grace_s = start_grace_s
        self._grace_s = grace_s
        self._forbidden = forbidden
        self._streak_start_step: int | None = None
        self._in_contact = False
        self.count = 0
        self.time_s = 0.0
        self.surface = ContactSurface.NONE
        """The surface that ended the run, once :meth:`update` has returned True."""

    def update(self, step: int, surface: ContactSurface) -> bool:
        """Fold in one tick's contact; return True if the run should end.

        Every contact counts toward :attr:`count`, but only a forbidden surface
        can end the run or accrue :attr:`time_s` — touching the wall this
        challenge permits is recorded, not punished.
        """
        touching = surface is not ContactSurface.NONE
        if touching and not self._in_contact:
            self.count += 1
        self._in_contact = touching

        if surface not in self._forbidden:
            self._streak_start_step = None
            return False

        if self._streak_start_step is None:
            self._streak_start_step = step
        self.time_s += self._dt
        streak_s = (step - self._streak_start_step) * self._dt

        if self._grace_s is not None:
            ended = streak_s >= self._grace_s
        else:
            began_at_start = (self._streak_start_step - 1) * self._dt <= self._start_window_s
            ended = not began_at_start or streak_s >= self._start_grace_s
        if ended:
            self.surface = surface
        return ended

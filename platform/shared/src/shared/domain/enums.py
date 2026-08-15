"""Shared domain enumerations.

Canonical home for cross-context domain enums (track sections, robot
direction, risk classification, robot/runtime state). Using enums instead of
bare strings eliminates typo-prone comparisons and provides IDE autocomplete
throughout the codebase.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self


class FromStringEnum(StrEnum):
    """Mixin providing a ``from_string`` classmethod to any ``StrEnum`` subclass.

    Eliminates the identical try/except boilerplate that every enum in this
    module was repeating.
    """

    @classmethod
    def from_string(cls, value: str) -> Self:
        """Parse a member from its string value, case-insensitively."""
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            msg = f"Invalid {cls.__name__.lower()}: {value!r}. Expected one of {options}"
            raise ValueError(msg) from err


class Section(FromStringEnum):
    """Four navigable corridors of the WRO 2026 track."""

    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    @property
    def capitalized(self) -> str:
        """Return the section name with an initial capital letter."""
        return self.value.capitalize()

    @classmethod
    def canonical(cls) -> Section:
        """The section a robot assumes when it hasn't been told which one it is in.

        Any choice works: the robot defines its own world frame by declaring
        its starting corridor to be this one, and everything else -- corridor
        geometry, sign routing -- follows self-consistently, since the whole
        map is just the true one rotated to match. See
        ``src.navigation.start_conditions``'s module docstring for why this
        is safe for section but NOT for direction, which cannot be assumed
        the same way (getting it wrong is a reflection, not a rotation).

        A fixed choice, not a claim about where the robot physically is.
        """
        return cls.SOUTH


class Direction(FromStringEnum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"


class ScenarioType(FromStringEnum):
    """WRO 2026 challenge type."""

    OPEN = "open"
    OBSTACLES = "obstacles"


class CorridorSide(FromStringEnum):
    """Which of a corridor's two boundaries something is measured toward.

    Every corridor on this track is bounded by the mat's outer wall on one side
    and a face of the inner block on the other, whichever cardinal section it
    is. Naming the side rather than a compass direction keeps the meaning the
    same for all four.
    """

    INNER = "inner"
    OUTER = "outer"


class CorridorWidthType(FromStringEnum):
    """Which width a corridor is built to.

    Open Challenge corridors are randomised to one of exactly two legal
    widths, so there the estimator's job is to classify which of the two it
    is seeing, not to measure a continuous value. Obstacles Challenge
    corridors are never randomised or estimated at all -- the Go generator
    (``simconfig.WidthTypeFixed``) always builds them at one fixed width, so
    ``FIXED`` exists only to label scenario metadata and is never a target
    the corridor-width estimator classifies towards.

    The metres are deliberately not here. They belong to the mat, are generated
    into ``shared.config.constants.CorridorDimensions`` from ``track.toml``, and
    the domain layer does not import config. Anything needing the number reads
    it from there and uses this only to name which one.
    """

    NARROW = "narrow"
    WIDE = "wide"
    FIXED = "fixed"

    @classmethod
    def blind_default(cls) -> CorridorWidthType:
        """The classification a corridor is assumed to be before it's measured.

        Narrow, not wide: a narrow corridor planned as if it were wide puts
        the path 0.15 m from the inner block face, closer than the chassis
        half-diagonal (0.180 m), so a corner would clip it mid-turn. The
        converse -- planning a wide corridor as narrow -- only pushes the
        path nearer the outer wall, which stays inside the true corridor.
        Unlike ``Section.canonical()``, this isn't an arbitrary label that
        happens to work either way: getting this one wrong costs a
        collision, not just a rotated map, so the direction of the guess
        matters and must always be this one.
        """
        return cls.NARROW


class RiskLevel(StrEnum):
    """Collision risk classification for navigation logic."""

    SAFE = "safe"
    CRITICAL = "critical"
    OBSTACLE = "obstacle"


class LightingScenario(StrEnum):
    """Realistic lighting scenarios for simulation."""

    DIRECT_SUNLIGHT = "direct_sunlight"
    CLOUDY = "cloudy"
    INDOOR_BRIGHT = "indoor_bright"
    INDOOR_DIM = "indoor_dim"
    EVENING = "evening"
    MIXED = "mixed"


class RobotState(FromStringEnum):
    """Robot state machine states."""

    BOOT_CHECK = "boot_check"
    READY = "ready"
    RACING = "racing"
    FINISHED = "finished"


class NodeHealth(FromStringEnum):
    """Telemetry node health status."""

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"


class NavigatorPhase(StrEnum):
    """Branch of ``CoreNavigator.step()`` that produced a snapshot.

    Which branch produced a given ``NavigatorDebugSnapshot`` -- see that
    model's docstring for why a field being ``None`` means "not computed on
    this phase", not "unknown".
    """

    NOT_YET_STEPPED = "not_yet_stepped"
    NO_POSE = "no_pose"
    BLIND_CREEP = "blind_creep"
    ACTIVE_MANEUVER = "active_maneuver"
    STUCK_ESCAPE_HOLDING = "stuck_escape_holding"
    STUCK_ESCAPE_MANEUVER = "stuck_escape_maneuver"
    FINISHED_HOLD = "finished_hold"
    PARKING = "parking"
    WAYPOINT_WRAP_FALLBACK = "waypoint_wrap_fallback"
    WAYPOINT_REACHED = "waypoint_reached"
    NORMAL_DRIVE = "normal_drive"
    ESCAPE_TRIGGERED = "escape_triggered"


class ManeuverType(StrEnum):
    """Kind of escape maneuver commanded by ``compute_escape_maneuver``.

    Lives here rather than in ``collision_avoidance_controller.py`` (which
    still owns the controller logic and re-exports this) because
    ``NavigatorDebugSnapshot.active_maneuver_type`` needs it: that model is
    the cross-context telemetry/bag-replay wire format, same reason
    ``RiskLevel``/``Direction``/``Section`` live here instead of their own
    producing modules.
    """

    K_TURN = "k_turn"
    SIDE_CORRECTION = "side_correction"
    STUCK_REVERSE = "stuck_reverse"
    STUCK_FORWARD = "stuck_forward"


class ParkPhase(StrEnum):
    """Parking maneuver phases.

    Lives here rather than in ``parking.py`` (which still owns the
    controller logic and re-exports this) for the same reason
    ``ManeuverType`` does: ``NavigatorDebugSnapshot.park_phase`` needs it.
    """

    STAGE = "stage"
    ENTER = "enter"
    DONE = "done"


# The class ids the retrained GMR traffic-sign detector emits, in the order the
# checkpoint itself declares them. Confirmed by running the checkpoint over the
# per-class image folders: green_prism images predict green, red_prism predict
# red. This is the single source of truth for that order.
#
# Do NOT take it from auto-annotator's data.yaml, which says (red, green,
# magenta) and is stale -- its `path` points at an archived directory. Consuming
# the HEF with that order swaps red and green, inverting the WRO pass-side rule
# on every obstacle, and nothing about it fails loudly.
#
# Regenerate after retraining with:
#   python -c "import onnx; print(onnx.load('hailo/data/gmr.onnx').metadata_props)"
GMR_CLASS_NAMES: dict[int, str] = {
    0: "green",
    1: "magenta",
    2: "red",
}

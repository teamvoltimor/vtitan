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

    @property
    def cardinal(self) -> str:
        """Short cardinal label (N/S/E/W) for display (e.g. the OLED)."""
        return self.name[0]

    @classmethod
    def loop_order(cls, start_section: Section, direction: Direction) -> list[Section]:
        """Corridor traversal order with ``start_section`` first.

        Returns the four corridors in the sequence the robot traverses them,
        beginning at ``start_section`` (corridor 1) and following ``direction``.
        The internal absolute order is anchored at EAST (the start/finish line
        sits on the east side of the mat -- see ``race_tracker.TRAVEL_DIRS``),
        but that anchor never leaks: callers always pass the *believed* start
        (measured once known, else ``Section.canonical()``), so the returned
        order is relative to the robot's actual corner.

        Used to assemble the waypoint loop and as the basis for ``loop_index``.
        """
        absolute = (
            [cls.EAST, cls.SOUTH, cls.WEST, cls.NORTH]
            if direction is Direction.CLOCKWISE
            else [cls.EAST, cls.NORTH, cls.WEST, cls.SOUTH]
        )
        rotated = list(absolute)
        while rotated[0] is not start_section:
            rotated.append(rotated.pop(0))
        return rotated

    def loop_index(self, start_section: Section, direction: Direction) -> int:
        """1-based position of this corridor in the loop from the start section.

        The robot's starting corridor is corridor 1; the next one it reaches
        (per ``loop_order``) is 2, and so on. Used for display (e.g. the OLED's
        "corridor n/4" line), where a positional index is more meaningful than
        the cardinal name.

        ``start_section`` and ``direction`` MUST be the robot's *believed* start
        (the measured start once known, else the assumed one -- e.g.
        ``Section.canonical()``) -- never a hardcoded corner. Passing an assumed
        start while the real one differs would number the corridors relative to
        the wrong corner.
        """
        return self.loop_order(start_section, direction).index(self) + 1

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
    BAY_EXIT = "bay_exit"
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


class ThreatDirection(StrEnum):
    """LIDAR threat sector a detected obstacle lies in, relative to the robot.

    Used by the collision-avoidance controller and the OLED/telemetry summaries
    to name which side is blocked. Centralised here so the raw strings
    ``"front"/"back"/"left"/"right"/"none"`` stop being re-typed across the
    navigation code, tests, and diag scripts.
    """

    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"
    NONE = "none"


class NodeNames(StrEnum):
    """Canonical ROS2 node names.

    Mirrors ``TfFrames``: a single source of truth so node names stop being
    raw strings scattered across ``src/ros2_ws/*`` node constructors.
    """

    TELEMETRY_BRIDGE = "telemetry_bridge"
    STATE_MACHINE = "state_machine"
    BAG_RECORDER = "bag_recorder"
    TRACK_NAVIGATOR = "track_navigator"
    VISION_DETECTOR = "vision_detector"
    OLED_DISPLAY = "oled_display"
    CHALLENGE_MODE = "challenge_mode"
    BUTTON = "button"
    ACKERMANN_MOTOR = "ackermann_motor"
    JOY_TELEOP = "joy_teleop"
    PI_ZERO_PERIPHERALS = "pi_zero_peripherals"
    HARDWARE = "hardware"


class CommandKind(StrEnum):
    """Command channel message kinds received from the gRPC bridge.

    The wire values come from a protobuf ``WhichOneof`` at the boundary, but the
    mapping to internal behaviour should compare against this enum rather than
    raw strings.
    """

    START_RACE = "start_race"
    STOP_RACE = "stop_race"
    EMERGENCY_STOP = "emergency_stop"
    SET_VISION_DEBUG = "set_vision_debug"
    DISABLE_COMMAND_CHANNEL = "disable_command_channel"
    SET_TELEMETRY_CHANNEL = "set_telemetry_channel"


class HoldKind(StrEnum):
    """Button hold classification used by the OLED/telemetry wire models."""

    SHORT_PRESS = "short_press"
    LONG_PRESS = "long_press"
    SHUTDOWN_PRESS = "shutdown_press"


class ConnectionStatus(StrEnum):
    """Diagnostics connection status string compared against in nodes."""

    CONNECTED = "connected"
    DISCONNECTED = "disconnected"


class Axis(StrEnum):
    """Which world coordinate a sign-routing table entry deforms."""

    X = "x"
    Y = "y"


from shared.domain.models import GMR_CLASS_NAMES  # noqa: E402,F401  (defined in models.py to avoid a cycle; re-exported here)

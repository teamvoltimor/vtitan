"""WRO 2026 simulation enumerations.

Defines strongly-typed domain enums for track sections, robot direction,
and challenge type.
Using enums instead of bare strings eliminates typo-prone comparisons and
provides IDE autocomplete throughout the codebase.
"""

from __future__ import annotations

from enum import Enum, StrEnum


class Section(Enum):
    """Four navigable corridors of the WRO 2026 track."""

    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> Section:
        """Look up a Section by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "north", "SOUTH").

        Returns:
            Matching Section enum member.

        Raises:
            ValueError: If value does not match any Section.
        """
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            error_message = f"Invalid section: {value!r}. Expected one of {options}"
            raise ValueError(error_message) from err

    @property
    def capitalized(self) -> str:
        """Return the section name with the first letter capitalised."""
        return self.value.capitalize()


class Direction(Enum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> Direction:
        """Look up a Direction by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "clockwise").

        Returns:
            Matching Direction enum member.

        Raises:
            ValueError: If value does not match any Direction.
        """
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(d.value for d in cls)
            error_message = f"Invalid direction: {value!r}. Expected one of {options}"
            raise ValueError(error_message) from err


class ScenarioType(StrEnum):
    """WRO 2026 challenge type.

    Inherits from ``str`` so values compare equal to their string
    representations, enabling transparent JSON round-trips.
    """

    OPEN = "open"
    OBSTACLES = "obstacles"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> ScenarioType:
        """Look up a ScenarioType by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "open", "obstacles").

        Returns:
            Matching ScenarioType enum member.

        Raises:
            ValueError: If value does not match any ScenarioType.
        """
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            error_message = (
                f"Invalid scenario type: {value!r}. Expected one of {options}"
            )
            raise ValueError(error_message) from err


class RiskLevel(StrEnum):
    """Collision risk classification for navigation logic.

    Inherits from ``str`` so values compare equal to their string
    representations.
    """

    SAFE = "safe"
    CRITICAL = "critical"
    OBSTACLE = "obstacle"

    def __str__(self) -> str:
        return self.value


class LightingScenario(StrEnum):
    """Realistic lighting scenarios for simulation.

    Each scenario represents different environmental lighting conditions
    with specific intensity, direction, and shadow characteristics.
    """

    DIRECT_SUNLIGHT = "direct_sunlight"
    CLOUDY = "cloudy"
    INDOOR_BRIGHT = "indoor_bright"
    INDOOR_DIM = "indoor_dim"
    EVENING = "evening"
    MIXED = "mixed"

    def __str__(self) -> str:
        return self.value


class RobotState(StrEnum):
    """Robot state machine states.

    Represents the primary operational states of the robot throughout
    its lifecycle from boot through completion.

    States flow: BOOT_CHECK → READY → RACING → FINISHED

    Attributes:
        BOOT_CHECK: Hardware verification and self-tests in progress
        READY: Robot ready for race, waiting for start signal (button press)
        RACING: Autonomous navigation in progress, actively following track
        FINISHED: Race complete, results displayed, ready for next run
    """

    BOOT_CHECK = "boot_check"
    READY = "ready"
    RACING = "racing"
    FINISHED = "finished"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> RobotState:
        """Look up a RobotState by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "boot_check", "RACING").

        Returns:
            Matching RobotState enum member.

        Raises:
            ValueError: If value does not match any RobotState.
        """
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            error_message = f"Invalid robot state: {value!r}. Expected one of {options}"
            raise ValueError(error_message) from err


class NodeHealth(StrEnum):
    """Telemetry node health status.

    Indicates the operational health of the telemetry/navigation node.
    Used for real-time diagnostics and failure detection.

    Attributes:
        NOMINAL: All systems operating within normal parameters
        WATCHDOG: Watchdog timer triggered, recovery in progress
        REPLANNING: Path replanning active due to obstacle or deviation
    """

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> NodeHealth:
        """Look up NodeHealth by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "nominal", "WATCHDOG").

        Returns:
            Matching NodeHealth enum member.

        Raises:
            ValueError: If value does not match any NodeHealth.
        """
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(n.value for n in cls)
            error_message = f"Invalid node health: {value!r}. Expected one of {options}"
            raise ValueError(error_message) from err

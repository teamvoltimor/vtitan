"""Navigation tuning parameters for WRO 2026 track following.

This module centralizes all robot navigation tuning constants to enable:
- Runtime configuration without code edits
- Multiple tuning profiles for A/B testing
- Consistent parameter sharing between robot and simulation
- Easy parameter experimentation during competition

All parameters have been extracted from robot/src/navigation/ modules
and consolidated into a single source of truth.

Example usage:
    from shared.config.navigation_tuning import NavigationTuning

    # Use defaults
    tuning = NavigationTuning()
    print(tuning.pursuit.LOOKAHEAD_SHORT)  # 0.20

    # Load custom profile from YAML
    tuning = NavigationTuning.load_from_yaml("tuning_profiles/aggressive.yaml")
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar


@dataclass(frozen=True)
class ClearanceZones:
    """LIDAR clearance thresholds for speed control.

    These distances define zones around the robot where speed is controlled
    based on obstacle proximity. Values are in meters.

    Attributes:
        CONTACT_DIST: Robot creeps forward (< 0.10m) - immediate danger
        SLOW_DIST: Robot enters slow zone (0.10-0.25m)
        MEDIUM_DIST: Robot enters medium speed zone (0.25-0.50m)
        FAST_DIST: Robot can go full speed (> 0.50m)
    """

    CONTACT_DIST: ClassVar[float] = 0.10  # Creep forward zone
    SLOW_DIST: ClassVar[float] = 0.25  # Reduced speed
    MEDIUM_DIST: ClassVar[float] = 0.50  # Normal speed
    FAST_DIST: ClassVar[float] = 1.00  # Full speed capability


@dataclass(frozen=True)
class HeadingErrorZones:
    """Heading error thresholds for speed modulation.

    These thresholds define how much heading error reduces speed. Radians.

    Attributes:
        CRAWL: Severe misalignment (> 1.0 rad) - crawl speed
        SLOW: Large error (0.7-1.0 rad) - slow speed
        MEDIUM: Moderate error (0.4-0.7 rad) - medium speed
        NORMAL: Small error (< 0.4 rad) - normal speed
    """

    CRAWL: ClassVar[float] = 1.0  # ~57° - worst case
    SLOW: ClassVar[float] = 0.7  # ~40°
    MEDIUM: ClassVar[float] = 0.4  # ~23°
    NORMAL: ClassVar[float] = 0.2  # ~11°


@dataclass(frozen=True)
class PurePursuitParams:
    """Pure pursuit controller parameters for waypoint following.

    Implements lookahead-based steering to follow waypoints with
    crosstrack error minimization.

    Attributes:
        LOOKAHEAD_SHORT: Lookahead distance for sharp corners (m)
        LOOKAHEAD_LONG: Lookahead distance for straights (m)
        LOOKAHEAD_TRANSITION: Crosstrack error threshold to switch modes (m)
        STEER_KP: Proportional gain for steering P-controller
        MAX_STEERING_RATE: Maximum steering command rate (rad/s)
    """

    LOOKAHEAD_SHORT: ClassVar[float] = 0.20  # Close to corner
    LOOKAHEAD_LONG: ClassVar[float] = 0.40  # Normal straight
    LOOKAHEAD_TRANSITION: ClassVar[float] = 0.30  # Crosstrack threshold
    STEER_KP: ClassVar[float] = 1.5  # Steering P-gain
    MAX_STEERING_RATE: ClassVar[float] = 2.0  # rad/s


@dataclass(frozen=True)
class SpeedControlParams:
    """Speed control parameters for different zones.

    Maps clearance zones and heading errors to commanded motor speeds.
    Values are normalized to [-1.0, 1.0] motor command range.

    Attributes:
        MIN_SPEED: Minimum forward speed to overcome friction
        MAX_SPEED: Maximum safe forward speed
        CREEP_SPEED: Speed in contact zone
        SLOW_SPEED: Speed in slow zone
        MEDIUM_SPEED: Speed in medium zone
        FAST_SPEED: Speed in fast/open zone
    """

    MIN_SPEED: ClassVar[float] = 0.05  # Minimum to move
    MAX_SPEED: ClassVar[float] = 0.50  # Maximum safe speed
    CREEP_SPEED: ClassVar[float] = 0.05  # Contact zone
    SLOW_SPEED: ClassVar[float] = 0.15  # Near obstacles
    MEDIUM_SPEED: ClassVar[float] = 0.30  # Moderate clearance
    FAST_SPEED: ClassVar[float] = 0.50  # Open track


@dataclass(frozen=True)
class EscapeManeuverParams:
    """Escape maneuver parameters for collision recovery.

    When collision risk is detected, the robot executes escape maneuvers
    (K-turn, slalom) to clear obstacles and resume navigation.

    Attributes:
        REV_SPEED: Reverse speed during escapes
        REV_STEERING_SCALE: Steering aggressiveness while reversing
        K_TURN_MIN_FRAMES: Minimum frames for K-turn maneuver
        K_TURN_MAX_FRAMES: Maximum frames for K-turn maneuver
        SLALOM_REVERSE_FRAMES: Frames spent reversing during slalom
        SLALOM_FORWARD_FRAMES: Frames spent forward turning during slalom
        STUCK_MOVE_THRESHOLD: Distance threshold to detect stuck (m)
        STUCK_TIMEOUT_FRAMES: Frames without movement before stuck (20Hz)
    """

    REV_SPEED: ClassVar[float] = -0.20  # Reverse speed
    REV_STEERING_SCALE: ClassVar[float] = 0.8  # Steering while reversing
    K_TURN_MIN_FRAMES: ClassVar[int] = 6  # Minimum K-turn duration
    K_TURN_MAX_FRAMES: ClassVar[int] = 12  # Maximum K-turn duration
    SLALOM_REVERSE_FRAMES: ClassVar[int] = 8  # Reverse duration in slalom
    SLALOM_FORWARD_FRAMES: ClassVar[int] = 10  # Forward turn duration
    STUCK_MOVE_THRESHOLD: ClassVar[float] = 0.03  # 3cm movement threshold
    STUCK_TIMEOUT_FRAMES: ClassVar[int] = 40  # ~2 seconds at 20Hz


@dataclass(frozen=True)
class NavigationTuning:
    """Complete navigation tuning configuration.

    Aggregates all tuning parameters into a single frozen dataclass
    for immutability and type safety.

    Can be instantiated with defaults or loaded from YAML files
    to support multiple tuning profiles.

    Example:
        # Use hardcoded defaults
        tuning = NavigationTuning()

        # Load from YAML for custom tuning
        tuning = NavigationTuning.load_from_yaml("aggressive.yaml")

        # Access parameters
        print(tuning.pursuit.LOOKAHEAD_SHORT)
        print(tuning.clearance.SLOW_DIST)
    """

    clearance: ClearanceZones = ClearanceZones()
    heading: HeadingErrorZones = HeadingErrorZones()
    pursuit: PurePursuitParams = PurePursuitParams()
    speed: SpeedControlParams = SpeedControlParams()
    escape: EscapeManeuverParams = EscapeManeuverParams()

    @classmethod
    def load_from_yaml(cls, path: Path | str) -> NavigationTuning:
        """Load tuning configuration from YAML file.

        Enables runtime configuration without code recompilation.
        Supports multiple tuning profiles for experimentation.

        Args:
            path: Path to YAML file with tuning parameters

        Returns:
            NavigationTuning instance with loaded parameters

        Raises:
            FileNotFoundError: If YAML file not found
            yaml.YAMLError: If YAML parsing fails
            ValueError: If YAML structure invalid

        Example YAML structure:
            clearance:
              CONTACT_DIST: 0.05
              SLOW_DIST: 0.20
              MEDIUM_DIST: 0.45
              FAST_DIST: 0.90
            pursuit:
              LOOKAHEAD_SHORT: 0.15
              STEER_KP: 2.0
            # ... etc
        """
        try:
            import yaml
        except ImportError as err:
            raise ImportError("PyYAML required for loading YAML tuning files") from err

        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Tuning file not found: {path}")

        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise ValueError("YAML must contain a mapping (dict)")

        # Reconstruct nested dataclasses from dict
        # Use get() with empty dict defaults to allow partial configs
        return cls(
            clearance=ClearanceZones(**data.get("clearance", {}))
            if "clearance" in data
            else ClearanceZones(),
            heading=HeadingErrorZones(**data.get("heading", {}))
            if "heading" in data
            else HeadingErrorZones(),
            pursuit=PurePursuitParams(**data.get("pursuit", {}))
            if "pursuit" in data
            else PurePursuitParams(),
            speed=SpeedControlParams(**data.get("speed", {}))
            if "speed" in data
            else SpeedControlParams(),
            escape=EscapeManeuverParams(**data.get("escape", {}))
            if "escape" in data
            else EscapeManeuverParams(),
        )

    @classmethod
    def load_from_json(cls, path: Path | str) -> NavigationTuning:
        """Load tuning configuration from JSON file.

        Similar to load_from_yaml but uses JSON format instead.

        Args:
            path: Path to JSON file with tuning parameters

        Returns:
            NavigationTuning instance with loaded parameters

        Raises:
            FileNotFoundError: If JSON file not found
            json.JSONDecodeError: If JSON parsing fails
        """
        import json

        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Tuning file not found: {path}")

        with open(path, encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError("JSON must contain a mapping (dict)")

        return cls(
            clearance=ClearanceZones(**data.get("clearance", {}))
            if "clearance" in data
            else ClearanceZones(),
            heading=HeadingErrorZones(**data.get("heading", {}))
            if "heading" in data
            else HeadingErrorZones(),
            pursuit=PurePursuitParams(**data.get("pursuit", {}))
            if "pursuit" in data
            else PurePursuitParams(),
            speed=SpeedControlParams(**data.get("speed", {}))
            if "speed" in data
            else SpeedControlParams(),
            escape=EscapeManeuverParams(**data.get("escape", {}))
            if "escape" in data
            else EscapeManeuverParams(),
        )

    def to_dict(self) -> dict[str, Any]:
        """Export configuration as nested dictionary.

        Useful for serialization or debugging.

        Returns:
            Dictionary representation of all parameters
        """
        import dataclasses

        return {
            "clearance": dataclasses.asdict(self.clearance),
            "heading": dataclasses.asdict(self.heading),
            "pursuit": dataclasses.asdict(self.pursuit),
            "speed": dataclasses.asdict(self.speed),
            "escape": dataclasses.asdict(self.escape),
        }

    def to_json(self) -> str:
        """Export configuration as JSON string.

        Returns:
            JSON string representation of all parameters
        """
        import json

        return json.dumps(self.to_dict(), indent=2)

    def to_yaml(self) -> str:
        """Export configuration as YAML string.

        Requires PyYAML to be installed.

        Returns:
            YAML string representation of all parameters

        Raises:
            ImportError: If PyYAML not available
        """
        try:
            import yaml
        except ImportError as err:
            raise ImportError("PyYAML required for YAML export") from err

        return yaml.dump(self.to_dict(), default_flow_style=False)

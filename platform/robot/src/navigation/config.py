"""Navigation tuning profiles and configuration management.

This module centralizes all navigation constants into typed dataclasses,
enabling A/B testing, simulation vs. real-robot tuning profiles, and hot-swappable
configuration without code recompilation.

Usage:
    config = NavigationConfig.load_profile(DrivingProfile.REAL_ROBOT)
    if forward_clearance < config.clearance.contact_dist:
        speed = config.speed.contact
"""

import json
import logging
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

logger = logging.getLogger(__name__)


class DrivingProfile(Enum):
    """Driving profile selection for different environments and tuning strategies."""

    SIMULATION = "simulation"  # Gazebo sim tuning
    REAL_ROBOT = "real_robot"  # Real track tuning
    CONSERVATIVE = "conservative"  # Safe fallback for unknown conditions


@dataclass(frozen=True)
class ClearanceZones:
    """Forward clearance thresholds (metres) for speed scaling."""

    contact_dist: float = 0.10  # creep speed zone (very close)
    slow_dist: float = 0.25  # reduced speed zone
    medium_dist: float = 0.50  # normal speed zone
    fast_dist: float = 1.00  # full speed zone


@dataclass(frozen=True)
class HeadingErrorZones:
    """Heading error thresholds (radians) for speed reduction due to misalignment."""

    crawl: float = 1.0  # severe misalignment (worst case)
    slow: float = 0.7  # large heading error
    medium: float = 0.4  # moderate heading error


@dataclass(frozen=True)
class SpeedFractions:
    """Speed command scaling by clearance and heading error zone."""

    contact: float = 0.15  # very close distance — creep forward
    slow: float = 0.35  # slow zone
    medium: float = 0.50  # medium zone
    fast: float = 0.70  # fast zone
    full: float = 1.00  # clear path — full speed
    err_crawl: float = 0.25  # worst-case heading misalignment
    err_slow: float = 0.35  # large heading error
    err_medium: float = 0.55  # moderate heading error


@dataclass(frozen=True)
class LookaheadDistances:
    """Pure-pursuit look-ahead distances (metres)."""

    short: float = 0.20  # near corners (forward clearance < threshold)
    long: float = 0.40  # open road / normal straight driving
    threshold: float = 0.30  # switch from long to short lookahead


@dataclass(frozen=True)
class EscapeManeuvers:
    """Wall escape & obstacle avoidance parameters."""

    rev_speed: float = -0.20  # m/s during K-turn reverse phase
    steer_scale: float = 0.8  # fraction of max_steering_angle during escape
    obs_rev_speed: float = -0.25  # obstacle reverse speed
    obs_fwd_speed: float = 0.15  # obstacle forward speed
    obs_steer_scale: float = 0.70  # obstacle steering scale
    obs_fwd_steer_scale: float = 0.50  # forward steering scale during obstacle avoidance
    obs_reverse_fraction: float = 0.70  # fraction of frames spent reversing


@dataclass(frozen=True)
class StuckDetection:
    """Stuck robot escape parameters."""

    move_threshold: float = 0.03  # metres — robot is stuck if moved less than this
    escape_duration: int = 15  # frames (~0.75s at 20 Hz)
    close_wall_dist: float = 0.18  # metres — escape direction safety override


@dataclass(frozen=True)
class CollisionAvoidance:
    """LIDAR-based collision avoidance gains and thresholds."""

    side_gain: float = 0.08  # mild wall push away from close walls
    obstacle_gain: float = 0.30  # forward obstacle correction strength
    active_fwd_dist: float = 0.35  # forward distance threshold for obstacle avoidance


@dataclass(frozen=True)
class SteeringControl:
    """Ackermann steering control parameters."""

    kp: float = 1.5  # pure-pursuit proportional gain


@dataclass(frozen=True)
class NavigationConfig:
    """Complete navigation tuning profile combining all sub-configurations.

    This is the single source of truth for all navigation parameters.
    Load profiles from JSON files for runtime tuning without recompilation.
    """

    profile: DrivingProfile
    clearance: ClearanceZones
    heading_error: HeadingErrorZones
    speed: SpeedFractions
    lookahead: LookaheadDistances
    escape: EscapeManeuvers
    stuck: StuckDetection
    collision: CollisionAvoidance
    steering: SteeringControl

    @staticmethod
    def load_profile(profile: DrivingProfile) -> "NavigationConfig":
        """Load tuning profile from JSON file.

        Args:
            profile: DrivingProfile enum value to load.

        Returns:
            NavigationConfig: Fully instantiated configuration object.

        Raises:
            FileNotFoundError: If profile JSON file doesn't exist.
            json.JSONDecodeError: If profile JSON is malformed.
            KeyError: If required fields are missing from profile.
        """
        config_dir = Path(__file__).parent / "tuning_profiles"
        config_file = config_dir / f"{profile.value}.json"

        logger.info(f"Loading navigation profile: {profile.value} from {config_file}")

        if not config_file.exists():
            raise FileNotFoundError(f"Navigation profile not found: {config_file}")

        with open(config_file) as f:
            data = json.load(f)

        try:
            return NavigationConfig(
                profile=profile,
                clearance=ClearanceZones(**data["clearance"]),
                heading_error=HeadingErrorZones(**data["heading_error"]),
                speed=SpeedFractions(**data["speed"]),
                lookahead=LookaheadDistances(**data["lookahead"]),
                escape=EscapeManeuvers(**data["escape"]),
                stuck=StuckDetection(**data["stuck"]),
                collision=CollisionAvoidance(**data["collision"]),
                steering=SteeringControl(**data["steering"]),
            )
        except KeyError as e:
            raise KeyError(f"Navigation profile missing required field: {e}") from e

    @staticmethod
    def default() -> "NavigationConfig":
        """Return default configuration (real_robot profile).

        This is a fallback if profile loading fails.
        """
        return NavigationConfig(
            profile=DrivingProfile.REAL_ROBOT,
            clearance=ClearanceZones(),
            heading_error=HeadingErrorZones(),
            speed=SpeedFractions(),
            lookahead=LookaheadDistances(),
            escape=EscapeManeuvers(),
            stuck=StuckDetection(),
            collision=CollisionAvoidance(),
            steering=SteeringControl(),
        )

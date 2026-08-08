"""Headless-simulator-only tuning group."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias


class SimulationParams(BaseModel):
    """Knobs that exist only in the headless simulator.

    These do not describe the robot or the mat, so they belong neither in
    robot.toml nor track.toml -- but they decide what a simulated run scores,
    which makes them exactly the kind of value that must not be a literal
    buried in a module. The contact policy in particular governs whether a
    legal start already touching a wall is a failure or a recoverable state.

    Attributes:
        START_COLLISION_WINDOW_S: Opening seconds during which contact is
            treated as a start-position artefact rather than a crash, because
            a legal starting cell may already sit against a wall.
        START_COLLISION_GRACE_S: How long such an opening contact may persist
            before it counts as a real failure.
        LIDAR_INVALID_RAY_RATE: Fraction of rays returning no measurement.
            A placeholder: the real rate depends on the mat's surface and is
            worth measuring from a recorded bag rather than guessed at.
        DETECTION_CONFIDENCE: Confidence stamped on emulated camera
            detections.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    START_COLLISION_WINDOW_S: float = Field(
        default=2.0, validation_alias=_alias("START_COLLISION_WINDOW_S")
    )
    START_COLLISION_GRACE_S: float = Field(
        default=15.0, validation_alias=_alias("START_COLLISION_GRACE_S")
    )
    LIDAR_INVALID_RAY_RATE: float = Field(
        default=0.01, validation_alias=_alias("LIDAR_INVALID_RAY_RATE")
    )
    DETECTION_CONFIDENCE: float = Field(default=0.9, validation_alias=_alias("DETECTION_CONFIDENCE"))

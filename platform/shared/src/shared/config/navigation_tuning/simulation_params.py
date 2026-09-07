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
        COLLISION_MARGIN_M: How far past the visual wall face the chassis
            keep-out extends in TrackModel's collision check. Zero: the
            chassis may drive right up to the wall it can see, matching the
            real robot -- see TrackModel module docstring for why this is
            deliberately NOT the Go generator's fatter Gazebo collision mesh.
        AXIS_ALIGN_TOLERANCE: |cos(yaw)| below this counts as a quarter-turn
            for ObstacleBox.from_pose, so a block's extents are swapped
            rather than treated as axis-aligned.
        NO_PROGRESS_WINDOW_S: A run ends early, scored as ``stuck`` rather
            than run out to ``max_steps``, if the chassis never nets this
            many metres of straight-line displacement (``NO_PROGRESS_DISPLACEMENT_M``)
            within this many seconds. Sized well beyond any single escape
            maneuver (a few seconds) so a car that is actively recovering is
            never mistaken for one that never will.
        NO_PROGRESS_DISPLACEMENT_M: See ``NO_PROGRESS_WINDOW_S``. Small
            enough that ordinary sensor/steering noise while parked or
            creeping cannot itself satisfy it.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    START_COLLISION_WINDOW_S: float = Field(default=2.0, validation_alias=_alias("START_COLLISION_WINDOW_S"))
    START_COLLISION_GRACE_S: float = Field(default=15.0, validation_alias=_alias("START_COLLISION_GRACE_S"))
    LIDAR_INVALID_RAY_RATE: float = Field(default=0.01, validation_alias=_alias("LIDAR_INVALID_RAY_RATE"))
    DETECTION_CONFIDENCE: float = Field(default=0.9, validation_alias=_alias("DETECTION_CONFIDENCE"))

    VISION_THROUGH_PINHOLE: bool = Field(default=False, validation_alias=_alias("VISION_THROUGH_PINHOLE"))
    """Emulate camera BOUNDING BOXES and decode them with the shipped code.

    Off, the emulator hands the router world coordinates built from the TRUE
    range and bearing, so the simulator never runs ``_detection_to_world`` at
    all -- no pinhole, no bearing formula, no bbox-height gate, no aspect test,
    no frame-clip test. Every one of those is live on the robot.

    That gap is not hypothetical. The camera's bearing formula was MIRRORED
    until 2026-09-06 -- positive for a box on the RIGHT of the image, against a
    robot frame where left is positive -- so every sign was reflected across the
    heading axis onto the far wall of a 1 m corridor, and the 256-scenario
    corpus could not see it. It survived because this emulator reproduced the
    true geometry directly and the router's unit tests built their boxes by
    INVERTING the same formula; both agreed with the error. Only a hardware bag
    disagreed.

    On, the emulated box round-trips EXACTLY (1e-15 m) through
    ``detection_to_observation`` when the decode is correct, so the arm is not a
    noise source -- it is a wiring change. What it adds is that a wrong decode
    now shows up in the corpus.

    **Ships False until the corpus has been re-baselined on it**, since it
    changes which detections survive the discovery gates and therefore every
    Obstacles number. Still deliberately optimistic about everything else: no
    false positives, no wall confusion, no occlusion, no dropout.
    """
    COLLISION_MARGIN_M: float = Field(default=0.0, validation_alias=_alias("COLLISION_MARGIN_M"))
    AXIS_ALIGN_TOLERANCE: float = Field(default=1e-6, validation_alias=_alias("AXIS_ALIGN_TOLERANCE"))
    NO_PROGRESS_WINDOW_S: float = Field(default=30.0, validation_alias=_alias("NO_PROGRESS_WINDOW_S"))
    NO_PROGRESS_DISPLACEMENT_M: float = Field(default=0.08, validation_alias=_alias("NO_PROGRESS_DISPLACEMENT_M"))

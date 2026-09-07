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

    MIN_TURN_RADIUS_M: float = Field(default=0.0, ge=0.0, validation_alias=_alias("MIN_TURN_RADIUS_M"))
    """Floor on the chassis's turn radius, in metres. 0 disables (the old model).

    The bicycle term has no floor: at the shipped 85 deg lock it gives
    ``L_eff / (tan(85) * yaw_gain)`` = **1.5 cm** of radius, which a 30 x 19.4 cm
    four-wheeled chassis cannot do. The real car SATURATES instead.

    Measured from `/joint_states` drive-wheel travel against pose yaw over five
    hardware bags:

    | steer band | effective R | model R | ratio |
    |---|---|---|---|
    | 15-30 deg | 66.0 cm | 41.7 cm | 1.6x |
    | 30-45 deg | 38.2 cm | 22.5 cm | 1.7x |
    | **75-90 deg** | **28.9 cm** | **2.3 cm** | **12.7x** |

    Per bay exit, all at full lock, the effective radius came out 27-70 cm --
    and 746 cm on run_20260907_044405, which spent 44.1 s and rotated 2.2 deg.
    Steering past ~30 deg buys the real car almost nothing while the model keeps
    rewarding lock, so EVERY full-lock manoeuvre in simulation is optimistic by
    more than an order of magnitude. The in-bay exit is deterministic in sim (91
    ticks, 42 reverse, 44 forward, in all 16 scenarios, always rotating the
    intended way) where hardware ranges 5-44 s and sometimes rotates the WRONG
    way entirely.

    **Ships at 0.0 until the corpus is re-baselined on it**, because it changes
    every contact- and corner-dependent number in the repo. 0.29 is the measured
    value to use. Two shipped constants were sized against the un-floored model
    and should be re-derived once it is on: ``BAY_EXIT_ARC_STEER_NORM`` (1.0,
    whose "cliff at full lock" was a sim result) and ``max_corner_steer_deg``
    (21.25, chosen so a predicted 0.45 m arc fits a 0.60 m commit clearance --
    the real arc at that angle is ~0.66 m and does NOT fit).
    """

    VISION_RANGE_MODEL: bool = Field(default=False, validation_alias=_alias("VISION_RANGE_MODEL"))
    """Make the emulated camera GO BLIND with distance, the way the real one does.

    Off, a sign is detected out to ``CAMERA_FAR_CLIP`` -- **10 m** -- with no
    misses, no dropout and fixed 0.9 confidence. The real detector's measured
    range distribution over the 09-07 runs is **p50 0.70 m, p90 1.06-1.31 m,
    with 3.6% of detections beyond 1.4 m**: the deployed HEF stops resolving a
    pillar at a bit over a metre (running the tracked ONNX on identical pixels
    finds 1.6-2.0x more far detections, so it is the quantized model, not the
    framing). The simulated camera therefore sees roughly TEN TIMES further than
    the real one.

    That single gap decides whether a sign experiment means anything. Anything
    that trades on the camera being blind at range -- ``SIGN_LIDAR_PROPOSE``
    above all, whose entire value is that the LIDAR spots a pillar at a median
    1.31 m -- has NO measurable benefit in a sim where the camera already saw it
    at 10 m, and will read as pure cost. An A/B run with this off can refute
    such a feature only on cost, never confirm it on benefit.

    OFF by default because turning it on invalidates every existing Obstacles
    baseline: the robot loses sign vision it was scored with and never had on
    the mat. Those numbers were always optimistic; this is the sim getting
    honest, not a regression.
    """

    VISION_DETECT_R50_M: float = Field(default=1.10, gt=0.0, validation_alias=_alias("VISION_DETECT_R50_M"))
    """Range at which a sign is detected on half of frames (``VISION_RANGE_MODEL``).

    A per-frame logistic, not a hard cutoff, because the real failure is
    gradual: the detector's recall falls off with range rather than stopping.

    Calibrated by replaying ONE fixed trajectory's in-frame sign ranges and
    subsampling them offline, NOT by sweeping the live simulator: each candidate
    changes how the robot drives and therefore which geometry it samples, which
    made a live sweep non-monotonic (a tighter model returned a LONGER median
    range than no model at all). Calibrate against a fixed trajectory.

        fitted 1.10 / 0.15   p50 0.55 m   p90 1.14 m   1.4% beyond 1.4 m
        hardware target      p50 0.70 m   p90 1.06-1.31 m   3.6% beyond 1.4 m

    KNOWN LIMITATION: p50 comes out ~0.15 m low because this curve is monotone
    in range, so it can only ever REMOVE far detections. The real detector also
    loses NEAR ones -- a pillar being closed on overflows the frame and the
    aspect gate rejects the clipped box (61% of the reds it rejects are
    frame-clipped). Modelling that would need a near-field term as well; until
    then the simulated camera is slightly too good up close.
    """

    VISION_DETECT_FALLOFF_M: float = Field(
        default=0.15, gt=0.0, validation_alias=_alias("VISION_DETECT_FALLOFF_M")
    )
    """Width of the logistic falloff around ``VISION_DETECT_R50_M``.

    Smaller is a sharper cliff. Detection probability is
    ``1 / (1 + exp((range - R50) / FALLOFF))``.
    """

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

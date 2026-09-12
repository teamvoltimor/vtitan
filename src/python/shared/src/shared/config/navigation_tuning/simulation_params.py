"""Headless-simulator-only tuning group."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.constants.robot import RobotSpecs
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
    OBSTACLES_INNER_WALL_TERMINAL: bool = Field(
        default=True, validation_alias=_alias("OBSTACLES_INNER_WALL_TERMINAL")
    )
    """Whether inner-wall contact ENDS an Obstacles run, as this simulator scores it.

    It does in the competition's rules only for the OPEN challenge's outer
    boundary. Rule 9.18: "if the vehicle touches or bumps the walls, and the
    walls are not moved, the vehicle may continue the round, and no penalties
    will be incurred". ``TERMINAL_SURFACES``'s own docstring has said so since
    2026-09-03 and kept the strict scoring anyway, because relaxing it re-bases
    every Obstacles figure in the repo at once.

    Operator-confirmed 2026-09-11, and stated as the rule set rather than as one
    exception:

    * OPEN: the OUTER wall may not be touched. Unchanged by this flag -- Open's
      terminal set is ``{OUTER_WALL}`` and stays that way.
    * BOTH: a wall may not be MOVED if it is not fixed. In practice that takes
      considerable force, so a scrape at this chassis's mass and speed does not
      reach it. This is the fact that makes relaxing the flag sound rather than
      merely permitted.
    * OBSTACLES: the PARKING LOT may not be touched (9.24.7). Terminal here
      regardless of this flag.

    Also terminal regardless: displacing a sign out of its 85 mm placement
    circle (9.20, already softened by ``_score_obstacle_contact``) and passing on
    the wrong side (9.19/9.24.5). Those three are the whole of what ends an
    Obstacles round.

    Why it matters beyond bookkeeping: the strict scoring is what REFUTED the
    one lever that would widen the sign lane. ``SIGN_LANE_OFFSET_FRAC`` at full
    offset was rejected on wall collisions going 3 -> 23, and
    ``clamp_lateral``'s wall margin is sized off the chassis half-diagonal
    (0.1786 m, correct at 45 deg) rather than half-width plus margin (0.137 m on
    a straight). The lane's plateau reaches +0.172 m on failing crossings
    against a +0.304 m intent, and the counterfactual for a yaw-aware clearance
    is +0.228 m with coverage above 0.15 m going 52.7% -> 76.7%. If those 23
    contacts are legal, the refutation does not apply to the competition.

    DEFAULTS TRUE, i.e. the strict scoring every figure in this repo was
    measured under. Set it False for an A/B that scores the ACTUAL rule, and say
    which scoring a result used -- a number taken under one and compared against
    the other is meaningless.
    """
    LIDAR_INVALID_RAY_RATE: float = Field(default=0.01, validation_alias=_alias("LIDAR_INVALID_RAY_RATE"))
    DETECTION_CONFIDENCE: float = Field(default=0.9, validation_alias=_alias("DETECTION_CONFIDENCE"))

    MIN_TURN_RADIUS_TRACKS_SPEED: bool = Field(
        default=False, validation_alias=_alias("MIN_TURN_RADIUS_TRACKS_SPEED")
    )
    """Floor the curvature at ``min(cap, intercept + slope * |v|)`` instead of a constant.

    ``MIN_TURN_RADIUS_M`` is not a property of the chassis, it is that curve's
    value at ONE speed. Re-measured 2026-09-10 over 33 bags in free space at
    lock >= 30 deg (``scripts/bag/diag_bay_slip.py``):

    | mean speed m/s | 0.025 | 0.079 | 0.132 | 0.168 | 0.227 | 0.270 | 0.324 |
    |---|---|---|---|---|---|---|---|
    | R achieved m | 0.105 | 0.202 | 0.298 | 0.391 | 0.412 | 0.445 | 0.429 |

    0.29 m is what that reads at 0.118 m/s, near corridor speed. The BAY EXIT
    runs at creep end to end, where the constant is nearly 2x too large -- which
    is why `72e7172b` took the in-bay exit from 16/16 to 0/16 while hardware
    kept getting out of the pocket in 48 of 98 recorded runs.

    The constants live in robot.toml beside ``min_turn_radius_m``, because they
    describe this chassis rather than the simulator.

    Ships FALSE. Turning it on changes every contact- and corner-dependent
    number in the repo, exactly as shipping the constant floor did, so it wants
    a corpus A/B against this commit's PARENT and not a bay-only reading.
    """

    MIN_TURN_RADIUS_M: float = Field(
        default=RobotSpecs.MIN_TURN_RADIUS_M, ge=0.0, validation_alias=_alias("MIN_TURN_RADIUS_M")
    )
    """Simulator's override of the chassis turn-radius floor. 0 disables it.

    The VALUE is a physical property and lives in robot.toml, resolved through
    ``RobotSpecs.MIN_TURN_RADIUS_M``; restating the 0.29 here would make two
    names for one measurement and let them drift. What stays here is the
    ability to run the simulator at a DIFFERENT floor than the robot has, which
    is what an A/B against the un-floored model needs -- and note that this
    knob moves only the physics, never ``BayExit``'s dead reckoning, which
    reads the robot constant directly.

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

    **NOW SHIPS AT THE MEASURED 0.29** (2026-09-07). It changes every contact- and
    corner-dependent number in the repo, so ANY Obstacles figure recorded before
    this date was measured on a chassis that could pivot in 1.5 cm and is not
    comparable. Measured cost of the floor alone, 16 scenarios x 3 seeds: in-time
    29 -> 26 and collisions 5 -> 8. That is the sim becoming honest, not a
    regression -- and it is what finally exposed the escape manoeuvre being
    unable to rotate out of a corner (see `escape.MAX_ESCAPE_S`). Two shipped constants were sized against the un-floored model
    and should be re-derived once it is on: ``BAY_EXIT_ARC_STEER_NORM`` (1.0,
    whose "cliff at full lock" was a sim result) and ``max_corner_steer_deg``
    (21.25, chosen so a predicted 0.45 m arc fits a 0.60 m commit clearance --
    the real arc at that angle is ~0.66 m and does NOT fit).
    """

    VISION_RANGE_MODEL: bool = Field(default=True, validation_alias=_alias("VISION_RANGE_MODEL"))
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

    ON since 2026-09-07. It was off on the assumption that turning it on would
    invalidate every existing Obstacles baseline -- the robot losing sign vision
    it was scored with and never had on the mat. Priced with
    ``scripts/sim/diag_vision_range_ab.py`` (16 fixtures x 6 seeds x 2 arms,
    blind), that cost IS NOT THERE:

        off   in_time 59   laps3 69   collided 21   pass_side 1
        on    in_time 59   laps3 71   collided 18   pass_side 0

    In-time is identical and the rest moves the favourable way by the margin
    this corpus moves under reseeding, so it is FREE rather than better. Likely
    because at 10 m the emulator was feeding the router detections from OTHER
    corridors, and sign tracks are keyed on the robot's own corridor -- blinding
    it removes cross-corridor phantoms about as fast as real early sightings.
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

    Verified IN THE CLOSED LOOP at the shipped 1.10 / 0.15, not just offline:

        sim              p50 0.36 m   p90 1.07 m   1.0% beyond 1.4 m
        hardware target  p50 0.70 m   p90 1.06-1.31 m   3.6% beyond 1.4 m

    **The FAR end matches, which is the end that matters** -- whether the camera
    can see a sign EARLY is what every range-dependent feature trades on.

    KNOWN LIMITATION: p50 lands ~0.34 m low, and this is NOT fixable by tuning
    R50 -- sweeping 0.55 to 1.10 moves p50 only 0.31 -> 0.36 while p90 moves
    0.52 -> 1.07. A monotone curve can only REMOVE FAR detections, whereas
    hardware also loses NEAR ones: the aspect gate rejects a frame-clipped box,
    and 61% of the reds it rejects are clipped. Matching p50 needs a near-field
    loss term, not a different R50.

    MEASURE THIS POPULATION CAREFULLY -- it broke three attempts. The detected
    range distribution is signs passing BOTH the range model AND the emulator's
    own far-clip/HFOV test, measured as TRUE range from TRUE pose. Counting
    signs behind the robot inflates the tail; and measuring an observation's
    world position against the true robot state compares the BELIEVED frame to
    the TRUE one, which in blind mode are rotated apart -- that read 1.16 m
    where the truth was 0.36 m.
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

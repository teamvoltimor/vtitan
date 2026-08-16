"""Sweep harness for the Obstacles Challenge sign-avoidance investigation.

Runs all 16 Go-generated obstacles fixtures under a parameterised
``NavigationTuning`` / ``SignRouterConfig`` and reports the four metrics that
must always be read together (see
``platform/robot/docs/sign-avoidance-investigation.md``): collisions, laps>=1,
laps>=3 and timeouts. Tracking collisions alone has already produced one wrong
conclusion in that log.

``success`` is deliberately not the headline: it also requires ``parked``, and
parking is independently blocked by chassis-vs-pocket geometry, so it stays
0/16 regardless of any driving change. ``laps>=3`` is the driving-success
metric.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/sim/diag_sign_sweep.py diagnose          # run this one first
    python scripts/sim/diag_sign_sweep.py lookahead 0.12 0.20 0.30 0.40
    python scripts/sim/diag_sign_sweep.py arc 0.25 0.30 0.35 0.40 0.45
    python scripts/sim/diag_sign_sweep.py crosstrack 0.12 0.20

Swept modes (``lookahead`` ``arc`` ``speed`` ``steer-rate`` ``creep`` ``offset``
``unsplit-offset`` ``masked-offset`` ``buffer`` ``wall`` ``mask-radius``
``corridor-flip`` ``crosstrack``) take the values to sweep as positional
arguments. Fixed comparison modes (``baseline``
``profile`` ``diagnose`` ``ghost`` ``lidar`` ``pin-guard`` ``pin-heading-guard``)
ignore them.

A flat sweep here has twice turned out to be a disconnected knob rather than a
real result. If a mode reads byte-identical across a wide range, confirm the
override actually reaches the navigator before concluding anything — see
``_apply_patches`` and ``SweepConfig.tuning`` for the two that failed silently.
"""

from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, DictKeys
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Waypoint

import src.navigation.planning.sign_router as sign_router_module
import src.simulation.scenario_simulator as gateway_module
from src.navigation.geometry import chassis_half_diagonal_m
from src.navigation.track_geometry import corridor_widths_from_metadata, cross_track_error
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.track_model import TrackModel, obstacles_from_metadata

if TYPE_CHECKING:
    from collections.abc import Callable


class SweepMode(StrEnum):
    """Diagnostic sweep modes for sign-avoidance tuning."""

    LOOKAHEAD = "lookahead"
    ARC = "arc"
    SPEED = "speed"
    OFFSET = "offset"
    MASKED_OFFSET = "masked-offset"
    BUFFER = "buffer"
    UNSPLIT_OFFSET = "unsplit-offset"
    MASK_RADIUS = "mask-radius"
    CORRIDOR_FLIP = "corridor-flip"
    WALL = "wall"
    ACTIVATION = "activation"
    WALL_TUNED = "wall-tuned"
    REACH_BLIND = "reach-blind"
    BUFFER_BLIND = "buffer-blind"
    REACH = "reach"
    ACTIVATION_BUF = "activation-buf"
    BUFFER_TUNED = "buffer-tuned"
    BASELINE = "baseline"
    PROFILE = "profile"
    DIAGNOSE = "diagnose"
    GHOST = "ghost"
    LIDAR = "lidar"
    BLIND = "blind"
    BLIND_SOURCE = "blind-source"
    NO_PARK = "no-park"
    HYSTERESIS = "hysteresis"
    CROSSTRACK = "crosstrack"

MAX_STEPS = 6000
"""Matches ``tests/unit/test_obstacles_challenge_sim.py``."""

CORPUS_DIR = Path(__file__).resolve().parents[2] / ".corpus" / "obstacles" / "scenarios"
"""Pinned-seed sweep corpus, built by ``task gen:corpus CHALLENGE=obstacles``.

Gitignored and regenerated rather than committed -- same generator, same seed,
identical output. Pass ``--corpus`` to use it instead of the committed 16.
Attributions must come from here: the 16 gave the right aggregate but two wrong
diagnoses (see docs/sign-avoidance-investigation.md).
"""

_TARGET_LAPS = 3
"""Laps a scenario must finish to count as a driving success."""



_RESULT_LABEL_WIDTH = 32
_RESULT_METRIC_WIDTH = 2
_DETAIL_LABEL_WIDTH = 34
_DETAIL_COLLISION_WIDTH = 9
_COLLISION_PRECISION = 2
_FORMAT_2F = ".2f"
_FORMAT_3F = ".3f"
_FORMAT_1F = ".1f"
_CROSSTRACK_LABEL_WIDTH = 24
_CROSSTRACK_PERCENTILE_WIDTH = 5
_CROSSTRACK_PERCENTILE_PRECISION = "5.1f"
_DEFAULT_SHORT_LOOKAHEAD = 0.20
_DEFAULT_LONG_LOOKAHEAD = 0.40
_LOOKAHEAD_MULTIPLIER = 2.0
_ACTIVATION_PASSED_DIST_GAP = 0.20
_DEFORM_DEPTH_BUFFER_TUNED = 0.50
_ACTIVATION_DIST_TUNED_1 = 1.00
_ACTIVATION_DIST_TUNED_2 = 1.40
_PASSED_DIST_EXTENDED = 1.20
_PASSED_DIST_CEILING = 1.30
_ACTIVATION_DIST_FOR_BLIND_REACH = 1.00
_PASSED_DIST_FOR_BLIND_REACH = 1.20
_OFFSET_ZERO_ROUTER_OFF = 0.0
_ACTIVATION_FOR_PROFILE = 0.12
_PASSED_DIST_FOR_PROFILE = 0.24
# The removed for_obstacles profile asked for FAST_SPEED 0.30 m/s. Speeds are
# fractions of the 0.156 m/s ceiling now, and 0.30 m/s was already above it --
# which is precisely why that half of the profile was measured to be inert.
# 1.0 is the same command the robot actually received.
_SPEED_FOR_PROFILE = 1.0


class CollisionKind(StrEnum):
    """Categorization of what obstacle the chassis collided with."""

    WALL = "wall"
    SIGN = "sign"
    PARKING = "parking"
    NONE = "none"


def _with(group: Any, **fields: float | None) -> Any:
    """Copy a frozen pydantic tuning group, applying only the non-None fields."""
    updates = {name: value for name, value in fields.items() if value is not None}
    return group.model_copy(update=updates) if updates else group


@dataclass(frozen=True, slots=True)
class SweepConfig:
    """One point in a parameter sweep."""

    label: str
    lookahead_short: float | None = None
    lookahead_long: float | None = None
    fast_frac: float | None = None
    creep_frac: float | None = None
    """Override ``SpeedParams.CREEP_FRAC``.

    The tier the heading limiter drops to past ``HeadingErrorZones.CRAWL``,
    which exists because the steering actuator's slew rate is fixed and cannot
    track a sharp demand at speed. Being a FRACTION of ``MAX_SPEED_MPS``, it is
    rescaled by any hardware speed profile -- so the guard speeds up in lockstep
    with the thing it guards against, while the actuator does not. Sweep this to
    hold the rung at an absolute speed across profiles.
    """
    arc_radius: float | None = None
    steer_kp: float | None = None
    max_steering_rate: float | None = None
    strip_obstacles: bool = False
    """Delete signs and the parking lot from the metadata entirely.

    Isolates the path tracker from sign avoidance: the corridor layout, start
    pose and lap count stay identical, but there is nothing to route around and
    nothing to hit. If a scenario still fails here, the failure is not a sign
    problem.
    """

    strip_signs: bool = False
    """Delete only the traffic signs, keeping the parking lot physical."""

    strip_parking: bool = False
    """Delete only the parking lot, keeping the traffic signs physical."""

    ghost_signs: bool = False
    """Keep signs in the metadata — so ``SignRouter`` still routes around them —
    but remove them from the track's collision/raycast set.

    Separates "the router aims badly" from "the sign stopped the run". With
    physical signs a run dies at its first bad pass, hiding every later one.
    """

    lateral_offset: float | None = None
    """Override ``SignRouterConfig.lateral_offset`` (default 0.28 m)."""

    lidar_blind: bool = False
    """Keep signs physically collidable but invisible to the LIDAR.

    Splits the two things ``fd33fd5`` made true at once. If results move, the
    failure runs through perception and the collision controller's escape
    maneuver rather than through the waypoint path the sign router deforms.
    """

    deform_depth_buffer: float | None = None
    """Override ``SignRouterParams.DEFORM_DEPTH_BUFFER_M`` (shipped 0.5 m).

    How far past the inner square's own [CORNER_MIN, CORNER_MAX] span the
    lookahead target may sit and still be deformed. Since the target leads the
    robot by up to ``LOOKAHEAD_LONG``, too small a buffer switches avoidance off
    during the final approach to any sign at grid depth 1.0 or 2.0.

    Applied through tuning. It used to be set with ``setattr`` on a
    ``sign_router`` module global that the constants centralisation removed, so
    every arm using it raised ``AttributeError`` before running a scenario.
    """

    wall_clearance: float | None = None
    """Override the deformation's TOTAL wall clearance (shipped ~0.220 m).

    Chassis half-diagonal + ``WALL_CLEARANCE_MARGIN_M``. Lowering it lets a
    deformation push further toward the wall (more sign clearance, less wall
    clearance); raising it does the reverse. Read the wall/sign split, never the
    total — this knob trades directly between the two.

    Expressed as the TOTAL because that is the quantity the geometry arguments
    in the investigation doc are written in, but the tunable is the MARGIN, so
    :meth:`tuning` subtracts the chassis half-diagonal before applying it.
    """

    blind: bool = False
    """Withhold the corridor widths, the travel direction AND the sign layout.

    The competition configuration: no scenario file exists on the mat, so the
    router has to discover the signs from the camera (see sign_discovery) rather
    than be handed them. Every other figure in this harness is SIGHTED, so any
    navigation result has to be re-read against this before it can be claimed
    for the real robot.
    """

    passed_dist: float | None = None
    """Override ``SignRouterParams.PASSED_DIST_M`` (default 1.20 m).

    Must stay ABOVE ``activation_dist``. The two are coupled by
    ``_active_sign_candidates``, which engages a sign at ``activation_dist``
    and retires it beyond ``passed_dist`` in the same pass: invert the order and
    every sign is engaged and marked passed on the same tick, from a metre away,
    permanently. Nothing in the router guards this -- it is the 256/256-collision
    cliff at activation 1.30.
    """

    activation_dist: float | None = None
    """Override ``SignRouterParams.ACTIVATION_DIST_M`` (default 0.80 m).

    How far out a sign starts deforming the waypoint. The trace of scenario 5
    showed the deformed line being tracked correctly but converged to only
    asymptotically: the chassis drew abreast of the sign 0.25 m short of the
    commanded lateral target, because pure pursuit closes cross-track error
    over distance and there was not enough of it left. Engaging earlier buys
    that distance without touching the deformation itself.
    """

    park: bool = True
    """Attempt the parking maneuver after the final lap.

    ``False`` scores the run on laps alone. The parking blocks stay on the mat
    and stay collidable — only the maneuver is skipped. Sign avoidance is the
    open problem and parking sits downstream of it, so while avoidance is being
    measured a clean three-lap run should read as a clean three-lap run instead
    of as a ParkController give-up.
    """

    known_signs: bool = False
    """Hand the router the sign layout even in a blind run.

    Only meaningful with ``blind=True``, where it holds back the corridor
    widths and travel direction but skips discovery.
    """

    scenarios_dir: str | None = None
    """Run against a generated corpus instead of the committed 16 fixtures.

    Every figure in ``docs/sign-avoidance-investigation.md`` before 2026-08-01
    was taken over the same 16, which is a small sample for a space this size
    and under-represents configurations that need two signs interacting. A
    corpus is reproducible from the generator and its seed rather than
    committed:

        go run ./cmd/simgen generate --challenge obstacles             --num-scenarios 200 --seed 2026 --output-dir <dir>
    """

    commit_hysteresis: bool | None = None
    """Override ``SignRouterParams.COMMIT_HYSTERESIS``.

    ``False`` restores the per-tick nearest-wins race, where the commanded
    lateral line can jump between two signs mid-approach. Both arms belong in
    ONE harness run: editing the router between two runs silently mixes old and
    new code across an already-warm process pool.
    """

    depth_pin: bool | None = None
    """Override ``SignRouterParams.DEPTH_PIN`` (default True).

    ``False`` is the pre-pin arm every figure in the investigation doc older
    than 2026-08-01 was measured against.
    """

    pin_corner_guard: bool | None = None
    """Override ``SignRouterParams.PIN_CORNER_GUARD`` (default True).

    ``False`` restores the depth pin exactly as it was measured on 2026-08-01,
    before the robot-position squareness re-check landed. Only meaningful with
    ``depth_pin=True``.
    """

    stale_target_rescue: bool | None = None
    """Override ``SignRouterParams.STALE_TARGET_RESCUE`` (default False).

    Advances the waypoint index past a waypoint reading as behind the
    chassis in local frame, fixing the stale-target/wall-clip mechanism
    root-caused under `wideonly`. Obstacles-only by construction (gated on
    sign_router presence) regardless of this override.
    """

    sign_aware_speed: bool | None = None
    """Override ``SignRouterParams.SIGN_AWARE_SPEED`` (default False).

    Caps speed at the slow tier whenever the router actually deformed the
    target this tick by more than sign_deform_speed_threshold, giving the
    pursuit controller more time to close the ~6.5cm asymptotic shortfall.
    """

    sign_deform_speed_threshold: float | None = None
    """Override ``SignRouterParams.SIGN_DEFORM_SPEED_THRESHOLD_M`` (default 0.02).

    Only meaningful with ``sign_aware_speed=True``.
    """

    sign_aware_lookahead: bool | None = None
    """Override ``SignRouterParams.SIGN_AWARE_LOOKAHEAD`` (default False).

    Arms the short pursuit lookahead whenever a routed sign is within
    activation distance, since crosstrack (measured against the raw path)
    never rises during a sign pass to arm it on its own. Traced as the likely
    cause of a consistent ~6.5cm shortfall between the commanded avoidance
    line and the chassis when it draws level with a sign.
    """

    pin_heading_guard: bool | None = None
    """Override ``SignRouterParams.PIN_HEADING_GUARD`` (default False).

    Releases the depth pin once the robot's heading has rotated more than
    ``pin_heading_guard_deg`` since the pin engaged on the current sign, even
    if ``pin_corner_guard``'s position check still reads squarely-in-corridor.
    Traced on go_obstacles_0049 (subset64, sighted): the pin held a commanded
    point frozen for 46 ticks while the robot's yaw rotated 67 deg mid-corner,
    because the position-only guard never tripped. Only meaningful with
    ``depth_pin=True``.
    """

    pin_heading_guard_deg: float | None = None
    """Override ``SignRouterParams.PIN_HEADING_GUARD_DEG`` (default 35.0).

    Only meaningful with ``pin_heading_guard=True``.
    """

    sign_lane_planner: bool | None = None
    """Override ``SignRouterParams.SIGN_LANE_PLANNER`` (default False).

    Shifts the PLANNED PATH onto a pass-side lane through each signed
    corridor, instead of only overriding the pursuit target within
    ``activation_dist``. Structurally different from every other knob in this
    dataclass: the rest change when or how hard the existing carrot-chase
    fires, and all of them have measured flat or worse against the ~6.5cm
    shortfall. See ``navigation.planning.sign_lane``.
    """

    sign_lane_ramp: float | None = None
    """Override ``SignRouterParams.SIGN_LANE_RAMP_M`` (default 0.70 m).

    Along-corridor distance the lane takes to transition on and off the
    centreline. Only meaningful with ``sign_lane_planner=True``.
    """

    sign_lane_hold: float | None = None
    """Override ``SignRouterParams.SIGN_LANE_HOLD_M`` (default 0.25 m).

    Half-width of the full-offset plateau either side of a sign's own depth.
    Only meaningful with ``sign_lane_planner=True``.
    """

    sign_lane_offset_frac: float | None = None
    """Override ``SignRouterParams.SIGN_LANE_OFFSET_FRAC`` (default 1.0).

    Fraction of the avoidance offset the LANE carries; the carrot override
    still commands the full value at the pass. The lever against the wall
    collisions the lane costs -- see the field's own docstring for the
    clearance arithmetic. Only meaningful with ``sign_lane_planner=True``.
    """

    sign_lane_corner_entry: float | None = None
    """Override ``SignRouterParams.SIGN_LANE_CORNER_ENTRY_M`` (default 0.0).

    Corner-arc runway the lane may borrow to transition over. The lever
    against the lane's inner-square collisions: 1211 of the corpus's 1282
    signs sit at a section boundary, where the straight has no near-side
    runway at all. Only meaningful with ``sign_lane_planner=True``.
    """

    sign_lane_suppress_deform: bool | None = None
    """Override ``SignRouterParams.SIGN_LANE_SUPPRESS_DEFORM`` (default True).

    ``False`` keeps the carrot-level deformation running on top of the laned
    path, which asks for the offset twice. Both arms belong in one harness
    invocation. Only meaningful with ``sign_lane_planner=True``.
    """

    escape_mask_radius: float | None = None
    """Override ``SignRouterParams.ESCAPE_MASK_RADIUS_M`` (default 0.12 m).

    How close a LIDAR return must land to a sign the router is routing around
    to be withheld from the CRITICAL escape trigger. ``0.0`` disables the
    mapped/unmapped split, restoring the behaviour where the escape maneuver
    fires on every sign pass — which is the comparison every measurement of the
    split has to be read against.
    """

    corridor_flip_ticks: int | None = None
    """Override ``SignRouterParams.CORRIDOR_FLIP_TICKS`` (default 5).

    ``1`` is the pre-fix arm: a discovered sign's corridor — and therefore the
    world axis its deformation treats as lateral — was reassigned on every tick
    from an estimate that keeps moving, so a sign on a corner boundary flipped
    between two orthogonal axes at 20 Hz.
    """

    def tuning(self) -> NavigationTuning:
        """Materialise the ``NavigationTuning`` this config asks for.

        Starts from plain defaults, not ``for_obstacles()`` — the point of the
        sweep is to re-derive that profile, so it must not be baked into the
        baseline.

        ``NavigationTuning`` is a dataclass but its GROUPS are frozen pydantic
        models, so ``dataclasses.replace`` works on the former and raises
        ``TypeError`` on the latter. This previously used ``replace`` for both,
        which meant every mode that overrides a tuning value (``lookahead``,
        ``arc``, ``speed``) raised before running a single fixture. Groups are
        therefore updated with ``model_copy``.
        """
        base = NavigationTuning.load_default()
        pursuit = _with(
            base.pursuit,
            LOOKAHEAD_SHORT=self.lookahead_short,
            LOOKAHEAD_LONG=self.lookahead_long,
            STEER_KP=self.steer_kp,
            MAX_STEERING_RATE=self.max_steering_rate,
        )
        speed = _with(base.speed, FAST_FRAC=self.fast_frac, CREEP_FRAC=self.creep_frac)
        waypoints = _with(base.waypoints, ARC_RADIUS=self.arc_radius)
        sign_router = _with(
            base.sign_router,
            ESCAPE_MASK_RADIUS_M=self.escape_mask_radius,
            COMMIT_HYSTERESIS=self.commit_hysteresis,
            ACTIVATION_DIST_M=self.activation_dist,
            PASSED_DIST_M=self.passed_dist,
            DEPTH_PIN=self.depth_pin,
            PIN_CORNER_GUARD=self.pin_corner_guard,
            PIN_HEADING_GUARD=self.pin_heading_guard,
            PIN_HEADING_GUARD_DEG=self.pin_heading_guard_deg,
            SIGN_AWARE_LOOKAHEAD=self.sign_aware_lookahead,
            SIGN_AWARE_SPEED=self.sign_aware_speed,
            STALE_TARGET_RESCUE=self.stale_target_rescue,
            SIGN_LANE_PLANNER=self.sign_lane_planner,
            SIGN_LANE_RAMP_M=self.sign_lane_ramp,
            SIGN_LANE_HOLD_M=self.sign_lane_hold,
            SIGN_LANE_SUPPRESS_DEFORM=self.sign_lane_suppress_deform,
            SIGN_LANE_OFFSET_FRAC=self.sign_lane_offset_frac,
            SIGN_LANE_CORNER_ENTRY_M=self.sign_lane_corner_entry,
            SIGN_DEFORM_SPEED_THRESHOLD_M=self.sign_deform_speed_threshold,
            CORRIDOR_FLIP_TICKS=self.corridor_flip_ticks,
            DEFORM_DEPTH_BUFFER_M=self.deform_depth_buffer,
            # The field is the margin BEYOND the chassis half-diagonal; the knob
            # is the total. Converted here rather than at every call site so the
            # sweep values stay comparable with the doc's geometry tables.
            WALL_CLEARANCE_MARGIN_M=(
                None if self.wall_clearance is None else self.wall_clearance - chassis_half_diagonal_m()
            ),
        )
        return replace(base, pursuit=pursuit, speed=speed, waypoints=waypoints, sign_router=sign_router)


@dataclass(frozen=True, slots=True)
class ScenarioOutcome:
    """Per-scenario result, reduced to what the sweep reports on."""

    label: str
    collided: bool
    laps: int
    timed_out: bool
    collision_xy: Waypoint | None
    collision_kind: CollisionKind
    collision_step: int
    steps: int
    sim_time_s: float = 0.0
    """Simulated seconds the run took.

    Needed because ``laps >= 3`` is not the same as passing: the official round
    limit is ``CompetitionSpecs.ROUND_TIME_LIMIT_S`` (180 s) while this
    harness's own budget is ``MAX_STEPS * CONTROL_DT`` = 300 s, so a run could
    take 250 s, be counted a three-lap success here, and be stopped by the
    judges. The gap only became reachable once the kinematics started clamping
    to the measured 0.156 m/s drivetrain: a clean three-lap run now takes
    ~130 s, leaving 50 s of margin instead of the ~140 s it had at the speed
    profile's unreachable 0.5 m/s.
    """


def _without_parking(metadata: dict[str, Any]) -> dict[str, Any]:
    """Return metadata dict with parking lot removed."""
    return {k: v for k, v in metadata.items() if k != DictKeys.PARKING_LOT}


def _without_signs(metadata: dict[str, Any]) -> dict[str, Any]:
    """Return metadata dict with signs removed."""
    return {k: v for k, v in metadata.items() if k != DictKeys.SIGN_POSITIONS}


def _classify_collision(metadata: dict[str, Any], pose: tuple[float, float, float]) -> CollisionKind:
    """Name what the chassis was overlapping when the run ended.

    "Collisions" is a single counter covering three unrelated failures — outer
    wall, inner keep-out block, and an actual sign or parking block. Sweeping a
    sign-avoidance parameter against a number dominated by wall contacts
    measures the wrong thing, so every sweep reports the split.

    The two obstacle probes are built by REMOVAL, so each one tests the class it
    is not named after: ``_without_signs`` leaves the parking blocks standing.
    Pairing them the other way round — which this did — reported every sign
    strike as ``park`` and every parking strike as ``sign``, i.e. it inverted
    the split it exists to provide, and made a sign-avoidance sweep look like it
    was moving nothing but parking outcomes.
    """
    x, y, yaw = pose
    widths = corridor_widths_from_metadata(metadata)
    if TrackModel(widths).footprint_collides(x, y, yaw):
        return CollisionKind.WALL
    signs_only = _without_parking(metadata)
    if TrackModel(widths, obstacles=obstacles_from_metadata(signs_only)).footprint_collides(x, y, yaw):
        return CollisionKind.SIGN
    parking_only = _without_signs(metadata)
    if TrackModel(widths, obstacles=obstacles_from_metadata(parking_only)).footprint_collides(x, y, yaw):
        return CollisionKind.PARKING
    return CollisionKind.NONE


def _apply_patches(config: SweepConfig, metadata: dict[str, Any]) -> list[tuple[Any, str, Any]]:
    """Monkeypatch the knobs with no public seam; return the restore list.

    Split out of ``_run_one`` so the patches live in one place: several of these
    target module-level names that must be patched on the module that *resolves*
    them, not the one that defines them, and getting that wrong fails silently
    as an inert knob rather than as an error (see ``lateral_offset`` below).
    """
    restore: list[tuple[Any, str, Any]] = []
    if config.known_signs:
        # Blind withholds the track, the direction AND the signs at once.
        # ``ScenarioSimulator`` derives ``discover_signs`` from ``blind``
        # inline, so the only way to hold the first two and hand back the third
        # is to intercept the router's construction. Splits "blind fails
        # because discovery is too slow/wrong" from "blind fails because the
        # estimated layout costs path accuracy" — two very different fixes.
        original_router = gateway_module.SignRouter
        restore.append((gateway_module, "SignRouter", original_router))
        known = sign_router_module.signs_from_metadata(metadata)

        def _router_with_known_signs(_signs: Any, **kwargs: Any) -> Any:
            return original_router(known, **{**kwargs, "discover": False})

        gateway_module.SignRouter = _router_with_known_signs
    if config.ghost_signs:
        original = gateway_module.obstacles_from_metadata
        restore.append((gateway_module, "obstacles_from_metadata", original))

        # Forward *args/**kwargs rather than restating the signature. Both this
        # patch and the lidar_blind one below pinned the exact parameter list
        # they were written against, so when the real functions gained a
        # ``tuning`` argument every arm that used them started dying with
        # TypeError inside a worker process -- which surfaces as a bare
        # traceback from ProcessPoolExecutor, not as "this knob is stale".
        def _obstacles_without_signs(md: dict[str, Any], *args: Any, **kwargs: Any) -> Any:
            return original({k: v for k, v in md.items() if k != DictKeys.SIGN_POSITIONS}, *args, **kwargs)

        gateway_module.obstacles_from_metadata = _obstacles_without_signs
    if config.lateral_offset is not None:
        # Patch the name ``scenario_simulator`` itself resolves, and patch the
        # constructor it actually calls.
        #
        # This previously replaced ``sign_router.SignRouterConfig`` with a
        # zero-arg factory, on the premise that "SignRouter calls
        # SignRouterConfig() with no arguments". Both halves are now false:
        # ``scenario_simulator`` binds the name at import time (so patching the
        # defining module never reached it) and builds the config explicitly via
        # ``SignRouterConfig.from_tuning(...)`` (so the zero-arg default path is
        # dead code). The override therefore silently did nothing, which is
        # exactly what an "inert knob" looks like — every `offset` and
        # `masked-offset` figure taken while that was true measured the default
        # 0.20 four times over. Patch both modules so neither binding can
        # reintroduce the same silent no-op.
        original_cfg = gateway_module.SignRouterConfig
        restore.append((gateway_module, "SignRouterConfig", original_cfg))
        restore.append((sign_router_module, "SignRouterConfig", sign_router_module.SignRouterConfig))
        offset = config.lateral_offset

        class _OffsetOverride:
            """Stands in for ``SignRouterConfig``, forcing ``lateral_offset``."""

            @staticmethod
            def from_tuning(params: Any) -> Any:
                return replace(original_cfg.from_tuning(params), lateral_offset=offset)

            def __new__(cls) -> Any:
                return replace(original_cfg(), lateral_offset=offset)

        gateway_module.SignRouterConfig = _OffsetOverride
        sign_router_module.SignRouterConfig = _OffsetOverride
    if config.lidar_blind:
        original_track = gateway_module.TrackModel
        restore.append((gateway_module, "TrackModel", original_track))
        def _track_blind_to_obstacles(*args: Any, **kwargs: Any) -> Any:
            return original_track(*args, **{**kwargs, "lidar_sees_obstacles": False})

        gateway_module.TrackModel = _track_blind_to_obstacles
    # deform_depth_buffer and wall_clearance used to be patched onto sign_router
    # module globals here. Both moved into SignRouterParams and the globals were
    # deleted, so this block raised AttributeError on every arm that set them.
    # They now go through SweepConfig.tuning() like every other tunable.
    return restore


def _scenarios(config: SweepConfig) -> list[Any]:
    """The scenario set this config runs over."""
    return all_obstacles_demo_scenarios(Path(config.scenarios_dir) if config.scenarios_dir else None)


def _run_one(args: tuple[int, SweepConfig]) -> ScenarioOutcome:
    index, config = args
    scenario = _scenarios(config)[index]
    metadata = scenario.metadata
    dropped = set()
    if config.strip_obstacles or config.strip_signs:
        dropped.add(DictKeys.SIGN_POSITIONS)
    if config.strip_obstacles or config.strip_parking:
        dropped.add(DictKeys.PARKING_LOT)
    if dropped:
        metadata = {k: v for k, v in metadata.items() if k not in dropped}

    restore = _apply_patches(config, metadata)

    try:
        sim = ScenarioSimulator(
            metadata,
            num_laps=scenario.laps,
            seed=scenario.seed,
            tuning=config.tuning(),
            blind=config.blind,
            park=config.park,
        )
        result = sim.run(max_steps=MAX_STEPS)
    finally:
        for module, name, value in restore:
            setattr(module, name, value)

    classify_meta = metadata
    if config.ghost_signs:
        classify_meta = _without_signs(classify_meta)
    kind = _classify_collision(classify_meta, result.final_pose) if result.collided else CollisionKind.NONE
    collision_xy = None if result.collision_xy is None else Waypoint(*result.collision_xy)
    return ScenarioOutcome(
        label=scenario.label,
        collided=result.collided,
        laps=result.laps_completed,
        timed_out=result.timed_out,
        collision_xy=collision_xy,
        collision_kind=kind,
        collision_step=result.steps,
        steps=result.steps,
        sim_time_s=result.sim_time_s,
    )


@dataclass(frozen=True, slots=True)
class SweepResult:
    """Aggregate of one config across all 16 fixtures."""

    config: SweepConfig
    outcomes: list[ScenarioOutcome]

    @property
    def collisions(self) -> int:
        """Scenarios that ended in a terminal collision."""
        return sum(1 for o in self.outcomes if o.collided)

    @property
    def laps_ge_1(self) -> int:
        """Scenarios that completed at least one lap."""
        return sum(1 for o in self.outcomes if o.laps >= 1)

    @property
    def laps_ge_3(self) -> int:
        """Scenarios that completed the full three laps — the driving-success metric."""
        return sum(1 for o in self.outcomes if o.laps >= _TARGET_LAPS)

    @property
    def laps_ge_3_in_time(self) -> int:
        """Three laps AND inside the official round limit -- the competition result.

        Reported alongside ``laps>=3`` rather than replacing it: the difference
        between the two is exactly the set of runs that drive correctly but too
        slowly, which is a different failure from driving into a sign and wants
        a different fix.
        """
        return sum(
            1
            for o in self.outcomes
            if o.laps >= _TARGET_LAPS and o.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
        )

    @property
    def timeouts(self) -> int:
        """Scenarios that ran out of step budget."""
        return sum(1 for o in self.outcomes if o.timed_out)

    def kind(self, name: CollisionKind) -> int:
        """Scenarios whose collision was of the given kind (wall/sign/parking)."""
        return sum(1 for o in self.outcomes if o.collision_kind == name)

    def row(self) -> str:
        """The one-line summary: all four metrics plus the collision-kind split."""
        n = len(self.outcomes)
        return (
            f"RESULT {self.config.label:<{_RESULT_LABEL_WIDTH}} "
            f"collisions {self.collisions:>{_RESULT_METRIC_WIDTH}}/{n} "
            f"(wall {self.kind(CollisionKind.WALL):>{_RESULT_METRIC_WIDTH}} sign {self.kind(CollisionKind.SIGN):>{_RESULT_METRIC_WIDTH}} park {self.kind(CollisionKind.PARKING):>{_RESULT_METRIC_WIDTH}})  "
            f"laps>=1 {self.laps_ge_1:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"laps>=3 {self.laps_ge_3:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"in-time {self.laps_ge_3_in_time:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"timeouts {self.timeouts:>{_RESULT_METRIC_WIDTH}}/{n}"
        )

    def detail(self) -> str:
        """Per-scenario rows, for when an aggregate needs breaking down."""
        return "\n".join(
            f"DETAIL   {o.label:<{_DETAIL_LABEL_WIDTH}} {o.collision_kind:<{_DETAIL_COLLISION_WIDTH}} laps={o.laps} steps={o.steps} "
            f"at={None if o.collision_xy is None else (round(o.collision_xy.x, _COLLISION_PRECISION), round(o.collision_xy.y, _COLLISION_PRECISION))}"
            for o in self.outcomes
        )


def run_sweep(configs: list[SweepConfig], workers: int, verbose: bool = False) -> list[SweepResult]:
    """Run every config over every fixture, fixtures fanned out across processes."""
    scenario_count = len(_scenarios(configs[0])) if configs else 0
    results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for config in configs:
            outcomes = list(pool.map(_run_one, [(i, config) for i in range(scenario_count)]))
            result = SweepResult(config, outcomes)
            if verbose:
                print(result.detail(), flush=True)
            print(result.row(), flush=True)
            results.append(result)
    return results


# Cross-track error measurement


def _cross_track_errors(args: tuple[int, float | None]) -> list[float]:
    """Per-tick distance from the robot to its own planned path.

    Run with signs and parking stripped, deliberately. With signs present the
    router deforms the lookahead target on purpose, so distance-to-nominal-path
    measures intended avoidance rather than tracking error — and the run ends
    in a collision after a few hundred ticks anyway. Stripping them gives the
    honest "how far off its own line does the chassis sit" number that the
    +-6.7 cm sign-pass slack budget has to be compared against.

    ``lookahead`` dominates this measurement — a longer lookahead cuts corners —
    so it is an explicit argument rather than whatever the default happens to
    be. ``None`` means the shipped default.
    """
    index, lookahead = args
    scenario = all_obstacles_demo_scenarios()[index]
    metadata = {k: v for k, v in scenario.metadata.items() if k not in (DictKeys.SIGN_POSITIONS, DictKeys.PARKING_LOT)}
    config = SweepConfig(
        "crosstrack", lookahead_short=lookahead, lookahead_long=None if lookahead is None else lookahead * _LOOKAHEAD_MULTIPLIER
    )
    sim = ScenarioSimulator(metadata, num_laps=scenario.laps, seed=scenario.seed, tuning=config.tuning())
    path = [Waypoint(*w) if isinstance(w, tuple) else w for w in sim.waypoints]
    errors: list[float] = []

    def record(state: Any, _scan: Any) -> None:
        errors.append(cross_track_error(path, state.x, state.y))

    sim.run(max_steps=MAX_STEPS, on_step=record)
    return errors


def _percentile(values: list[float], q: float) -> float:
    """Linear-interpolated percentile ``q`` (0-1) of ``values``."""
    if not values:
        return math.nan
    ordered = sorted(values)
    pos = q * (len(ordered) - 1)
    lo = math.floor(pos)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def report_cross_track(workers: int, lookaheads: list[float]) -> None:
    """Print pooled cross-track error, one row per lookahead setting.

    With no values given, measures the shipped default only.
    """
    scenario_count = len(all_obstacles_demo_scenarios())
    settings: list[float | None] = list(lookaheads) if lookaheads else [None]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for lookahead in settings:
            per_scenario = pool.map(_cross_track_errors, [(i, lookahead) for i in range(scenario_count)])
            pooled = [e for errors in per_scenario for e in errors]
            label = f"default ({_DEFAULT_SHORT_LOOKAHEAD:{_FORMAT_2F}}/{_DEFAULT_LONG_LOOKAHEAD:{_FORMAT_2F}})" if lookahead is None else f"lookahead {lookahead:{_FORMAT_2F}}/{lookahead * _LOOKAHEAD_MULTIPLIER:{_FORMAT_2F}}"
            print(
                f"CROSSTRACK {label:<{_CROSSTRACK_LABEL_WIDTH}} "
                f"median {_percentile(pooled, 0.5) * 100:{_CROSSTRACK_PERCENTILE_PRECISION}}cm  "
                f"p90 {_percentile(pooled, 0.9) * 100:{_CROSSTRACK_PERCENTILE_PRECISION}}cm  "
                f"max {max(pooled) * 100:{_CROSSTRACK_PERCENTILE_PRECISION}}cm",
                flush=True,
            )


_SWEPT_MODES: dict[str, Callable[[float], SweepConfig]] = {
    "lookahead": lambda v: SweepConfig(
        f"lookahead {v:{_FORMAT_2F}}/{v * _LOOKAHEAD_MULTIPLIER:{_FORMAT_2F}}",
        lookahead_short=v,
        lookahead_long=v * _LOOKAHEAD_MULTIPLIER,
    ),
    "arc": lambda v: SweepConfig(f"arc_radius {v:{_FORMAT_2F}}", arc_radius=v),
    # Runway the lane takes to move on and off the centreline. The trade is
    # legible from the geometry: too short and the lane reproduces the very
    # late correction it replaces, too long and the corridor's straight is
    # spent off-centre for its whole length. Run `lane` first -- this only
    # means anything once the mechanism itself is worth tuning.
    "lane-ramp": lambda v: SweepConfig(
        f"lane ramp {v:{_FORMAT_2F}}",
        sign_lane_planner=True,
        sign_lane_ramp=v,
    ),
    "lane-hold": lambda v: SweepConfig(
        f"lane hold {v:{_FORMAT_2F}}",
        sign_lane_planner=True,
        sign_lane_hold=v,
    ),
    # How much of the avoidance offset the LANE carries, the carrot override
    # still commanding the full value at the pass. This is the wall-collision
    # lever: at 1.0 a centred sign pins the lane on clamp_lateral's floor for
    # a whole straight (wall 3 -> 23 over the corpus). Read the SIGN column
    # against the WALL column here -- the whole question is where the two
    # curves cross, not whether either moves.
    "lane-frac": lambda v: SweepConfig(
        f"lane offset frac {v:{_FORMAT_2F}}",
        sign_lane_planner=True,
        sign_lane_offset_frac=v,
    ),
    # clamp_lateral's total clearance (chassis half-diagonal + margin),
    # crossed with the lane. Named "wall" throughout, but every wall collision
    # traced under the lane on subset64 was against the INNER SQUARE, not the
    # outer wall -- go_obstacles_0020/0032/0042/0044/0061/0063 all died within
    # ~0.17 m of the block, i.e. about one chassis half-diagonal, and 4 cm
    # PAST the clamped lane rather than on it. Green signs route inward, so
    # the lane parks the chassis beside the block for a whole straight and any
    # tracking overshoot clips its corner. clamp_lateral applies this margin to
    # both sides, so this is the direct lever on that. Shipped total is
    # 0.179 + 0.04 = 0.219.
    # Corner-arc runway the lane may borrow. The targeted lever at the traced
    # mechanism: 1211 of 1282 corpus signs sit at a section boundary, so for
    # ~94% of them the straight has NO near-side runway and the lane starts at
    # full offset against a centred arc. Corners are provably empty (0 of 1282
    # signs), so the arc is free to transition through. Read the wall column
    # first here -- this exists to buy back the 3 -> 23 the lane cost.
    "lane-entry": lambda v: SweepConfig(
        f"lane corner entry {v:{_FORMAT_2F}}",
        sign_lane_planner=True,
        sign_lane_corner_entry=v,
    ),
    "lane-wall": lambda v: SweepConfig(
        f"lane wall clearance {v:{_FORMAT_3F}}",
        sign_lane_planner=True,
        wall_clearance=v,
    ),
    "speed": lambda v: SweepConfig(f"fast_frac {v:{_FORMAT_2F}}", fast_frac=v),
    # The one knob mechanically coupled to a hardware speed profile. It is in
    # rad/SECOND while every speed tier is a fraction of MAX_SPEED_MPS, so a
    # faster profile leaves the steering actuator exactly as quick while giving
    # it less distance to act over. Sweep it alongside VTITAN_HARDWARE_PROFILE,
    # not on its own -- at the base speed the shipped 2.0 is already tuned.
    "steer-rate": lambda v: SweepConfig(f"max_steering_rate {v:{_FORMAT_2F}}", max_steering_rate=v),
    # The other half of the same coupling, and the half that is not a fraction
    # problem but becomes one: pass the fraction that holds the CRAWL rung at
    # the absolute speed it was tuned at. Under fastwide that is 0.1014/0.234 =
    # 0.43, against the shipped 0.65.
    "creep": lambda v: SweepConfig(f"creep_frac {v:{_FORMAT_2F}}", creep_frac=v),
    "offset": lambda v: SweepConfig(f"lateral_offset {v:{_FORMAT_3F}}", lateral_offset=v),
    # The offset sweep CROSSED with lidar_blind. This was how the escape layer
    # was first identified as the gate, back when it was the only way to make
    # the offset knob move.
    #
    # Superseded as a diagnostic by `unsplit-offset`, which asks the same
    # question without blinding the safety layer to a whole obstacle class: it
    # toggles only whether a ROUTED sign can trigger the escape. Prefer it.
    # `masked-offset` is kept because the historical table in
    # docs/sign-avoidance-investigation.md cites it — but note that table was
    # taken while the offset override was silently disconnected, so it does not
    # currently reproduce and should not be trusted without re-measuring.
    "masked-offset": lambda v: SweepConfig(
        f"lateral_offset {v:{_FORMAT_3F}}, lidar blind to signs",
        lateral_offset=v,
        lidar_blind=True,
    ),
    "buffer": lambda v: SweepConfig(f"depth_buffer {v:{_FORMAT_2F}}", deform_depth_buffer=v),
    # The offset sweep with the mapped/unmapped escape split DISABLED. Pair it
    # with `offset` (split enabled at its default radius) to read what the split
    # is worth: the two differ only in whether a routed sign can trigger the
    # reactive escape maneuver.
    "unsplit-offset": lambda v: SweepConfig(
        f"lateral_offset {v:{_FORMAT_3F}}, escape split off",
        lateral_offset=v,
        escape_mask_radius=_OFFSET_ZERO_ROUTER_OFF,
    ),
    "mask-radius": lambda v: SweepConfig(f"escape_mask_radius {v:{_FORMAT_3F}}", escape_mask_radius=v),
    # Sweep 1 (per-tick reassignment, the pre-fix arm) against 5+ to read what
    # holding a sign's corridor steady is worth. Both arms in ONE invocation:
    # the axis flip this targets is a property of the router's own state, so
    # comparing two separately-launched runs would mix process pools warmed
    # against different code.
    #
    # BLIND deliberately, and this knob is meaningless without it: a sighted
    # run takes its signs from scenario metadata and never revises them, so no
    # corridor is ever reassigned and every value reads identical. Sweeping it
    # sighted would report a flat knob and mean nothing by it.
    # park=False for the same reason the doc keeps laps>=3 as the headline:
    # with parking on, every one of the 16 ends its run on the parking block and
    # the collision kind reads "park 16/16" whatever the router did, which hides
    # exactly the sign-pass difference this arm exists to measure.
    "corridor-flip": lambda v: SweepConfig(
        f"BLIND corridor_flip_ticks {int(v)}",
        corridor_flip_ticks=int(v),
        blind=True,
        park=False,
    ),
    "wall": lambda v: SweepConfig(f"wall_clearance {v:{_FORMAT_3F}}", wall_clearance=v),
    "activation": lambda v: SweepConfig(f"activation_dist {v:{_FORMAT_2F}}", activation_dist=v),
    # Wall clearance measured at the TUNED activation distance, not the stock
    # 0.80. The two are coupled: activation distance buys convergence runway,
    # wall clearance sets how far there is to converge to. Sweeping either
    # against the other's untuned value measures a corner of the space nobody
    # would ship.
    "wall-tuned": lambda v: SweepConfig(f"wall {v:{_FORMAT_3F}} @ act {_ACTIVATION_DIST_TUNED_1:{_FORMAT_2F}}", wall_clearance=v, activation_dist=_ACTIVATION_DIST_TUNED_1),
    # The corner dead zone, measured at the tuned activation distance. Tracing
    # the 1.20->1.30 cliff showed deform_waypoint returning the waypoint
    # UNTOUCHED 0.20 m from a sign at grid depth 1.0: the lookahead target had
    # crossed into the corner, _is_squarely_in_corridor rejected every
    # candidate, and avoidance switched itself off during the final approach.
    # This buffer is how far past the corner span a target may sit and still be
    # deformed, so it is the direct control on that dead zone.
    # Re-measure the activation curve on top of the tuned buffer. The 1.20->1.30
    # collapse was traced to the corner dead zone, so if that diagnosis is right
    # a wider buffer should flatten the cliff rather than merely shift it --
    # which is also what decides whether the activation peak is safe to adopt on
    # hardware, where pose error would otherwise tip runs across it.
    # Activation and passed distance raised TOGETHER, holding the 0.10 m gap
    # that keeps activation below passed. The plateau ends at 1.20 only because
    # PASSED_DIST_M sits there: past it a sign is engaged and marked passed on
    # the same tick, from a metre away, and retired for the rest of the run --
    # which is the 256/0 cliff, not any geometric limit. This asks whether the
    # pair wants to move up, or whether 1.00 is a real optimum.
    # THE COMPETITION CONFIGURATION. No scenario file exists on the mat, so the
    # router discovers signs from the camera rather than being handed them.
    # activation/passed/buffer were all tuned on SIGHTED arms, which is a
    # configuration that never occurs in a round -- blind improving too was
    # luck, not design. Any operating point meant for the robot has to be
    # chosen here.
    "reach-blind": lambda v: SweepConfig(
        f"BLIND activation {v:{_FORMAT_2F}} / passed {v + _ACTIVATION_PASSED_DIST_GAP:{_FORMAT_2F}} @ buffer {_DEFORM_DEPTH_BUFFER_TUNED:{_FORMAT_2F}}",
        activation_dist=v,
        passed_dist=v + _ACTIVATION_PASSED_DIST_GAP,
        deform_depth_buffer=_DEFORM_DEPTH_BUFFER_TUNED,
        blind=True,
    ),
    "buffer-blind": lambda v: SweepConfig(
        f"BLIND depth_buffer {v:{_FORMAT_2F}} @ act {_ACTIVATION_DIST_TUNED_2:{_FORMAT_2F}}", deform_depth_buffer=v, activation_dist=_ACTIVATION_DIST_TUNED_2, blind=True,
    ),
    "reach": lambda v: SweepConfig(
        f"activation {v:{_FORMAT_2F}} / passed {v + _ACTIVATION_PASSED_DIST_GAP:{_FORMAT_2F}} @ buffer {_DEFORM_DEPTH_BUFFER_TUNED:{_FORMAT_2F}}",
        activation_dist=v,
        passed_dist=v + _ACTIVATION_PASSED_DIST_GAP,
        deform_depth_buffer=_DEFORM_DEPTH_BUFFER_TUNED,
    ),
    "activation-buf": lambda v: SweepConfig(
        f"activation {v:{_FORMAT_2F}} @ buffer {_DEFORM_DEPTH_BUFFER_TUNED:{_FORMAT_2F}}", activation_dist=v, deform_depth_buffer=_DEFORM_DEPTH_BUFFER_TUNED,
    ),
    "buffer-tuned": lambda v: SweepConfig(
        f"depth_buffer {v:{_FORMAT_2F}} @ act {_ACTIVATION_DIST_TUNED_1:{_FORMAT_2F}}", deform_depth_buffer=v, activation_dist=_ACTIVATION_DIST_TUNED_1,
    ),
}
"""Modes that sweep one numeric knob across the values given on the CLI."""

_FIXED_MODES: dict[str, list[SweepConfig]] = {
    "baseline": [SweepConfig("defaults")],
    "profile": [
        SweepConfig("defaults"),
        SweepConfig("for_obstacles (removed)", lookahead_short=_ACTIVATION_FOR_PROFILE, lookahead_long=_PASSED_DIST_FOR_PROFILE, fast_frac=_SPEED_FOR_PROFILE),
    ],
    # Separates "the tracker broke" from "sign avoidance failed" — run this
    # first on any change; no aggregate collision count can tell them apart.
    "diagnose": [
        SweepConfig("everything physical"),
        SweepConfig("no signs, parking physical", strip_signs=True),
        SweepConfig("signs physical, no parking", strip_parking=True),
        SweepConfig("nothing physical", strip_obstacles=True),
    ],
    "ghost": [
        SweepConfig("ghost signs, router on", ghost_signs=True),
        SweepConfig("ghost signs, router off", ghost_signs=True, lateral_offset=_OFFSET_ZERO_ROUTER_OFF),
        SweepConfig("physical signs, router off", lateral_offset=_OFFSET_ZERO_ROUTER_OFF),
    ],
    "lidar": [
        SweepConfig("lidar sees signs (default)"),
        SweepConfig("lidar blind to signs", lidar_blind=True),
        SweepConfig("lidar blind, router off", lidar_blind=True, lateral_offset=0.0),
    ],
    # Does the sighted result survive the competition configuration? Every other
    # figure here is sighted, and the escape split in particular depends on the
    # router knowing where the signs are -- which blind has to DISCOVER before
    # it can own them. A sign not yet confirmed by ObservedSignMap is not on
    # routed_sign_positions, so it keeps the full reactive guard, which is the
    # conservative direction but means blind cannot be assumed to match.
    "blind": [
        SweepConfig("sighted (signs from metadata)"),
        SweepConfig("blind (track, direction, signs)", blind=True),
    ],
    # What the escape split + half-diagonal offset are worth IN BLIND, measured
    # in one tree so nothing else that has landed since can be mistaken for
    # them. Comparing today's blind run against a blind figure recorded in an
    # earlier session does not do this: unrelated changes (e.g. the
    # replace_path lap-seam fix) move blind lap counts on their own.
    # Which half of "blind" costs the sighted result? Middle row holds the
    # track and direction back but hands over the signs, so the gap between it
    # and the outer rows attributes the loss to discovery or to layout error.
    "blind-source": [
        SweepConfig("sighted (everything known)"),
        SweepConfig("blind track+direction, signs known", blind=True, known_signs=True),
        SweepConfig("fully blind (signs discovered)", blind=True),
    ],
    # Sign avoidance on its own, with parking deferred until it is solved.
    # Blocks stay on the mat and stay collidable; only the maneuver is skipped.
    "no-park": [
        SweepConfig("laps only, sighted", park=False),
        SweepConfig("laps only, blind", blind=True, park=False),
        SweepConfig("with parking, sighted"),
        SweepConfig("with parking, blind", blind=True),
    ],
    # What commit hysteresis is worth. Both arms in one run, deliberately.
    "hysteresis": [
        SweepConfig("per-tick race (hysteresis off)", commit_hysteresis=False),
        SweepConfig("committed sign held (hysteresis on)", commit_hysteresis=True),
        SweepConfig("blind, hysteresis off", blind=True, commit_hysteresis=False),
        SweepConfig("blind, hysteresis on", blind=True, commit_hysteresis=True),
    ],
    # Does the offset arrive in time? 182/182 surviving collisions are lag
    # (diag_failure_split.py), and the symmetric taper is only at 0.125 when a
    # sign activates. Baseline 0.0 shares the invocation, so the comparison is
    # immune to the warm-pool hazard.
    # What the depth pin is worth. Both arms in one run, deliberately: the pin
    # first appeared mid-session and its 182 -> 119 was briefly mis-attributed
    # to an unrelated harness fix measured in a different invocation.
    "pin": [
        SweepConfig("blind, pin off (pre-pin baseline)", blind=True, park=False, depth_pin=False),
        SweepConfig("blind, pin on", blind=True, park=False, depth_pin=True),
        SweepConfig("sighted, pin off", park=False, depth_pin=False),
        SweepConfig("sighted, pin on", park=False, depth_pin=True),
    ],
    # Item 2a: the 11 wall collisions the depth pin introduced (0 -> 11), and
    # whether the robot-position squareness re-check in `_pin_depth` closes them.
    # That re-check landed 2026-08-11 inside an unrelated commit, ten days after
    # the 11 was measured, so nothing here has ever been read against it.
    #
    # Three arms, ONE invocation, in the same configuration the pin was
    # attributed in (blind, park=False): pin off reproduces the 182/0-wall
    # baseline, guard off must reproduce 11 wall / 108 sign / 137 in-time or the
    # knob is not wired to the thing being toggled, and guard on is what ships.
    # The guard can only cost sign collisions -- it suppresses the pin on a
    # subset of ticks -- so read the wall AND sign columns together.
    "pin-guard": [
        SweepConfig("blind, pin off", blind=True, park=False, depth_pin=False),
        SweepConfig("blind, pin on, corner guard OFF", blind=True, park=False, depth_pin=True, pin_corner_guard=False),
        SweepConfig("blind, pin on, corner guard ON (shipped)", blind=True, park=False, depth_pin=True, pin_corner_guard=True),
    ],
    # SIGHTED, unlike pin-guard above -- the freeze this targets was traced on
    # go_obstacles_0049 sighted (subset64), not blind: the depth pin froze the
    # commanded point for 46 ticks while the robot's yaw rotated 67 deg mid-
    # corner, because PIN_CORNER_GUARD's position-only check never tripped.
    "pin-heading-guard": [
        SweepConfig("sighted, pin on, heading guard OFF (pre-fix)", pin_heading_guard=False),
        SweepConfig("sighted, pin on, heading guard ON 25deg", pin_heading_guard=True, pin_heading_guard_deg=25.0),
        SweepConfig("sighted, pin on, heading guard ON 30deg", pin_heading_guard=True, pin_heading_guard_deg=30.0),
        SweepConfig("sighted, pin on, heading guard ON 35deg (shipped)", pin_heading_guard=True, pin_heading_guard_deg=35.0),
        SweepConfig("sighted, pin on, heading guard ON 40deg", pin_heading_guard=True, pin_heading_guard_deg=40.0),
        SweepConfig("sighted, pin on, heading guard ON 45deg", pin_heading_guard=True, pin_heading_guard_deg=45.0),
    ],
    "sign-aware-lookahead": [
        SweepConfig("sighted, sign-aware lookahead OFF (shipped)", sign_aware_lookahead=False),
        SweepConfig("sighted, sign-aware lookahead ON", sign_aware_lookahead=True),
    ],
    "stale-target-rescue": [
        SweepConfig("sighted, stale-target rescue OFF (shipped)", stale_target_rescue=False),
        SweepConfig("sighted, stale-target rescue ON", stale_target_rescue=True),
    ],
    "sign-aware-speed": [
        SweepConfig("sighted, sign-aware speed OFF (shipped)", sign_aware_speed=False),
        SweepConfig("sighted, sign-aware speed ON, threshold 0.02", sign_aware_speed=True, sign_deform_speed_threshold=0.02),
        SweepConfig("sighted, sign-aware speed ON, threshold 0.05", sign_aware_speed=True, sign_deform_speed_threshold=0.05),
        SweepConfig("sighted, sign-aware speed ON, threshold 0.10", sign_aware_speed=True, sign_deform_speed_threshold=0.10),
    ],
    # The lane planner moves the PATH instead of the carrot, so unlike every
    # other sign mode here the arms differ in mechanism, not magnitude. Read
    # the wall column as carefully as the sign one: a lane is a deliberately
    # off-centre line through a 1.0 m corridor, which is the trade the offset
    # sweep already lost once (raising lateral_offset to 0.33 bought sign hits
    # back at the price of 12 new wall hits).
    #
    # Measured on the FULL 256 corpus, sighted, with SIGN_LANE_CORNER_ENTRY_M
    # at its 0.50 default (the parameter that dominates everything else here
    # -- see its docstring; without it the whole feature is worth ~2%):
    #   OFF (baseline)     234 collisions (wall 3 sign 231)  laps>=3  22  in-time  19
    #   lane, no override   70            (wall 5 sign  65)  laps>=3 186  in-time 128
    #   lane + override     64            (wall 7 sign  57)  laps>=3 192  in-time  82
    # Read the in-time column, not the collision column: the override buys 6
    # more three-lap finishes and loses 46 inside the round limit, because its
    # depth pin holds the commanded point abeam a sign rather than letting it
    # advance. That answer REVERSES without corner runway, where the lane
    # cannot reach its own line unaided and the override is what rescues it
    # (subset64: lane alone 61/64, lane + override 55/64, baseline 56/64) --
    # so never read these two arms without checking which runway they ran at.
    "lane": [
        SweepConfig("sighted, lane planner OFF (shipped)", sign_lane_planner=False),
        SweepConfig("sighted, lane ON, override suppressed (default)", sign_lane_planner=True),
        SweepConfig("sighted, lane ON + override", sign_lane_planner=True, sign_lane_suppress_deform=False),
    ],
    # The lane under the REAL competition condition: no scenario file on the
    # mat, so signs come from camera discovery through the believed pose. The
    # lane is rebuilt whenever the discovered layout changes
    # (SignRouter.lane_fingerprint), so unlike sighted mode it is being
    # replanned mid-run off estimates that move -- and, per
    # obstacles_blind_localizer_rotational_lock, off a believed pose that can
    # be confidently rotated. Read this against the sighted `lane` numbers:
    # a gain that survives sighted but vanishes blind is a localizer result,
    # not a lane result.
    # Measured subset64: OFF 64/64 collisions, laps>=3 1, in-time 0; lane ON
    # 59/64, laps>=3 5, in-time 3. Real but small -- and the contrast with
    # sighted (234 -> 70 over the corpus) is the finding, not the 5. The lane
    # is aimed through the believed pose, so a pose locked onto a rotated
    # solution puts a correct maneuver in the wrong place. Blind is bounded by
    # the localizer, not by the avoidance maneuver; measured, not inferred.
    # Before corner runway existed this arm was 64/64 either way, i.e. exactly
    # zero -- so the lane does now reach blind, it just cannot outrun a wrong
    # pose.
    "lane-blind": [
        SweepConfig("blind, lane planner OFF (shipped)", blind=True, sign_lane_planner=False),
        SweepConfig("blind, lane ON (default, override suppressed)", blind=True, sign_lane_planner=True),
    ],
    "blind-split": [
        SweepConfig("blind, pre-fix (offset 0.20, split off)", blind=True, lateral_offset=0.20, escape_mask_radius=0.0),
        SweepConfig("blind, split only (offset 0.20)", blind=True, lateral_offset=0.20),
        SweepConfig("blind, offset only (0.28, split off)", blind=True, lateral_offset=0.28, escape_mask_radius=0.0),
        SweepConfig("blind, both (shipped defaults)", blind=True),
    ],
}
"""Modes with a fixed comparison set, ignoring any CLI values."""

MODES = ("crosstrack", *_FIXED_MODES, *_SWEPT_MODES)


def _build_configs(mode: str, values: list[float]) -> list[SweepConfig]:
    """Map a CLI mode plus its numeric arguments to the configs to run."""
    if mode in _FIXED_MODES:
        return _FIXED_MODES[mode]
    if mode in _SWEPT_MODES:
        return [_SWEPT_MODES[mode](v) for v in values]
    msg = f"unhandled mode {mode!r}"
    raise ValueError(msg)


def main() -> None:
    """Parse arguments and run the requested sweep."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        choices=MODES,
    )
    parser.add_argument("values", nargs="*", type=float)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--verbose", action="store_true", help="print per-scenario detail rows")
    parser.add_argument(
        "--scenarios-dir",
        default=None,
        help="directory of generated *_metadata.json to run instead of the committed 16",
    )
    parser.add_argument("--corpus", action="store_true", help=f"shorthand for --scenarios-dir {CORPUS_DIR}")
    args = parser.parse_args()

    if args.mode == "crosstrack":
        report_cross_track(args.workers, args.values)
        return

    configs = _build_configs(args.mode, args.values)
    scenarios_dir = args.scenarios_dir or (str(CORPUS_DIR) if args.corpus else None)
    if scenarios_dir:
        configs = [replace(c, scenarios_dir=scenarios_dir) for c in configs]
    run_sweep(configs, args.workers, verbose=args.verbose)


if __name__ == "__main__":
    main()

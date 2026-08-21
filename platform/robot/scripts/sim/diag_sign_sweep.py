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
    python scripts/sim/diag_sign_sweep.py lane-geometry --corpus 0 -0.1 0.1  # planner only, no sim

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
import json
import math
import sys
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from enum import StrEnum
from itertools import pairwise
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import (
    CompetitionSpecs,
    DictKeys,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
)
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import NavigatorPhase, Section
from shared.domain.models import (
    CorridorWidthEntry,
    CorridorWidths,
    ScenarioMetadata,
    Waypoint,
)

import src.navigation.planning.sign_router as sign_router_module
import src.simulation.scenario_simulator as gateway_module
from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from scripts.common.stats import percentile
from src.navigation.geometry import chassis_half_diagonal_m
from src.navigation.planning.sign_discovery import SignSpec
from src.navigation.planning.sign_lane import (
    SignLaneParams,
    _axis_coords,
    _control_points,
    _in_lane_span,
    _interpolate,
    apply_sign_lanes,
)
from src.navigation.planning.waypoints import calculate_waypoints, corridor_for_position
from src.navigation.track_geometry import corridor_widths_from_metadata, cross_track_error, project_onto_path
from src.navigation.utils import wrap_angle
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.track_model import TrackModel, obstacles_from_metadata

if TYPE_CHECKING:
    from collections.abc import Callable

    from src.navigation.core_navigator import CoreNavigator
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState


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




_RESULT_LABEL_WIDTH = 32
_RESULT_METRIC_WIDTH = 2
_DETAIL_LABEL_WIDTH = 34
_DETAIL_COLLISION_WIDTH = 9
_COLLISION_PRECISION = 2
_CLEARANCE_PRECISION = 3


def _round_or_none(value: float | None) -> float | None:
    """``round`` that passes ``None`` through, for optional per-scenario fields."""
    return None if value is None else round(value, _COLLISION_PRECISION)


def _round_mm(value: float | None) -> float | None:
    """``round`` to millimetres, for the sub-centimetre lane clearances.

    The shared ``_round_or_none`` quantises to 1 cm, which is coarser than the
    quantities the lane analysis turns on -- a 5 mm and a 14 mm planned gap are
    different findings but round to the same 0.01.
    """
    return None if value is None else round(value, _CLEARANCE_PRECISION)


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

    retrace_escape: bool | None = None
    """Override ``SignRouterParams.RETRACE_ESCAPE`` (default False).

    Reverse along ground the chassis just occupied instead of along an arc.
    Aimed at keeping mask-off's sign gain (57 -> 41) without its wall cost
    (0 -> 13), and at removing the reverse guard's dependence on a rear sector
    the next chassis may not have.
    """

    retrace_dist: float | None = None
    """Override ``SignRouterParams.RETRACE_DIST_M`` (default 0.25 m)."""

    sign_contact_evade: bool | None = None
    """Override ``SignRouterParams.SIGN_CONTACT_EVADE`` (default False).

    Steer away and creep when the RAW scan reads CRITICAL but the MASKED one
    does not -- i.e. the imminent contact is a sign the router owns. The middle
    rung between today's two options (ignore it, or reverse into a wall).
    """

    sign_contact_steer: float | None = None
    """Override ``SignRouterParams.SIGN_CONTACT_STEER`` (default 0.35)."""

    sign_lane_commit_ahead: float | None = None
    """Override ``SignRouterParams.SIGN_LANE_COMMIT_AHEAD_M`` (default 0.0, off).

    How much of the path ahead of the chassis a lane rebuild may not move.
    Blind-only in effect: sighted runs build the path once and never rebuild.
    """

    explore_lap_speed_frac: float | None = None
    """Override ``SignRouterParams.EXPLORE_LAP_SPEED_FRAC`` (default 1.0, off).

    Speed ceiling for the first lap of a BLIND Obstacles run, as a fraction of
    the normal ceiling. Targets the measured shape of blind failure: 78% of it
    happens during lap 1, because a corridor's signs cannot be seen until the
    robot is inside that corridor -- but they persist for laps 2-3.
    """

    ingest_range: float | None = None
    """Override ``SignDiscoveryParams.MAX_INGEST_RANGE_M`` (default 2.0 m).

    How far away a camera observation may be accepted into discovery at all.
    Measured as the binding constraint on blind runs: publish distance tops
    out at 1.99 m against this 2.0 m cap, and 11 of 44 published signs only
    existed once the robot was ALREADY inside ``ACTIVATION_DIST_M``. The lane
    planner's whole advantage is runway, so a sign that appears at 1.5 m has
    already lost most of it.
    """

    min_hits: int | None = None
    """Override ``SignDiscoveryParams.MIN_HITS`` (default 3).

    Confirming observations before a track is published. Lower publishes
    sooner (more runway) at the cost of acting on weaker evidence -- read the
    SIGN column against any gain, since a spurious sign deforms the path
    toward a hazard that is not there.
    """

    obstacles_center_bias: float | None = None
    """Override ``WaypointParams.OBSTACLES_CENTER_BIAS_M`` (default 0.0, centred).

    How far the planned centreline sits toward the INNER block on Obstacles.
    Open keeps its own ``CENTER_BIAS_M`` regardless. Centred is the right
    answer geometrically -- all Obstacles corridors are 1.0 m and signs sit
    0.10 m either side of centre, so 0.0 leaves symmetric room -- but the
    chassis is documented to drift OUTWARD while tracking
    (centre_bias_is_tracking_not_sign), which makes a nonzero inner bias
    compensation rather than preference. Sweep it rather than assuming either.
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
        waypoints = _with(
            base.waypoints,
            ARC_RADIUS=self.arc_radius,
            OBSTACLES_CENTER_BIAS_M=self.obstacles_center_bias,
        )
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
            EXPLORE_LAP_SPEED_FRAC=self.explore_lap_speed_frac,
            SIGN_LANE_COMMIT_AHEAD_M=self.sign_lane_commit_ahead,
            RETRACE_ESCAPE=self.retrace_escape,
            RETRACE_DIST_M=self.retrace_dist,
            SIGN_CONTACT_EVADE=self.sign_contact_evade,
            SIGN_CONTACT_STEER=self.sign_contact_steer,
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
        sign_discovery = _with(
            base.sign_discovery,
            MAX_INGEST_RANGE_M=self.ingest_range,
            MIN_HITS=self.min_hits,
        )
        return replace(
            base,
            pursuit=pursuit,
            speed=speed,
            waypoints=waypoints,
            sign_router=sign_router,
            sign_discovery=sign_discovery,
        )


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
    uturns: int = 0
    """Heading reversals detected during the run — see ``_UTurnDetector``."""

    corner_uturns: int = 0
    """How many of ``uturns`` happened in a corner zone rather than a straight."""

    escape_starts: int = 0
    """How many times the escape machinery engaged during the run.

    Counted per ENGAGEMENT, not per tick: a latched maneuver holds its phase
    for its whole duration, so ticking would report duration, not frequency.
    Normalise by laps driven before comparing arms -- an arm that survives
    longer gets more escapes for free (see the corner-escape rate mistake in
    the 2026-08-16 notes)."""

    steps_since_escape: int | None = None
    """Ticks between the last escape engagement and the run ending.

    The attribution signal for the reverse arc: an escape that fires and is
    followed within ``_ESCAPE_ATTRIBUTION_STEPS`` by a collision is evidence
    the escape drove into something, as opposed to a collision the escape
    never had a chance to prevent. ``None`` when no escape ever ran, which is
    itself the answer for those scenarios."""

    sign_masked: bool | None = None
    """For a SIGN collision, whether the struck sign was already in the
    router's ``routed_sign_positions`` at the moment of collision -- i.e.
    whether ``ESCAPE_MASK_RADIUS_M`` would have withheld its returns from the
    escape trigger. ``None`` for a non-sign collision or a run with no
    router. Answers the open question behind the proximity-gated unmask idea:
    do blind's escape-never-fired collisions actually involve a MASKED sign,
    or is the escape silent for some other reason entirely?"""

    sign_ahead_m: float | None = None
    """Along-track distance from the final pose to the struck sign, in the
    chassis's own heading frame at the moment of collision. Same geometry as
    ``_sign_evade_steer``'s trigger test. ``None`` for a non-sign collision."""

    sign_lateral_m: float | None = None
    """Cross-track distance from the final pose to the struck sign, signed
    the same way as ``_sign_evade_steer``'s ``lateral``. ``None`` for a
    non-sign collision."""

    sign_lane_clamped: bool | None = None
    """For a SIGN collision, whether ``clamp_lateral`` bound on the STRUCK
    sign's lane plateau -- see ``_lane_is_clamped``.

    Decides whether the geometry thread has anything left in it. A clamped
    plateau sits at an 18.14 cm gap and is already saturated, so it can only
    be improved by relaxing the clamp bound itself; a free one sits at
    27.86 cm with ~8 cm of margin, where plan geometry cannot be the cause and
    the collision must be tracking error. ``None`` for a non-sign collision.
    """

    sign_lane_boundary: bool | None = None
    """For a SIGN collision, whether the struck sign is at a section BOUNDARY.

    Cross-tabulated with ``sign_lane_clamped`` because the two are NOT the
    same split: boundary signs run roughly half clamped, half free, so
    "198/199 collisions are at boundaries" does not by itself establish that
    the collisions are at squeezed plateaux."""

    collision_crosstrack_m: float | None = None
    """|crosstrack| against the navigator's own planned path on the last tick
    before a collision.

    The test that separates the two collision populations. A FREE sign carries
    ~10.6 cm of margin at the median collision yaw, so a collision there
    requires an excursion far past the 6.55 cm p90 -- if these really are
    gross excursions this reads large, and the sub-centimetre clearance-budget
    model simply does not apply to them."""

    planned_gap_m: float | None = None
    """Distance from the struck sign to the navigator's ACTUAL planned path at
    the collision tick.

    The check on ``sign_lane_clamped``, which reports what the plateau WOULD
    be from geometry alone. ``apply_sign_lanes`` only rewrites waypoints that
    pass ``_in_lane_span``, so a plateau whose depth falls outside that window
    is never applied at all and the path stays on the centreline -- roughly
    0.10 m from a sign. Where this reads far below the theoretical 18.14 /
    27.86 cm, the lane did not materialise, and the collision is a planner
    coverage failure rather than a clearance-budget or tracking one."""

    struck_corridor_count: int | None = None
    """How many DISTINCT corridors the struck sign was routed into at the
    collision tick.

    Normally 1. A discovery landing near the corner diagonal gets an ambiguous
    ``corridor_for_position``, and the same physical sign can end up as two
    routed entries under two corridors. ``apply_sign_lanes`` groups by
    corridor and applies each group in turn to a progressively-mutated path,
    and the two groups shift DIFFERENT axes (a NORTH corridor's lateral is y,
    an EAST corridor's is x), so they compose into a diagonal displacement
    neither intended -- traced on go_obstacles_0255 as moving the planned gap
    from 4.62 cm (centreline, untouched) to 0.75 cm, i.e. the lane steering
    INTO the sign it was meant to avoid."""

    planned_outward_m: float | None = None
    """Signed lateral offset of the planned path from the struck sign, positive
    OUTWARD -- see ``_outward_pass_offset``.

    The disambiguator for a small ``planned_gap_m``. Positive-but-short means
    the lane fell short or never applied and the path sat near the centreline;
    NEGATIVE means the plan crossed to the sign's inward side, which extra
    clearance on the intended side cannot fix."""

    collision_phase: Any = None
    """Navigator phase on the last tick before a collision.

    Names the excursion rather than merely sizing it: an escape or u-turn
    driving into a sign is a controller-arbitration failure, while a plain
    pursuit phase at 10+ cm of crosstrack is a tracking failure. Those need
    opposite fixes, and ``collision_crosstrack_m`` alone cannot tell them
    apart."""

    sign_color_match: bool | None = None
    """When ``sign_masked``, whether the discovered track's voted colour
    agrees with the struck sign's true colour. ``False`` means the pass-side
    plan itself was wrong (steered to the wrong side of a sign it otherwise
    knew about correctly), not that the plan wasn't executed in time --
    two different bugs the mask-vs-unmasked split alone cannot tell apart.
    ``None`` when not masked or not a sign collision."""

    sim_time_s: float = 0.0
    """Simulated seconds the run took.

    Needed because ``laps >= 3`` is not the same as passing: the official round
    limit is ``CompetitionSpecs.ROUND_TIME_LIMIT_S`` (180 s) while this
    harness's own budget is ``OBSTACLES_MAX_STEPS * CONTROL_DT`` = 300 s, so a run could
    take 250 s, be counted a three-lap success here, and be stopped by the
    judges. The gap only became reachable once the kinematics started clamping
    to the measured 0.156 m/s drivetrain: a clean three-lap run now takes
    ~130 s, leaving 50 s of margin instead of the ~140 s it had at the speed
    profile's unreachable 0.5 m/s.
    """


_UTURN_WINDOW_TICKS = 60
"""~3 s at 20 Hz. Long enough to contain a whole reversal, short enough that
two legitimate consecutive corners (90 deg each, and never that close together
on this track) cannot sum past the threshold below."""

_UTURN_THRESHOLD_RAD = math.radians(150.0)
"""How much net heading change counts as a reversal rather than a corner.

A planned corner is 90 deg. 150 deg is comfortably past that and comfortably
short of 180, so it catches a genuine turn-around without flagging a corner
taken wide, which is the distinction the whole metric exists to make."""

_UTURN_DEBOUNCE_TICKS = 40
"""Ticks to wait before another reversal may be counted, so one long spin is
reported as one event rather than as however many windows it spans."""


class _UTurnDetector:
    """Counts heading reversals from the per-tick pose stream.

    Deliberately measured against the chassis's OWN recent heading rather than
    against the planned path: a robot that has turned around is a failure
    whether or not the path agrees, and keying on path bearing would make the
    metric silent in exactly the case where the path itself is the problem.
    """

    def __init__(self) -> None:
        self._deltas: deque[float] = deque(maxlen=_UTURN_WINDOW_TICKS)
        self._prev_yaw: float | None = None
        self._cooldown = 0
        self.events: list[tuple[float, float]] = []

    def update(self, x: float, y: float, yaw: float) -> None:
        """Fold one tick's pose in, recording an event if a reversal completed."""
        if self._prev_yaw is not None:
            self._deltas.append(wrap_angle(yaw - self._prev_yaw))
        self._prev_yaw = yaw
        if self._cooldown > 0:
            self._cooldown -= 1
            return
        # Net, not absolute: a corner entered and exited cleanly nets ~90 deg,
        # while weaving back and forth cancels toward zero. Only a sustained
        # turn in ONE direction accumulates past the threshold.
        if abs(sum(self._deltas)) > _UTURN_THRESHOLD_RAD:
            self.events.append((x, y))
            self._deltas.clear()
            self._cooldown = _UTURN_DEBOUNCE_TICKS

    @property
    def corner_events(self) -> int:
        """Events that happened in a corner zone (outside the inner square's span on BOTH axes)."""
        return sum(1 for x, y in self.events if _in_corner_zone(x, y))


_ESCAPE_PHASES = frozenset(
    {
        NavigatorPhase.ESCAPE_TRIGGERED,
        NavigatorPhase.ACTIVE_MANEUVER,
        NavigatorPhase.STUCK_ESCAPE_MANEUVER,
        NavigatorPhase.STUCK_ESCAPE_HOLDING,
    },
)
"""Phases that mean the escape machinery, not pure pursuit, is driving."""

_ESCAPE_ATTRIBUTION_STEPS = 40
"""~2 s at 20 Hz: how recently an escape must have run for a collision to be
attributable to it. Long enough to cover a latched maneuver plus the tick or
two of re-acquisition after it, short enough that an escape a whole corridor
ago does not get the blame."""


class _EscapeTracker:
    """When the escape machinery last engaged, so a collision can be attributed.

    Reads the navigator's own phase rather than re-deriving "is it escaping"
    from the pose stream: the phase is what the navigator actually decided,
    and a reverse arc that drives into a wall looks, from outside, exactly
    like ordinary bad tracking.
    """

    def __init__(self, navigator: CoreNavigator) -> None:
        self._navigator = navigator
        self._engaged = False
        self.step = 0
        self.starts = 0
        self.last_step: int | None = None

    def update(self) -> None:
        """Fold one tick of navigator phase in."""
        self.step += 1
        engaged = self._navigator.debug_snapshot.phase in _ESCAPE_PHASES
        if engaged:
            if not self._engaged:
                self.starts += 1
            self.last_step = self.step
        self._engaged = engaged

    @property
    def steps_since_escape(self) -> int | None:
        """Ticks from the last engagement to now, or None if none ever ran."""
        return None if self.last_step is None else self.step - self.last_step


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


def _into_believed_frame(
    sim: ScenarioSimulator, true_xy: tuple[float, float], final_pose: tuple[float, float, float]
) -> tuple[float, float]:
    """Map a TRUE world position into the frame the navigator believes it is in.

    Everything the router produces -- routed positions, lane specs -- is
    reprojected through the robot's own pose estimate, never ground truth.
    Comparing any of it against a true position mixes two frames, which is
    incoherent regardless of which one is "right", and under the blind
    rotational lock the gap is 1-2+ m. Rotating the true position by the
    believed-vs-true pose offset puts both sides of such a comparison in the
    same frame.

    Falls back to the identity when the localizer has no estimate yet, which
    is the only case where the two frames are not meaningfully different.
    """
    believed = sim.gateway.get_current_pose()
    if believed is None:
        return true_xy
    true_x, true_y, true_yaw = final_pose
    dyaw = wrap_angle(believed.yaw - true_yaw)
    cos_d, sin_d = math.cos(dyaw), math.sin(dyaw)
    dx, dy = true_xy[0] - true_x, true_xy[1] - true_y
    return (believed.x + dx * cos_d - dy * sin_d, believed.y + dx * sin_d + dy * cos_d)


_LANE_DELIVERED_FRAC = 0.9
"""Fraction of its plateau a lane must reach to count as fully delivered.

Not 1.0: the plan is a polyline sampled at coarse waypoint spacing, and the
closest point to a sign can sit slightly off the plateau's flat top even for a
lane that ramped correctly. Loose enough not to punish that, tight enough that
the "half-delivered" reading under investigation falls well outside it.
"""

_SIGN_MATCH_DIST_M = 0.30
"""How close a routed position must land to the struck sign's true position
to count as the same sign. Matches ``SignDiscoveryParams.DETECTION_MATCH_DIST_M``
-- generous enough to cover discovery estimate error in blind mode, tight
enough that it can't accidentally match a different, nearby sign."""


def _sign_mask_attribution(
    sim: ScenarioSimulator,
    metadata: dict[str, Any],
    collision_xy: tuple[float, float],
    final_pose: tuple[float, float, float],
) -> tuple[bool | None, float | None, float | None, bool | None]:
    """Was the sign the chassis struck already masked from the escape trigger?

    Finds the true sign nearest the collision point, then checks whether the
    router had it in ``routed_sign_positions`` (masked) at run end, plus the
    along-track/lateral geometry ``_sign_evade_steer`` would have seen. Exists
    to test the proximity-gated-unmask premise directly, instead of assuming
    it from the escape-never-fired correlation alone: a collision the escape
    never reacted to could be a masked sign (the mask hypothesis), or a sign
    never routed at all (a discovery-timing problem the mask can't fix).

    Also reports ``color_match`` when masked: whether the discovered track's
    voted colour agrees with the true sign's. A masked-but-still-collided run
    can mean the pass-side plan was correct and simply not executed in time
    (no runway), or that the plan itself was wrong because discovery's colour
    vote landed on the wrong side -- two different bugs with the same
    symptom, so the split matters. ``lane_specs`` is used rather than
    ``routed_sign_positions`` because it includes passed signs too, and a
    struck sign could in principle have just been marked passed.

    ``routed_sign_positions``/``lane_specs`` are BELIEVED-frame -- discovery
    reprojects every observation through the robot's own pose estimate (see
    ``vision_emulator.emulate_sign_observations``'s docstring), never ground
    truth. Comparing them directly against ``struck``'s TRUE position mixes
    two different reference frames, which is incoherent regardless of which
    frame is "right" -- the same anti-pattern that docstring calls out and
    was fixed for the navigation stack itself. Under the blind-mode
    rotational-lock bug the believed pose can be 1-2+ m off true, which made
    this read "never routed" for a sign that WAS genuinely routed and being
    avoided inside the robot's own self-consistent frame (traced directly on
    go_obstacles_0009: the routed track and the struck sign are the same
    physical sign, related by the exact same rigid transform as the
    believed/true pose gap). Transform ``struck`` into the believed frame
    first -- using the offset between ``final_pose`` (true) and the
    localizer's own current estimate, a stable rigid rotation for the whole
    run -- before comparing, so both sides of every comparison below are in
    the same frame.
    """
    signs = sign_router_module.signs_from_metadata(metadata)
    if not signs:
        return None, None, None, None
    cx, cy = collision_xy
    struck = min(signs, key=lambda s: math.hypot(s.x - cx, s.y - cy))
    struck_x, struck_y = _into_believed_frame(sim, (struck.x, struck.y), final_pose)

    router = sim.navigator.sign_router
    routed = router.routed_sign_positions if router is not None else []
    masked = any(math.hypot(rx - struck_x, ry - struck_y) < _SIGN_MATCH_DIST_M for rx, ry in routed)

    color_match = None
    if masked and router is not None:
        for spec, _corridor in router.lane_specs:
            if math.hypot(spec.x - struck_x, spec.y - struck_y) < _SIGN_MATCH_DIST_M:
                color_match = spec.color == struck.color
                break

    robot_x, robot_y, robot_yaw = final_pose
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    dx, dy = struck.x - robot_x, struck.y - robot_y
    ahead = dx * cos_yaw + dy * sin_yaw
    lateral = -dx * sin_yaw + dy * cos_yaw
    return masked, ahead, lateral, color_match


_MIN_POLYLINE_VERTICES = 2
"""A polyline needs two vertices before it has a segment to measure against."""

_CLAMP_BIND_EPS_M = 1e-9
"""Tolerance for "``clamp_lateral`` moved the requested lane at all"."""


def _in_corner_zone(x: float, y: float) -> bool:
    """Whether ``(x, y)`` lies in a corner, i.e. outside the inner square on BOTH axes.

    The track's four corridors are the faces of a square annulus, so a point
    inside the span on one axis is on a straight; outside on both puts it in the
    turn. ``corridor_for_position`` cannot answer this -- it folds corners into
    whichever face is nearest and never reports the turn itself.
    """
    return not (TrackDimensions.CORNER_MIN <= x <= TrackDimensions.CORNER_MAX) and not (
        TrackDimensions.CORNER_MIN <= y <= TrackDimensions.CORNER_MAX
    )


def _closest_on_polyline(px: float, py: float, path: list[Any] | None) -> tuple[float, float, float, int] | None:
    """Closest point on ``path`` to ``(px, py)`` as ``(distance, x, y, segment)``.

    Measured against SEGMENTS rather than vertices: waypoint spacing is coarse
    relative to the clearances in play here, so a nearest-vertex distance would
    overstate the gap by most of a segment length and manufacture margin that
    the chassis never actually has.

    ``segment`` is the index of the segment's FIRST vertex, carried so callers
    can place the point along the path rather than only in the plane -- the
    difference between "the path comes within 5 cm of the sign" and "it does so
    HERE, half a metre from where the lane was laid".
    """
    if not path or len(path) < _MIN_POLYLINE_VERTICES:
        return None
    best: tuple[float, float, float, int] = (float("inf"), 0.0, 0.0, 0)
    for index, (a, b) in enumerate(pairwise(path)):
        vx, vy = b.x - a.x, b.y - a.y
        wx, wy = px - a.x, py - a.y
        seg_sq = vx * vx + vy * vy
        t = 0.0 if seg_sq == 0.0 else max(0.0, min(1.0, (wx * vx + wy * vy) / seg_sq))
        dist = math.hypot(wx - t * vx, wy - t * vy)
        if dist < best[0]:
            best = (dist, a.x + t * vx, a.y + t * vy, index)
    return best


def _lane_is_clamped(spec: Any) -> bool | None:
    """True if ``clamp_lateral`` binds on this sign's lane plateau.

    The split that decides whether any LATERAL widening is reachable at all.
    ``sign_lane`` asks for ``sign_lateral + mult * lateral_offset`` and hands
    the result to ``clamp_lateral``; because the clamp bound is fixed
    (``CORNER_MIN - (half_diagonal + wall_margin)``) and WRO signs sit at
    fixed laterals, the resulting lane-to-sign gap is binary rather than a
    distribution -- 18.14 cm where the clamp binds, 27.86 cm where it does
    not, with nothing in between.

    That makes this the decisive attribute for a struck sign. A CLAMPED sign's
    plateau is already saturated, so raising ``lateral_offset`` (yaw-aware or
    otherwise) moves it by exactly zero; only the clamp bound itself is a
    lever there. A FREE sign carries ~8 cm of margin against the ~19.96 cm
    worst-case-yaw requirement, well beyond the 6.55 cm p90 crosstrack, so a
    collision at one cannot be explained by plan geometry.
    """
    corridor = sign_router_module.corridor_for_position(spec.x, spec.y)
    plateau = _lane_plateau_m(spec, corridor)
    if plateau is None:
        return None
    return abs(plateau - _unclamped_lane_offset_m()) > _CLAMP_BIND_EPS_M


def _unclamped_lane_offset_m() -> float:
    """The outward offset ``sign_lane`` asks for, before ``clamp_lateral``."""
    return (
        chassis_half_diagonal_m()
        + TrafficSignSpecs.WIDTH / 2
        + NavigationTuning.load_default().sign_router.SIGN_CLEARANCE_MARGIN_M
    )


def _lane_plateau_m(spec: Any, corridor: Section) -> float | None:
    """The outward sign-to-lane offset this sign's lane should deliver.

    The yardstick the measured offsets are read against, computed per sign
    rather than quoted as the 18.14 / 27.86 cm pair: which of the two applies
    depends on whether ``clamp_lateral`` binds, and that in turn depends on the
    sign's own lateral, so a single literal would be wrong for half the corpus
    and would restate config besides.
    """
    rule = sign_router_module.outward_lateral_axis(corridor, spec.color)
    if rule is None:
        return None
    axis, mult = rule
    sign_lateral = spec.y if axis is sign_router_module.Axis.Y else spec.x
    want = sign_lateral + mult * _unclamped_lane_offset_m()
    return mult * (sign_router_module.clamp_lateral(want, corridor) - sign_lateral)


def _planned_lane_attribution(
    sim: ScenarioSimulator,
    metadata: dict[str, Any],
    collision_xy: tuple[float, float],
    final_pose: Any,
    planned_path: list[Any] | None,
) -> tuple[bool | None, bool | None, float | None, int | None, float | None]:
    """Describe the struck sign's lane: ``(clamped, boundary, gap, corridors, outward)``.

    The struck sign is identified from its TRUE position, deliberately: unlike
    the mask attribution (a question about what the ROUTER believed, hence
    believed-frame), clamped/boundary are questions about the PLANNED lane's
    geometry, which the planner derives from the same true layout the corpus
    defines.
    """
    true_signs = sign_router_module.signs_from_metadata(metadata)
    if not true_signs:
        return None, None, None, None, None
    cx, cy = collision_xy
    hit = min(true_signs, key=lambda s: math.hypot(s.x - cx, s.y - cy))
    # BELIEVED frame on both sides of the gap measurement. `_waypoints` is the
    # navigator's own plan, expressed in the pose estimate it is steering
    # against; `hit` is ground truth. Comparing them directly is the exact
    # frame-mixing error `_sign_mask_attribution` documents and was fixed for
    # -- it would read "the lane never materialised" for a lane that
    # materialised correctly inside the robot's own self-consistent frame.
    hit_bx, hit_by = _into_believed_frame(sim, (hit.x, hit.y), final_pose)
    router = sim.navigator.sign_router
    matches = (
        []
        if router is None
        else [
            (spec, corridor)
            for spec, corridor in router.lane_specs
            if math.hypot(spec.x - hit_bx, spec.y - hit_by) < _SIGN_MATCH_DIST_M
        ]
    )
    corridors = None if router is None else len({corridor for _, corridor in matches})
    closest = _closest_on_polyline(hit_bx, hit_by, planned_path)
    return (
        _lane_is_clamped(hit),
        not _is_middle_sign(hit.x, hit.y),
        None if closest is None else closest[0],
        corridors,
        _outward_pass_offset(hit, (hit_bx, hit_by), closest, matches[0] if matches else None),
    )


def _outward_pass_offset(
    spec: Any,
    believed_xy: tuple[float, float],
    closest: tuple[float, float, float, int] | None,
    matched: tuple[Any, Section] | None,
) -> float | None:
    """Signed lateral offset of the plan from the sign, positive = OUTWARD.

    Separates the two ways a small ``planned_gap_m`` can arise, which the
    unsigned gap cannot tell apart and which need opposite fixes. A lane that
    never materialised leaves the path near the centreline on the sign's
    outward side, so this reads positive but short of the plateau; a lane that
    composed wrongly puts the path on the INWARD side, and this goes negative
    -- the plan crossing to the wrong side of the sign entirely, which no
    amount of extra clearance on the intended side would fix.

    Measured along the corridor's lateral axis (``outward_lateral_axis``'s
    ``mult`` carries the outward direction), so it is directly comparable to
    the 18.14 / 27.86 cm plateaux ``_lane_is_clamped`` splits on.

    The axis has to come from the ROUTER's own settled corridor for the matched
    sign, not from the true position, and the two are not interchangeable here.
    Both the path and the sign estimate live in the believed frame, which under
    the rotational lock is a 90 deg rotation of the true one -- so a true-frame
    corridor names the axis that is LATERAL in truth but ALONG-track in the
    frame the difference is taken in. Since ``closest`` is by construction
    perpendicular to the path, reading its along-track component returns
    approximately zero for any lane whatsoever, well-formed or not. That is a
    measurement that cannot fail, which makes it useless. Falls back to the
    true-derived corridor only when the sign has no lane spec to match, where
    there is no plan to describe anyway.
    """
    if closest is None:
        return None
    if matched is not None:
        matched_spec, corridor = matched
        color = matched_spec.color
        origin_x, origin_y = matched_spec.x, matched_spec.y
    else:
        corridor = sign_router_module.corridor_for_position(spec.x, spec.y)
        color = spec.color
        origin_x, origin_y = believed_xy
    rule = sign_router_module.outward_lateral_axis(corridor, color)
    if rule is None:
        return None
    axis, mult = rule
    _, path_x, path_y, _ = closest
    if axis is sign_router_module.Axis.Y:
        return mult * (path_y - origin_y)
    return mult * (path_x - origin_x)


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
        uturns = _UTurnDetector()
        escapes = _EscapeTracker(sim.navigator)
        # Last tick's tracking state, kept so the COLLISION tick can be
        # described. `sim.run` returns only the final pose, and by then the
        # navigator has stopped, so anything about what the tracker was doing
        # when it hit has to be latched on the way past.
        last_crosstrack: list[float | None] = [None]
        last_phase: list[Any] = [None]
        # The path polyline itself, not a copy: `replace_path`/`apply_sign_lanes`
        # both REBIND `_waypoints` to a fresh list rather than mutating in
        # place, so holding the reference is safe and costs nothing per tick.
        last_path: list[Any] = [None]

        def _on_step(state: AckermannState, _scan: LidarScan) -> None:
            uturns.update(state.x, state.y, state.yaw)
            escapes.update()
            snapshot = sim.navigator.debug_snapshot
            if snapshot.crosstrack_error_m is not None:
                last_crosstrack[0] = snapshot.crosstrack_error_m
            last_phase[0] = getattr(snapshot, "phase", None)
            last_path[0] = sim.navigator._waypoints  # noqa: SLF001 - diagnostic needs the post-lane polyline

        result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=_on_step)
    finally:
        for module, name, value in restore:
            setattr(module, name, value)

    classify_meta = metadata
    if config.ghost_signs:
        classify_meta = _without_signs(classify_meta)
    kind = _classify_collision(classify_meta, result.final_pose) if result.collided else CollisionKind.NONE
    collision_xy = None if result.collision_xy is None else Waypoint(*result.collision_xy)
    sign_masked, sign_ahead_m, sign_lateral_m, sign_color_match = (
        _sign_mask_attribution(sim, metadata, result.collision_xy, result.final_pose)
        if kind == CollisionKind.SIGN and result.collision_xy is not None
        else (None, None, None, None)
    )
    (
        sign_lane_clamped,
        sign_lane_boundary,
        planned_gap_m,
        struck_corridor_count,
        planned_outward_m,
    ) = (
        _planned_lane_attribution(sim, metadata, result.collision_xy, result.final_pose, last_path[0])
        if kind == CollisionKind.SIGN and result.collision_xy is not None
        else (None, None, None, None, None)
    )
    return ScenarioOutcome(
        label=scenario.label,
        collided=result.collided,
        laps=result.laps_completed,
        timed_out=result.timed_out,
        collision_xy=collision_xy,
        collision_kind=kind,
        collision_step=result.steps,
        steps=result.steps,
        uturns=len(uturns.events),
        corner_uturns=uturns.corner_events,
        escape_starts=escapes.starts,
        steps_since_escape=escapes.steps_since_escape,
        sign_masked=sign_masked,
        sign_ahead_m=sign_ahead_m,
        sign_lateral_m=sign_lateral_m,
        sign_color_match=sign_color_match,
        sign_lane_clamped=sign_lane_clamped,
        sign_lane_boundary=sign_lane_boundary,
        collision_crosstrack_m=abs(last_crosstrack[0]) if result.collided and last_crosstrack[0] is not None else None,
        collision_phase=last_phase[0] if result.collided else None,
        planned_gap_m=planned_gap_m,
        struck_corridor_count=struck_corridor_count,
        planned_outward_m=planned_outward_m,
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
        return sum(1 for o in self.outcomes if o.laps >= CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS)

    @property
    def uturns(self) -> int:
        """Total heading reversals across every scenario in this arm."""
        return sum(o.uturns for o in self.outcomes)

    @property
    def corner_uturns(self) -> int:
        """How many of those happened in a corner zone."""
        return sum(o.corner_uturns for o in self.outcomes)

    @property
    def scenarios_with_uturn(self) -> int:
        """Scenarios that reversed at least once — the breadth, against the total's depth."""
        return sum(1 for o in self.outcomes if o.uturns)

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
            if o.laps >= CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS and o.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
        )

    @property
    def timeouts(self) -> int:
        """Scenarios that ran out of step budget."""
        return sum(1 for o in self.outcomes if o.timed_out)

    @property
    def escape_starts(self) -> int:
        """Escape engagements across every scenario in this arm."""
        return sum(o.escape_starts for o in self.outcomes)

    @property
    def escape_linked_collisions(self) -> int:
        """Collisions that happened within ``_ESCAPE_ATTRIBUTION_STEPS`` of an escape.

        Separates "the escape drove into something" from "the escape never got
        a chance": both land in the same collision counter, and they want
        opposite fixes -- one says make the maneuver safer, the other says
        make it fire earlier or at all.
        """
        return sum(
            1
            for o in self.outcomes
            if o.collided and o.steps_since_escape is not None and o.steps_since_escape <= _ESCAPE_ATTRIBUTION_STEPS
        )

    @property
    def laps_driven(self) -> int:
        """Total laps completed across the arm, the denominator for escape rate.

        Escape totals are not comparable across arms without it: an arm that
        survives longer earns more escapes for free.
        """
        return sum(o.laps for o in self.outcomes)

    def kind(self, name: CollisionKind) -> int:
        """Scenarios whose collision was of the given kind (wall/sign/parking)."""
        return sum(1 for o in self.outcomes if o.collision_kind == name)

    @property
    def sign_collisions_masked(self) -> int:
        """Sign collisions where the struck sign was already routed (masked from escape).

        The direct test of the proximity-gated-unmask premise: a collision
        here means the escape trigger's silence is attributable to the mask,
        not to the sign never having been routed at all.
        """
        return sum(1 for o in self.outcomes if o.sign_masked is True)

    @property
    def sign_collisions_unmasked(self) -> int:
        """Sign collisions where the struck sign was never routed at all.

        Not a masking problem -- the router never had this sign, so no mask
        change can fix these; the gap is upstream, in discovery/routing.
        """
        return sum(1 for o in self.outcomes if o.sign_masked is False)

    @property
    def sign_lane_clamp_split(self) -> tuple[int, int, int, int]:
        """Sign collisions as ``(clamped_boundary, clamped_middle, free_boundary, free_middle)``.

        Answers whether any LATERAL widening is reachable. A collision in a
        CLAMPED cell sits on a saturated plateau (18.14 cm gap), so the only
        geometric lever left there is the clamp bound itself; a collision in a
        FREE cell sits at 27.86 cm with ~8 cm of margin, which plan geometry
        cannot explain at all. Compare against the corpus exposure -- boundary
        signs are roughly half clamped, half free -- rather than reading the
        raw counts, or the layout's own skew will look like a result.
        """
        cells = [0, 0, 0, 0]
        for o in self.outcomes:
            if o.sign_lane_clamped is None or o.sign_lane_boundary is None:
                continue
            cells[(0 if o.sign_lane_clamped else 2) + (0 if o.sign_lane_boundary else 1)] += 1
        return cells[0], cells[1], cells[2], cells[3]

    def clamp_population_report(self) -> str:
        """Multi-line breakdown of sign collisions by clamped/free plateau.

        Reports crosstrack and phase within each, because the headline split
        establishes only that ~39% of collisions happen where the plan had
        ~10.6 cm of margin -- it does not say what consumed it. Escape and
        u-turn phases are broken out separately: those are controller
        arbitration driving into a sign, which no amount of tracking accuracy
        or lane geometry addresses.
        """
        lines = []
        for label, want_clamped in (("CLAMPED (18.14cm gap)", True), ("FREE (27.86cm gap)", False)):
            group = [
                o for o in self.outcomes if o.sign_lane_clamped is want_clamped and o.collision_crosstrack_m is not None
            ]
            if not group:
                continue
            xt = sorted(o.collision_crosstrack_m for o in group if o.collision_crosstrack_m is not None)
            phases: dict[str, int] = {}
            for o in group:
                phases[str(o.collision_phase)] = phases.get(str(o.collision_phase), 0) + 1
            escape_linked = sum(
                1 for o in group if o.steps_since_escape is not None and o.steps_since_escape <= _ESCAPE_ATTRIBUTION_STEPS
            )
            top = ", ".join(f"{k}={v}" for k, v in sorted(phases.items(), key=lambda kv: -kv[1])[:4])
            gaps = sorted(o.planned_gap_m for o in group if o.planned_gap_m is not None)
            expected = 0.1814 if want_clamped else 0.2786
            # A lane that never materialised leaves the path on the centreline,
            # ~0.10 m out; anything at or below that is a coverage failure
            # rather than a thin plateau, so count them rather than let the
            # median hide them among correctly-offset passes.
            unlaned = sum(1 for g in gaps if g < expected - 0.03)
            gap_txt = (
                f"planned-gap median={percentile(gaps, 0.5) * 100:5.2f}cm "
                f"(expected {expected * 100:.2f}cm)  below-expected={unlaned:>3}/{len(gaps)}"
                if gaps
                else "planned-gap n/a"
            )
            # As a FRACTION of the same plateau the pass control reports, so
            # the two populations can be read against each other directly --
            # which is the only way either number means anything, since a
            # collision-only reading is circular (the chassis hit the sign, so
            # its plan was necessarily near it). See `_SignPassSample.
            # pass_outward` and the `lane delivered at PASSES` row.
            outs = sorted(o.planned_outward_m for o in group if o.planned_outward_m is not None)
            out_txt = f"lane-delivered median={percentile(outs, 0.5) / expected:+5.2f}x" if outs else "lane-delivered n/a"
            dual = sum(1 for o in group if (o.struck_corridor_count or 0) > 1)
            lines.append(
                f"  {label:<22} n={len(group):>3}  dual-corridor={dual:>3}  "
                f"crosstrack median={percentile(xt, 0.5) * 100:5.2f}cm p90={percentile(xt, 0.9) * 100:5.2f}cm "
                f"max={xt[-1] * 100:5.2f}cm  escape-linked={escape_linked:>3}\n"
                f"  {'':<22} {gap_txt}  {out_txt}  phases: {top}"
            )
        return "\n".join(lines)

    @property
    def sign_collisions_wrong_color(self) -> int:
        """Masked sign collisions where the discovered colour vote was wrong.

        The plan sent the robot to the wrong side of a sign it otherwise knew
        about correctly -- a discovery/voting bug, not an execution-timing one.
        """
        return sum(1 for o in self.outcomes if o.sign_color_match is False)

    @property
    def sign_collisions_right_color(self) -> int:
        """Masked sign collisions where the colour vote was correct but the
        robot still hit it -- consistent with not enough runway to execute
        the avoidance in time, not a wrong plan.
        """
        return sum(1 for o in self.outcomes if o.sign_color_match is True)

    def row(self) -> str:
        """The one-line summary: all four metrics plus the collision-kind split."""
        n = len(self.outcomes)
        cb, cm, fb, fm = self.sign_lane_clamp_split
        return (
            f"RESULT {self.config.label:<{_RESULT_LABEL_WIDTH}} "
            f"collisions {self.collisions:>{_RESULT_METRIC_WIDTH}}/{n} "
            f"(wall {self.kind(CollisionKind.WALL):>{_RESULT_METRIC_WIDTH}} sign {self.kind(CollisionKind.SIGN):>{_RESULT_METRIC_WIDTH}} park {self.kind(CollisionKind.PARKING):>{_RESULT_METRIC_WIDTH}})  "
            f"laps>=1 {self.laps_ge_1:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"laps>=3 {self.laps_ge_3:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"in-time {self.laps_ge_3_in_time:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"timeouts {self.timeouts:>{_RESULT_METRIC_WIDTH}}/{n}  "
            f"uturns {self.uturns:>3} ({self.corner_uturns:>3} at corners, {self.scenarios_with_uturn:>{_RESULT_METRIC_WIDTH}}/{n} runs)  "
            f"escapes {self.escape_starts:>4} ({self._escapes_per_lap:.2f}/lap, "
            f"{self.escape_linked_collisions:>{_RESULT_METRIC_WIDTH}} collisions within {_ESCAPE_ATTRIBUTION_STEPS} ticks)  "
            f"sign-mask (masked {self.sign_collisions_masked:>{_RESULT_METRIC_WIDTH}} unmasked {self.sign_collisions_unmasked:>{_RESULT_METRIC_WIDTH}})  "
            f"masked-color (wrong {self.sign_collisions_wrong_color:>{_RESULT_METRIC_WIDTH}} right {self.sign_collisions_right_color:>{_RESULT_METRIC_WIDTH}})  "
            f"lane-clamp (clamped {cb:>{_RESULT_METRIC_WIDTH}}b/{cm}m free {fb:>{_RESULT_METRIC_WIDTH}}b/{fm}m)"
        )

    @property
    def _escapes_per_lap(self) -> float:
        """Escape engagements per lap actually driven, 0.0 when nothing drove."""
        return self.escape_starts / self.laps_driven if self.laps_driven else 0.0

    def detail(self) -> str:
        """Per-scenario rows, for when an aggregate needs breaking down."""
        return "\n".join(
            f"DETAIL   {o.label:<{_DETAIL_LABEL_WIDTH}} {o.collision_kind:<{_DETAIL_COLLISION_WIDTH}} laps={o.laps} steps={o.steps} "
            f"at={None if o.collision_xy is None else (round(o.collision_xy.x, _COLLISION_PRECISION), round(o.collision_xy.y, _COLLISION_PRECISION))} "
            f"escapes={o.escape_starts} since_escape={o.steps_since_escape} "
            f"sign_masked={o.sign_masked} ahead={_round_or_none(o.sign_ahead_m)} lateral={_round_or_none(o.sign_lateral_m)} "
            f"color_match={o.sign_color_match} "
            f"clamped={o.sign_lane_clamped} corridors={o.struck_corridor_count} "
            f"gap={_round_mm(o.planned_gap_m)} outward={_round_mm(o.planned_outward_m)} "
            f"xt={_round_or_none(o.collision_crosstrack_m)} phase={o.collision_phase}"
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
            population = result.clamp_population_report()
            if population:
                print(population, flush=True)
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

    sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=record)
    return errors


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
                f"median {percentile(pooled, 0.5) * 100:{_CROSSTRACK_PERCENTILE_PRECISION}}cm  "
                f"p90 {percentile(pooled, 0.9) * 100:{_CROSSTRACK_PERCENTILE_PRECISION}}cm  "
                f"max {max(pooled) * 100:{_CROSSTRACK_PERCENTILE_PRECISION}}cm",
                flush=True,
            )


_SIGN_PASS_WINDOW_M = 0.30
"""How close to a routed sign a tick must be to count as "during a pass".

Loose enough to span the whole abeam moment at the sim's step size, tight
enough that the ramp on and off the lane -- where the chassis is SUPPOSED to
be moving laterally -- does not dominate the sample.
"""


_MIDDLE_DEPTH_TOLERANCE_M = 0.25
"""Half-window around depth 1.50 that counts as a MIDDLE sign.

Corpus depths take exactly three values -- 1.00, 1.50, 2.00 -- so anything
short of 0.25 separates them cleanly, and the estimate error these are
classified from is 0.5 cm (see this module's sign-crosstrack results).
"""


def _is_middle_sign(x: float, y: float) -> bool:
    """True if this sign sits mid-section rather than on a section BOUNDARY.

    The split that separates the two candidate causes of pass yaw. 1211 of the
    corpus's 1282 signs sit at a boundary, immediately beside a corner, where
    the chassis may still be rotating out of the turn; a middle sign is the
    only case with a corner-free approach on both sides. If the yaw is
    concentrated at boundaries it is corner-driven, and the lever is corner
    exit; if it is flat across both, the lane ramp itself is doing it and the
    lever is ramp length and approach speed.

    Depth is the along-corridor coordinate: x for the SOUTH/NORTH corridors,
    y for EAST/WEST, matching ``sign_lane._axis_coords``.
    """
    corridor = corridor_for_position(x, y)
    depth = x if corridor in (Section.SOUTH, Section.NORTH) else y
    return abs(depth - 1.5) < _MIDDLE_DEPTH_TOLERANCE_M


def _corridor_yaw_deg(yaw_rad: float) -> float:
    """Chassis angle to the nearest corridor axis, in [0, 45] degrees.

    This is the angle the simulator's collision test actually responds to.
    Signs are axis-aligned 0.05 m boxes and corridors run along the world
    axes, so the chassis half-extent facing a sign is
    ``0.15*|sin| + 0.097*|cos|`` of THIS angle -- folded modulo 90 deg
    because all four corridors are equivalent under the track's own symmetry,
    which also makes it immune to the blind rotational lock.

    Distinct from the path-relative heading below: on a lane ramp the path
    deliberately runs at an angle to the corridor, so a chassis tracking that
    ramp perfectly still presents a widened profile to the sign.
    """
    folded = math.degrees(yaw_rad) % 90.0
    return abs(folded - 90.0 if folded > 45.0 else folded)


def _radial_offset(proj: Any, px: float, py: float) -> float | None:
    """Signed distance from the path, positive when the chassis sits OUTSIDE it.

    "Outward" is taken radially from the track centre rather than from the
    path's left-normal, so it needs no travel direction and cannot be flipped
    by a mis-inferred one -- which matters here because these runs are blind.

    The whole quantity is computed in the BELIEVED frame: the path is the one
    the navigator is tracking and the pose is the one it is tracking with, so
    this is "did the chassis go where it meant to", independent of where that
    frame sits relative to truth. Under the rotational lock the believed frame
    is a 90 deg rotation of the true one, which maps corridors onto corridors,
    so the answer is the same in both.
    """
    centre = TrackDimensions.MAX_COORD / 2
    radial_x, radial_y = proj.x - centre, proj.y - centre
    norm = math.hypot(radial_x, radial_y)
    if norm == 0.0:
        return None
    return ((px - proj.x) * radial_x + (py - proj.y) * radial_y) / norm


@dataclass(frozen=True, slots=True)
class _SignPassSample:
    """One scenario's tracking error, at sign passes and overall."""

    near_abs: list[float]
    """|crosstrack| on ticks within ``_SIGN_PASS_WINDOW_M`` of a routed sign."""

    near_outward: list[float]
    """Signed radial offset on those same ticks, positive = outside the path."""

    all_abs: list[float]
    """|crosstrack| on every driving tick, as the contrast."""

    yaw_boundary: list[float]
    """Corridor-relative yaw on ticks passing a section-BOUNDARY sign."""

    yaw_middle: list[float]
    """Corridor-relative yaw on ticks passing a MID-SECTION sign."""

    head_boundary: list[float]
    """Heading error against the PATH at a boundary sign.

    Read together with ``yaw_boundary``: a large corridor-yaw beside a small
    path-heading error means the chassis is correctly driving a curve, and the
    angle is geometry rather than a tracking fault.
    """

    head_middle: list[float]
    """Heading error against the PATH at a mid-section sign."""

    pass_branches: list[LaneBranch]
    """Which ``apply_sign_lanes`` branch settled each PASSED sign's lane.

    The control for ``struck_branch``. A branch that appears at collisions and
    nowhere else is a cause; one that appears at both in the same proportion is
    just what the planner does, and the difference has to be elsewhere.
    """

    struck_branch: LaneBranch | None
    """The same, for the sign that ended the run. ``None`` if no sign was hit."""

    pass_branch_delivery: list[tuple[LaneBranch, float]]
    """Per PASSED sign, its branch PAIRED with what the plan actually delivered.

    Kept as pairs rather than as the two existing parallel lists because the
    question is a cross-tab, and ``pass_outward`` and ``pass_branches`` cannot be
    zipped: the first holds every sample, the second only those a branch could be
    computed for.

    What it settles: the branch classifier diagnoses the target sign's corridor
    group ALONE, while the plan is the composition of every group over the whole
    path. So ``DELIVERED`` asserts only that this group in isolation would reach
    its plateau -- if those signs then measure short on the actual polyline, the
    difference is cross-group composition, which no branch here names.
    ``CLAMPED_SHIFT`` is the control: its shortfall is predicted by the branch
    itself, so it should read short in BOTH columns.
    """

    struck_delivered: float | None
    """The same delivery fraction, for the sign that ended the run."""

    pass_branch_compose: list[tuple[LaneBranch, float]]
    """Per PASSED sign, its branch paired with its composition gap.

    Paired for the same reason the delivery cross-tab is: the claim is that
    composition is what separates a ``DELIVERED`` pass from a ``DELIVERED``
    collision, and that is a contrast within one branch, not a level.
    """

    struck_compose: float | None
    """The composition gap at the sign that ended the run."""

    pass_approach: list[_Approach]
    """Per PASSED sign, everything the closest approach says about where it sits."""

    struck_approach: _Approach | None
    """The same, for the sign that ended the run."""

    pass_stale: list[float]
    """Per PASSED sign, how far the held plan sits from a fresh rebuild there.

    The control for ``struck_stale``, and needed for the same reason the branch
    columns are paired: some baseline disagreement is expected everywhere,
    because discovery refines estimates continuously and the gate only fires
    when they cross a threshold. Staleness only explains the collisions if it is
    LARGER at them.
    """

    pass_fp_stale: list[bool]
    """Per PASSED sign, whether the fingerprint gate was un-fired at that tick."""

    struck_stale: float | None
    """The same as ``pass_stale``, for the sign that ended the run."""

    struck_fp_stale: bool | None
    """Whether the fingerprint gate was un-fired when the struck sign was passed.

    The discriminator. Read beside ``struck_stale``: a large stale distance with
    the gate un-fired is an ordinary rebuild that had not happened yet, while a
    large one with the gate FIRED means the navigator believes its plan is
    current and it is not -- which at shipped config can only be the
    ``lane_fingerprint`` rounding, since nothing else rewrites ``_waypoints``.
    """

    struck_middle: bool | None
    """Whether the sign that ended the run was mid-section. ``None`` if no sign
    collision. Read against the 1211/71 boundary/middle exposure, not raw."""

    near_yaw_deg: list[float]
    """Chassis angle to the corridor axis on those same ticks.

    The variable crosstrack cannot see: a chassis can sit dead on its lane and
    still be angled across it, and the clamped plan clears a squeezed sign
    only within +/-28.2 deg.
    """

    pass_outward: list[float]
    """Fraction of its own lane plateau the plan delivered at each PASSED sign.

    A ratio rather than a raw offset because the plateau is per-sign: 18.14 cm
    where ``clamp_lateral`` binds and 27.86 cm where it does not, so pooling
    raw centimetres across the corpus would mix two different targets. 1.0 is
    the lane fully delivered; 0.0 is the plan running straight through the
    sign's own lateral; negative is the plan on the side the pass is forbidden
    to use. An untouched centreline is NOT 0 but roughly -0.4 to -0.6, since
    WRO signs sit 10 cm off-centre toward that forbidden side -- so the
    interesting threshold for "the lane did nothing" is negative, and any
    positive reading is some amount of lane actually delivered.

    The control for the collision-side reading of the same quantity, and the
    thing that decides whether that reading means anything. Measuring only at
    collisions is circular: the chassis hit the sign, so of course its plan
    ran close to it. The non-circular claim is a CONTRAST -- the plan is short
    of its 18.14 / 27.86 cm plateau specifically where it fails. If passes read
    at the plateau and collisions read near zero, the lane under-delivers and
    that is the bug; if passes read near zero too, then near zero is simply
    what this planner produces everywhere, the collisions are not distinguished
    by it, and the cause is elsewhere.

    Sampled once per sign at CLOSEST APPROACH rather than per tick, so a sign
    the chassis crawled past cannot outvote one it drove by, and taken from
    ``lane_specs`` (which retains passed signs) rather than
    ``routed_sign_positions`` (which drops them at the moment of interest).
    """

    collided_with_sign: bool

    at_collision: float | None

    yaw_at_collision: float | None
    """Corridor-relative chassis angle on the last tick before contact."""

    estimate_err_m: float | None
    """Distance from the struck sign to the routed position the lane was built
    around, both in the believed frame.

    The lane is planned around the DISCOVERY ESTIMATE, not the sign, and it
    carries only ~3.4 cm of slack at zero yaw. Any lateral estimate error eats
    that directly, and it is invisible to every other measure here: in its own
    frame the chassis tracks its plan perfectly and passes the sign it thinks
    is there. ``_SIGN_MATCH_DIST_M`` tolerates 0.30 m of this before it stops
    calling the collision "correctly routed" at all.
    """
    """|crosstrack| on the last tick before a sign collision ended the run.

    The pooled per-run figures dilute the thing being asked: a run that hits
    one sign on lap 3 still contributes every clean pass that preceded it, so
    a real difference at the moment of contact averages away. This is the
    single tick that actually went wrong.
    """


class LaneBranch(StrEnum):
    """Which decision inside ``apply_sign_lanes`` settled one sign's lane.

    The measured offset says the plan passes through the struck sign's own
    lateral; it cannot say why. Every path from ``apply_sign_lanes``'s inputs to
    that outcome runs through exactly one of these, and they need different
    fixes -- ``NO_SPAN`` is a corridor/waypoint mismatch, ``OFF_PROFILE`` a
    depth-window one, ``CLAMPED_SHIFT`` a bound that was already saturated, and
    ``SHORT_PROFILE`` says the profile itself asked for too little, which points
    at ``base_lateral`` rather than at any of the tunables.

    ``DELIVERED`` is the interesting one to find at a collision: it would mean
    the planner does lay a correct lane and the plan the tracker holds is not
    the one it laid, i.e. staleness rather than geometry.
    """

    MULTI_CORRIDOR = "multi-corridor"
    NO_RULE = "no-rule"
    NO_SPAN = "no-span"
    NO_PROFILE = "no-profile"
    OFF_PROFILE = "off-profile"
    CLAMPED_SHIFT = "clamped-shift"
    SHORT_PROFILE = "short-profile"
    DELIVERED = "delivered"


_LANE_BRANCH_EPS_M = 0.01
"""Slack allowed before a lane counts as short of its plateau, or a clamp as
having bound. Centimetre scale because the plateaux it separates are 18.14 and
27.86 cm and the shortfall being diagnosed is the whole offset, not a trim."""


@dataclass(frozen=True)
class _LaneSample:
    """Everything one sign's closest approach has to say about its lane."""

    gap_m: float
    """Distance from the chassis to the sign at this sample, the record being
    kept: a later tick only replaces this one if it is nearer."""

    delivered_frac: float
    """Fraction of the sign's own plateau the plan delivered here."""

    branch: LaneBranch | None
    """Which ``apply_sign_lanes`` decision settled the rebuilt lane."""

    compose_m: float | None
    """How far the full rebuild sits from what this sign's corridor group alone
    would have built, at the waypoint the branch verdict was reached on.

    Signed by the sign's own outward direction: negative is the composition
    pulling the path back toward the sign. See ``_composition_gap_m``.
    """

    approach: _Approach | None
    """Where the polyline actually runs nearest this sign, relative to the
    waypoint the lane was laid on. See ``_approach_offset``."""

    stale_m: float | None
    """How far the plan the tracker holds sits from the one the planner would
    build RIGHT NOW from the same base path and the same sign estimates,
    measured at the waypoint nearest this sign.

    The quantity that separates the two remaining explanations for a collision
    that classifies ``DELIVERED``. Near zero means the tracker really is holding
    the lane the branch classification describes, and the shortfall is geometry
    the branch already names. Plateau-sized means it is not -- the planner lays
    a correct lane and the tracker drives a different polyline -- and no amount
    of tuning the lane geometry can reach that.

    ``None`` when the plan and the base path differ in length, which happens
    only across a ``replace_path`` and makes an elementwise diff meaningless.
    """

    fingerprint_stale: bool
    """Whether ``lane_fingerprint`` disagrees with the one the navigator last
    rebuilt on.

    Reads the gate directly, and it is what makes ``stale_m`` diagnostic rather
    than merely descriptive. Both post-lane transforms are inert at shipped
    config -- ``SIGN_LANE_COMMIT_AHEAD_M`` is 0.0 so ``_hold_committed_path``
    returns immediately, and ``_apply_path_wall_budget`` sets a controller
    budget without touching ``_waypoints`` -- so a stale plan has nowhere else
    to come from. TRUE says the gate simply has not fired yet on this tick's
    estimates; FALSE beside a large ``stale_m`` says the gate fired, believes
    itself current, and is wrong, which is the cm-rounding signature.
    """


@dataclass(frozen=True)
class _LaneProfile:
    """One corridor group's rebuilt lane, enough of it to ask what it delivers."""

    axis: Any
    """The world axis this corridor treats as lateral."""

    mult: int
    """+1/-1 carrying the TARGET sign's own outward direction, which is not
    necessarily the group's: a corridor holding two opposite-coloured signs
    builds one profile with an S-bend through it."""

    base_lateral: float
    """The corridor centreline the profile is applied as a shift from."""

    profile: list[tuple[float, float]]
    """``(depth, lateral)`` control points, as ``_control_points`` returns them."""

    waypoints: list[Waypoint]
    """Only the waypoints the lane may move -- the ones ``_in_lane_span``
    admitted. Carried rather than re-derived so the delivery half cannot
    disagree with the structural half about which points are in play."""

    indices: list[int]
    """Those same waypoints' positions in the base path, in the same order.

    Needed because the composition diff has to index the FULL rebuild at the
    waypoint the branch verdict was reached on, and a ``Waypoint`` cannot say
    where it came from. Kept parallel to ``waypoints`` rather than replacing it
    so neither the branch nor the diff has to re-derive the other's view.
    """


def _lane_params(tuning: NavigationTuning) -> SignLaneParams:
    """The lane geometry ``CoreNavigator`` builds, rebuilt from the same tuning.

    Restated rather than read off the navigator because it does not keep the
    params it passes -- they are constructed inline at the call to
    ``apply_sign_lanes``. Derived from the live ``NavigationTuning`` on every
    term, so a sweep arm that overrides one of them is diagnosed under the
    value it actually ran with rather than under the shipped default.
    """
    sr = tuning.sign_router
    return SignLaneParams(
        lateral_offset=(chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2 + sr.SIGN_CLEARANCE_MARGIN_M)
        * sr.SIGN_LANE_OFFSET_FRAC,
        ramp_m=sr.SIGN_LANE_RAMP_M,
        hold_m=sr.SIGN_LANE_HOLD_M,
        corner_entry_m=sr.SIGN_LANE_CORNER_ENTRY_M,
    )


def _lane_branch(
    index: int,
    base_waypoints: list[Waypoint],
    specs: list[tuple[Any, Section]],
    params: SignLaneParams,
    fresh: list[Waypoint] | None = None,
) -> _BranchResult:
    """Re-run ``apply_sign_lanes``'s decisions for one sign and name the branch.

    Mirrors the loop rather than instrumenting it, so the production transform
    stays free of diagnostic hooks; every predicate is the real one imported
    from ``sign_lane``, so the only thing restated here is the loop skeleton.

    Deliberately diagnoses the target sign's corridor group ALONE. Signs that
    route into two corridors compose across groups -- an earlier group's shift
    is what the later one reads as its waypoint lateral -- and that interaction
    is already a confirmed mechanism for part of the corpus, so it is reported
    as its own branch instead of being modelled here and folded in twice.
    """
    spec, corridor = specs[index]
    built = _lane_profile_for(spec, corridor, base_waypoints, specs, params)
    if isinstance(built, LaneBranch):
        # A lane that was never built has no single-group prediction for the
        # full build to differ FROM, so composition is not a question here.
        return _BranchResult(branch=built, compose_m=None, waypoint_index=None)
    target = _lane_group_target(spec, corridor, built)
    return _BranchResult(
        branch=_lane_delivery_branch(spec, corridor, built),
        compose_m=_composition_gap_m(spec, corridor, built, fresh),
        waypoint_index=None if target is None else target[0],
    )


def _lane_profile_for(
    spec: Any,
    corridor: Section,
    base_waypoints: list[Waypoint],
    specs: list[tuple[Any, Section]],
    params: SignLaneParams,
) -> LaneBranch | _LaneProfile:
    """Rebuild this corridor group's lane profile, or name the branch that stopped it.

    The half of the classification that asks whether a lane can be BUILT at
    all, kept apart from what it then delivers: the four ways out here are
    structural (the sign is in two groups, has no pass-side rule, or the
    corridor matched no waypoints) and every one of them means no lane exists,
    while the branches on the other side all describe a lane that does.
    """
    if any(c is not corridor and math.hypot(s.x - spec.x, s.y - spec.y) < _SIGN_MATCH_DIST_M for s, c in specs):
        return LaneBranch.MULTI_CORRIDOR

    group = [(s, c) for s, c in specs if c is corridor]
    rule = sign_router_module.outward_lateral_axis(corridor, group[0][0].color)
    target_rule = sign_router_module.outward_lateral_axis(corridor, spec.color)
    if rule is None or target_rule is None:
        return LaneBranch.NO_RULE
    axis, _ = rule

    indices = [i for i, wp in enumerate(base_waypoints) if _in_lane_span(wp, corridor, axis, params.corner_entry_m)]
    if not indices:
        return LaneBranch.NO_SPAN
    straight = [i for i in indices if _in_lane_span(base_waypoints[i], corridor, axis, 0.0)]
    laterals = sorted(_axis_coords(base_waypoints[i], axis)[0] for i in (straight or indices))
    base_lateral = laterals[len(laterals) // 2]

    profile = _control_points(group, corridor, axis, base_lateral, params)
    if not profile:
        return LaneBranch.NO_PROFILE
    return _LaneProfile(
        axis=axis,
        mult=target_rule[1],
        base_lateral=base_lateral,
        profile=profile,
        waypoints=[base_waypoints[i] for i in indices],
        indices=indices,
    )


@dataclass(frozen=True)
class _BranchResult:
    """The branch verdict for one sign, with the readings taken alongside it."""

    branch: LaneBranch
    compose_m: float | None
    waypoint_index: int | None
    """Base-path index of the waypoint the verdict was reached on, so the
    approach measurement can ask how far that sits from where the path actually
    runs nearest the sign. ``None`` when no lane was built."""


_BEND_CROSS_EPS = 1e-9
"""Cross-product magnitude above which a waypoint counts as bending.

Waypoints are rounded to millimetres at generation, so a straight run's cross
product is exactly zero rather than merely small; this only has to clear
floating-point noise.
"""


@dataclass(frozen=True)
class _Approach:
    """Where the polyline runs nearest a sign, relative to the lane's own waypoint.

    Read in the frame the navigator holds its plan in, deliberately. A blind run
    can be rotationally locked, so anything compared against a metadata section
    label carries that lock as a confound; every field here is derived from the
    held plan's own geometry and none of them needs to know which corridor the
    robot thinks it is in.
    """

    gap_m: float
    """Along-path metres from the laned waypoint to the closest approach."""

    in_corner_box: bool
    """``_in_corner_zone``: both coordinates outside the inner square.

    A COORDINATE test, not a turn test -- kept only so the earlier corner
    contrast stays comparable. On the sighted plan it disagrees with ``on_arc``
    on 291 of 1282 signs, and unlike ``on_arc`` it moves with the believed
    corridor width. Read ``on_arc`` instead. See ``report_lane_geometry``.
    """

    on_arc: bool
    """Whether the plan is actually TURNING at the closest point."""

    bend_gap_m: float | None
    """Along-path metres from the laned waypoint to the nearest bend vertex.

    How far the sign sits from a corner as the held plan believes it, which is
    the quantity ``gap_m`` was suspected of being a constant multiple of.
    """

    arc_radius_m: float | None
    """Radius of the arc the closest point sits on, when it sits on one.

    A frame-free readout of the believed corridor width: ``_corner_arc_radius``
    returns ``width/2 - center_bias`` until the ``ARC_RADIUS`` cap binds, so the
    radius inverts to the belief without ever naming a section. Taken from the
    UNLANED base path, whose arc vertices lie exactly on their circle -- the
    lane shift varies with depth and would bend them off it.

    A value ABOVE the ``ARC_RADIUS`` cap is not a corner arc at all: no corner
    can plan one, so the bend is the lap-seam splice, where ``_rotate_to_start``
    joins the partial first segment to the rest of the lap at a shallow angle.
    The report separates those rather than bucketing them as a wide corner.
    """


def _circumradius(a: Any, b: Any, c: Any) -> float | None:
    """Radius of the circle through three points, or ``None`` if collinear."""
    ax, ay = b.x - a.x, b.y - a.y
    bx, by = c.x - b.x, c.y - b.y
    cross = ax * by - ay * bx
    if abs(cross) < _BEND_CROSS_EPS:
        return None
    return (
        math.hypot(ax, ay)
        * math.hypot(bx, by)
        * math.hypot(c.x - a.x, c.y - a.y)
        / (2 * abs(cross))
    )


def _bend_flags(path: list[Any]) -> list[bool]:
    """Per vertex, whether the polyline changes direction there.

    Waypoints are rounded to millimetres at generation, so a straight run's
    cross product is exactly zero and this only has to clear float noise.
    """
    bend = [False] * len(path)
    for i in range(1, len(path) - 1):
        ax, ay = path[i].x - path[i - 1].x, path[i].y - path[i - 1].y
        bx, by = path[i + 1].x - path[i].x, path[i + 1].y - path[i].y
        bend[i] = abs(ax * by - ay * bx) > _BEND_CROSS_EPS
    return bend


def _path_station_m(plan: list[Any], index: int) -> float:
    """Arc length along ``plan`` from its start to vertex ``index``.

    Open polyline, no closing segment, matching ``_closest_on_polyline`` so the
    two stations are on one ruler. The lap seam is therefore not traversable
    here, which is harmless for a sign mid-corridor and would only matter for
    one sitting on the seam itself.
    """
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in pairwise(plan[: index + 1]))


def _path_stations_m(plan: list[Any]) -> list[float]:
    """``_path_station_m`` for every vertex at once, on the same ruler."""
    stations = [0.0]
    for a, b in pairwise(plan):
        stations.append(stations[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return stations


def _approach_offset(
    plan: list[Any],
    waypoint_index: int | None,
    closest: tuple[float, float, float, int] | None,
    base: list[Any] | None = None,
) -> _Approach | None:
    """How far along the path the closest approach sits from the laned waypoint.

    The measurement the last three refuted hypotheses were missing. Branch,
    composition and staleness all evaluate the lane AT A WAYPOINT and agree it
    is at full plateau there; the delivered offset evaluates the POLYLINE and
    reads zero. Both hold if the path runs nearest the sign somewhere other than
    that waypoint, and this is the distance between the two.

    Near zero means the two agree and the contradiction is elsewhere after all.
    A large value in a turn means the lane is laid on the straight while the
    unlaned arc is what grazes the sign.

    ``base`` is the UNLANED path the transform ran on. Given it, the reading
    also carries the turn test and the arc radius, which is what decomposes the
    gap against the width belief -- see ``_Approach``. Without it the two
    structural fields come back ``None`` and only the gap is measured.
    """
    if closest is None or waypoint_index is None or not plan or waypoint_index >= len(plan):
        return None
    _, hit_x, hit_y, segment = closest
    if segment >= len(plan):
        return None
    # Cumulative once rather than `_path_station_m` per lookup: the bend scan
    # below asks for a station per bend vertex, and this runs inside the sweep's
    # per-tick loop.
    stations = _path_stations_m(plan)
    station_hit = stations[segment] + math.hypot(hit_x - plan[segment].x, hit_y - plan[segment].y)
    gap = abs(station_hit - stations[waypoint_index])

    on_arc = False
    bend_gap: float | None = None
    radius: float | None = None
    # The 1:1 length guard `_lane_staleness_m` needs for the same reason: across
    # a `replace_path` the two lists index different paths, and every structural
    # reading below is an index lookup into `base` keyed off `plan`.
    if base is not None and len(base) == len(plan) and segment + 1 < len(base):
        bend = _bend_flags(base)
        on_arc = bend[segment] and bend[segment + 1]
        here = stations[waypoint_index]
        bend_gaps = [abs(stations[i] - here) for i, flag in enumerate(bend) if flag]
        bend_gap = min(bend_gaps) if bend_gaps else None
        if on_arc and 0 < segment < len(base) - 1:
            radius = _circumradius(base[segment - 1], base[segment], base[segment + 1])
    return _Approach(
        gap_m=gap,
        in_corner_box=_in_corner_zone(hit_x, hit_y),
        on_arc=on_arc,
        bend_gap_m=bend_gap,
        arc_radius_m=radius,
    )


def _lane_group_target(
    spec: Any,
    corridor: Section,
    built: _LaneProfile,
) -> tuple[int, float, float] | None:
    """Where this corridor group ALONE would put the path at ``spec``.

    Returns ``(base-path index, unclamped lateral, clamped lateral)``, or
    ``None`` if the sign's depth falls outside the group's own profile.

    Factored out so the branch verdict and the composition diff are forced
    through the same waypoint and the same arithmetic. They are two readings of
    one prediction -- what this group would build if it were the only one -- and
    letting each pick its own nearest waypoint would let them disagree about
    which point the lane is even expressed at.
    """
    _, sign_depth = _axis_coords(Waypoint(spec.x, spec.y), built.axis)
    lane = _interpolate(built.profile, sign_depth)
    if lane is None:
        return None
    # The waypoint the plateau actually lands on, not the sign's own depth: the
    # lane is only ever expressed at waypoints, so a profile that is correct
    # between two of them still delivers whatever the nearer one got.
    position = min(
        range(len(built.waypoints)),
        key=lambda i: abs(_axis_coords(built.waypoints[i], built.axis)[1] - sign_depth),
    )
    lateral, _ = _axis_coords(built.waypoints[position], built.axis)
    want = lateral + (lane - built.base_lateral)
    return built.indices[position], want, sign_router_module.clamp_lateral(want, corridor)


def _composition_gap_m(
    spec: Any,
    corridor: Section,
    built: _LaneProfile,
    fresh: list[Waypoint] | None,
) -> float | None:
    """How far the FULL rebuild sits from what this group alone would have built.

    The quantity that decides whether ``DELIVERED`` collisions are explained.
    ``_lane_profile_for`` diagnoses the target sign's corridor group in
    isolation, but ``apply_sign_lanes`` composes every group across the whole
    path -- and where two corridors' spans overlap, an earlier group's shift is
    what the later one reads as its waypoint lateral. So a sign can be
    ``DELIVERED`` on its own group's arithmetic and carry no lane at all on the
    polyline that results.

    Non-zero here IS that composition, measured rather than argued: the same
    waypoint, the same axis, single-group prediction against full-build reality.
    Zero would mean the two agree and the collapse at collisions is something
    neither the branch nor composition explains.

    Signed by the sign's own outward direction, so positive is the composition
    pushing the path further out than the group asked and negative is it pulling
    the path back toward -- and through -- the sign. Only the negative direction
    can cause a collision, which a magnitude would hide.
    """
    target = _lane_group_target(spec, corridor, built)
    if target is None or fresh is None:
        return None
    position, _, shifted = target
    if position >= len(fresh):
        return None
    actual, _ = _axis_coords(fresh[position], built.axis)
    return built.mult * (actual - shifted)


def _lane_delivery_branch(
    spec: Any,
    corridor: Section,
    built: _LaneProfile,
) -> LaneBranch:
    """Name what a successfully-built lane actually delivers at ``spec``.

    Split from the structural half so that "no lane" and "a lane that falls
    short" stay separate questions; they have nothing in common but their
    symptom, and only the second one is about geometry the tunables can move.
    """
    target = _lane_group_target(spec, corridor, built)
    if target is None:
        return LaneBranch.OFF_PROFILE
    _, want, shifted = target
    sign_lateral, _ = _axis_coords(Waypoint(spec.x, spec.y), built.axis)
    plateau = _lane_plateau_m(spec, corridor)
    if plateau and built.mult * (shifted - sign_lateral) >= plateau - _LANE_BRANCH_EPS_M:
        return LaneBranch.DELIVERED
    if abs(shifted - want) > _LANE_BRANCH_EPS_M:
        return LaneBranch.CLAMPED_SHIFT
    return LaneBranch.SHORT_PROFILE


def _lane_staleness_m(
    spec: Any,
    plan: list[Any],
    fresh: list[Waypoint] | None,
) -> float | None:
    """Distance between the held plan and a fresh rebuild, at this sign's waypoint.

    Local rather than a whole-path maximum on purpose. A rebuild that moved some
    far corner has nothing to do with why this sign was hit; the only waypoint
    that can put the chassis into it is the one the pass happens at. Taken as a
    plane distance rather than along the lane axis so it stays frame-free -- any
    disagreement at all shows up, including one on an axis the lane never
    intended to move.
    """
    if fresh is None or len(fresh) != len(plan):
        return None
    index = min(range(len(plan)), key=lambda i: math.hypot(plan[i].x - spec.x, plan[i].y - spec.y))
    return math.hypot(plan[index].x - fresh[index].x, plan[index].y - fresh[index].y)


def _sample_lane_delivery(
    navigator: Any,
    pose_xy: tuple[float, float],
    plan: list[Any],
    per_sign: dict[int, _LaneSample],
    lane_base: list[Waypoint] | None = None,
    params: SignLaneParams | None = None,
) -> None:
    """Record each sign's lane-delivery fraction at its CLOSEST APPROACH.

    Keeps a running best per sign rather than appending every tick: a sign the
    chassis crawled past would otherwise contribute far more samples than one
    it drove by, weighting the corpus figure by speed instead of by sign. The
    closest approach is also where the plateau is supposed to be flat, so it is
    the one tick where "did the lane deliver" has a clean answer, away from the
    ramps on either side.
    """
    router = navigator.sign_router
    specs = router.lane_specs
    # Built at most once per tick and only if some sign actually samples, since
    # it is a whole-path transform and this runs inside the sweep's hot loop.
    fresh: list[Waypoint] | None = None
    rebuilt = False
    for index, (spec, corridor) in enumerate(specs):
        gap = math.hypot(spec.x - pose_xy[0], spec.y - pose_xy[1])
        previous = per_sign.get(index)
        if gap > _SIGN_PASS_WINDOW_M or (previous is not None and gap >= previous.gap_m):
            continue
        closest = _closest_on_polyline(spec.x, spec.y, plan)
        outward = _outward_pass_offset(spec, (spec.x, spec.y), closest, (spec, corridor))
        plateau = _lane_plateau_m(spec, corridor)
        if outward is None or not plateau:
            continue
        # Sampled at the same tick as the offset, not once at the end: the
        # branch is a function of the sign estimates and the base path as
        # they stood when this pass happened, and blind discovery moves
        # both. Reading it later would diagnose a lane the chassis never
        # drove.
        branch: LaneBranch | None = None
        compose: float | None = None
        approach: _Approach | None = None
        if lane_base is not None and params is not None:
            # Built BEFORE the branch call, not after: the composition diff
            # inside it reads this exact rebuild, and the caching only exists to
            # keep a whole-path transform out of the per-sign loop.
            if not rebuilt:
                fresh, rebuilt = apply_sign_lanes(lane_base, specs, params), True
            verdict = _lane_branch(index, lane_base, specs, params, fresh)
            branch, compose = verdict.branch, verdict.compose_m
            approach = _approach_offset(plan, verdict.waypoint_index, closest, lane_base)
        per_sign[index] = _LaneSample(
            gap_m=gap,
            delivered_frac=outward / plateau,
            branch=branch,
            compose_m=compose,
            approach=approach,
            stale_m=_lane_staleness_m(spec, plan, fresh),
            fingerprint_stale=router.lane_fingerprint != navigator._lane_fingerprint,  # noqa: SLF001
        )


def _sign_pass_crosstrack(args: tuple[int, SweepConfig]) -> _SignPassSample:
    """Measure tracking error DURING sign passes, blind, under ``config``.

    Distinct from ``_cross_track_errors`` above, which deliberately strips the
    signs: that answers "how well does the chassis hold a clean line", and it
    has to strip them because under the carrot-deformation regime the router
    biases the target away from the path on purpose, so distance-to-path
    measures intended avoidance rather than error.

    Under ``SIGN_LANE_PLANNER`` that is no longer true -- the lane IS the
    planned polyline -- so with signs present, distance to the navigator's own
    current path is honest tracking error, and it can finally be measured at
    the moment it matters instead of inferred from a sign-free run.
    """
    index, config = args
    scenario = _scenarios(config)[index]
    restore = _apply_patches(config, scenario.metadata)
    near_abs: list[float] = []
    near_outward: list[float] = []
    near_yaw: list[float] = []
    yaw_boundary: list[float] = []
    yaw_middle: list[float] = []
    head_boundary: list[float] = []
    head_middle: list[float] = []
    all_abs: list[float] = []
    last: list[float | None] = [None]
    last_yaw: list[float | None] = [None]
    # Keyed by the sign's index in `lane_specs`, which is append-only and
    # survives `reset_for_new_lap`, so it identifies the same physical sign for
    # the whole run. Value is (distance at closest approach, offset there,
    # which branch of `apply_sign_lanes` produced that offset).
    per_sign: dict[int, _LaneSample] = {}

    try:
        sim = ScenarioSimulator(
            scenario.metadata,
            num_laps=scenario.laps,
            seed=scenario.seed,
            tuning=config.tuning(),
            blind=config.blind,
            park=config.park,
        )
        # Hoisted out of the callback: the params are fixed for the run, and
        # rebuilding them per tick would put an allocation in the hot loop of a
        # sweep that already costs an hour per arm on the corpus.
        lane_params = _lane_params(sim.navigator._tuning)  # noqa: SLF001

        def _on_step(state: AckermannState, _scan: LidarScan) -> None:
            # The controller's OWN error signal, not a re-derivation of it --
            # None whenever the navigator is not in a tracking phase at all
            # (escape, creep), which is exactly when it has no path to be off.
            crosstrack = sim.navigator.debug_snapshot.crosstrack_error_m
            if crosstrack is None:
                return
            all_abs.append(crosstrack)
            last[0] = crosstrack
            # TRUE yaw, not the believed one: the collision is resolved against
            # the real sign box. Under the rotational lock the two differ by a
            # multiple of 90 deg, which _corridor_yaw_deg folds away anyway --
            # taking truth just removes the assumption that the lock is exact.
            last_yaw[0] = _corridor_yaw_deg(state.yaw)

            router = sim.navigator.sign_router
            pose = sim.gateway.get_current_pose()
            if router is None or pose is None:
                return

            # Ahead of the `routed` guard below on purpose. `routed` drops a
            # sign the moment it is marked passed, which is at or just after
            # the closest approach this wants to sample -- gating on it would
            # systematically lose the pass samples nearest the sign, which are
            # the only ones the comparison is about.
            plan = sim.navigator._waypoints  # noqa: SLF001
            # The centreline the lane is laid over, and the geometry it is laid
            # with. Both are reached for the same way the plan is: neither has
            # a public seam, and re-deriving the base path here would diagnose a
            # different path from the one the transform actually ran on.
            _sample_lane_delivery(
                sim.navigator,
                (pose.x, pose.y),
                plan,
                per_sign,
                sim.navigator._lane_base_waypoints,  # noqa: SLF001
                lane_params,
            )

            routed = router.routed_sign_positions
            if not routed:
                return
            nearest = min(routed, key=lambda p: math.hypot(p[0] - pose.x, p[1] - pose.y))
            if math.hypot(nearest[0] - pose.x, nearest[1] - pose.y) > _SIGN_PASS_WINDOW_M:
                return

            near_abs.append(crosstrack)
            yaw_deg = _corridor_yaw_deg(state.yaw)
            near_yaw.append(yaw_deg)
            # Classified from the BELIEVED position, which is the frame this
            # whole callback works in -- and at 0.5 cm of estimate error the
            # depth it lands on is the true one anyway.
            (yaw_middle if _is_middle_sign(*nearest) else yaw_boundary).append(yaw_deg)
            # Reaching past the public snapshot for the path (see `plan`
            # above): it carries the magnitude but not the projection, and both
            # remaining questions need the projection -- which side of the path
            # the chassis sits on, and whether its heading agrees with the
            # path's own.
            if len(plan) < 2:
                return
            proj = project_onto_path(plan, pose.x, pose.y)
            outward = _radial_offset(proj, pose.x, pose.y)
            if outward is not None:
                near_outward.append(outward)

            # Heading error against the PATH, not the corridor. The pair that
            # decides the fix: a chassis angled to the corridor but aligned
            # with its own path is correctly driving a curve, and no amount of
            # slowing down or extra steering authority will straighten it --
            # only asking for more clearance where it is known to be angled.
            # A chassis angled to its own path is lagging the turn, which
            # speed and steering rate CAN fix.
            heading = abs(math.degrees(wrap_angle(pose.yaw - proj.tangent_rad)))
            (head_middle if _is_middle_sign(*nearest) else head_boundary).append(heading)

        result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=_on_step)
    finally:
        for module, name, value in restore:
            setattr(module, name, value)

    struck_sign = result.collided and _classify_collision(scenario.metadata, result.final_pose) == CollisionKind.SIGN
    estimate_err: float | None = None
    struck_middle: bool | None = None
    struck_branch: LaneBranch | None = None
    struck_delivered: float | None = None
    struck_compose: float | None = None
    struck_approach: _Approach | None = None
    struck_stale: float | None = None
    struck_fp_stale: bool | None = None
    if struck_sign and result.collision_xy is not None:
        signs = sign_router_module.signs_from_metadata(scenario.metadata)
        router = sim.navigator.sign_router
        routed = router.routed_sign_positions if router is not None else []
        if signs and router is not None:
            cx, cy = result.collision_xy
            struck = min(signs, key=lambda s: math.hypot(s.x - cx, s.y - cy))
            # TRUE position here: the struck sign's own depth is a fact about
            # the layout, so take it from metadata rather than an estimate.
            struck_middle = _is_middle_sign(struck.x, struck.y)
            bx, by = _into_believed_frame(sim, (struck.x, struck.y), result.final_pose)
            if routed:
                estimate_err = min(math.hypot(rx - bx, ry - by) for rx, ry in routed)
            # The sign that ended the run is not a pass, and it is the one the
            # control exists to be compared AGAINST -- leaving it in would
            # contaminate the control with the very population it contrasts.
            # Its own reading is reported separately, by the sweep's
            # `planned_outward_m`.
            specs = router.lane_specs
            hits = [
                i for i, (s, _) in enumerate(specs) if math.hypot(s.x - bx, s.y - by) < _SIGN_MATCH_DIST_M
            ]
            # Lifted before the pop, not after: this is the whole point of the
            # split. The struck sign's branch is the one being explained, and
            # the pass population is the control it is read against.
            for i in hits:
                sample = per_sign.pop(i, None)
                if sample is None or struck_branch is not None:
                    continue
                if sample.branch is not None:
                    struck_branch = sample.branch
                    struck_delivered = sample.delivered_frac
                    struck_compose = sample.compose_m
                    struck_approach = sample.approach
                    struck_stale = sample.stale_m
                    struck_fp_stale = sample.fingerprint_stale

    return _SignPassSample(
        near_abs=near_abs,
        near_outward=near_outward,
        all_abs=all_abs,
        near_yaw_deg=near_yaw,
        pass_outward=[s.delivered_frac for s in per_sign.values()],
        pass_branches=[s.branch for s in per_sign.values() if s.branch is not None],
        pass_branch_delivery=[(s.branch, s.delivered_frac) for s in per_sign.values() if s.branch is not None],
        struck_delivered=struck_delivered,
        pass_branch_compose=[
            (s.branch, s.compose_m) for s in per_sign.values() if s.branch is not None and s.compose_m is not None
        ],
        struck_compose=struck_compose,
        pass_approach=[s.approach for s in per_sign.values() if s.approach is not None],
        struck_approach=struck_approach,
        pass_stale=[s.stale_m for s in per_sign.values() if s.stale_m is not None],
        pass_fp_stale=[s.fingerprint_stale for s in per_sign.values() if s.branch is not None],
        struck_branch=struck_branch,
        struck_stale=struck_stale,
        struck_fp_stale=struck_fp_stale,
        yaw_boundary=yaw_boundary,
        yaw_middle=yaw_middle,
        head_boundary=head_boundary,
        head_middle=head_middle,
        struck_middle=struck_middle,
        collided_with_sign=struck_sign,
        at_collision=last[0] if struck_sign else None,
        yaw_at_collision=last_yaw[0] if struck_sign else None,
        estimate_err_m=estimate_err,
    )


def _report_lane_branches(samples: list[_SignPassSample]) -> None:
    """Print which ``apply_sign_lanes`` branch produced each lane, passes vs collisions.

    Two columns rather than one, and that is the whole design. The offset
    measurements establish THAT the lane is missing at collisions; a
    collision-only branch tally would then be circular in the same way the
    original offset measurement was, because whichever branch is commonest
    overall will also be commonest among the failures. Only the CONTRAST
    between the columns identifies a cause: a branch that dominates both
    equally is just what the planner does.
    """
    pass_branches = [b for s in samples for b in s.pass_branches]
    struck_branches = [s.struck_branch for s in samples if s.struck_branch is not None]
    if not pass_branches and not struck_branches:
        return
    for branch in LaneBranch:
        n_pass = sum(1 for b in pass_branches if b is branch)
        n_hit = sum(1 for b in struck_branches if b is branch)
        if not n_pass and not n_hit:
            continue
        share_pass = n_pass / len(pass_branches) * 100 if pass_branches else 0.0
        share_hit = n_hit / len(struck_branches) * 100 if struck_branches else 0.0
        print(
            f"LANE-BRANCH {branch:<16} "
            f"passes {n_pass:>5} ({share_pass:5.1f}%)   "
            f"collisions {n_hit:>4} ({share_hit:5.1f}%)",
            flush=True,
        )
    print(
        f"LANE-BRANCH {'TOTAL':<16} passes {len(pass_branches):>5}            "
        f"collisions {len(struck_branches):>4}",
        flush=True,
    )
    _report_delivery_by_branch(samples)
    _report_composition(samples)
    _report_approach(samples)
    _report_lane_staleness(samples)


def _report_approach(samples: list[_SignPassSample]) -> None:
    """Print how far the closest approach sits from the waypoint the lane was laid on.

    Tests the one reading that reconciles the contradiction. Branch, composition
    and staleness all evaluate the lane at a WAYPOINT and agree it is at full
    plateau; the delivered offset evaluates the POLYLINE and reads zero. If the
    path runs nearest the sign well away from that waypoint, both are true and
    the lane is simply being laid in the wrong place.

    ``in corner`` is the mechanism test on top of that: the lane spans only
    ``SIGN_LANE_CORNER_ENTRY_M`` into a turn, so if closest approach happens in
    the corner the offset was never applied where it was needed. Paired columns,
    since a large gap at passes too would make it normal geometry rather than a
    fault.
    """
    passes = [a for s in samples for a in s.pass_approach]
    struck = [s.struck_approach for s in samples if s.struck_approach is not None]
    if not passes and not struck:
        return
    columns = (("passes", passes), ("collisions", struck))
    for label, group in columns:
        if not group:
            continue
        gaps = [a.gap_m for a in group]
        print(
            f"LANE-APPROACH {label:<11} n={len(gaps):>5}  "
            f"median {percentile(gaps, 0.5) * 100:6.2f}cm  "
            f"p90 {percentile(gaps, 0.9) * 100:6.2f}cm  "
            f"max {max(gaps) * 100:6.2f}cm",
            flush=True,
        )
    for label, group in columns:
        if not group:
            continue
        n_box = sum(1 for a in group if a.in_corner_box)
        n_arc = sum(1 for a in group if a.on_arc)
        print(
            f"LANE-APPROACH {label:<11} in corner BOX {n_box:>5}/{len(group)} "
            f"({n_box / len(group) * 100:5.1f}%)   ON ARC {n_arc:>5}/{len(group)} "
            f"({n_arc / len(group) * 100:5.1f}%)",
            flush=True,
        )
    _report_approach_decomposition(columns)


def _report_approach_decomposition(columns: tuple[tuple[str, list[_Approach]], ...]) -> None:
    """Split the approach gap against the two things that can set it.

    The gap is NOT a planner constant -- on the sighted plan, with true widths
    and true sign positions, it reads 2.22 cm median across all 1282 corpus
    signs and never clusters at 21 (see ``report_lane_geometry``). So the tight
    collision band has to be produced at run time, and there are only two
    candidates in a blind run: where the sign sits relative to a corner as the
    held plan believes it, and how wide the plan believes the corridor is.

    ``bend gap`` is the first: along-path distance from the laned waypoint to
    the nearest turn. If the 21 cm band is "the closest approach is one bend
    away", it shows up here and the pass column will not share it.

    ``arc radius`` is the second, and it is the width belief without the frame
    risk of reading a section label under a rotational lock:
    ``_corner_arc_radius`` returns ``width/2 - center_bias`` until the
    ``ARC_RADIUS`` cap binds, so bucketing by radius buckets by belief. A band
    that survives inside every radius bucket is not the width; one that
    disappears is.
    """
    for label, group in columns:
        bend_gaps = [a.bend_gap_m for a in group if a.bend_gap_m is not None]
        if bend_gaps:
            print(
                f"LANE-APPROACH {label:<11} bend gap  n={len(bend_gaps):>5}  "
                f"median {percentile(bend_gaps, 0.5) * 100:6.2f}cm  "
                f"p90 {percentile(bend_gaps, 0.9) * 100:6.2f}cm",
                flush=True,
            )
    cap = NavigationTuning.load_default().waypoints.ARC_RADIUS
    buckets: dict[float, dict[str, list[float]]] = {}
    seam = 0
    for label, group in columns:
        for approach in group:
            if approach.arc_radius_m is None:
                continue
            if approach.arc_radius_m > cap + _LANE_BRANCH_EPS_M:
                seam += 1
                continue
            buckets.setdefault(round(approach.arc_radius_m, 2), {}).setdefault(label, []).append(approach.gap_m)
    if seam:
        print(
            f"LANE-APPROACH dropped {seam} reading(s) whose bend radius exceeds the "
            f"{cap:.2f}m ARC_RADIUS cap -- the lap-seam splice, not a corner",
            flush=True,
        )
    for radius in sorted(buckets):
        parts = []
        for label, _ in columns:
            gaps = buckets[radius].get(label)
            parts.append(
                f"{label} n={len(gaps):>4} median {percentile(gaps, 0.5) * 100:6.2f}cm"
                if gaps
                else f"{label} n=   0"
            )
        print(f"LANE-APPROACH arc r={radius:.2f}m  " + "   ".join(parts), flush=True)


def report_lane_geometry(scenarios_dir: str | None, width_errors: list[float]) -> None:
    """Census the same approach gap on the PLANNED path alone -- no simulation.

    ``_report_approach`` measures the gap during a blind run, where the plan is
    built from a believed corridor width and the sign specs are discovery
    estimates. This strips both: ground-truth widths, ground-truth sign
    positions, one canonical lap, ``apply_sign_lanes`` applied once. Whatever
    survives here is planner geometry; whatever does not is produced at run time
    by belief, and no amount of reading the planner will find it.

    ``width_errors`` shifts every corridor's believed width by the given
    metres before planning, leaving the signs where they truly are. That is the
    one belief the geometry is most sensitive to: the width sets the centreline
    AND the corner arc radius (``_corner_arc_radius``, ``w/2 - bias`` until the
    ``ARC_RADIUS`` cap binds), so it decides where the waypoint grid falls
    relative to a sign's depth.

    Reports ``on arc`` next to ``in corner box``. They are not the same test --
    ``_in_corner_zone`` asks whether both coordinates are outside the inner
    square, which is a coordinate box, while ``on arc`` asks whether the
    polyline is actually turning at the closest point. The box count moves with
    the believed width while the turn count does not.
    """
    directory = Path(scenarios_dir) if scenarios_dir else None
    paths = sorted((directory or CORPUS_DIR).glob("*_metadata.json"))
    tuning = NavigationTuning.load_default()
    sr = tuning.sign_router
    params = SignLaneParams(
        lateral_offset=chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2 + sr.SIGN_CLEARANCE_MARGIN_M,
        ramp_m=sr.SIGN_LANE_RAMP_M,
        hold_m=sr.SIGN_LANE_HOLD_M,
        corner_entry_m=sr.SIGN_LANE_CORNER_ENTRY_M,
    )
    for width_error in width_errors or [0.0]:
        gaps: list[float] = []
        in_box: list[bool] = []
        on_arc: list[bool] = []
        for path in paths:
            meta = ScenarioMetadata.model_validate(json.loads(path.read_text()))
            if not meta.sign_positions:
                continue
            gaps_here = _lane_geometry_for(meta, params, tuning, width_error)
            for gap, boxed, arced in gaps_here:
                gaps.append(gap)
                in_box.append(boxed)
                on_arc.append(arced)
        if not gaps:
            continue
        n_box, n_arc = sum(in_box), sum(on_arc)
        print(
            f"LANE-GEOMETRY werr {width_error:+.2f}m  n={len(gaps):>5}  "
            f"median {percentile(gaps, 0.5) * 100:6.2f}cm  "
            f"p90 {percentile(gaps, 0.9) * 100:6.2f}cm  "
            f"max {max(gaps) * 100:6.2f}cm  "
            f"in corner box {n_box / len(gaps) * 100:5.1f}%  "
            f"on arc {n_arc / len(gaps) * 100:5.1f}%",
            flush=True,
        )


def _lane_geometry_for(
    meta: ScenarioMetadata,
    params: SignLaneParams,
    tuning: NavigationTuning,
    width_error: float,
) -> list[tuple[float, bool, bool]]:
    """Per sign: ``(approach gap m, closest point in corner box, on an arc)``."""
    plan_meta = meta if not width_error else _with_width_error(meta, width_error)
    try:
        base = calculate_waypoints(
            plan_meta,
            num_laps=1,
            tuning=tuning,
            center_bias_m=tuning.waypoints.OBSTACLES_CENTER_BIAS_M,
        )
    except ValueError:
        return []
    # A segment is on an arc when BOTH its end vertices bend. Either-end would
    # also catch the last straight segment, whose far end is the arc entry.
    bend = _bend_flags(base)
    arc_seg = [bend[i] and bend[i + 1] for i in range(len(base) - 1)]

    specs = [
        (SignSpec(x=s.x, y=s.y, color=s.color), corridor_for_position(s.x, s.y))
        for s in meta.sign_positions
    ]
    planned = apply_sign_lanes(base, specs, params)
    out: list[tuple[float, bool, bool]] = []
    for spec, corridor in specs:
        rule = sign_router_module.outward_lateral_axis(corridor, spec.color)
        closest = _closest_on_polyline(spec.x, spec.y, planned)
        if rule is None or closest is None:
            continue
        axis, _ = rule
        admitted = [
            i for i, wp in enumerate(base) if _in_lane_span(wp, corridor, axis, params.corner_entry_m)
        ]
        if not admitted:
            continue
        _, sign_depth = _axis_coords(Waypoint(spec.x, spec.y), axis)
        index = min(admitted, key=lambda i: abs(_axis_coords(base[i], axis)[1] - sign_depth))
        _, hit_x, hit_y, segment = closest
        station = _path_station_m(planned, segment) + math.hypot(
            hit_x - planned[segment].x, hit_y - planned[segment].y
        )
        out.append(
            (
                abs(station - _path_station_m(planned, index)),
                _in_corner_zone(hit_x, hit_y),
                arc_seg[segment],
            ),
        )
    return out


def _with_width_error(meta: ScenarioMetadata, width_error: float) -> ScenarioMetadata:
    """``meta`` with every corridor width shifted, signs left where they are."""
    widths = meta.corridor_widths
    believed = {
        side: max(RobotSpecs.WIDTH, getattr(widths, side).width_mm / 1000.0 + width_error)
        for side in ("north", "south", "east", "west")
    }
    return meta.replanned_with(
        corridor_widths=CorridorWidths(
            **{side: CorridorWidthEntry(width_mm=round(w * 1000)) for side, w in believed.items()},
        ),
        starting_conditions=meta.starting_conditions,
    )


def _report_composition(samples: list[_SignPassSample]) -> None:
    """Print how far the full rebuild diverges from single-group arithmetic.

    The test of the one explanation left standing for the ``DELIVERED``
    collisions. Staleness is refuted and the branch verdict is computed on the
    target sign's corridor group alone, so if those signs carry no lane on the
    real polyline, the composition of the other groups is what removed it.

    Negative is the composition pulling the path back toward the sign, which is
    the only direction that can cause contact. Reported in centimetres against
    the 18.14 / 27.86 cm plateaux, split by branch, passes against collisions --
    the claim being tested is a contrast WITHIN ``DELIVERED``, so a gap that is
    equally large in both columns refutes it just as a zero would.
    """
    pairs = [p for s in samples for p in s.pass_branch_compose]
    struck = [
        (s.struck_branch, s.struck_compose)
        for s in samples
        if s.struck_branch is not None and s.struck_compose is not None
    ]
    if not pairs and not struck:
        return
    for branch in LaneBranch:
        got_pass = [c for b, c in pairs if b is branch]
        got_hit = [c for b, c in struck if b is branch]
        if not got_pass and not got_hit:
            continue
        as_pass = f"{percentile(got_pass, 0.5) * 100:+6.2f}cm (n={len(got_pass):>4})" if got_pass else "  n/a        "
        as_hit = f"{percentile(got_hit, 0.5) * 100:+6.2f}cm (n={len(got_hit):>3})" if got_hit else "  n/a       "
        print(f"LANE-COMPOSE {branch:<16} passes {as_pass}   collisions {as_hit}", flush=True)


def _report_delivery_by_branch(samples: list[_SignPassSample]) -> None:
    """Print what each branch actually DELIVERED on the plan, passes vs collisions.

    The branch tally says which decision settled a lane; this says whether that
    decision predicted the outcome. The two can disagree, and where they do the
    branch is not the mechanism: ``_lane_profile_for`` rebuilds the target sign's
    corridor group ALONE, while the plan the chassis drives is
    ``apply_sign_lanes`` composing every group across the whole path. A
    ``DELIVERED`` sign measuring short on the polyline is that gap, and with
    staleness refuted at 0.22 cm p90 there is nothing else left for it to be.

    Read 1.0 as the full plateau and anything below ~0 as the plan running on the
    forbidden side. ``CLAMPED_SHIFT`` is the built-in control: it is short by its
    own definition, so it should read short in both columns, and a branch that
    reads at plateau for passes but near zero for collisions is the interesting
    one.
    """
    pairs = [p for s in samples for p in s.pass_branch_delivery]
    struck = [(s.struck_branch, s.struck_delivered) for s in samples if s.struck_branch is not None]
    struck = [(b, d) for b, d in struck if d is not None]
    if not pairs and not struck:
        return
    for branch in LaneBranch:
        got_pass = [d for b, d in pairs if b is branch]
        got_hit = [d for b, d in struck if b is branch]
        if not got_pass and not got_hit:
            continue
        as_pass = f"{percentile(got_pass, 0.5):+5.2f}x (n={len(got_pass):>4})" if got_pass else "   n/a       "
        as_hit = f"{percentile(got_hit, 0.5):+5.2f}x (n={len(got_hit):>3})" if got_hit else "   n/a      "
        print(f"LANE-DELIVERY {branch:<16} passes {as_pass}   collisions {as_hit}", flush=True)


def _report_lane_staleness(samples: list[_SignPassSample]) -> None:
    """Print how far the held plan sits from a fresh rebuild, passes vs collisions.

    Settles what the branch tally cannot. A collision classifying ``DELIVERED``
    says the planner WOULD lay a full lane from the inputs it has, while the
    measured offset says the plan running through that sign has none -- and only
    one of those can describe the polyline the tracker actually drove. This
    measures the two against each other directly, elementwise, at the waypoint
    the pass happens on.

    Paired columns for the same anti-circularity reason as the branch report: a
    continuously-refining estimate makes some disagreement normal everywhere, so
    the claim is a contrast, not a level.
    """
    pass_stale = [d for s in samples for d in s.pass_stale]
    struck_stale = [s.struck_stale for s in samples if s.struck_stale is not None]
    if not pass_stale and not struck_stale:
        return
    for label, values in (("passes", pass_stale), ("collisions", struck_stale)):
        if not values:
            continue
        # Centimetres, against the 18.14 / 27.86 cm plateaux: the question is
        # whether the disagreement is the whole lane or a rounding crumb.
        print(
            f"LANE-STALE {label:<11} n={len(values):>5}  "
            f"median {percentile(values, 0.5) * 100:6.2f}cm  "
            f"p90 {percentile(values, 0.9) * 100:6.2f}cm  "
            f"max {max(values) * 100:6.2f}cm",
            flush=True,
        )
    # The gate's own state, which decides WHICH staleness bug this is.
    pass_fp = [f for s in samples for f in s.pass_fp_stale]
    struck_fp = [s.struck_fp_stale for s in samples if s.struck_fp_stale is not None]
    for label, flags in (("passes", pass_fp), ("collisions", struck_fp)):
        if not flags:
            continue
        un_fired = sum(1 for f in flags if f)
        print(
            f"LANE-STALE {label:<11} fingerprint un-fired {un_fired:>5}/{len(flags)} "
            f"({un_fired / len(flags) * 100:5.1f}%)",
            flush=True,
        )


def report_sign_pass_crosstrack(workers: int, configs: list[SweepConfig]) -> None:
    """Print tracking error at sign passes, split by outcome.

    Three questions, in order, because each one only matters if the previous
    answered yes: is the error big relative to the ~3.1 cm of lateral room the
    planner has to give; is it a systematic outward bias (fixable aim) or
    symmetric scatter (not); and do the runs that hit a sign carry more of it
    than the runs that do not.
    """
    count = len(_scenarios(configs[0]))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        if len(configs) > 1:
            # Screening several knobs: the per-arm detail blocks below would
            # bury the comparison, so print one row each. Boundary path-heading
            # error is the mechanism being targeted and collisions are the
            # outcome -- a knob that moves the second without the first did it
            # by some other route and should be treated with suspicion.
            for arm in configs:
                got = list(pool.map(_sign_pass_crosstrack, [(i, arm) for i in range(count)]))
                head_b = [h for smp in got for h in smp.head_boundary]
                yaw_b = [y for smp in got for y in smp.yaw_boundary]
                hits = sum(1 for smp in got if smp.collided_with_sign)
                print(
                    f"YAW-SCREEN {arm.label:<38} sign-collisions {hits:>3}/{count}  "
                    f"boundary path-heading {percentile(head_b, 0.5):5.2f}deg "
                    f"(p90 {percentile(head_b, 0.9):5.2f})  "
                    f"boundary yaw {percentile(yaw_b, 0.5):5.2f}deg",
                    flush=True,
                )
            return
        samples = list(pool.map(_sign_pass_crosstrack, [(i, configs[0]) for i in range(count)]))

    near = [e for s in samples for e in s.near_abs]
    everywhere = [e for s in samples for e in s.all_abs]
    outward = [e for s in samples for e in s.near_outward]
    if not near:
        print("SIGN-CROSSTRACK no sign-pass ticks recorded", flush=True)
        return

    def _row(label: str, values: list[float]) -> None:
        print(
            f"SIGN-CROSSTRACK {label:<26} n {len(values):>7}  "
            f"median {percentile(values, 0.5) * 100:6.2f}cm  "
            f"p90 {percentile(values, 0.9) * 100:6.2f}cm  "
            f"max {max(values) * 100:6.2f}cm",
            flush=True,
        )

    _row("|error| at sign passes", near)
    _row("|error| everywhere", everywhere)

    outward_share = sum(1 for e in outward if e > 0) / len(outward) if outward else 0.0
    print(
        f"SIGN-CROSSTRACK {'signed radial at passes':<26} n {len(outward):>7}  "
        f"mean {sum(outward) / len(outward) * 100:+6.2f}cm  "
        f"median {percentile(outward, 0.5) * 100:+6.2f}cm  "
        f"outward {outward_share * 100:5.1f}%  (+ = outside its own path)",
        flush=True,
    )

    # The control for the sweep's `planned_outward_m`. Read the two together or
    # neither means anything: a collision-only reading is circular, because a
    # chassis that hit a sign necessarily planned a path near it. What is not
    # circular is the contrast against the plateau the lane is supposed to
    # deliver -- 18.14 cm where `clamp_lateral` binds, 27.86 cm where it does
    # not, against -10 cm for an untouched centreline, since WRO signs sit
    # 10 cm off-centre and always on the forbidden side.
    passes = [e for s in samples for e in s.pass_outward]
    if passes:
        full = sum(1 for e in passes if e >= _LANE_DELIVERED_FRAC) / len(passes)
        print(
            f"SIGN-CROSSTRACK {'lane delivered at PASSES':<26} n {len(passes):>7}  "
            f"median {percentile(passes, 0.5):+6.2f}x  "
            f"p10 {percentile(passes, 0.1):+6.2f}x  "
            f"at-plateau {full * 100:5.1f}%  (1.0 = full lane, <0 = wrong side)",
            flush=True,
        )

    _report_lane_branches(samples)

    struck = [e for s in samples if s.collided_with_sign for e in s.near_abs]
    clean = [e for s in samples if not s.collided_with_sign for e in s.near_abs]
    if struck and clean:
        _row("|error|, runs that hit", struck)
        _row("|error|, runs that did not", clean)

    # The decisive comparison: the tick contact happened on, against every
    # pass that did not end in contact. If tracking error is what puts the
    # chassis into a sign, these two must separate. If they do not, the
    # collisions are being caused by something the chassis's distance from
    # its own path does not capture, and tuning the tracker cannot fix them.
    at_collision = [s.at_collision for s in samples if s.at_collision is not None]
    if at_collision:
        _row("|error| AT the collision tick", at_collision)
        _row("|error| at clean passes", clean)

    # The variable crosstrack is blind to. A squeezed plateau clears its sign
    # only within +/-28.2 deg of the corridor axis, so if collisions sit above
    # that band while clean passes sit below it, the lever is the approach
    # ANGLE -- speed, steering rate, ramp shape -- and neither lateral
    # placement nor tracking accuracy can reach it.
    yaw_near = [y for s in samples for y in s.near_yaw_deg]
    yaw_hit = [s.yaw_at_collision for s in samples if s.yaw_at_collision is not None]
    yaw_clean = [y for s in samples if not s.collided_with_sign for y in s.near_yaw_deg]
    if yaw_near:
        _YAW_LIMIT_DEG = 28.2

        def _yaw_row(label: str, values: list[float]) -> None:
            over = sum(1 for v in values if v > _YAW_LIMIT_DEG) / len(values)
            print(
                f"SIGN-CROSSTRACK {label:<26} n {len(values):>7}  "
                f"median {percentile(values, 0.5):6.2f}deg  "
                f"p90 {percentile(values, 0.9):6.2f}deg  "
                f"over {_YAW_LIMIT_DEG}deg {over * 100:5.1f}%",
                flush=True,
            )

        _yaw_row("yaw at sign passes", yaw_near)
        _yaw_row("yaw at clean passes", yaw_clean)
        if yaw_hit:
            _yaw_row("yaw AT the collision tick", yaw_hit)

        # THE SPLIT: is the pass yaw corner-driven or lane-driven? A boundary
        # sign sits beside a corner the chassis may still be rotating out of;
        # a middle sign has a corner-free approach on both sides. Same lane
        # ramp in both cases, so a large gap is the corner and a small one is
        # the ramp.
        boundary = [y for s in samples for y in s.yaw_boundary]
        middle = [y for s in samples for y in s.yaw_middle]
        if boundary and middle:
            _yaw_row("yaw at BOUNDARY signs", boundary)
            _yaw_row("yaw at MIDDLE signs", middle)
            head_b = [h for s in samples for h in s.head_boundary]
            head_m = [h for s in samples for h in s.head_middle]
            if head_b and head_m:
                _yaw_row("path-heading err, BOUNDARY", head_b)
                _yaw_row("path-heading err, MIDDLE", head_m)
            # Collisions per pass-tick, since boundary signs outnumber middle
            # ones ~17:1 in the corpus and raw counts would say nothing.
            hits_mid = sum(1 for s in samples if s.struck_middle is True)
            hits_bnd = sum(1 for s in samples if s.struck_middle is False)
            print(
                f"SIGN-CROSSTRACK {'collisions by sign depth':<26} "
                f"boundary {hits_bnd:>4} over {len(boundary):>6} ticks "
                f"({hits_bnd / len(boundary) * 1e4:5.2f} per 10k)   "
                f"middle {hits_mid:>4} over {len(middle):>6} ticks "
                f"({hits_mid / len(middle) * 1e4:5.2f} per 10k)",
                flush=True,
            )

    # How far the lane was planned from the sign it was meant to clear. The
    # plan's own slack is ~3.4cm, so anything approaching that is enough on
    # its own -- and unlike yaw or crosstrack it is invisible from inside the
    # robot's frame, where the pass looks correct.
    err = [s.estimate_err_m for s in samples if s.estimate_err_m is not None]
    if err:
        over = sum(1 for e in err if e > 0.034) / len(err)
        print(
            f"SIGN-CROSSTRACK {'sign estimate error':<26} n {len(err):>7}  "
            f"median {percentile(err, 0.5) * 100:6.2f}cm  "
            f"p90 {percentile(err, 0.9) * 100:6.2f}cm  "
            f"max {max(err) * 100:6.2f}cm  over 3.4cm {over * 100:5.1f}%",
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
    # Where the Obstacles centreline sits, toward the inner block. Run WITH the
    # lane on, since that is the configuration it has to hold up in. Geometry
    # says 0.0 (signs sit 0.10 m either side of a 1.0 m corridor's centre, so
    # centred is symmetric); the tracker's documented outward drift says
    # otherwise. This is the arbitration.
    # How far out discovery may ingest an observation, BLIND with the lane on.
    # The measured bottleneck: publish distance is capped at the 2.0 m default
    # and 25% of signs are published already inside ACTIVATION_DIST_M, so the
    # lane never gets the runway that makes it work sighted. Watch the SIGN
    # column: reaching further means acting on smaller, noisier bounding boxes
    # (see MIN_RELIABLE_BBOX_HEIGHT_PX), so a gain here can be paid for in
    # mis-placed signs deforming the path toward a hazard that is not there.
    # Reconnaissance-lap speed cap, BLIND with the lane on. Read laps>=1 first:
    # this exists to survive lap 1, and everything downstream is conditional on
    # that. Then read in-time, because a slow first lap is paid for in clock --
    # the round limit is 180 s and a clean three-lap run already takes ~130 s,
    # so there is not much to spend.
    # How much of the path ahead a lane rebuild may not move, BLIND. Targets
    # the ramp-behind-the-chassis failure: a sign discovered 1.5 m into a
    # corridor rebuilds a lane whose approach ramp is already behind the
    # robot. Sighted is untouched by construction (it never rebuilds), so any
    # movement here is genuinely a blind result.
    # Does the escape mask cost more than it saves in BLIND? The mask withholds
    # LIDAR returns near a ROUTED sign from the CRITICAL escape trigger, on the
    # reasoning that the planner already has that sign handled -- true sighted,
    # where the lane is planned a corridor in advance. In blind lap 1 the
    # planner provably cannot handle a sign that only appeared 1.5 m away, so
    # the mask may be suppressing the last-resort reactive layer for precisely
    # the signs nothing else is covering. 0.0 disables the mask entirely.
    # The middle rung against the mask trade. Read all three columns: this
    # aims to keep mask-off's sign gain (57 -> 41) WITHOUT its wall cost
    # (0 -> 13), so a result that just moves collisions between the two
    # columns has not achieved anything.
    # How far back a retrace runs before the robot drives forward again. Too
    # short and it has not cleared the sign it backed away from; too long and
    # it is spending clock and re-approaching from further out. Run with the
    # mask off, i.e. the configuration retracing exists to make safe.
    "blind-retrace-dist": lambda v: SweepConfig(
        f"blind, retrace dist {v:{_FORMAT_2F}}",
        blind=True,
        sign_lane_planner=True,
        escape_mask_radius=0.0,
        retrace_escape=True,
        retrace_dist=v,
    ),
    "blind-evade": lambda v: SweepConfig(
        f"blind, evade steer {v:{_FORMAT_2F}}",
        blind=True,
        sign_lane_planner=True,
        sign_contact_evade=v > 0.0,
        sign_contact_steer=v,
    ),
    "blind-mask": lambda v: SweepConfig(
        f"blind, escape mask {v:{_FORMAT_3F}}",
        blind=True,
        sign_lane_planner=True,
        escape_mask_radius=v,
    ),
    "blind-commit": lambda v: SweepConfig(
        f"blind, commit-ahead {v:{_FORMAT_2F}}",
        blind=True,
        sign_lane_planner=True,
        sign_lane_commit_ahead=v,
    ),
    "blind-explore": lambda v: SweepConfig(
        f"blind, explore-lap speed {v:{_FORMAT_2F}}",
        blind=True,
        sign_lane_planner=True,
        explore_lap_speed_frac=v,
    ),
    "blind-reach": lambda v: SweepConfig(
        f"blind, ingest range {v:{_FORMAT_2F}}",
        blind=True,
        sign_lane_planner=True,
        ingest_range=v,
    ),
    "blind-hits": lambda v: SweepConfig(
        f"blind, min hits {int(v)}",
        blind=True,
        sign_lane_planner=True,
        min_hits=int(v),
    ),
    "lane-bias": lambda v: SweepConfig(
        f"obstacles centre bias {v:{_FORMAT_2F}}",
        sign_lane_planner=True,
        obstacles_center_bias=v,
    ),
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
    # blind-source, re-run with the lane planner ON -- the attribution that
    # decides whether any further blind work belongs in the localizer or in
    # sign discovery. Same three arms, so the middle row is the informative
    # one: it withholds the track layout and travel direction (believed pose
    # comes from the assumed start + LIDAR matching) but HANDS OVER the sign
    # positions.
    #   middle ~= fully blind  -> knowing the signs does not help, so the
    #                             believed POSE is the binding constraint
    #   middle ~= sighted      -> the pose is fine and DISCOVERY is what fails
    # Worth running before attempting either fix: it says which one is worth a
    # session, and bounds what fixing it can possibly buy.
    "lane-blind-source": [
        SweepConfig("lane, sighted (everything known)", sign_lane_planner=True),
        SweepConfig("lane, blind track+direction, signs known", sign_lane_planner=True, blind=True, known_signs=True),
        SweepConfig("lane, fully blind (signs discovered)", sign_lane_planner=True, blind=True),
    ],
    # Are the observed heading reversals the PARKING maneuver or the driving?
    # Traced on go_obstacles_0000 (subset64, lane on): all 9 reversals landed
    # after the third lap was already counted, at (1.7-1.8, 2.4-2.6) beside the
    # north parking bay, spinning ~186 deg every ~2 s; the laps themselves were
    # clean. Parking is a known-blocked problem (chassis-vs-pocket geometry),
    # so if the reversals vanish with park=False they are its failure mode and
    # not a cornering defect -- and the in-time shortfall is then mostly clock
    # burnt after the driving is already done, which is a different fix.
    # Blind arms included because parking is only ever attempted AFTER three
    # laps, so a park=False arm isolates the DRIVING phase exactly: any
    # reversal it still reports happened while the robot was lapping. Sighted
    # park=False measured 0/64 -- if blind park=False is also 0, there is no
    # cornering defect anywhere and every reversal ever seen is the parking
    # maneuver.
    "lane-park": [
        SweepConfig("lane, sighted, parking ON", sign_lane_planner=True),
        SweepConfig("lane, sighted, parking OFF", sign_lane_planner=True, park=False),
        SweepConfig("lane, blind, parking ON", sign_lane_planner=True, blind=True),
        SweepConfig("lane, blind, parking OFF", sign_lane_planner=True, blind=True, park=False),
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
    # Lookahead is the only one of seven knobs screened (`yaw-screen`) that
    # reduces the boundary-sign tracker lag, which is ~2/3 of the pass yaw and
    # the larger half of a 5.94 cm clearance budget. But the screen reports
    # SIGN collisions only, and the relationship is non-monotonic there --
    # 0.16 gives 195/256 against a 199 baseline while the shorter 0.12, which
    # cuts the lag furthest, gives 202. Shortening the lookahead changes
    # cornering everywhere, so the wall column and laps>=3 are what decide
    # whether 0.16 is an improvement or another sign-for-wall trade.
    "blind-lookahead": [
        SweepConfig("blind, lookahead 0.20/0.40 (shipped)", blind=True, sign_lane_planner=True),
        *(
            SweepConfig(
                f"blind, lookahead {v:{_FORMAT_2F}}/{v * _LOOKAHEAD_MULTIPLIER:{_FORMAT_2F}}",
                blind=True,
                sign_lane_planner=True,
                lookahead_short=v,
                lookahead_long=v * _LOOKAHEAD_MULTIPLIER,
            )
            for v in (0.14, 0.16, 0.18)
        ),
    ],
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
    # One arm, the shipped blind default, run for its ATTRIBUTION rather than
    # for a comparison: with the lane shipped on, blind's remaining failure is
    # 230/256 sign collisions, and the open question is how many of those the
    # escape drove into versus how many it never got a chance to prevent. Read
    # `escapes N/lap` and `collisions within 40 ticks` from the RESULT row, and
    # `since_escape` per scenario with --verbose. Normalise by laps driven --
    # raw escape totals are not comparable across arms.
    #
    # Measured on the FULL 256 corpus: only 20/230 sign collisions land within
    # 40 ticks of an escape (0.78 escapes/lap) -- the escape is silent for the
    # other 210, matching the 2026-08-16 finding that blind collisions mostly
    # happen with the escape never firing at all. The `sign-mask` split in the
    # same RESULT row answers WHY: 108/230 struck signs were already in
    # `routed_sign_positions` (masked from the CRITICAL trigger, so the
    # proximity-gated-unmask idea can plausibly reach them) but 122/230 were
    # never routed at all (a discovery/routing gap upstream of the mask --
    # unmasking changes nothing for these). Roughly even split: the mask is a
    # real, sizeable lever, but not the majority of the remaining failure.
    "blind-arc": [
        SweepConfig("blind, shipped defaults (lane ON)", blind=True),
    ],
    "lane": [
        SweepConfig("sighted, lane planner OFF (pre-2026-08-17 shipped)", sign_lane_planner=False),
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
    # Third arm matters: SIGN_LANE_SUPPRESS_DEFORM was decided in SIGHTED mode,
    # where the lane has a full corridor of runway and the carrot override is
    # only a time tax. Blind has no such runway -- a sign discovered 1.5 m into
    # a corridor cannot be planned around, only reacted to -- so the sighted
    # answer should not be assumed to carry over.
    # The point of the exercise: mask OFF so the escape may fire on a sign
    # (the only measured blind gain, sign 57 -> 41), with the reverse RETRACED
    # rather than swung, to avoid the 13 wall collisions that gain cost. Read
    # the WALL column against the middle arm, not against the shipped default
    # -- the comparison that matters is "same sign gain, less wall damage".
    "blind-retrace": [
        SweepConfig("blind, mask ON (shipped)", blind=True, sign_lane_planner=True),
        SweepConfig("blind, mask OFF, arc reverse", blind=True, sign_lane_planner=True, escape_mask_radius=0.0),
        SweepConfig(
            "blind, mask OFF, retrace reverse",
            blind=True,
            sign_lane_planner=True,
            escape_mask_radius=0.0,
            retrace_escape=True,
        ),
    ],
    "lane-blind": [
        SweepConfig("blind, lane planner OFF (pre-2026-08-17 shipped)", blind=True, sign_lane_planner=False),
        SweepConfig("blind, lane ON, override suppressed", blind=True, sign_lane_planner=True),
        SweepConfig(
            "blind, lane ON + override",
            blind=True,
            sign_lane_planner=True,
            sign_lane_suppress_deform=False,
        ),
    ],
    "blind-split": [
        SweepConfig("blind, pre-fix (offset 0.20, split off)", blind=True, lateral_offset=0.20, escape_mask_radius=0.0),
        SweepConfig("blind, split only (offset 0.20)", blind=True, lateral_offset=0.20),
        SweepConfig("blind, offset only (0.28, split off)", blind=True, lateral_offset=0.28, escape_mask_radius=0.0),
        SweepConfig("blind, both (shipped defaults)", blind=True),
    ],
}
"""Modes with a fixed comparison set, ignoring any CLI values."""

# Candidate levers against the boundary-sign tracker lag, which is ~2/3 of the
# pass yaw (11.33 deg path-heading error at a boundary sign against 5.82 at a
# mid-section one) and the larger half of a 5.94 cm clearance budget. Screened
# on whether they move that angle, not just the collision count: the angle is
# the mechanism, and a knob that moves collisions WITHOUT it did so some other
# way and needs explaining before it is trusted.
_YAW_SCREEN_ARMS = [
    SweepConfig("shipped", blind=True, sign_lane_planner=True),
    SweepConfig("lookahead 0.12/0.24", blind=True, sign_lane_planner=True, lookahead_short=0.12, lookahead_long=0.24),
    SweepConfig("lookahead 0.16/0.32", blind=True, sign_lane_planner=True, lookahead_short=0.16, lookahead_long=0.32),
    # Purpose-built and never measured on the corpus. Caps to slow_mps while
    # the router has a correction in flight -- more time to rotate through the
    # corner-adjacent pass. Watch in-time: it slows within 1.40 m of EVERY
    # sign and in-time is only 27/256 to begin with.
    SweepConfig("sign-aware speed", blind=True, sign_lane_planner=True, sign_aware_speed=True),
    # A longer ramp spreads the same lateral travel over more distance, so the
    # path itself bends less where the chassis is already busy with a corner.
    SweepConfig("ramp 1.20", blind=True, sign_lane_planner=True, sign_lane_ramp=1.20),
    SweepConfig("ramp 1.50", blind=True, sign_lane_planner=True, sign_lane_ramp=1.50),
    # Directly raises how fast the chassis MAY rotate, which is the actuator
    # limit a lag runs into. Note the standing warning against raising the
    # corridor follower's gain/cap -- this is the steering rate, not that gain,
    # but treat an improvement here sceptically until oscillation is ruled out.
    SweepConfig(
        "steer-rate 1.5x",
        blind=True,
        sign_lane_planner=True,
        max_steering_rate=NavigationTuning.load_default().pursuit.MAX_STEERING_RATE * 1.5,
    ),
    # The classic cause of heading lag in pure pursuit, and the one absent
    # from the first screen: a long lookahead aims at a point beyond the turn,
    # so the chassis cuts the corner and its heading trails the path's. If
    # anything here reduces the boundary lag it should be this. Swept both
    # ways -- the pair is gated together by `_LOOKAHEAD_MULTIPLIER`.
    # Sets the heading RATE the corner demands. A wider arc asks for less
    # rotation per metre, which is the demand side of the same lag the
    # steering-rate arm attacked from the supply side.
    SweepConfig("arc 0.35", blind=True, sign_lane_planner=True, arc_radius=0.35),
    SweepConfig("arc 0.45", blind=True, sign_lane_planner=True, arc_radius=0.45),
]

MODES = ("crosstrack", "sign-crosstrack", "yaw-screen", "lane-geometry", *_FIXED_MODES, *_SWEPT_MODES)


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

    scenarios_dir = args.scenarios_dir or (str(CORPUS_DIR) if args.corpus else None)
    if args.mode == "lane-geometry":
        report_lane_geometry(scenarios_dir, args.values)
        return

    if args.mode in ("sign-crosstrack", "yaw-screen"):
        arms = (
            _YAW_SCREEN_ARMS
            if args.mode == "yaw-screen"
            else [SweepConfig("sign-crosstrack", blind=True, sign_lane_planner=True)]
        )
        # `yaw-screen N` runs only the first N arms. Each arm is a full pass
        # over the scenario set, so screening nine of them against the corpus
        # costs over an hour -- once the fixtures have narrowed the field,
        # re-running the refuted arms buys nothing.
        if args.values:
            arms = arms[: int(args.values[0])]
        report_sign_pass_crosstrack(args.workers, [replace(a, scenarios_dir=scenarios_dir) for a in arms])
        return

    configs = _build_configs(args.mode, args.values)
    if scenarios_dir:
        configs = [replace(c, scenarios_dir=scenarios_dir) for c in configs]
    run_sweep(configs, args.workers, verbose=args.verbose)


if __name__ == "__main__":
    main()

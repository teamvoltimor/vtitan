"""Split the surviving sign collisions between the two known failure modes.

Both modes are diagnosed (see docs/sign-avoidance-investigation.md) but their
SHARE is not, and that is what decides where the next fix goes. Fixing the
geometry (Mode A) is worthless if most runs die with no deformation at all.

At the tick the run ends, ask what the router was doing:

* **Mode B -- avoidance was OFF.** ``deform_waypoint`` returned the waypoint
  untouched: either no sign was committed, or the corner guard rejected every
  candidate. Nothing the deformation geometry does can help here.
* **Mode A -- avoidance was ON but insufficient.** A sign was committed and the
  waypoint was deformed, yet the chassis hit anyway. Sub-split by whether the
  commanded lateral line could ever have cleared the sign:
  - ``A-clamped``: the deformed target was itself closer to the sign than a
    yawed chassis needs, so following it perfectly still collides. This is the
    wall-clamp shortfall the yaw-aware clamp would fix.
  - ``A-lag``: the target was far enough out, but the robot had not converged
    onto it yet -- pure pursuit closes cross-track error over distance and the
    chassis drew abreast of the sign first.

``--yaw`` additionally splits the ``A-clamped`` group by the chassis heading at
the fatal tick, which is what decides whether "arrive square" is a real lever
(investigation doc, Next item 2b). The clearance a pass needs scales with the
angle ``th`` off the corridor axis as ``(L/2)|sin th| + (W/2)|cos th| +
sign_half``, so the same commanded line that fails while turning can be ample
square. Each ``A-clamped`` collision is re-asked as: given the clearance the
line actually offered, was it short only because the chassis was yawed
(``recoverable`` -- arriving square fixes it) or short even square
(``hard`` -- no heading fix reaches it)?

Every lateral figure here is reported in TWO frames, because "off the corridor
axis" and "off the planned path" are the same question only on a straight:

* **axis** -- measured against the sign's straight corridor axis. What every
  figure published before this used.
* **path** -- measured against the planned path the robot was actually asked to
  follow, via :func:`~src.navigation.track_geometry.project_onto_path`.

Two-thirds of legal sign positions sit on a corner, and there the axis frame
charges the robot for the turn it was supposed to be making: a chassis
perfectly on its arc reads as both heavily yawed and steadily diverging. That
inflated yaw then inflates the clearance the pass is said to have needed, so
the confound reaches the ``adequate``/``short`` split as well as the tracking
numbers. Only the path frame is a tracking error. The axis column is kept
beside it so the size of the correction stays visible rather than numbers
changing silently.

The straight-approach subset is the wiring check: with the path parallel to the
corridor axis the frames differ only by a constant origin, and every figure here
is a difference, so they should agree. Where they do not, read the per-tick
JUMP figure first -- a commanded line that alternates between two positions at
20 Hz is resolved against a different global coordinate on alternate ticks by
the axis frame, and is not one line the chassis failed to reach in either.

Runs in the competition configuration (blind) by default, since that is the one
that counts.

Usage (from ``platform/robot``, PYTHONPATH=.)::

    python scripts/sim/diag_failure_split.py --corpus
    python scripts/sim/diag_failure_split.py --corpus --sighted
    python scripts/sim/diag_failure_split.py --corpus --yaw
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Axis
from shared.domain.models import Waypoint

import src.navigation.planning.sign_router as sign_router_module
from scripts.common.sign_router_capture import patched_deform_waypoint
from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from scripts.common.stats import median
from src.navigation.track_geometry import project_onto_path
from src.navigation.utils import wrap_angle
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

CHASSIS_HALF_DIAGONAL = math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)
_PASS_CLEARANCE = CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2
"""Centre-to-centre separation a mid-turn pass needs.

The yawed figure deliberately, not the aligned one: a target closer than this
cannot be followed safely at an arbitrary heading, which is the condition
``A-clamped`` is testing for.
"""

_SQUARE_CLEARANCE = RobotSpecs.WIDTH / 2 + TrafficSignSpecs.WIDTH / 2
"""Centre-to-centre separation a pass needs with the chassis ON the corridor axis.

The floor of the yaw-dependent requirement: at ``th = 0`` the length term drops
out entirely and only the half-WIDTH presents. Any commanded line at or above
this could have been followed cleanly by a square chassis, so a collision on
such a line is a heading failure, not a clamp failure.
"""

_DEFAULT_WORKERS = 8

_YAW_BUCKETS_DEG = (10.0, 20.0, 28.0, 40.0)
"""Histogram edges. 28 deg is the measured budget at the clamp's 0.181 m."""


_MOSTLY = 0.5
"""Threshold for calling an approach abeam-dominated rather than incidental."""

_ABEAM_LEAD_M = 0.10
"""Below this along-track lead, pure pursuit has no useful forward component."""

_MIN_APPROACH_TICKS = 10
"""Below this there is no convergence to speak of, only the impact itself."""

_MIN_PATH_WAYPOINTS = 2
"""Below two waypoints there is no segment, so no tangent and no path frame."""

_SKEW_DEG = 20.0
"""Angle between the corridor axis and the path above which they are not the
same measurement at all. Measured: on approaches the robot drove dead straight,
skew above this separates every frame disagreement from every agreement."""

_STRAIGHT_TURN_DEG = 10.0
"""Path heading change over an approach below which it counts as a straight.

The whole point of the frame split. Two-thirds of legal sign positions sit on a
corner, and across a turning approach the corridor axis and the path have
rotated apart by exactly this angle -- so this is also the size of the error the
axis frame introduces. Ten degrees puts the axis-frame inflation under a
centimetre at the offsets involved, which is below the noise on everything else
here.
"""


@dataclass(frozen=True, slots=True)
class OffsetSeries:
    """Commanded line and chassis as lateral offsets from the sign, in one frame.

    Signed and sharing an origin, so they difference directly: ``target`` is
    the line the router commanded, ``robot`` is where the chassis actually was.
    Which frame decides whether the difference means anything -- see
    :class:`Approach`.
    """

    target: list[float]
    robot: list[float]

    @property
    def line_travel_m(self) -> float:
        """How far the commanded line itself moved during the approach.

        Pure-pursuit convergence assumes something to converge TO; if the line
        is travelling as fast as the chassis can close on it, no amount of
        lookahead or gain tuning helps, and the fix is upstream in what the
        router commands.
        """
        return max(self.target) - min(self.target)

    @property
    def error_start_m(self) -> float:
        """Cross-track error to the commanded line when the approach began."""
        return abs(self.target[0] - self.robot[0])

    @property
    def error_end_m(self) -> float:
        """Cross-track error to the commanded line at impact."""
        return abs(self.target[-1] - self.robot[-1])

    @property
    def closed_m(self) -> float:
        """Cross-track error actually removed over the approach. Negative = grew."""
        return self.error_start_m - self.error_end_m

    @property
    def chatter_m(self) -> float:
        """Median tick-to-tick jump in the commanded line.

        Bounds what every other figure here can mean. ``line_travel_m`` reads a
        line that moved steadily and one that alternated between two positions
        identically, and only the first is something pure pursuit can converge
        onto. The sign-corridor label can flip between two axes on consecutive
        ticks (``docs/sign-avoidance-investigation.md``), which deforms along
        the depth axis instead of the lateral one and drops the commanded
        lateral line back onto the path every other tick -- so this is not a
        hypothetical.
        """
        return median([abs(b - a) for a, b in itertools.pairwise(self.target)]) if len(self.target) > 1 else 0.0


@dataclass(frozen=True, slots=True)
class Approach:
    """The final unbroken run of ticks spent committed to the fatal sign.

    Carries the same approach in BOTH frames. ``axis`` measures against the
    sign's straight corridor axis, which is what every figure published before
    this used; ``path`` measures against the planned path the robot was
    actually asked to follow. On a straight approach the two should agree, since
    they then differ only by a constant origin. On a corner they do not, and
    only ``path`` is a tracking error -- ``axis`` additionally counts the turn
    the robot was supposed to be making, so a chassis on its arc reads as
    diverging. Both are kept so the correction stays auditable rather than
    replacing numbers silently.
    """

    axis: OffsetSeries
    path: OffsetSeries

    approach_turn_rad: float = 0.0
    """Heading the planned path swept between the first and last tick.

    The size of the disagreement between the two frames, and the handle for
    splitting straights from corners.
    """

    target_leads: list[float] = field(default_factory=list)
    """Along-track distance from chassis to commanded target, per tick.

    Pure pursuit converts lateral error into heading change by aiming at a point
    AHEAD; the conversion weakens as that lead shrinks and is meaningless once
    the target is abeam. ``pin_depth`` deliberately holds the commanded point
    level with the sign, so this is where to look for a target that stopped
    leading -- a chassis chasing sideways cannot close cross-track error however
    much runway is left.
    """

    committed_ticks_total: int = 0
    """Ticks committed to this sign across the WHOLE run, gaps included."""

    dropouts: int = 0
    """Times the router let go of this sign and later re-took it.

    Pure pursuit closes cross-track error over distance, so what matters is not
    how long the sign was engaged in total but how much UNBROKEN runway the
    chassis had to converge on one line. Every dropout restarts that.
    """

    engage_distance_m: float = 0.0
    """Robot-to-sign distance the FIRST time this sign was ever committed.

    Read against ``ACTIVATION_DIST_M`` (1.40 m). The activation radius is an
    upper bound on the runway, not the runway itself: a sign the router has not
    discovered yet cannot be committed however close it is, so in blind mode
    this is the number that actually decides how much distance pure pursuit has
    to work with.
    """

    ticks_in_range: int = 0
    """Ticks the robot spent inside ``ACTIVATION_DIST_M`` of the fatal sign.

    The denominator for ``committed_ticks_total``. The gap between the two is
    approach distance during which the sign was in range and eligible but the
    router was NOT deforming for it -- runway the pursuit controller never got
    offered, and which no pursuit-side knob can recover.
    """

    @property
    def ticks(self) -> int:
        """Length of the final unbroken committed run."""
        return len(self.axis.target)

    @property
    def engaged_fraction(self) -> float:
        """Share of the in-range approach actually spent deforming."""
        return self.committed_ticks_total / self.ticks_in_range if self.ticks_in_range else 0.0

    @property
    def on_a_straight(self) -> bool:
        """Whether the path held its heading across the approach.

        Where it did, the axis frame was never wrong and its numbers stand.
        """
        return math.degrees(self.approach_turn_rad) < _STRAIGHT_TURN_DEG

    @property
    def lead_start_m(self) -> float:
        """Along-track lead at the start of the approach."""
        return self.target_leads[0] if self.target_leads else 0.0

    @property
    def lead_end_m(self) -> float:
        """Along-track lead at impact."""
        return self.target_leads[-1] if self.target_leads else 0.0

    @property
    def abeam_fraction(self) -> float:
        """Share of the approach with the target effectively abeam or behind.

        Judged against a tenth of a metre rather than zero: the conversion from
        lateral error to heading is already negligible well before the lead
        reaches zero, and a strict sign test would call a target 2 cm ahead
        "leading".
        """
        if not self.target_leads:
            return 0.0
        return sum(1 for a in self.target_leads if a < _ABEAM_LEAD_M) / len(self.target_leads)


@dataclass(frozen=True, slots=True)
class Frame:
    """The fatal tick's clearance geometry, in one reference frame.

    Both frames answer the same questions from the same three points; they
    differ only in what "lateral" means. The AXIS frame reads it off the sign's
    straight corridor axis, which is what every figure published before this
    used. The PATH frame reads it off the planned path, which changes two terms
    on a corner:

    * the **yaw** is measured against the path tangent, so a chassis square to
      the arc it is driving no longer reads as 40 deg yawed -- and every degree
      of that inflated the clearance it was said to need;
    * the **clearance** is the lateral gap in that same rotated frame.

    Both feed ``line_was_adequate``, which is the premise the whole item-2b
    split rests on, so sharing one shape is what makes the two comparable.
    """

    sign_offset_m: float
    target_offset_m: float
    robot_offset_m: float
    yaw_deg: float

    @property
    def clearance_m(self) -> float:
        """Lateral gap the commanded line left the sign."""
        return abs(self.target_offset_m - self.sign_offset_m)

    @property
    def tracking_error_m(self) -> float:
        """How far the chassis sat off the line it was commanded to hold.

        Differenced SIGNED. Unsigned it would collapse a chassis sitting on the
        wrong side of the sign entirely onto one that merely undershot, and the
        wrong-side case is the larger error of the two.
        """
        return abs(self.target_offset_m - self.robot_offset_m)

    @property
    def needed_at_yaw_m(self) -> float:
        """Clearance the chassis needed at the yaw it actually held."""
        return _needed_clearance(self.yaw_deg)

    @property
    def line_was_adequate(self) -> bool:
        """Whether the commanded line cleared the sign at the yaw actually held.

        ``A-clamped`` is judged against the half-DIAGONAL, i.e. the worst yaw
        the chassis could possibly present. A line short of that can still be
        ample at the heading the robot was really on, and when it is, the
        commanded line did not cause the collision -- the chassis not being on
        it did.
        """
        return self.clearance_m >= self.needed_at_yaw_m

    @property
    def recoverable_by_squaring(self) -> bool | None:
        """Whether arriving square would have cleared this sign.

        Only meaningful for a genuinely short line: if the line was already
        adequate at the held yaw, squaring the chassis was never the missing
        ingredient.
        """
        if self.line_was_adequate:
            return None
        return self.clearance_m >= _SQUARE_CLEARANCE


@dataclass(frozen=True, slots=True)
class Verdict:
    """One scenario's outcome, plus the geometry behind an ``A-clamped`` label.

    Carries the yaw terms rather than a pre-computed boolean so the "arrive
    square" question can be re-asked at a different threshold without re-running
    256 scenarios -- the same separation of run from verdict ``_label`` exists
    for, after a wrong verdict cost a full sweep once already.
    """

    kind: str
    label: str
    axis: Frame | None = None
    """Geometry against the sign's straight corridor axis -- the published frame."""

    path: Frame | None = None
    """The same geometry against the planned path. ``None`` when no path could
    be resolved, which is the only case where the axis frame is all there is."""

    approach: Approach | None = None

    @property
    def frames(self) -> list[tuple[str, Frame]]:
        """Both resolved frames, path first, for reporting side by side."""
        return [(name, f) for name, f in (("path frame", self.path), ("axis frame", self.axis)) if f]


def _yaw_off_axis(robot_yaw: float, lateral_axis: Axis) -> float:
    """Angle between the chassis and the corridor axis, folded into [0, 90] deg.

    The lateral axis is the one the deformation moves along, so the corridor
    runs along the OTHER one: a y-lateral corridor (south/north) is travelled
    along x, heading 0. Folded because the clearance requirement is symmetric --
    a chassis 30 deg off the axis presents the same footprint whichever way it
    leans, and whether it is nose-first or reversed does not matter either.
    """
    axis_heading = 0.0 if lateral_axis == "y" else math.pi / 2
    off = abs(math.atan2(math.sin(robot_yaw - axis_heading), math.cos(robot_yaw - axis_heading)))
    return math.degrees(min(off, math.pi - off))


def _needed_clearance(yaw_off_axis_deg: float) -> float:
    """Centre-to-centre separation a chassis at this heading needs to clear a sign."""
    th = math.radians(yaw_off_axis_deg)
    half_footprint = (RobotSpecs.LENGTH / 2) * abs(math.sin(th)) + (RobotSpecs.WIDTH / 2) * abs(math.cos(th))
    return half_footprint + TrafficSignSpecs.WIDTH / 2


@dataclass(frozen=True, slots=True)
class Tick:
    """One ``deform_waypoint`` call, kept raw for resolution afterwards.

    The lateral axis is only known once the fatal sign is, so resolving offsets
    per tick would bake in whichever corridor happened to be current then.
    """

    committed: int | None
    raw: Any
    deformed: Any
    pos: Any
    lead: float
    waypoints: list[Waypoint]
    """The path in force on this tick.

    Snapshotted per tick rather than read once at the end: the navigator
    replans on width updates and at the lap seam, rebinding the list, and an
    approach measured against a path that only came into being after the
    collision is measured against the wrong thing.
    """


def _classify(index: int, fixtures: Path | None, blind: bool) -> Verdict:
    """Run one scenario and name the failure mode at the tick it ended."""
    scenario = all_obstacles_demo_scenarios(fixtures)[index]

    last: dict[str, Any] = {}
    history: list[Tick] = []
    live: dict[str, Any] = {}

    def make_capturing(original_deform: Any) -> Any:
        def capturing(router: Any, waypoint: Any, robot_pos: Any, robot_yaw: float, *args: Any, **kwargs: Any) -> Any:
            result = original_deform(router, waypoint, robot_pos, robot_yaw, *args, **kwargs)
            last["raw"] = waypoint
            last["deformed"] = result
            last["committed"] = router._committed  # noqa: SLF001 - a probe, by design
            last["signs"] = router._signs  # noqa: SLF001
            last["corridors"] = router._sign_corridors  # noqa: SLF001
            last["direction"] = router._direction  # noqa: SLF001
            last["pos"] = robot_pos
            last["yaw"] = robot_yaw
            last["waypoints"] = _live_waypoints(live)
            # Per-tick history of the approach, for --approach.
            #
            # Along-track lead of the commanded target, in the chassis frame. Pure
            # pursuit converts lateral error into heading change only while it has
            # something AHEAD to aim at; a target gone abeam (lead -> 0) leaves the
            # controller chasing sideways, which is what pin_depth risks by holding
            # the commanded point level with the sign.
            dx = result[0] - robot_pos[0]
            dy = result[1] - robot_pos[1]
            lead = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
            history.append(Tick(router._committed, waypoint, result, robot_pos, lead, last["waypoints"]))  # noqa: SLF001
            return result

        return capturing

    with patched_deform_waypoint(make_capturing):
        sim = ScenarioSimulator(
            scenario.metadata,
            num_laps=scenario.laps,
            seed=scenario.seed,
            blind=blind,
            park=False,
        )
        live["sim"] = sim
        result = sim.run(max_steps=OBSTACLES_MAX_STEPS)

    if not result.collided:
        return Verdict("no collision", scenario.label)
    return _verdict(last, scenario.label, history)


def _live_waypoints(live: dict[str, Any]) -> list[Waypoint]:
    """The planned path currently in force, as a list of :class:`Waypoint`.

    The simulator hands out tuples in some paths and Waypoints in others, and
    the projection needs one shape. Read through a holder because the hook is
    defined before the simulator it reads from exists.
    """
    sim = live.get("sim")
    if sim is None:
        return []
    raw = sim.waypoints
    if live.get("raw") is not raw:
        # Identity, not equality: replanning rebinds the list, so a changed
        # object IS the replan signal and the conversion need not run per tick.
        live["raw"] = raw
        live["path"] = [w if isinstance(w, Waypoint) else Waypoint(*w) for w in raw]
    return live["path"]


def _verdict(
    last: dict[str, Any], label: str, history: list[tuple[int | None, Any, Any, float]] | None = None
) -> Verdict:
    """Name the failure mode, and for Mode A attach the yaw geometry.

    Geometry is attached to the whole of Mode A, not just ``A-clamped``: the two
    sub-labels are decided against the worst-case yaw, so which side of that
    line a run falls on says little about what actually went wrong at the
    heading it really held.
    """
    kind = _label(last)
    if not kind.startswith("A-"):
        return Verdict(kind, label)

    routing = sign_router_module.ROUTING_TABLE.get((last["corridors"][last["committed"]], last["direction"]))  # noqa: SLF001
    if routing is None:
        return Verdict(kind, label)
    axis = 1 if routing.axis is Axis.Y else 0
    sign = last["signs"][last["committed"]]
    sign_xy = (sign.x, sign.y)
    sign_lat = sign_xy[axis]
    return Verdict(
        kind,
        label,
        axis=Frame(
            # The sign is the origin of the axis frame by construction, so its
            # own offset is zero and the other two are measured from it.
            sign_offset_m=0.0,
            target_offset_m=last["deformed"][axis] - sign_lat,
            robot_offset_m=last["pos"][axis] - sign_lat,
            yaw_deg=_yaw_off_axis(last["yaw"], routing.axis),
        ),
        path=_path_frame(last.get("waypoints") or [], last["deformed"], last["pos"], last["yaw"], sign_xy),
        approach=_approach(history, last["committed"], axis, sign_lat, sign_xy),
    )


def _path_frame(
    waypoints: list[Waypoint],
    deformed: Any,
    pos: Any,
    robot_yaw: float,
    sign_xy: tuple[float, float],
) -> Frame | None:
    """Re-measure the fatal tick against the planned path instead of the axis.

    All three points are projected onto the SAME polyline, so their offsets
    share an origin and difference cleanly however the path is curving under
    them. The yaw is taken off the tangent at the sign -- the tangent the pass
    actually happens on -- rather than at the chassis, which would let a robot
    that has already turned away declare itself square.
    """
    if len(waypoints) < _MIN_PATH_WAYPOINTS:
        return None
    at_sign = project_onto_path(waypoints, *sign_xy)
    off_path = abs(wrap_angle(robot_yaw - at_sign.tangent_rad))
    return Frame(
        sign_offset_m=at_sign.signed_offset_m,
        target_offset_m=project_onto_path(waypoints, deformed[0], deformed[1]).signed_offset_m,
        robot_offset_m=project_onto_path(waypoints, pos[0], pos[1]).signed_offset_m,
        # Folded into [0, 90] for the same reason the axis version is: the
        # footprint a chassis presents is symmetric in lean and in nose/tail.
        yaw_deg=math.degrees(min(off_path, math.pi - off_path)),
    )


def _approach(
    history: list[Tick] | None,
    committed: int,
    axis: int,
    sign_lat: float,
    sign_xy: tuple[float, float],
) -> Approach | None:
    """Resolve the run of ticks spent committed to the fatal sign, in both frames.

    Only the FINAL unbroken run counts. A sign can be engaged, dropped and
    re-engaged, and the earlier spells were not the approach that ended the run
    — splicing them together would invent convergence that never happened.
    """
    if not history:
        return None
    run: list[Tick] = []
    for tick in reversed(history):
        if tick.committed != committed:
            break
        run.append(tick)
    if len(run) < _MIN_APPROACH_TICKS:
        return None
    run.reverse()

    engaged = [tick.committed == committed for tick in history]
    dropouts = sum(1 for prev, cur in itertools.pairwise(engaged) if prev and not cur)
    first_pos = history[engaged.index(True)].pos
    activation = NavigationTuning.load_default().sign_router.ACTIVATION_DIST_M
    in_range = sum(1 for t in history if math.hypot(t.pos[0] - sign_xy[0], t.pos[1] - sign_xy[1]) <= activation)

    frames = [_tick_offsets(tick, sign_xy) for tick in run]
    if any(f is None for f in frames):
        # A path-less tick cannot be measured in the path frame at all, and
        # dropping just that tick would splice a gap into a convergence series.
        return None
    tangents = [f[2] for f in frames if f is not None]
    return Approach(
        ticks_in_range=in_range,
        axis=OffsetSeries(
            target=[tick.deformed[axis] - sign_lat for tick in run],
            robot=[tick.pos[axis] - sign_lat for tick in run],
        ),
        path=OffsetSeries(
            target=[f[0] for f in frames if f is not None],
            robot=[f[1] for f in frames if f is not None],
        ),
        approach_turn_rad=abs(wrap_angle(tangents[-1] - tangents[0])),
        target_leads=[tick.lead for tick in run],
        committed_ticks_total=sum(engaged),
        dropouts=dropouts,
        engage_distance_m=math.hypot(first_pos[0] - sign_xy[0], first_pos[1] - sign_xy[1]),
    )


def _tick_offsets(tick: Tick, sign_xy: tuple[float, float]) -> tuple[float, float, float] | None:
    """One tick's target and robot offsets from the sign, in the path frame.

    Each point carries the offset from its OWN nearest stretch of path, which is
    what makes the pair curvature-free: a chassis correctly on its arc reads
    zero however hard the arc bends, where a global axis would have it drifting.
    Subtracting the sign's offset keeps the axis frame's origin, so every
    convergence property reads the same way in both.

    Returns the tangent under the chassis as well, to measure how far the path
    swept during the approach.
    """
    if len(tick.waypoints) < _MIN_PATH_WAYPOINTS:
        return None
    at_sign = project_onto_path(tick.waypoints, *sign_xy)
    target = project_onto_path(tick.waypoints, tick.deformed[0], tick.deformed[1])
    robot = project_onto_path(tick.waypoints, tick.pos[0], tick.pos[1])
    return (
        target.signed_offset_m - at_sign.signed_offset_m,
        robot.signed_offset_m - at_sign.signed_offset_m,
        robot.tangent_rad,
    )


def _label(last: dict[str, Any]) -> str:
    """Name the failure mode from the router state captured at the fatal tick.

    Split out from ``_classify`` so the run and the verdict stay separable: the
    verdict has been wrong once already (it compared Euclidean gaps, see below)
    and re-deriving it should not mean re-running 256 scenarios.
    """
    if not last:
        return "B-never-ran"

    committed = last.get("committed")
    raw, deformed = last.get("raw"), last.get("deformed")
    if committed is None or raw == deformed:
        # No sign selected, or the corner guard rejected every candidate and the
        # waypoint came back untouched. Avoidance was not running.
        return "B-no-deform"

    # Avoidance WAS running. Could the line it commanded ever have cleared?
    signs = last.get("signs") or []
    if committed >= len(signs):
        return "A-lag"
    sign = signs[committed]

    # Compare LATERAL separations, not Euclidean distances. The deformation only
    # moves the waypoint on the corridor's lateral axis, and the target leads the
    # robot by a lookahead along the DEPTH axis -- so a Euclidean gap is inflated
    # by an along-track term that has nothing to do with clearing the sign. Using
    # it made ``A-clamped`` almost unreachable: a line clamped to 0.181 m of real
    # clearance still measured >0.205 m once the lookahead was folded in, so
    # every run classified as ``A-lag`` and the clamp looked exonerated when it
    # is in fact saturated at half of all legal sign/colour combinations.
    routing = sign_router_module.ROUTING_TABLE.get((last["corridors"][committed], last["direction"]))  # noqa: SLF001
    if routing is None:
        return "A-other"
    axis = 1 if routing.axis is Axis.Y else 0
    sign_lat = (sign.x, sign.y)[axis]
    target_lat = abs(deformed[axis] - sign_lat)
    robot_lat = abs(last["pos"][axis] - sign_lat)
    if target_lat < _PASS_CLEARANCE:
        return "A-clamped"
    return "A-lag" if robot_lat < target_lat else "A-other"


def _job(args: tuple[int, str | None, bool]) -> Verdict:
    index, fixtures, blind = args
    return _classify(index, Path(fixtures) if fixtures else None, blind)


def main() -> None:
    """Classify every failing scenario and report the split."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", action="store_true", help="use the pinned-seed 256 corpus")
    parser.add_argument("--sighted", action="store_true", help="run sighted instead of the blind competition config")
    parser.add_argument("--yaw", action="store_true", help="also break A-clamped down by chassis yaw at the fatal tick")
    parser.add_argument("--approach", action="store_true", help="also report how the approach to the fatal sign went")
    parser.add_argument(
        "--by-sign-pair",
        action="store_true",
        help="split the collision rate by how the hardest same-section sign PAIR is "
        "arranged. A same-colour pair on both laterals is the case the pass-side rule "
        "cannot spread out, and the router deforms off one sign only.",
    )
    parser.add_argument("--workers", type=int, default=_DEFAULT_WORKERS)
    args = parser.parse_args()

    fixtures = CORPUS_DIR if args.corpus else None
    blind = not args.sighted
    count = len(all_obstacles_demo_scenarios(fixtures))
    jobs = [(i, str(fixtures) if fixtures else None, blind) for i in range(count)]

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        verdicts = list(pool.map(_job, jobs))
    tally = Counter(v.kind for v in verdicts)

    mode = "BLIND (competition)" if blind else "sighted"
    print(f"\nFAILURE SPLIT over {count} scenarios -- {mode}")
    collisions = sum(v for k, v in tally.items() if k != "no collision")
    for kind, n in tally.most_common():
        share = "" if kind == "no collision" else f"  ({100 * n / collisions:.0f}% of collisions)"
        print(f"  {kind:<14} {n:>4}{share}")
    a = sum(v for k, v in tally.items() if k.startswith("A-"))
    b = sum(v for k, v in tally.items() if k.startswith("B-"))
    if collisions:
        print(f"\n  Mode A (deformation ran, insufficient): {a}/{collisions} ({100 * a / collisions:.0f}%)")
        print(f"  Mode B (deformation absent):            {b}/{collisions} ({100 * b / collisions:.0f}%)")

    if args.by_sign_pair:
        scenarios = all_obstacles_demo_scenarios(fixtures)
        split: dict[str, list[int]] = defaultdict(lambda: [0, 0])
        for scenario, verdict in zip(scenarios, verdicts, strict=False):
            cell = split[_sign_pair_kind(scenario.metadata)]
            cell[0] += 1
            cell[1] += int(verdict.kind != "no collision")
        print()
        print("BY SIGN-PAIR ARRANGEMENT -- does a same-colour pair cost more?")
        print(f"  {'arrangement':<28}{'n':>6}{'collisions':>12}{'rate':>9}")
        for kind, (n, bad) in sorted(split.items(), key=lambda kv: -kv[1][0]):
            print(f"  {kind:<28}{n:>6}{bad:>12}{bad / n:>9.1%}")

    if args.yaw:
        _report_yaw([v for v in verdicts if v.frames])
    if args.approach:
        _report_approach([v for v in verdicts if v.approach is not None])


def _sign_pair_kind(metadata: object) -> str:
    """How the hardest sign PAIR in this scenario is arranged, or "none".

    The WRO lattice puts at most two signs in a section, at lateral 0.4 or 0.6
    of a 1.0 m corridor. A pair of the SAME COLOUR on BOTH laterals is the case
    the pass-side rule cannot spread out: red is passed outward and green
    inward regardless of travel direction, so both signs push the chassis to
    the same edge and the gap it must thread is what is left over. Worth
    splitting the failures on because `deform_waypoint` commands off exactly
    ONE sign -- the nearest -- and has no notion of two bounding a gap.
    """

    # The catalog hands scenarios their metadata as a plain dict, but callers
    # elsewhere pass the parsed model, so read either rather than assuming.
    def _get(obj: object, name: str, default: object = None) -> object:
        return obj.get(name, default) if isinstance(obj, dict) else getattr(obj, name, default)

    by_section: dict[str, list[tuple[str, float]]] = defaultdict(list)
    for spec in _get(metadata, "sign_positions", []) or []:
        x = float(_get(spec, "x", 0.0))
        y = float(_get(spec, "y", 0.0))
        raw_colour = _get(spec, "color", "")
        colour = str(getattr(raw_colour, "value", raw_colour))
        if y <= 0.6:
            by_section["south"].append((colour, y))
        elif y >= 2.4:
            by_section["north"].append((colour, 3.0 - y))
        elif x <= 0.6:
            by_section["west"].append((colour, x))
        elif x >= 2.4:
            by_section["east"].append((colour, 3.0 - x))
    best = "none"
    for items in by_section.values():
        if len(items) < 2:
            continue
        colours = {c for c, _ in items}
        laterals = {round(lat, 2) for _, lat in items}
        if len(colours) == 1 and len(laterals) > 1:
            return f"{next(iter(colours))} pair, both laterals"
        if best == "none":
            best = "same-colour, one lateral" if len(colours) == 1 else "mixed pair"
    return best


def _report_approach(tracked: list[Verdict]) -> None:
    """Report whether the commanded line held still long enough to be followed."""
    print(f"\nTHE APPROACH -- {len(tracked)} collisions with a resolvable committed run")
    if not tracked:
        return

    runs = [v.approach for v in tracked if v.approach]
    engage = median([a.engage_distance_m for a in runs])
    activation = NavigationTuning.load_default().sign_router.ACTIVATION_DIST_M
    print(f"  first committed at:                median {engage:.2f} m  (activation radius {activation:.2f} m)")
    print(f"  ticks IN RANGE of the sign:        median {median([float(a.ticks_in_range) for a in runs]):.0f}")
    print(f"  ticks committed IN TOTAL:          median {median([float(a.committed_ticks_total) for a in runs]):.0f}")
    print(f"    i.e. deforming for {100 * median([a.engaged_fraction for a in runs]):.0f}% of the approach")
    print(f"  ticks in the final UNBROKEN run:   median {median([float(a.ticks) for a in runs]):.0f}")
    print(f"  dropouts (engaged, let go, re-took): median {median([float(a.dropouts) for a in runs]):.0f}")

    # Both frames, because the axis one is what every earlier figure used and a
    # correction nobody can see the size of is not a correction.
    straight = [a for a in runs if a.on_a_straight]
    corner = [a for a in runs if not a.on_a_straight]
    print(
        f"\n  path swept during the approach:    median {math.degrees(median([a.approach_turn_rad for a in runs])):.0f} deg"
    )
    print(
        f"    on a STRAIGHT (<{_STRAIGHT_TURN_DEG:.0f} deg): {len(straight)}/{len(runs)}   MID-CORNER: {len(corner)}/{len(runs)}"
    )

    subsets = [("all approaches", runs)]
    if straight and corner:
        # The straight subset is the wiring check. With the path parallel to the
        # corridor axis the two frames differ only by a constant origin, and
        # every figure here is a difference, so a residual disagreement is
        # either a miswired path frame or the axis flip above resolving alternate
        # ticks against the wrong global coordinate. The flip shows up here and
        # in nothing else, which is the only reason it is worth printing.
        subsets += [
            ("straight approaches (frames agree once the flip is out)", straight),
            ("mid-corner approaches", corner),
        ]
    for name, subset in subsets:
        print(f"\n  {name} -- {len(subset)}")
        for frame, label in ((lambda a: a.path, "path"), (lambda a: a.axis, "axis")):
            print(f"    {label} frame")
            _report_convergence(subset, frame, indent="      ")

    # Can pure pursuit act at all? It needs a target AHEAD to turn lateral error
    # into heading; pin_depth holds the commanded point level with the sign.
    print(f"\n  target LEAD at engage:             median {1000 * median([a.lead_start_m for a in runs]):.0f} mm")
    print(f"  target LEAD at impact:             median {1000 * median([a.lead_end_m for a in runs]):.0f} mm")
    abeam = [a.abeam_fraction for a in runs]
    print(
        f"  share of approach with target ABEAM (<{100 * _ABEAM_LEAD_M:.0f} cm ahead): median {100 * median(abeam):.0f}%"
    )
    mostly_abeam = sum(1 for a in abeam if a > _MOSTLY)
    print(f"    runs abeam for >{100 * _MOSTLY:.0f}% of the approach: {mostly_abeam}/{len(runs)}")

    # The discriminator. If the line moves about as far as the chassis manages to
    # close, the chassis is chasing a moving target and no pursuit-side knob
    # (lookahead, gain, arc radius, speed) can be expected to fix it.
    #
    # Adequacy judged in the path frame: the axis frame overstates the yaw of a
    # chassis that is merely cornering, and an overstated yaw inflates the
    # clearance the line is said to have needed.
    adequate = [v.approach for v in tracked if v.path and v.path.line_was_adequate and v.approach]
    if adequate:
        print(f"\n  on the {len(adequate)} with an ADEQUATE line at the held yaw (path frame):")
        travel = median([a.path.line_travel_m for a in adequate])
        closed_ok = median([a.path.closed_m for a in adequate])
        print(f"    line travel {1000 * travel:.0f} mm vs error closed {1000 * closed_ok:.0f} mm")


def _report_convergence(runs: list[Approach], frame: Callable[[Approach], OffsetSeries], indent: str = "  ") -> None:
    """Print the line-versus-chassis convergence of ``runs`` in one frame."""
    series = [frame(a) for a in runs]
    # First, because it bounds what the rest can mean: a line jumping tick to
    # tick is not one line the chassis failed to reach, it is two.
    print(f"{indent}commanded line JUMPED per tick:    median {1000 * median([s.chatter_m for s in series]):.0f} mm")
    print(
        f"{indent}commanded line MOVED by:           median {1000 * median([s.line_travel_m for s in series]):.0f} mm"
    )
    print(
        f"{indent}cross-track error at engage:       median {1000 * median([s.error_start_m for s in series]):.0f} mm"
    )
    print(f"{indent}cross-track error at impact:       median {1000 * median([s.error_end_m for s in series]):.0f} mm")
    closed = [s.closed_m for s in series]
    print(f"{indent}error actually CLOSED:             median {1000 * median(closed):.0f} mm")
    diverged = sum(1 for c in closed if c < 0)
    print(f"{indent}  runs where the error GREW: {diverged}/{len(runs)} ({100 * diverged / len(runs):.0f}%)")


def _report_yaw(mode_a: list[Verdict]) -> None:
    """Report what the chassis was doing at the fatal tick, and what it implies."""
    print(f"\nYAW AT THE FATAL TICK -- {len(mode_a)} Mode A collisions")
    if not mode_a:
        return

    print(f"  a square pass needs {_SQUARE_CLEARANCE:.3f} m centre-to-centre")

    # How far apart the two frames are pointing. Non-zero means the sign's
    # corridor label is not parallel to the path the robot is on, so the axis
    # frame's "lateral" offset is partly an along-track distance -- the same
    # inflation that made ``A-clamped`` unreachable when this diagnostic
    # compared Euclidean gaps, arriving by a different route.
    skews = sorted(abs(v.path.yaw_deg - v.axis.yaw_deg) for v in mode_a if v.path and v.axis)
    if skews:
        badly = sum(1 for s in skews if s > _SKEW_DEG)
        print(f"  frame SKEW (corridor axis vs path): median {median(skews):.1f} deg, max {skews[-1]:.1f} deg")
        print(f"    over {_SKEW_DEG:.0f} deg, where the axis frame measures mostly along-track: {badly}/{len(skews)}")

    # Reported in both frames throughout. Two-thirds of legal sign positions sit
    # on a corner, where a chassis square to the arc it is driving reads as
    # heavily yawed against the corridor axis -- and every degree of that
    # inflates the clearance the pass is then said to have needed. The axis
    # column is kept so the size of the correction is visible.
    for name in ("path frame", "axis frame"):
        frames = [f for v in mode_a for label, f in v.frames if label == name]
        if not frames:
            continue
        print(f"\n  {name.upper()} -- {len(frames)} of {len(mode_a)} resolvable")
        _report_frame(frames)


def _report_frame(frames: list[Frame]) -> None:
    """Report the yaw histogram and the line-adequacy split for one frame."""
    yaws = sorted(f.yaw_deg for f in frames)
    print(f"    yaw at the fatal tick: median {median(yaws):.1f} deg, max {yaws[-1]:.1f} deg")
    lo = 0.0
    for edge in (*_YAW_BUCKETS_DEG, 90.0):
        n = sum(1 for y in yaws if lo <= y < edge)
        print(f"      {lo:>4.0f}-{edge:<4.0f} deg  {n:>4}  ({100 * n / len(yaws):.0f}%)")
        lo = edge

    # The question item 2b actually turns on: was the commanded line the problem?
    adequate = [f for f in frames if f.line_was_adequate]
    short = [f for f in frames if not f.line_was_adequate]
    print(
        f"    line ADEQUATE at the held yaw: {len(adequate)}/{len(frames)} ({100 * len(adequate) / len(frames):.0f}%)"
    )
    print("      the geometry was there and the chassis was not on it -- a tracking failure")
    if adequate:
        errs = sorted(f.tracking_error_m for f in adequate)
        print(f"      median offset from the commanded line: {1000 * median(errs):.0f} mm")
        wrong_side = sum(
            1 for f in adequate if (f.target_offset_m - f.sign_offset_m) * (f.robot_offset_m - f.sign_offset_m) < 0
        )
        print(f"      of which on the WRONG SIDE of the sign entirely: {wrong_side}/{len(adequate)}")
    print(f"    line SHORT at the held yaw:    {len(short)}/{len(frames)} ({100 * len(short) / len(frames):.0f}%)")
    if short:
        recoverable = sum(1 for f in short if f.recoverable_by_squaring)
        print(f"      of which squaring would fix: {recoverable}/{len(short)}")
        deficits = sorted(f.needed_at_yaw_m - f.clearance_m for f in short)
        print(f"      median shortfall: {1000 * median(deficits):.0f} mm")


if __name__ == "__main__":
    main()

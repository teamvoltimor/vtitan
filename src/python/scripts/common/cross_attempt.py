r"""Bag-replay crossing-pass reconstruction, shared by the crossing diagnostics.

``diag_bag_cross_attempt`` replayed the shipped router over one bag and kept
every tick of every committed pillar, then classified each pillar into
ROUTING / EXECUTION / ok. ``diag_bag_reverse_budget`` and
``diag_bag_lane_reconstruct`` imported its private ``_classify``/``_replay``
(and ``Pillar``) to build on that reconstruction instead of replaying the
router a third and fourth time. The records and the two functions are one API;
they live here so the replay has a single definition.

The pass-side convention is used throughout: positive is the legal side of the
sign. ``lane_off`` is the recorded ``steer_target``'s lateral offset (what the
laned path actually commanded), ``deform_off`` the router's deformed target
(which the shipped navigator discards under ``SIGN_LANE_SUPPRESS_DEFORM``), and
``pose_off`` where the chassis really was.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.domain.enums import Axis, Direction
from shared.domain.models import Pose, SignColor

from scripts.common.bag_io import decode_detections, scan_to_ranges_angles, settled_direction
from scripts.common.stats import nearest_by_time
from src.navigation.planning.sign_discovery import detection_to_observation
from src.navigation.planning.sign_router import SignRouter
from src.navigation.planning.sign_router.routing import clamp_lateral, pass_side_lateral_axis

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.models import NavigatorDebugSnapshot


@dataclass
class Tick:
    """One tick of a pass, raw. Projected onto the pass-side axis only at the end."""

    rel: float
    rng: float | None
    committed: bool
    pose_x: float
    pose_y: float
    yaw: float
    steer_x: float | None
    steer_y: float | None
    deform_x: float | None
    deform_y: float | None
    steering: float | None
    maneuver: str | None
    man_steering: float | None
    speed: float | None
    crosstrack: float | None
    lap: int


@dataclass
class Pillar:
    run: str
    ticks: list[Tick] = field(default_factory=list)
    colour: SignColor = SignColor.UNKNOWN
    corridor: object = None
    sign_x: float = 0.0
    sign_y: float = 0.0
    best_rng: float = 1e9
    lane_specs: list = field(default_factory=list)
    """router.lane_specs as it stood at closest approach -- what
    apply_sign_lanes would have been handed on that tick."""


def replay(
    run: str,
    rows: list[tuple[float, NavigatorDebugSnapshot]],
    frames: list[tuple[float, list[dict]]],
    scans: list[tuple[float, object]],
    tuning: NavigationTuning,
    per_lap: bool = False,
) -> tuple[list[Pillar], Direction]:
    """Replay the shipped router over one bag, keeping every tick of every pass.

    per_lap keys a pillar by (position, lap) instead of position alone. The
    shipped diag_bag_pass_side keys by position, so one physical pillar
    passed on three laps collapses into ONE verdict taken at its best approach
    -- which is what makes the default here reproduce that script's counts, and
    also what makes the default UNABLE to test anything lap-dependent. Under
    per_lap each lap's pass is judged on its own, which is the only form in
    which 'the lane is already built from lap 2' is a testable claim.
    """
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    frame_i = 0
    scan_times = [t for t, _ in scans]
    pillars: dict[tuple[float, float], Pillar] = {}
    orphans: list[Tick] = []

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            obs.extend(
                o
                for det in decode_detections(frames[frame_i][1])
                if (o := detection_to_observation(det, pose, tuning, ranges, angles)) is not None
            )
            frame_i += 1
        if d.steer_target_x is None or d.steer_target_y is None:
            # Manoeuvre-owned (or otherwise pre-router) tick. Parked here and
            # folded back into whichever pass's window it falls inside, below.
            orphans.append(
                Tick(
                    rel=rel,
                    rng=None,
                    committed=False,
                    pose_x=d.pose_x,
                    pose_y=d.pose_y,
                    yaw=d.pose_yaw,
                    steer_x=None,
                    steer_y=None,
                    deform_x=None,
                    deform_y=None,
                    steering=d.commanded_steering_norm,
                    maneuver=str(d.active_maneuver_type) if d.active_maneuver_type is not None else None,
                    man_steering=d.maneuver_steering,
                    speed=d.commanded_speed_mps,
                    crosstrack=d.crosstrack_error_m,
                    lap=d.laps_completed,
                )
            )
            continue
        deformed = router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )
        committed = router.committed_sign_position
        if committed is None:
            continue
        rng = math.hypot(committed.x - d.pose_x, committed.y - d.pose_y)
        key = (round(committed.x, 1), round(committed.y, 1), d.laps_completed if per_lap else 0)
        p = pillars.setdefault(key, Pillar(run=run))
        p.ticks.append(
            Tick(
                rel=rel,
                rng=rng,
                committed=True,
                pose_x=d.pose_x,
                pose_y=d.pose_y,
                yaw=d.pose_yaw,
                # The bag's OWN recorded steer target: what steering really chased.
                steer_x=d.steer_target_x,
                steer_y=d.steer_target_y,
                # The replayed deformation, which the shipped navigator discards
                # under SIGN_LANE_SUPPRESS_DEFORM. Recorded to price what the
                # suppressed branch would have commanded on the same tick.
                deform_x=deformed[0],
                deform_y=deformed[1],
                steering=d.commanded_steering_norm,
                maneuver=str(d.active_maneuver_type) if d.active_maneuver_type is not None else None,
                man_steering=d.maneuver_steering,
                speed=d.commanded_speed_mps,
                crosstrack=d.crosstrack_error_m,
                lap=d.laps_completed,
            )
        )
        if rng < p.best_rng:
            p.best_rng = rng
            p.sign_x, p.sign_y = committed.x, committed.y
            p.corridor = d.current_corridor
            p.lane_specs = [(spec.color, spec.x, spec.y, corr) for spec, corr in router.lane_specs]
            p.colour = next(
                (s.color for s in router.signs if abs(s.x - committed.x) < 1e-9 and abs(s.y - committed.y) < 1e-9),
                SignColor.UNKNOWN,
            )
    for p in pillars.values():
        p.ticks.sort(key=lambda t: t.rel)
        if not p.ticks:
            continue
        closest = min(p.ticks, key=lambda t: t.rng)
        lo, hi = p.ticks[0].rel, closest.rel
        p.ticks.extend(t for t in orphans if lo <= t.rel <= hi)
        p.ticks.sort(key=lambda t: t.rel)
    return list(pillars.values()), direction


@dataclass
class Row:
    """One pillar, classified and traced."""

    run: str
    verdict: str
    crossing: bool
    commit_rng: float
    commit_off: float
    n_approach: int
    duration_s: float
    path_m: float
    lane_ticks: int
    lane_max: float
    lane_at_commit: float
    asked_ticks: int
    """Ticks where the LANE asked for a crossing the chassis had not made: steer
    target on the legal side by > thresh while the pose was still wrong-side.
    The literal answer to 'did the planner ask'."""
    deform_ticks: int
    deform_max: float
    steer_toward: int
    steer_run: int
    steer_integral: float
    steer_sat: int
    """Approach ticks at |steering| > 0.9 -- the actuator at its stop."""
    steer_late: float
    """Toward-legal fraction over the LAST 20% of the approach."""
    man_ticks: int
    man_hidden: int
    """Approach ticks the escape layer owned OUTRIGHT (no steer target at all)."""
    man_types: dict[str, int]
    man_opposes: int
    reverse_ticks: int
    pose_gain: float
    final_off: float
    mean_speed: float
    lap: int
    lane_block: str
    """Which gate inside apply_sign_lanes would have refused this sign's
    corridor, evaluated on the router state at closest approach."""
    asked_first_rng: float | None
    """Range at which the plan FIRST asked for the cross -- the runway it left."""


def classify(p: Pillar, direction: Direction, lane_thresh: float) -> Row | None:
    rule = pass_side_lateral_axis(p.corridor, p.colour, direction)
    committed = [t for t in p.ticks if t.committed]
    if rule is None or not committed:
        return None
    axis, want = rule
    idx = 0 if axis is Axis.X else 1
    sign_lat = p.sign_x if idx == 0 else p.sign_y

    def lat(x: float, y: float) -> float:
        return ((x if idx == 0 else y) - sign_lat) * want

    closest = min(committed, key=lambda t: t.rng)
    first = committed[0]
    achieved = lat(closest.pose_x, closest.pose_y)
    commanded = lat(closest.deform_x, closest.deform_y)
    verdict = "routing" if commanded < 0 else ("execution" if achieved < 0 else "ok")

    approach = [t for t in p.ticks if first.rel <= t.rel <= closest.rel]
    n = len(approach)
    path_m = sum(
        math.hypot(b.pose_x - a.pose_x, b.pose_y - a.pose_y) for a, b in zip(approach, approach[1:], strict=False)
    )

    lane_offs = [lat(t.steer_x, t.steer_y) for t in approach if t.steer_x is not None]
    deform_offs = [lat(t.deform_x, t.deform_y) for t in approach if t.deform_x is not None]
    asked = sum(
        1
        for t in approach
        if t.steer_x is not None and lat(t.steer_x, t.steer_y) > lane_thresh and lat(t.pose_x, t.pose_y) < 0
    )

    def wants_left_of(t: Tick) -> float:
        """Legal-side world direction projected onto the chassis's own left."""
        left_x, left_y = -math.sin(t.yaw), math.cos(t.yaw)
        legal = (float(want), 0.0) if idx == 0 else (0.0, float(want))
        return legal[0] * left_x + legal[1] * left_y

    toward = 0
    best_run = run_len = 0
    integral = 0.0
    sat = 0
    late_toward = late_n = 0
    late_cut = int(n * 0.8)
    for i, t in enumerate(approach):
        if t.steering is None:
            run_len = 0
            continue
        if abs(t.steering) > 0.9:
            sat += 1
        wl = wants_left_of(t)
        # A zero steering command votes for neither -- it is not a side.
        if wl == 0.0 or t.steering == 0.0:
            run_len = 0
            continue
        integral += t.steering * (1.0 if wl > 0 else -1.0)
        hit = (t.steering > 0) == (wl > 0)
        if i >= late_cut:
            late_n += 1
            late_toward += int(hit)
        if hit:
            toward += 1
            run_len += 1
            best_run = max(best_run, run_len)
        else:
            run_len = 0

    man_types: dict[str, int] = {}
    man_opposes = 0
    for t in approach:
        if t.maneuver is None:
            continue
        man_types[t.maneuver] = man_types.get(t.maneuver, 0) + 1
        if t.man_steering:
            wl = wants_left_of(t)
            if wl != 0.0 and (t.man_steering > 0) != (wl > 0):
                man_opposes += 1

    speeds = [abs(t.speed) for t in approach if t.speed is not None]
    return Row(
        run=p.run,
        verdict=verdict,
        crossing=lat(first.pose_x, first.pose_y) < 0,
        commit_rng=first.rng,
        commit_off=lat(first.pose_x, first.pose_y),
        n_approach=n,
        duration_s=closest.rel - first.rel,
        path_m=path_m,
        lane_ticks=sum(1 for v in lane_offs if v > lane_thresh),
        lane_max=max(lane_offs) if lane_offs else 0.0,
        lane_at_commit=lane_offs[0] if lane_offs else 0.0,
        asked_ticks=asked,
        deform_ticks=sum(1 for v in deform_offs if v > lane_thresh),
        deform_max=max(deform_offs) if deform_offs else 0.0,
        steer_toward=toward,
        steer_run=best_run,
        steer_integral=integral,
        steer_sat=sat,
        steer_late=(late_toward / late_n) if late_n else 0.0,
        man_ticks=sum(1 for t in approach if t.maneuver is not None),
        man_hidden=sum(1 for t in approach if not t.committed),
        man_types=man_types,
        man_opposes=man_opposes,
        reverse_ticks=sum(1 for t in approach if t.speed is not None and t.speed < 0),
        pose_gain=achieved - lat(first.pose_x, first.pose_y),
        final_off=achieved,
        mean_speed=sum(speeds) / len(speeds) if speeds else 0.0,
        lap=first.lap,
        lane_block=_lane_block(p, direction),
        asked_first_rng=next(
            (
                t.rng
                for t in approach
                if t.steer_x is not None and lat(t.steer_x, t.steer_y) > lane_thresh and lat(t.pose_x, t.pose_y) < 0
            ),
            None,
        ),
    )


def _lane_block(p: Pillar, direction: Direction) -> str:
    """Re-run apply_sign_lanes' own gates for this pillar and name the blocker.

    Only the gates that can be evaluated from router state are checked; the
    waypoint-level ones (indices empty, shifted == lateral) need the
    planned path, which the bag does not carry. So a verdict of built means
    'no ROUTER-side gate refused it', not 'the lane definitely moved' -- which
    is exactly why it is reported beside the measured lane offset rather than
    instead of it.
    """
    same = [e for e in p.lane_specs if e[3] == p.corridor]
    if not same:
        return "sign not in lane_specs"
    # apply_sign_lanes takes the AXIS from a colour-keyed lookup on an
    # ARBITRARY member of the corridor (corridor_signs[0]), even though the
    # axis itself depends only on the corridor. An UNKNOWN first member
    # therefore drops the whole corridor's lane, including identified signs.
    if pass_side_lateral_axis(p.corridor, SignColor(same[0][0]), direction) is None:
        return "corridor axis lookup refused (first sign UNKNOWN)"
    own = next((e for e in same if abs(e[1] - p.sign_x) < 1e-9 and abs(e[2] - p.sign_y) < 1e-9), None)
    if own is None:
        return "sign not in its own corridor group"
    rule = pass_side_lateral_axis(p.corridor, SignColor(own[0]), direction)
    if rule is None:
        return "own colour UNKNOWN (no plateau)"
    axis, mult = rule
    sign_lat = own[1] if axis is Axis.X else own[2]
    target = clamp_lateral(sign_lat + mult * 0.28, p.corridor)
    if (target - sign_lat) * mult <= 0.0:
        return "clamp put the target on the forbidden side"
    return "built"

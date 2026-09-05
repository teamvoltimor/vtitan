"""Probe the IN-BAY start — the legal start no corpus scenario exercises.

The rules allow two starts: inside the parking lot, or parallel to it in the
same section. **Every one of the 256 corpus scenarios uses the second.** Verified
here as well as in the corpus: the along-corridor offset between start and bay is
0.0 in every scenario, the start sits on the corridor centreline and the bay
against the outer wall, so the two differ only in the across-corridor coordinate.

That makes the in-bay start invisible to every sweep in the repo, which is why it
went unnoticed that a robot placed there **never moves** (8/8 probed, dist=0.00m,
stuck): forward reads 0.05-0.19m against a parking fin, below the gate that
authorises the initial creep, and there is no rear sensing to reverse on
(``compute_rear_clearance`` fails open, there is no rear slot).

This script places the robot in the pocket and reports what it does. It ALWAYS
runs the parallel start as a control on the same scenarios -- the in-bay
diagnosis previously shipped a wrong cause precisely because the working case was
not run through the same probe, and ``not_yet_stepped`` at tick 1 turned out to
be snapshot lag rather than a state. Read the two arms side by side.

The bay pose is derived, not hardcoded: the lot's two magenta fins stand
perpendicular to the outer wall at ``parking_lot.block1_position`` and
``block2_position``, so their midpoint is the pocket centre. Heading is taken
from the scenario's own start, which is already parallel to the wall.

Usage::

    PYTHONPATH="." pixi run -e dev python scripts/sim/diag_bay_start.py --limit 8
    PYTHONPATH="." pixi run -e dev python scripts/sim/diag_bay_start.py --corpus --limit 32
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, ParkingLotSpecs, RobotSpecs  # noqa: E402
from shared.domain.models import ScenarioMetadata  # noqa: E402

from scripts.common.diag_base import print_pool_progress, resolve_jobs, run_pool  # noqa: E402
from scripts.common.sim_defaults import CORPUS_DIR  # noqa: E402
from src.config.tuning_helpers import tuning_with_overrides  # noqa: E402
from src.navigation.track_geometry import parking_bay_centre  # noqa: E402
from src.navigation.utils import wrap_angle  # noqa: E402
from src.simulation.scenario_simulator import ScenarioSimulator  # noqa: E402
from src.simulation.track_model import ObstacleBox  # noqa: E402

if TYPE_CHECKING:
    from collections.abc import Sequence

    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

_COMMITTED_DIR = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"
# Seconds of continuous wall contact allowed before the run is judged a real
# failure rather than a chassis still working itself clear. Only meaningful
# with solid walls; the escape being measured IS a sustained scrape.
_CONTACT_GRACE_S = 8.0
# Below this separation the bay and the parallel start coincide on the axis
# perpendicular to the wall, leaving "out of the bay" without a direction.
_DEGENERATE_AXIS_M = 1e-6
# Clearance at or below which the chassis counts as TOUCHING a fin. Not zero:
# once the fins are solid the chassis is stopped AT the surface rather than
# through it, so the measured gap settles on a hair of floating-point positive.
# A judge calls that contact and 9.24.7 ends the round for it, so scoring it as
# clear would replace one false negative (ghosting through) with another.
_TOUCH_EPSILON_M = 1e-4


@dataclass(frozen=True, slots=True)
class BayStartRow:
    """One scenario's outcome from one start, as produced by ``_run_case``."""

    id: str
    skipped: bool
    moved: bool = False
    fx: float = 0.0
    fy: float = 0.0
    dyaw_deg: float = 0.0
    net_m: float = 0.0
    exit_m: float | None = None
    bex: int = 0
    rev_ticks: int = 0
    fwd_ticks: int = 0
    rev_m: float = 0.0
    flips: int = 0
    dist: float = 0.0
    laps: int = 0
    collided: bool = False
    stuck: bool = False
    timed_out: bool = False
    pass_side: bool = False
    fin_m: float | None = None
    """Smallest measured clearance to a lot fin DURING the bay-exit phase.

    The whole legality question in one number. 9.24.7 ends the round when the
    robot touches the parking lot limitations, and both shipped exits are built
    around a stall detector that fires ON contact -- so whether the manoeuvre is
    legal is not a matter of which knob is set but of whether the chassis ever
    reaches a fin while escaping. Measured over the exit ticks only; the parking
    phase at the END of a round legitimately approaches the lot and would swamp
    a whole-run minimum. ``None`` when the scenario has no lot or the exit never
    ran.
    """
    hand_yaw_deg: float | None = None
    """Chassis yaw at the tick the manoeuvre HANDED OVER, against the start heading.

    The release pose, which is what the outcome actually turns on. Outward
    displacement was excluded as the discriminator on 2026-09-04: every arc
    releases at ~0.07 m out, and gating on that made the exit strictly worse
    (7/8 -> 0/8, all stuck), because the ratchet cannot finish alone -- the
    navigator drives the last of it. So what separates a run that laps from one
    that dies inside 0.35 m has to be the rest of the pose, and heading is the
    part the wall clip was holding.
    """
    hand_out_m: float | None = None
    """Outward displacement at handover, on the bay's outward axis."""

    sign_color: str = ""
    """Colour of the first sign AHEAD of the chassis at handover, or "" if none is.

    The remaining discriminator, by elimination. The release pose is identical
    in all 256 scenarios -- yaw 25.5-25.6 deg, displacement -0.200 m -- and the
    bay sits at a fixed place on the mat, so the chassis is handed over in the
    SAME position and heading every time. Nothing about the manoeuvre can
    therefore explain why 44 runs die on a pass-side violation inside the first
    metre and 210 do not. What differs between scenarios is the layout ahead,
    and the first sign is what the router commits a lane for.
    """
    sign_range_m: float | None = None
    """Distance to that sign at handover. How much room the router had to work with."""
    sign_lat_m: float | None = None
    """Its lateral offset from the chassis's heading axis, POSITIVE TO THE LEFT.

    Colour alone cannot say whether the required side was the reachable one:
    the rule is travel-relative (see the 2026-09-03 pass-side correction), so
    what matters is the colour together with which side the sign already sits
    on when the navigator takes over.
    """

    viol_sign: int | None = None
    """Index of the sign that ENDED the run under 9.24.5, or ``None``."""
    viol_ahead_m: float | None = None
    """That sign's along-corridor offset from the START, positive in the travel direction.

    The decisive number for the in-bay arm. The scorer fires when the chassis
    COMPLETELY crosses a sign's radius -- the line across the corridor at the
    sign -- while on the forbidden side, and it only tests signs within
    ``_PASS_SIDE_APPROACH_M`` (1.20 m). The bay sits at the same along-corridor
    depth as the parallel start and differs from it only ACROSS the corridor,
    against the outer wall. So a sign whose radius lies at or behind the start
    is one the in-bay chassis crosses from the outer side without ever driving
    up to it, while the control crosses the same line from the centreline. A
    value near zero or negative means the run was lost to where the bay IS, not
    to anything the navigator did.
    """
    viol_lat_m: float | None = None
    """Its lateral offset from the start axis, positive LEFT. Which side had to be taken."""

    surface: str = ""
    """Which surface ENDED the run, or "" if contact did not end it.

    A bare ``collided`` flag cannot decide whether an in-bay start is worth its
    7 points, because the rules do not treat contacts alike: 9.18 lets the
    vehicle touch a wall it does not move and continue, while 9.24.7 ends the
    round on the parking lot. Counting them together reads a legal brush as a
    failure.
    """


def bay_outward_axis(
    centre: tuple[float, float],
    start_xy: tuple[float, float],
    start_yaw: float,
) -> tuple[float, float] | None:
    """Unit vector from the pocket towards the corridor, perpendicular to the wall.

    Derived, not hardcoded: the scenario's own (parallel) start sits on the
    corridor centreline and the bay against the outer wall, so the bay-to-start
    vector points out of the pocket. Its along-corridor part is 0.0 in every
    corpus scenario, but it is projected out anyway rather than trusted -- the
    axis this measurement is taken on must be the one perpendicular to the wall
    even if a future generator offsets the start along the corridor.

    Returns ``None`` when the two coincide on that axis, which would leave the
    direction undefined; the caller then reports no exit verdict rather than a
    guessed one.
    """
    vx, vy = start_xy[0] - centre[0], start_xy[1] - centre[1]
    along = vx * math.cos(start_yaw) + vy * math.sin(start_yaw)
    px, py = vx - along * math.cos(start_yaw), vy - along * math.sin(start_yaw)
    norm = math.hypot(px, py)
    if norm < _DEGENERATE_AXIS_M:
        return None
    return px / norm, py / norm


def bay_exit_clearance(
    pose: tuple[float, float, float],
    centre: tuple[float, float],
    outward: tuple[float, float],
) -> float:
    """Metres by which the whole chassis footprint clears the pocket mouth.

    The two fins stand ``ParkingLotSpecs.LENGTH`` (0.20 m) out from the outer
    wall, and the pocket centre sits ``WALL_OFFSET`` (0.10 m) from it, so the
    mouth is 0.10 m outboard of where the chassis is placed. The verdict is
    taken on the footprint CORNER nearest the wall, at the final heading, not on
    the centre: a chassis that has driven its nose out while its tail is still
    between the fins has not left the bay, and a centre-only test would call it
    out. Negative means still (partly) in the pocket.
    """
    x, y, yaw = pose
    hx, hy = math.cos(yaw), math.sin(yaw)
    half_l, half_w = RobotSpecs.LENGTH / 2.0, RobotSpecs.WIDTH / 2.0
    nearest = min(
        ParkingLotSpecs.WALL_OFFSET
        + (x + sl * half_l * hx + sw * half_w * -hy - centre[0]) * outward[0]
        + (y + sl * half_l * hy + sw * half_w * hx - centre[1]) * outward[1]
        for sl in (-1.0, 1.0)
        for sw in (-1.0, 1.0)
    )
    return nearest - ParkingLotSpecs.LENGTH


def _chassis_corners(pose: tuple[float, float, float]) -> list[tuple[float, float]]:
    """The four chassis corners at ``pose`` (world frame)."""
    x, y, yaw = pose
    hx, hy = math.cos(yaw), math.sin(yaw)
    half_l, half_w = RobotSpecs.LENGTH / 2.0, RobotSpecs.WIDTH / 2.0
    return [
        (x + sl * half_l * hx - sw * half_w * hy, y + sl * half_l * hy + sw * half_w * hx)
        for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1))
    ]


def _convex_gap(poly_a: Sequence[tuple[float, float]], poly_b: Sequence[tuple[float, float]]) -> float:
    """Lower bound on the gap between two convex polygons; negative means overlap.

    The largest separation found over both polygons' edge normals. That is the
    exact distance when the closest features are edge-to-vertex and an
    UNDER-estimate when they are vertex-to-vertex, so it never reports more
    clearance than there is -- the right direction for a margin that has to hold
    against a rule which ends the round on contact.
    """
    best = -math.inf
    for poly in (poly_a, poly_b):
        count = len(poly)
        for i in range(count):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % count]
            ax, ay = y2 - y1, x1 - x2
            norm = math.hypot(ax, ay)
            if norm == 0.0:
                continue
            ax, ay = ax / norm, ay / norm
            a_lo = min(px * ax + py * ay for px, py in poly_a)
            a_hi = max(px * ax + py * ay for px, py in poly_a)
            b_lo = min(px * ax + py * ay for px, py in poly_b)
            b_hi = max(px * ax + py * ay for px, py in poly_b)
            best = max(best, b_lo - a_hi, a_lo - b_hi)
    return best


def _fin_polygons(raw: dict) -> list[list[tuple[float, float]]]:
    """The two lot marker fins as polygons, or empty when the scenario has no lot."""
    parking = raw.get("parking_lot")
    if not parking:
        return []
    polys = []
    for pos_key, yaw_key in (("block1_position", "block1_yaw"), ("block2_position", "block2_yaw")):
        block = parking[pos_key]
        box = ObstacleBox.from_pose(
            cx=float(block["x"]),
            cy=float(block["y"]),
            length=ParkingLotSpecs.LENGTH,
            width=ParkingLotSpecs.WIDTH,
            yaw=float(parking.get(yaw_key, 0.0)),
            is_parking_lot=True,
        ).to_box()
        polys.append([(c.x, c.y) for c in box.corners()])
    return polys


def _first_sign_ahead(pose: tuple[float, float, float], signs: Sequence[dict]) -> tuple[str, float, float] | None:
    """Colour, range and signed lateral offset of the nearest sign in FRONT of ``pose``.

    "In front" is the half-plane the chassis is facing, not the nearest sign
    outright: a sign the robot has already passed cannot be the one whose lane
    the router is about to commit. Lateral offset is positive to the LEFT, in
    the chassis frame, so it reads directly against the travel-relative
    pass-side rule. ``None`` when the scenario has no sign ahead at all, which
    is a real outcome rather than missing data -- the run then cannot fail on
    9.24.5 at handover.
    """
    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    best: tuple[str, float, float] | None = None
    for sign in signs:
        dx, dy = float(sign["x"]) - x, float(sign["y"]) - y
        ahead = dx * cos_yaw + dy * sin_yaw
        if ahead <= 0.0:
            continue
        left = -dx * sin_yaw + dy * cos_yaw
        rng = math.hypot(dx, dy)
        if best is None or rng < best[1]:
            best = (str(sign.get("color", "?")), rng, left)
    return best


def _run_case(payload: tuple[str, bool, int, dict[str, float], bool, bool, bool, float, float, float]) -> BayStartRow:
    """Run one scenario from one start. Returns a row, never raises on outcome."""
    (
        path_str,
        in_bay,
        laps,
        changes,
        known_start,
        solid_walls,
        slide,
        scrub,
        bay_offset_m,
        no_progress_s,
    ) = payload
    path = Path(path_str)
    raw = json.loads(path.read_text())

    start = raw["starting_conditions"]
    centre = parking_bay_centre(raw)
    # Captured before the in-bay branch overwrites it: the parallel start is the
    # reference the outward axis is derived from, and it must be the ORIGINAL
    # one on both arms so the two report the exit on the same axis.
    parallel_xy = (start["position"]["x"], start["position"]["y"])
    moved = False
    if in_bay:
        if centre is None:
            return BayStartRow(id=raw["scenario_id"], skipped=True)
        # Heading is left alone: the scenario's own start is already parallel to
        # the outer wall, which is the only orientation the 0.20m-deep pocket
        # admits for a 0.194m-wide chassis.
        # Offset ALONG THE WALL from the pocket centre. The corpus always places
        # the chassis dead centre, but on the day the TEAM places it and 9.9
        # allows physical adjustment during preparation, so where in the pocket
        # it starts is a free parameter -- and the pocket has 65 mm of slack at
        # each end against a manoeuvre that misses by a fraction of a
        # millimetre. Positive is along the start heading.
        start["position"]["x"] = centre[0] + bay_offset_m * math.cos(start["yaw"])
        start["position"]["y"] = centre[1] + bay_offset_m * math.sin(start["yaw"])
        moved = True

    meta = ScenarioMetadata.model_validate(raw)
    # The bay ratchet nets millimetres per second by construction, so the
    # simulator's own no-progress bailout -- 0.08 m of net displacement inside
    # 30 s -- ends the run long before the manoeuvre can be judged. That bound
    # is an artefact of the harness, not a rule: 9.4 gives the round three
    # minutes. Raised only when asked for, so every other arm stays comparable
    # with every previously measured one.
    tuning = tuning_with_overrides(changes)
    if no_progress_s > 0.0:
        tuning = tuning_with_overrides({"NO_PROGRESS_WINDOW_S": no_progress_s}, group="simulation", base=tuning)
    sim = ScenarioSimulator(
        meta,
        num_laps=laps,
        tuning=tuning,
        seed=raw["scenario_id"],
        blind=True,
        known_start=known_start,
        solid_walls=solid_walls,
        slide_on_contact=slide,
        scrub_yaw_gain=scrub,
    )
    # Wall contact is legal on OBSTACLES and not on Open, so a scraping escape is
    # only a real result on this challenge -- which is also the only one with a
    # parking lot to start in. With solid walls the chassis is stopped by the
    # wall instead of passing through it, and `allowed_step` then limits the TURN
    # while keeping the translation, which is what lets a cornered chassis peel
    # away. Without it the run simply stalls short of contact, which is what
    # every earlier bay-start number here measured.
    # Tracked over the whole run, not read off the final pose: a run that leaves
    # the pocket, drives its laps and then PARKS is back inside the bay at the
    # end, and a final-pose test scores that -- the best possible outcome -- as
    # never having left. Observed on scenario 0 of the committed set: 3 laps,
    # 26 m driven, final clearance -0.00 m.
    outward = bay_outward_axis(centre, parallel_xy, start["yaw"]) if centre else None
    best_exit_m: float | None = None
    fins = _fin_polygons(raw)
    min_fin_m: float | None = None
    prev_bex = 0
    # Pose on the LAST tick the manoeuvre drove, i.e. what it handed the
    # navigator. Rewritten every such tick rather than latched on a transition,
    # because the release is a fall-through in the caller and the observer never
    # sees a "handover" event of its own.
    hand_yaw: float | None = None
    hand_out: float | None = None
    hand_sign: tuple[str, float, float] | None = None
    signs = raw.get("sign_positions") or []

    def _observe(state: AckermannState, _scan: LidarScan) -> None:
        nonlocal best_exit_m, min_fin_m, prev_bex, hand_sign
        # Only ticks the MANOEUVRE drove. `bay_exit_ticks` advances exactly while
        # it holds control, so a tick that raised it is one it is answerable for
        # -- and the parking phase later in the round is correctly excluded.
        nonlocal hand_yaw, hand_out
        bex = sim.bay_exit_ticks
        if bex > prev_bex:
            if fins:
                corners = _chassis_corners((state.x, state.y, state.yaw))
                gap = min(_convex_gap(corners, fin) for fin in fins)
                min_fin_m = gap if min_fin_m is None else min(min_fin_m, gap)
            hand_yaw = math.degrees(wrap_angle(state.yaw - start["yaw"]))
            hand_sign = _first_sign_ahead((state.x, state.y, state.yaw), signs)
            if centre is not None and outward is not None:
                hand_out = bay_exit_clearance((state.x, state.y, state.yaw), centre, outward)
        prev_bex = bex
        if centre is None or outward is None:
            return
        clearance = bay_exit_clearance((state.x, state.y, state.yaw), centre, outward)
        best_exit_m = clearance if best_exit_m is None else max(best_exit_m, clearance)

    result = sim.run(contact_grace_s=_CONTACT_GRACE_S if solid_walls else None, on_step=_observe)

    # Where it STOPPED, not just how far it went. A distance alone cannot tell
    # "never left the pocket" from "drove out, crossed the corridor and stalled
    # facing the far wall" -- and those want opposite fixes. Yaw is reported
    # against the scenario's own start heading, which is parallel to the outer
    # wall, so ~90 deg means the chassis is broadside to the corridor it is
    # meant to be driving down.
    fx, fy, fyaw = result.final_pose
    sx, sy = start["position"]["x"], start["position"]["y"]
    # Measured from the PARALLEL start on both arms, never from the bay: the two
    # differ only across the corridor, so a common origin is what makes "the
    # offending sign is level with the start" mean the same thing in each.
    viol_sign = result.pass_side_violation_signs[0] if result.pass_side_violation_signs else None
    viol_ahead: float | None = None
    viol_lat: float | None = None
    if viol_sign is not None and viol_sign < len(signs):
        offender = signs[viol_sign]
        dx = float(offender["x"]) - parallel_xy[0]
        dy = float(offender["y"]) - parallel_xy[1]
        viol_ahead = dx * math.cos(start["yaw"]) + dy * math.sin(start["yaw"])
        viol_lat = -dx * math.sin(start["yaw"]) + dy * math.cos(start["yaw"])
    return BayStartRow(
        id=raw["scenario_id"],
        skipped=False,
        moved=moved,
        fx=fx,
        fy=fy,
        dyaw_deg=math.degrees(abs(wrap_angle(fyaw - start["yaw"]))),
        net_m=math.hypot(fx - sx, fy - sy),
        exit_m=best_exit_m,
        hand_yaw_deg=hand_yaw,
        hand_out_m=hand_out,
        sign_color=hand_sign[0] if hand_sign else "",
        sign_range_m=hand_sign[1] if hand_sign else None,
        sign_lat_m=hand_sign[2] if hand_sign else None,
        viol_sign=viol_sign,
        viol_ahead_m=viol_ahead,
        viol_lat_m=viol_lat,
        bex=sim.bay_exit_ticks,
        rev_ticks=sim.bay_exit.legs[0],
        fwd_ticks=sim.bay_exit.legs[1],
        rev_m=sim.bay_exit.legs[2],
        flips=sim.bay_exit.open_flips,
        dist=result.distance_m,
        laps=result.laps_completed,
        collided=result.collided,
        stuck=result.stuck,
        timed_out=result.timed_out,
        pass_side=result.pass_side_violation,
        surface=str(result.terminal_surface.value),
        fin_m=min_fin_m,
    )


def _summarise(name: str, rows: Sequence[BayStartRow]) -> None:
    """One line per scenario, then the aggregate that actually decides it."""
    live = [r for r in rows if not r.skipped]
    if not live:
        print(f"  {name}: no scenarios with a parking lot")
        return

    print(f"\n=== {name} ===")
    print(
        f"| {'#':>4} | {'dist m':>7} | {'net m':>6} | {'exit m':>6} | {'bex':>5} | {'rev':>5} | {'fwd':>5} | "
        f"{'rev m':>6} | {'flip':>5} | {'h.yaw':>6} | {'h.out':>6} | "
        f"{'s.col':>5} | {'s.rng':>6} | {'s.lat':>6} | {'dyaw':>6} | "
        f"{'end x':>6} | {'end y':>6} | {'laps':>4} | {'coll':>4} | {'stuck':>5} | {'t/o':>3} | {'pass':>4} |"
    )
    print(
        f"|{'-' * 6}|{'-' * 9}|{'-' * 8}|{'-' * 8}|{'-' * 7}|{'-' * 7}|{'-' * 7}|{'-' * 8}|"
        f"{'-' * 8}|{'-' * 8}|{'-' * 8}|{'-' * 7}|{'-' * 8}|{'-' * 8}|"
        f"{'-' * 8}|{'-' * 8}|{'-' * 8}|{'-' * 6}|{'-' * 6}|{'-' * 7}|{'-' * 5}|{'-' * 6}|"
    )
    for r in sorted(live, key=lambda x: int(x.id)):
        exit_cell = f"{r.exit_m:>6.2f}" if r.exit_m is not None else f"{'--':>6}"
        hyaw = f"{r.hand_yaw_deg:>6.1f}" if r.hand_yaw_deg is not None else f"{'--':>6}"
        hout = f"{r.hand_out_m:>6.3f}" if r.hand_out_m is not None else f"{'--':>6}"
        scol = f"{r.sign_color[:5] or '--':>5}"
        srng = f"{r.sign_range_m:>6.2f}" if r.sign_range_m is not None else f"{'--':>6}"
        slat = f"{r.sign_lat_m:>6.2f}" if r.sign_lat_m is not None else f"{'--':>6}"
        print(
            f"| {r.id:>4} | {r.dist:>7.2f} | {r.net_m:>6.2f} | {exit_cell} | {r.bex:>5} | "
            f"{r.rev_ticks:>5} | {r.fwd_ticks:>5} | {r.rev_m:>6.3f} | {r.flips:>5} | "
            f"{hyaw} | {hout} | {scol} | {srng} | {slat} | "
            f"{r.dyaw_deg:>6.1f} | {r.fx:>6.2f} | {r.fy:>6.2f} | {r.laps:>4} | "
            f"{'Y' if r.collided else '.':>4} | {'Y' if r.stuck else '.':>5} | "
            f"{'Y' if r.timed_out else '.':>3} | {'Y' if r.pass_side else '.':>4} |"
        )

    n = len(live)
    dists = [r.dist for r in live]
    immobile = sum(1 for d in dists if d < 0.01)
    # The headline for the in-bay arm. Laps and distance both answer "how well
    # did the run go afterwards"; this answers the prior question the pocket
    # actually poses, and a run can clear the bay and then stall or collide
    # without that making the exit itself a failure. The margin is the run's
    # BEST clearance, so it says how far out it got, not where it ended.
    clear = [r for r in live if r.exit_m is not None]
    if clear:
        out = [r for r in clear if r.exit_m > 0.0]  # type: ignore[operator]
        margins = sorted(r.exit_m for r in clear)  # type: ignore[misc]
        print(
            f"  OUT OF BAY {len(out)}/{len(clear)}  "
            f"clearance min {margins[0]:.2f} / median {margins[len(margins) // 2]:.2f} / max {margins[-1]:.2f} m"
        )
    print(
        f"  n={n}  immobile(<1cm)={immobile}  "
        f"dist min {min(dists):.2f} / median {sorted(dists)[n // 2]:.2f} / max {max(dists):.2f}  "
        f"laps>=1 {sum(1 for r in live if r.laps >= 1)}  "
        f"collided {sum(1 for r in live if r.collided)}  "
        f"stuck {sum(1 for r in live if r.stuck)}"
    )
    # Split by WHAT ended the run. The rules do not treat contacts alike -- 9.18
    # lets the vehicle touch a wall it does not move and carry on, 9.24.7 ends
    # the round on the parking lot -- so a single `collided` total cannot say
    # whether an in-bay start actually forfeits its 7 points or merely brushed
    # something legal on the way past.
    surfaces: dict[str, int] = {}
    for row in live:
        if row.collided and row.surface:
            surfaces[row.surface] = surfaces.get(row.surface, 0) + 1
    if surfaces:
        breakdown = "  ".join(f"{name}={count}" for name, count in sorted(surfaces.items()))
        print(f"  ended by surface: {breakdown}")
    # Does the FIRST SIGN AHEAD explain the pass-side deaths? By elimination it
    # is the only candidate left: the release pose is identical in all 256, so
    # the manoeuvre cannot be what separates them. Split by colour and by which
    # side the sign already sits on, because the rule is travel-relative -- a
    # colour that demands the far side is a different problem from one that
    # demands the side the chassis is already on.
    seen = [r for r in live if r.sign_range_m is not None]
    if seen:
        print("  first sign ahead at handover:")
        for color in sorted({r.sign_color for r in seen}):
            group = [r for r in seen if r.sign_color == color]
            for side, want_left in (("left", True), ("right", False)):
                arm = [r for r in group if ((r.sign_lat_m or 0.0) > 0.0) == want_left]
                if not arm:
                    continue
                bad = sum(1 for r in arm if r.pass_side)
                ranges = sorted(r.sign_range_m for r in arm)  # type: ignore[misc]
                print(
                    f"    {color:>6} on the {side:<5} n={len(arm):>3}  "
                    f"pass-side {bad}/{len(arm)} ({100.0 * bad / len(arm):.0f}%)  "
                    f"range min {ranges[0]:.2f} / median {ranges[len(ranges) // 2]:.2f} m"
                )
        blind = [r for r in live if r.sign_range_m is None]
        if blind:
            bad = sum(1 for r in blind if r.pass_side)
            print(f"    no sign ahead      n={len(blind):>3}  pass-side {bad}/{len(blind)}")
    # WHERE the offending sign was, which is the test that separates "the
    # navigator drove badly" from "the bay is on the wrong side of a line the
    # chassis was always going to cross". Printed for BOTH arms -- it needs only
    # the result and the layout, not a handover -- so the control answers the
    # same question on the same signs.
    # Split by whether the run had gone ANYWHERE, because the offsets only mean
    # what they look like for the runs that died on the spot. The corridor is a
    # loop with corners, so projecting a sign in another section onto the start
    # heading returns a number with no geometric meaning, and a run that drives
    # three laps legitimately reaches signs "behind" its start. Under a metre of
    # travel neither applies: the chassis is still beside the bay, so the
    # offending radius has to be one within reach of it.
    offenders = [r for r in live if r.viol_ahead_m is not None]
    early = [r for r in offenders if r.dist < 1.0]
    late = [r for r in offenders if r.dist >= 1.0]
    if offenders:
        print(f"  pass-side offenders n={len(offenders)}  died under 1 m {len(early)}  after {len(late)}")
    if early:
        aheads = sorted(r.viol_ahead_m for r in early)  # type: ignore[misc]
        lats = sorted(r.viol_lat_m for r in early)  # type: ignore[misc]
        print(
            f"    under 1 m -- offending sign along {aheads[0]:+.2f} / "
            f"{aheads[len(aheads) // 2]:+.2f} / {aheads[-1]:+.2f} m, "
            f"across {lats[0]:+.2f} / {lats[len(lats) // 2]:+.2f} / {lats[-1]:+.2f} m (+ = left of start)"
        )
    # The legality verdict. A single touch ends the round and voids the parking
    # points, so the aggregate that matters is the WORST margin any run got to,
    # not an average.
    fins = [r.fin_m for r in live if r.fin_m is not None]
    if fins:
        fins.sort()
        touched = sum(1 for f in fins if f <= _TOUCH_EPSILON_M)
        print(
            f"  fin clearance during exit: min {fins[0]:.4f} / median {fins[len(fins) // 2]:.4f} m"
            f"  TOUCHED={touched}/{len(fins)}"
        )


def _build_parser() -> argparse.ArgumentParser:
    """Assemble the CLI. Split from ``main`` purely to keep it readable."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=8, help="Scenario count (0 = all).")
    parser.add_argument("--laps", type=int, default=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--corpus", action="store_true", help=f"use {CORPUS_DIR} instead of the committed set")
    parser.add_argument("--scenarios-dir", default=None)
    parser.add_argument(
        "--hold-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_HOLD_STEER (0/1, in-bay arm only). Holds the forward leg's "
        "steering through the reverse instead of centring it, so the servo can actually "
        "reach the commanded angle within the pocket's 7.5 cm of stroke.",
    )
    parser.add_argument(
        "--cycle",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_CYCLE (0/1, in-bay arm only). Alternating steered-forward-arc "
        "and STRAIGHT reverse, repeated until clear. Asymmetric legs accumulate outward "
        "displacement without needing wall contact, which a constant-|steer| shuffle cannot.",
    )
    parser.add_argument(
        "--arc-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_ARC_STEER_NORM (with --cycle). Full lock is a 17 mm turn radius "
        "-- a pivot, not a translation; ~0.5 is 42 deg and ~0.21 m.",
    )
    parser.add_argument(
        "--cycle-reverse",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_CYCLE_REVERSE_M (with --cycle): how far the straight reverse "
        "runs. Its own constant -- BAY_EXIT_REVERSE_M belongs to the reverse-then-swing exit "
        "and is pinned at 0.05 there.",
    )
    parser.add_argument(
        "--forward-dist",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_FORWARD_M (with --cycle): how far the forward arc runs before the reverse leg.",
    )
    parser.add_argument(
        "--fallback",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_FALLBACK_FRAMES (in-bay arm only): ticks to give the configured "
        "exit before switching to the OTHER one. The two are complementary -- each scores "
        "254/256 under the contact model where the other scores 0 -- so covering both beats "
        "betting on one. 0 never switches.",
    )
    parser.add_argument(
        "--cycle-rev-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_CYCLE_REVERSE_STEER_NORM (with --cycle): steering on the reverse "
        "leg, applied OPPOSITE to the arc. 0 backs straight (keeps the heading the arc won); "
        "non-zero is the three-point turn, adding rotation on both legs at the cost of a full "
        "servo swing between them.",
    )
    parser.add_argument(
        "--exit-speed",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_SPEED_SCALE: extra scale on both cycle legs. The pocket is short "
        "of STOPPING distance -- a leg ends by commanding zero but the drivetrain coasts "
        "v*tau ~ 40 mm at creep, twice the 20 mm leg. Stopping distance is linear in speed.",
    )
    parser.add_argument(
        "--bay-offset",
        type=float,
        default=0.0,
        help="metres to offset the IN-BAY placement along the wall from the pocket centre. "
        "The corpus always places the chassis dead centre, but on the day the team places it "
        "and 9.9 permits physical adjustment during preparation -- so this is a free, legal "
        "parameter, and the pocket has 65 mm of slack at each end.",
    )
    parser.add_argument(
        "--clearance-guard",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_CLEARANCE_GUARD (0/1): bound the cycle legs by PREDICTED fin "
        "clearance, dead-reckoned from odometry, instead of by the stall backstop -- which "
        "fires ON contact and so is itself the 9.24.7 violation.",
    )
    parser.add_argument(
        "--leg-stall",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_LEG_STALL_TICKS (with --cycle): motionless ticks before a leg is "
        "judged jammed and handed over. Charged on EVERY reverse leg at shipped tuning, which "
        "ends on stall rather than on BAY_EXIT_CYCLE_REVERSE_M -- measured rev_m 0.041 against "
        "the 0.09 asked for -- so it is paid once per cycle whatever the distance bound says.",
    )
    parser.add_argument(
        "--latch-direction",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_LATCH_DIRECTION (0/1, in-bay arm only). Decides the open "
        "side once, on the first tick, rather than re-reading two rays that stop "
        "pointing across the pocket as soon as the chassis rotates.",
    )
    parser.add_argument(
        "--latch-reverse",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_LATCH_REVERSE (0/1, in-bay arm only). Makes the reverse leg "
        "a one-shot, so the forward turn is held instead of the gate flipping back and "
        "forth across its own threshold.",
    )
    parser.add_argument(
        "--max-frames",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_MAX_FRAMES (in-bay arm only). Ticks the bay-exit maneuver may "
        "hold control before handing over to CoreNavigator; 0 = forever (shipped). The "
        "maneuver currently never yields, so the escape ladder has never run from the pocket.",
    )
    parser.add_argument(
        "--solid-walls",
        action="store_true",
        help="make walls physically stop the chassis instead of ending the run on contact. "
        "Touching the outer wall is legal on OBSTACLES (not on Open), and `allowed_step` then "
        "caps the TURN while keeping the translation, so a cornered chassis can scrape and peel "
        "away. Without this the probe stalls short of contact and the wall can never help.",
    )
    parser.add_argument(
        "--slide",
        action="store_true",
        help="let a blocked translation slide ALONG the contacted surface instead of being "
        "scaled to nothing. Without it the chassis advances 0.125 mm per tick at 20 deg of "
        "incidence where a rubbing one gains 7.05 mm. Applies to both arms; every "
        "contact-dependent baseline in the repo was measured WITHOUT it.",
    )
    parser.add_argument(
        "--scrub",
        type=float,
        default=0.0,
        help="chassis yaw per radian of wheel turn while STATIONARY, modelling the servo "
        "scrubbing the tyres in place. The kinematics scale yaw with speed, so a stopped "
        "chassis cannot rotate at all -- yet the servo has 35-70x the torque needed to "
        "scrub a wheel. DELIBERATELY OPTIMISTIC: applied in the helpful direction with no "
        "friction threshold, so it bounds the benefit rather than modelling it.",
    )
    parser.add_argument(
        "--no-progress-window",
        type=float,
        default=0.0,
        help="seconds of under-0.08 m net displacement the simulator tolerates before calling "
        "the run stuck; 0 keeps the shipped 30 s. The wall ratchet nets ~10 mm in its first "
        "30 s and only accelerates after that, so the shipped bound ends it mid-manoeuvre and "
        "reports 'stuck' for a chassis that is working.",
    )
    parser.add_argument(
        "--parallel-only",
        action="store_true",
        help="skip the in-bay arm (control alone, to confirm the probe is inert)",
    )
    parser.add_argument(
        "--steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_STEER_NORM over these values (in-bay arm only). "
        "Full lock spins about the chassis centre; the pocket needs translation.",
    )
    parser.add_argument(
        "--reverse",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_REVERSE_M over these values (in-bay arm only).",
    )
    parser.add_argument(
        "--rev-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_REVERSE_STEER_NORM (in-bay arm only). 0 backs straight, "
        "which buys room ahead but nothing on the axis the pocket opens on.",
    )
    parser.add_argument(
        "--known-start",
        action="store_true",
        help="also run each in-bay arm with the believed pose seeded from truth. "
        "DIAGNOSTIC ONLY -- separates a failed exit maneuver from a good exit "
        "handing over to a plan built for a centreline the robot is not on.",
    )
    return parser


def main() -> None:
    """Run both starts over the same scenarios and print them side by side."""
    parser = _build_parser()
    args = parser.parse_args()

    directory = Path(args.scenarios_dir) if args.scenarios_dir else (CORPUS_DIR if args.corpus else _COMMITTED_DIR)
    paths = sorted(directory.glob("*_metadata.json"))
    if not paths:
        parser.error(f"no *_metadata.json under {directory}")
    if args.limit:
        paths = paths[: args.limit]

    jobs = resolve_jobs(args.jobs)
    print(f"{len(paths)} scenarios from {directory}, {jobs} workers, {args.laps} laps")

    # The control always runs at shipped tuning: it exists to show the probe is
    # inert on a normal start, which a swept control could not.
    arms: list[tuple[str, bool, dict[str, float], bool]] = [("parallel (control)", False, {}, False)]
    if not args.parallel_only:
        axes = (
            ("BAY_EXIT_STEER_NORM", "steer", args.steer),
            ("BAY_EXIT_REVERSE_M", "rev-dist", args.reverse),
            ("BAY_EXIT_REVERSE_STEER_NORM", "rev-steer", args.rev_steer),
            ("BAY_EXIT_MAX_FRAMES", "max-frames", args.max_frames),
            ("BAY_EXIT_HOLD_STEER", "hold-steer", args.hold_steer),
            ("BAY_EXIT_LATCH_REVERSE", "latch", args.latch_reverse),
            ("BAY_EXIT_LATCH_DIRECTION", "latch-dir", args.latch_direction),
            ("BAY_EXIT_CYCLE", "cycle", args.cycle),
            ("BAY_EXIT_FALLBACK_FRAMES", "fallback", args.fallback),
            ("BAY_EXIT_ARC_STEER_NORM", "arc", args.arc_steer),
            ("BAY_EXIT_FORWARD_M", "fwd", args.forward_dist),
            ("BAY_EXIT_CYCLE_REVERSE_M", "cyc-rev", args.cycle_reverse),
            ("BAY_EXIT_CYCLE_REVERSE_STEER_NORM", "back-steer", args.cycle_rev_steer),
            ("BAY_EXIT_LEG_STALL_TICKS", "stall", args.leg_stall),
            ("BAY_EXIT_CLEARANCE_GUARD", "guard", args.clearance_guard),
            ("BAY_EXIT_SPEED_SCALE", "spd", args.exit_speed),
        )
        combos: list[dict[str, float]] = [{}]
        labels: list[str] = [""]
        for field, short, values in axes:
            if not values:
                continue
            combos, labels = (
                [{**c, field: v} for c in combos for v in values],
                [f"{lbl} {short} {v:g}".strip() for lbl in labels for v in values],
            )
        for changes, label in zip(combos, labels, strict=True):
            arms.append((f"IN-BAY{' ' + label if label else ''}", True, changes, False))
            if args.known_start:
                # Diagnostic only -- seeds the believed pose from truth, which
                # the hardware cannot do. Isolates "the exit maneuver failed"
                # from "the exit worked and the plan was 0.4 m off", since blind
                # assumes a centreline start the in-bay robot is not at.
                arms.append((f"IN-BAY{' ' + label if label else ''} +known_start", True, changes, True))

    for name, in_bay, changes, known in arms:
        payloads = [
            (
                str(p),
                in_bay,
                args.laps,
                changes,
                known,
                args.solid_walls,
                args.slide,
                args.scrub,
                args.bay_offset,
                args.no_progress_window,
            )
            for p in paths
        ]
        rows = run_pool(_run_case, payloads, jobs, on_result=print_pool_progress(name))
        _summarise(name, rows)


if __name__ == "__main__":
    main()

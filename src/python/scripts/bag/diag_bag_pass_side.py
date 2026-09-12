r"""Which side did the robot pass each sign, and which side was it TOLD to?

Reported from the track on 2026-09-07: green signs passed on the RIGHT when the
rule requires the LEFT, repeatedly, across laps and across runs. Three causes
produce that same symptom and they need completely different fixes, so the point
of this script is to tell them apart rather than to confirm the symptom:

* **DIRECTION** -- the pass-side rule is travel-relative, and ``ROUTING_TABLE``'s
  clockwise rows are the exact negation of its counterclockwise ones. A router
  running the wrong direction does not degrade the lane, it MIRRORS it: every
  colour passes on the wrong side, consistently, all race. Tell: commanded side
  is wrong for EVERY sign, and flipping the direction makes them all correct.
* **COLOUR** -- a green seen as red (or the reverse) flips that sign's side and
  no other. Tell: commanded side is wrong only where the believed colour
  disagrees with the camera's own votes.
* **EXECUTION** -- the router commanded the correct side and the chassis did not
  get there. Tell: commanded side is RIGHT and achieved side is wrong, with a
  small lateral magnitude.

The believed sign colour is not in ``/nav_debug``, so the sign map is rebuilt by
replaying the real router over the recorded detections -- the same approach as
``diag_bag_sign_target_churn.py``, which reproduces the bag's own commitment
churn (7.6x against 7.4x measured independently).

MEASURED 2026-09-07 on run_233653 (clockwise) and run_234457 (counterclockwise),
47 pillars:

    commanded the WRONG side (routing):          11
    commanded right, chassis went wrong (exec):  10
    correct:                                     26

DIRECTION IS NOT THE CAUSE -- a mirrored table would flip reds and greens alike,
and reds are mostly legal. Both remaining causes are present in similar numbers.

TRAP, and it moves the answer: judge with the SIGN's corridor, not the robot's.
The router keys the deformation off the candidate sign's own corridor, and the
two disagree on 22 of these 47 pillars (they legitimately differ at corners).
Judging by the robot's corridor instead reports 17 routing / 6 execution -- a
different conclusion from the same data.

WHAT THE TRACE FOUND, which the counts do not show. On the wrong-side pass
reported from the track in run_234457, the router commanded the CORRECT side on
every tick (+0.25 to +0.29) and the chassis was on the wrong side throughout
(-0.32 closing to -0.16). It never crossed, because the sign was not committed
until 0.57 m -- and crossing sides inside 0.57 m with the measured 0.29 m turn
radius is geometrically impossible. The outcome was decided before the router
ever engaged, by the detector's range (p50 0.70 m).

And the colour was wrong for a structural reason: DUPLICATE TRACKS SPLIT THE
COLOUR EVIDENCE. That pillar carried nine tracks within half a metre; the router
committed to one with 4 hits and 2.29 of RED weight while its siblings held
20-30 hits and 18-24 of GREEN. Pooling votes within 0.30 m calls it green 40.8
to 11.2. Colour is resolved per FRAGMENT, but a fragment is not a physical
object.

TRAP: do NOT read ``wrong_side_pass_count`` for this. It judges from the
router's own believed layout, which on hardware includes phantom pillars and
positions off by half a metre; it has read 24 where the truth was 7.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_pass_side.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.enums import Axis, Direction  # noqa: E402
from shared.domain.models import Pose, SignColor, Waypoint  # noqa: E402

from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import (  # noqa: E402
    create_bags_parser,
    decode_detections,
    read_vision_rows_and_scans,
    settled_direction,
)
from scripts.common.stats import nearest_by_time  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_observation  # noqa: E402
from src.navigation.planning.sign_router import SignRouter  # noqa: E402
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis  # noqa: E402

PILLAR_M = 0.35
"""Commitment anchors closer than this are the same physical pillar."""


@dataclass
class Pass:
    """One pillar, as passed."""

    run: str
    colour: SignColor
    corridor: str
    commanded: int | None
    achieved: int
    lateral_m: float
    commanded_m: float
    """Lateral offset the router ASKED for at the SAME tick ``lateral_m`` is
    read at. ``commit_commanded_m`` below is the ask at commit, which is a
    different instant: pairing it against ``lateral_m`` compares two moments
    and cannot separate a bad plan from bad tracking. This one can."""

    # Everything below describes the FIRST tick this sign was committed to,
    # which is the moment the pass became a steering problem. The fields above
    # describe the closest tick, which is where the outcome is read. An
    # execution failure can only be diagnosed by holding both: the verdict says
    # the chassis ended up on the wrong side, and these say whether it ever had
    # the room to end up anywhere else.
    commit_range_m: float
    commit_lateral_m: float
    """Signed lateral offset of the CHASSIS at commit, positive = legal side."""
    commit_commanded_m: float
    """Signed lateral offset the router ASKED for at commit, same convention."""
    commit_speed_mps: float | None
    maneuver_during_pass: bool
    """An escape/stuck manoeuvre was latched at some point while committed."""

    maneuver_ticks: Counter[str]
    """Latched-manoeuvre ticks while committed, BY TYPE. ``maneuver_during_pass``
    collapses every manoeuvre into one bit, and that cannot tell a k_turn (a
    deliberate re-orientation) apart from side_correction (the reactive layer
    taking the wheel). Reported from the track on 2026-09-12: the corrections
    themselves often fail to reach the legal side or to avoid contact, which is
    a claim about ONE manoeuvre type and needs the type to test."""

    manoeuvre_agrees: int
    """Latched-manoeuvre ticks steering TOWARD the side the router asked for."""
    manoeuvre_opposes: int
    """...and ticks steering AWAY from it. The escape picks its side from
    left/right CLEARANCE (33be7da7); the router picks it from the rule. Near a
    pillar the two routinely disagree -- the clearer side is the one away from
    the pillar, which is the wrong side when the chassis must still cross."""

    sign_x: float
    sign_y: float
    """Where the router BELIEVED the pillar was. Believed, not true -- there is
    no ground truth in a bag -- but it is what the pass was planned against,
    and it is what snaps onto the 24-point legal lattice for a geometry
    classification that needs no ground truth to be meaningful."""


def _load(bag_dir: Path):  # noqa: ANN202
    """Read nav_debug rows, detection frames AND scans from one bag."""
    return read_vision_rows_and_scans(bag_dir)


_detections = decode_detections



def _passes(run: str, rows, frames, scans, tuning) -> tuple[list[Pass], int]:  # noqa: ANN001
    """Replay the router, then judge each pillar against the SHIPPED rule.

    Legality is evaluated in the WORLD frame with ``pass_side_lateral_axis``,
    not from a restatement of the rule: it returns the axis and the sign the
    deformed waypoint must take relative to the pillar, so the same call decides
    what was REQUIRED, what the router COMMANDED (the deformed waypoint it
    actually produced) and what the chassis ACHIEVED (where it really went).
    Those three separate a routing error from an execution one.

    Also returns the PEAK believed sign count over the replay. The track holds
    at most 8 pillars, so anything above that is the map inventing objects --
    the quantity ``SNAP_TO_LATTICE_M`` exists to bound, and the one that has to
    move for a routing verdict to mean anything.
    """
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)

    frame_i = 0
    peak_signs = 0
    scan_times = [t for t, _ in scans]
    best: dict[tuple[float, float], tuple] = {}
    first: dict[tuple[float, float], tuple] = {}
    manoeuvred: dict[tuple[float, float], bool] = {}
    man_types: dict[tuple[float, float], Counter[str]] = {}
    steer_vote: dict[tuple[float, float], list[int]] = {}

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        # The same tick's sweep, so LIDAR_RANGE_FUSION* can fire. Without it
        # detection_to_observation falls back to the pinhole range and an A/B
        # of the fusion measures nothing at all.
        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            obs.extend(
                o
                for det in _detections(frames[frame_i][1])
                if (o := detection_to_observation(det, pose, tuning, ranges, angles)) is not None
            )
            frame_i += 1
        if d.steer_target_x is None:
            continue
        deformed = router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )
        peak_signs = max(peak_signs, router.active_sign_count)
        committed = router.committed_sign_position
        if committed is None:
            continue
        rng = math.hypot(committed.x - d.pose_x, committed.y - d.pose_y)
        key = (round(committed.x, 1), round(committed.y, 1))
        # Recorded BEFORE the nearest-tick filter below, because the first
        # commitment is by definition not the nearest one.
        if key not in first:
            first[key] = (rng, (d.pose_x, d.pose_y), deformed, d.commanded_speed_mps)
        manoeuvred[key] = manoeuvred.get(key, False) or d.active_maneuver_type is not None
        if d.active_maneuver_type is not None:
            man_types.setdefault(key, Counter())[str(d.active_maneuver_type)] += 1
        # Does the latched manoeuvre steer toward the side the router asked
        # for? Compared in the ROBOT frame, because a steering sign is a
        # left/right command and the router's request is a world vector: the
        # deformed target minus the pose, projected onto the chassis's own left.
        if d.active_maneuver_type is not None and d.maneuver_steering is not None:
            left_x, left_y = -math.sin(d.pose_yaw), math.cos(d.pose_yaw)
            wants_left = (deformed[0] - d.pose_x) * left_x + (deformed[1] - d.pose_y) * left_y
            vote = steer_vote.setdefault(key, [0, 0])
            # A zero steering command votes for neither -- it is not a side.
            if d.maneuver_steering != 0.0 and wants_left != 0.0:
                same = (d.maneuver_steering > 0) == (wants_left > 0)
                vote[0 if same else 1] += 1
        if key in best and best[key][0] <= rng:
            continue
        colour = next(
            (s.color for s in router.signs if abs(s.x - committed.x) < 1e-9 and abs(s.y - committed.y) < 1e-9),
            SignColor.UNKNOWN,
        )
        rule = pass_side_lateral_axis(d.current_corridor, colour, direction)
        best[key] = (rng, colour, str(d.current_corridor), rule, committed, (d.pose_x, d.pose_y), deformed)

    out: list[Pass] = []
    for key, (rng, colour, corridor, rule, sign_pos, robot, deformed) in best.items():
        if rule is None:
            continue
        axis, want = rule
        idx = 0 if axis is Axis.X else 1
        sign_axis = sign_pos.x if idx == 0 else sign_pos.y
        achieved_delta = robot[idx] - sign_axis
        commanded_delta = deformed[idx] - sign_axis
        # Commit-time deltas are taken on the SAME axis the verdict is judged
        # on, not on whatever the corridor was at commit: the question is how
        # far the chassis had to travel to reach the side it is judged against.
        c_rng, c_robot, c_deformed, c_speed = first[key]
        out.append(
            Pass(
                run=run,
                colour=colour,
                corridor=corridor,
                commanded=(1 if commanded_delta > 0 else -1) * want,
                achieved=(1 if achieved_delta > 0 else -1) * want,
                lateral_m=abs(achieved_delta),
                commanded_m=abs(commanded_delta),
                commit_range_m=c_rng,
                commit_lateral_m=(c_robot[idx] - sign_axis) * want,
                commit_commanded_m=(c_deformed[idx] - sign_axis) * want,
                commit_speed_mps=c_speed,
                maneuver_during_pass=manoeuvred.get(key, False),
                maneuver_ticks=man_types.get(key, Counter()),
                manoeuvre_agrees=steer_vote.get(key, [0, 0])[0],
                manoeuvre_opposes=steer_vote.get(key, [0, 0])[1],
                sign_x=sign_pos.x,
                sign_y=sign_pos.y,
            )
        )
    return out, peak_signs


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help="override one sign_discovery field for the replay, e.g. --set SNAP_TO_LATTICE_M=0.40",
    )
    args = parser.parse_args()

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)
    if overrides:
        print(f"== sign_discovery overrides: {overrides}")

    rows_out = []
    ask_rows: list[tuple[str, str, str, float, float, float, float, str, bool]] = []
    peaks: list[tuple[str, int]] = []
    skipped: list[tuple[str, str]] = []
    tally = {"routing": 0, "execution": 0, "ok": 0}
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = _load(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            # Older bags in the archive carry a metadata version this rosbag2
            # cannot open. A corpus sweep must not die on one of them, so the
            # skip is reported and counted rather than raised.
            skipped.append((Path(bag).name, type(exc).__name__))
            continue
        direction = settled_direction(rows)
        run = Path(bag).name.replace("run_", "")
        passes, peak = _passes(run, rows, frames, scans, tuning)
        peaks.append((run, peak))
        for p in passes:
            # commanded/achieved are already expressed as +1 when they match the
            # rule's required side, so the reading needs no second convention.
            if p.commanded < 0:
                verdict = "ROUTING: commanded wrong side"
                tally["routing"] += 1
            elif p.achieved < 0:
                verdict = "EXECUTION: commanded right, went wrong"
                tally["execution"] += 1
            else:
                verdict = "ok"
                tally["ok"] += 1
            rows_out.append(
                [run, str(direction), p.corridor, str(p.colour), round(p.lateral_m, 3), verdict]
            )
            # Signed against the rule's legal side, so a negative ask is the
            # router itself planning the wrong side and a negative achievement
            # with a positive ask is the chassis failing to follow it.
            ask_rows.append(
                (run, str(p.colour), p.corridor,
                 (p.commanded or 0) * p.commanded_m, p.achieved * p.lateral_m,
                 p.commit_commanded_m, p.commit_lateral_m, verdict,
                 p.maneuver_during_pass)
            )
    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(n for n, _ in skipped[:6])}")
        print()
    print("== PEAK BELIEVED SIGNS per run (the track holds at most 8)")
    for run, peak in peaks:
        print(f"  {run}  peak={peak:3d}{'   OVER THE PHYSICAL MAX' if peak > 8 else ''}")
    over = sum(1 for _, p in peaks if p > 8)
    print(f"  runs over the physical max: {over}/{len(peaks)}   worst={max((p for _, p in peaks), default=0)}")
    print()

    if not rows_out:
        print("No sign passes reconstructed from these bags.")
        return
    print("== PASS SIDE, required vs commanded vs achieved")
    print_table(rows_out, ["run", "direction", "corridor", "colour", "clearance m", "verdict"])
    print()
    by_colour: dict[str, dict[str, int]] = {}
    for _run, _direction, _corr, colour, _lat, verdict in rows_out:
        bucket = by_colour.setdefault(colour, {"routing": 0, "execution": 0, "ok": 0})
        bucket["routing" if verdict.startswith("ROUTING") else "execution" if verdict.startswith("EXEC") else "ok"] += 1
    print("  by colour (routing / exec / ok, and the routing rate):")
    for colour, b in sorted(by_colour.items()):
        n_total = b["routing"] + b["execution"] + b["ok"]
        print(
            f"    {colour:>16}  {b['routing']:4d} / {b['execution']:4d} / {b['ok']:4d}"
            f"   routing {100 * b['routing'] / n_total:5.1f}%  of {n_total}"
        )
    print()
    # ASKED vs ACHIEVED, both read at the closest tick, so the gap between the
    # two columns is tracking error and nothing else. This is the control that
    # decides whether a near-miss is a PLAN failure (the router asked for the
    # clearance it got) or a TRACKING failure (it asked for much more).
    print("== ASKED vs ACHIEVED at the closest tick (signed, + = legal side)")
    ask_table = [
        [run, colour, corr, round(ask, 3), round(got, 3), round(got - ask, 3),
         round(c_ask, 3), round(c_got, 3), verdict.split(":")[0]]
        for run, colour, corr, ask, got, c_ask, c_got, verdict, _man in ask_rows
    ]
    print_table(
        ask_table,
        ["run", "colour", "corridor", "ask m", "got m", "got-ask", "ask@commit", "pose@commit", "verdict"],
    )
    print()
    grazes = [r for r in ask_rows if abs(r[4]) < 0.030]
    print(f"  passes grazing under 30 mm: {len(grazes)} of {len(ask_rows)}")
    for label, subset in (("ALL passes", ask_rows), ("grazes <30mm", grazes)):
        if not subset:
            continue
        asks = [abs(r[3]) for r in subset]
        gots = [abs(r[4]) for r in subset]
        errs = [abs(r[4] - r[3]) for r in subset]
        n = len(subset)
        print(
            f"    {label:>14}  n={n:3d}  mean |ask|={sum(asks) / n:.3f}"
            f"  mean |got|={sum(gots) / n:.3f}  mean |got-ask|={sum(errs) / n:.3f}"
            f"  max |got-ask|={max(errs):.3f}"
        )
    # The two worlds the measurement has to separate, counted on the grazes.
    plan_fail = sum(1 for r in grazes if abs(r[3]) < 0.030)
    track_fail = sum(1 for r in grazes if abs(r[3]) >= 0.030)
    print(f"    of the grazes: PLAN asked <30mm too: {plan_fail}   TRACKING asked >=30mm: {track_fail}")
    # A tracking failure has to come from somewhere. The open chain says the
    # planner is silent during manoeuvres, so split the SAME error by whether a
    # manoeuvre was latched: if the error lives in the manoeuvre ticks, the
    # open-loop arcs are the mechanism; if it does not, they are exonerated.
    print()
    print("  tracking error split by whether a manoeuvre was latched while committed:")
    for label, subset in (
        ("manoeuvred", [r for r in ask_rows if r[8]]),
        ("clean", [r for r in ask_rows if not r[8]]),
    ):
        if not subset:
            print(f"    {label:>12}  n=  0")
            continue
        errs = [abs(r[4] - r[3]) for r in subset]
        graze = sum(1 for r in subset if abs(r[4]) < 0.030)
        n = len(subset)
        print(
            f"    {label:>12}  n={n:3d}  mean |got-ask|={sum(errs) / n:.3f}"
            f"  grazes {graze}/{n} = {100 * graze / n:5.1f}%"
        )
    print()
    print(f"  commanded the WRONG side (routing):        {tally['routing']}")
    print(f"  commanded right, chassis went wrong (exec): {tally['execution']}")
    print(f"  correct:                                    {tally['ok']}")


if __name__ == "__main__":
    main()

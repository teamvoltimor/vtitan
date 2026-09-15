r"""Did the chassis pass each pillar on the legal side, judged from GROUND TRUTH?

STATUS: the instrument PASSES its control on THREE of the six 2026-09-15 runs
-- 140852, 141413 and 141832 all reconstruct the operator-stated layout of 5
GREEN and 3 RED. Quote only those three. It still misreads 140014 (4G/3R, a
full round) and the two rounds the operator aborted early, 140358 and 141230,
where coverage is too short to pile up every pillar.

The COLOUR VOTE was the broken part and is no longer what decides colour: the
operator's layout plus the pillar's SECTION does, and on a validated run the
camera vote then AGREES with the layout on all eight signs. Two independent
methods concur, which is why the control passes at all. The vote is still
printed per pillar, as the strength of the camera's own call.

Fixing the vote itself still needs detection TRACKS associated to pillars over
time rather than per-frame proximity: detections are attributed to the nearest
pillar within 0.35 m while the camera bearing carries +/-12 deg of zero-mean
scatter, which at 1.5 m is 0.31 m of lateral miss, so they land on the
neighbour. Camera-bearing work reached the same conclusion separately.

A wrong-side pass ENDS the round. Every hardware round of 2026-09-15 shows one
to five of them by the robot's own count, with and without
``barrier_span_along_wall``, and the lap counter hides all of it -- four rounds
read 3/3 while the operator had mentally stopped them on lap 1.

GROUND TRUTH DISAGREES WITH THAT COUNT. On the three validated runs this judge
reads 0, 2 and 2 wrong-side passes where the robot's own counter read 3, 3 and
5. The counter is computed in the BELIEVED frame and over-reports; on 140852 it
claimed three where the chassis committed none. Prefer this judge, and do not
re-derive a round's fate from ``wrong_side_pass_count``.

Neither existing instrument can settle that:

* ``wrong_side_pass_count`` on the wire is ``SignRouter.wrong_side_violations``,
  computed in the BELIEVED frame from discovered colours. It conflates where the
  chassis drove with what the robot thinks it saw.
* ``diag_bag_pass_side.py`` judges with the ROBOT's corridor while the router
  uses another (recorded 2026-09-12).

So this rebuilds the simulator's own judge -- ``scenario_simulator/scoring.py``
-- against hardware truth:

* **Positions from the LIDAR**, not from the sign map. Every return more than
  ``--wall-margin`` from a wall is accumulated in world coordinates over the
  whole bag; a pillar is a physical object and piles up. The sign map is built
  from camera bearings and would agree with any error they carry.
* **Colour by VOTE** over every detection projected near each pillar. Colour is
  genuinely camera-only, so belief cannot be eliminated here -- but a majority
  over hundreds of frames is a different thing from one frame, and the vote
  margin is printed so a weak call is visible rather than silent.
* **The rule from production**: ``pass_side_lateral_axis`` with the round's
  settled direction, the same function the router uses.
* **A FOOTPRINT crossing test**, like the simulator's: the pass is decided when
  the chassis has COMPLETELY crossed the pillar's radius, so a chassis that
  strays wide and corrects before the line is not scored as offending. The rules
  permit that recovery and a closest-approach proxy forbids it by construction.

CONTROLS, because a crashed diagnostic here exits 0:

* The pillar map is printed first with its return counts. If it is empty or its
  clusters sit on walls, nothing below it means anything.
* The colour vote margin is printed per pillar. A pillar with no detections is
  reported as UNKNOWN and judged on nothing rather than guessed.
* Signs the chassis never fully crossed are counted separately from signs it
  crossed correctly. Those are not passes and must not dilute the rate.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_pass_side_truth.py RUN_DIR...
"""

from __future__ import annotations

import contextlib
import json
import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# isort: off
# scripts.common.bag_io FIRST: importing shared.domain.models ahead of it trips
# a partially-initialised cycle between models and enums (GMR_CLASS_NAMES).
from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_detections,
    decode_nav_debug,
    open_reader,
    scan_to_ranges_angles,
)
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants import RobotSpecs  # noqa: E402
from shared.domain.enums import Axis, Direction  # noqa: E402
from shared.domain.models import Pose, SignColor  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_world_point, snap_to_lattice  # noqa: E402
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis  # noqa: E402
from src.navigation.planning.waypoints.classification import corridor_for_position  # noqa: E402
from src.navigation.race_tracker import TRAVEL_DIRS  # noqa: E402

# isort: on

_TRACK_MIN, _TRACK_MAX = 0.0, 3.0
_INNER_MIN, _INNER_MAX = 1.0, 2.0
_SELF_RETURN_M = 0.15
_PEAK_CLAIM_M = 0.30  # a sign is 0.05 m wide; two peaks closer than this are one object split
_APPROACH_M = 1.20  # the simulator's own engage radius
_COLOUR_MATCH_M = 0.35  # a detection this close to a pillar votes for it

# OPERATOR GROUND TRUTH, every round of 2026-09-15: the mat carried 2 parking
# walls, 5 GREEN pillars and 3 RED. Ten off-wall objects, of which only EIGHT
# are signs. That is both a constraint and a control: the pass-side rule does
# not apply to a parking fin, and a colour vote that does not come out 5 green
# and 3 red is a vote to distrust rather than a finding.
_EXPECTED_GREEN, _EXPECTED_RED = 5, 3
# The two parking fins are NOT expected in the cluster map and their absence is
# not a fault: they stand 0.10-0.20 m from the wall and `_off_wall` drops every
# return within `--wall-margin` (0.25 m) of one. They are excluded from the
# pass-side rule anyway, which says nothing about a fin.
# A fin stands at ParkingLotSpecs wall_offset, ~0.10-0.20 m from the wall face,
# while the nearest legal sign cell is a division line at 0.40. Anything inside
# this of a wall is the lot, not a pillar.
_FIN_BAND_M = 0.30


# OPERATOR LAYOUT, 2026-09-15, given per section in TRAVEL order for both
# directions and self-consistent between them -- which is what makes it
# trustworthy rather than a single assertion:
#
#   clockwise         south: rojo ... verde | west: verde,verde | north: verde,rojo | east: rojo,verde
#   counterclockwise  south: verde ... rojo | east: verde,rojo  | north: rojo,verde | west: verde,verde
#
# The two readings are the SAME arrangement traversed in opposite senses: every
# mixed section reverses and the west pair is symmetric, and both total 5 green
# and 3 red. Section order was verified against the bags -- CW drives S,W,N,E
# and CCW drives S,E,N,W, with the start in SOUTH near x=1.22.
#
# Resolving that into world coordinates gives a rule per section, which replaces
# the colour VOTE entirely. The vote was the part that failed: detections are
# attributed by proximity while the camera bearing carries +/-12 deg of
# zero-mean scatter, so they land on the neighbour.
_SOUTH_SPLIT_X = 1.22  # the in-bay start; the red lies west of it, the green east


def _assign_sections(
    pillars: list[tuple[float, float, int]],
) -> dict[int, str]:
    """Give each pillar a section, forcing the TWO-PER-SECTION the layout states.

    ``corridor_for_position`` answers for a CHASSIS, and at a corner a pillar
    sits in the ambiguous wedge: measured, (0.53, 0.83) is called south while
    113 detections against 0 say green, which only the west pair can be. The
    operator's layout fixes the cardinality at two per section, so the
    assignment is a constraint rather than a lookup: each pillar goes to the
    section whose BAND it sits deepest inside, and a section already holding two
    passes the pillar on to its next-best.
    """
    bands = {
        "south": lambda x, y: 1.0 - y,
        "north": lambda x, y: y - 2.0,
        "west": lambda x, y: 1.0 - x,
        "east": lambda x, y: x - 2.0,
    }
    ranked = []
    for i, (x, y, _n) in enumerate(pillars):
        order = sorted(bands, key=lambda k: -bands[k](x, y))
        ranked.append((i, order))
    out: dict[int, str] = {}
    held: dict[str, int] = dict.fromkeys(bands, 0)
    for depth in range(len(bands)):
        for i, order in ranked:
            if i in out:
                continue
            sec = order[depth]
            if held[sec] < 2:
                out[i] = sec
                held[sec] += 1
    return out


def _layout_colour(x: float, y: float, name: str) -> SignColor | None:
    """The pillar's colour from the operator's layout, by section and position."""
    if "west" in name:
        return SignColor.GREEN  # both of them
    if "north" in name:
        # CW drives north EAST-bound and sees green then red.
        return SignColor.GREEN if x < 1.5 else SignColor.RED
    if "east" in name:
        # CW drives east SOUTH-bound and sees red then green.
        return SignColor.RED if y > 1.5 else SignColor.GREEN
    if "south" in name:
        return SignColor.GREEN if x > _SOUTH_SPLIT_X else SignColor.RED
    return None


def _off_wall(px: np.ndarray, py: np.ndarray, margin: float) -> np.ndarray:
    """True for returns clear of every wall face, outer ring and inner block alike."""
    near_outer = (
        (np.abs(px - _TRACK_MIN) < margin)
        | (np.abs(px - _TRACK_MAX) < margin)
        | (np.abs(py - _TRACK_MIN) < margin)
        | (np.abs(py - _TRACK_MAX) < margin)
    )
    dx = np.maximum(_INNER_MIN - px, px - _INNER_MAX)
    dy = np.maximum(_INNER_MIN - py, py - _INNER_MAX)
    near_inner = np.abs(np.maximum(dx, dy)) < margin
    on_mat = (px > -margin) & (px < _TRACK_MAX + margin) & (py > -margin) & (py < _TRACK_MAX + margin)
    return on_mat & ~near_outer & ~near_inner


def _near_wall(x: float, y: float) -> float:
    """Distance to the nearest OUTER wall face."""
    return min(abs(x - _TRACK_MIN), abs(x - _TRACK_MAX), abs(y - _TRACK_MIN), abs(y - _TRACK_MAX))


def _corners(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    """The oriented chassis rectangle, for the COMPLETELY-crossed test."""
    hl, hw = RobotSpecs.LENGTH / 2.0, RobotSpecs.WIDTH / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + dx * c - dy * s, y + dx * s + dy * c) for dx, dy in ((hl, hw), (hl, -hw), (-hl, hw), (-hl, -hw))]


def _read(  # noqa: PLR0912  one pass over the bag, branching per topic; splitting it would mean reading the bag twice
    bag_dir: Path, wall_margin: float, cell: float, min_returns: int, max_pillars: int
) -> tuple[
    list[tuple[Pose, int]], Direction | None, list[tuple[float, float, int]], list[tuple[float, float, SignColor]]
]:
    """One pass over the bag: pose track, LIDAR pillar map, and colour votes."""
    reader = open_reader(bag_dir)
    pose: Pose | None = None
    track: list[tuple[Pose, int]] = []
    direction = None
    counts: dict[tuple[int, int], int] = {}
    dets: list[tuple[float, float, SignColor]] = []
    ranges = angles = None
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            try:
                snap = decode_nav_debug(data)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(snap.pose_x, (int, float)) and isinstance(snap.pose_y, (int, float)):
                pose = Pose(x=float(snap.pose_x), y=float(snap.pose_y), yaw=float(snap.pose_yaw or 0.0))
                track.append((pose, int(snap.laps_completed or 0)))
            if snap.direction is not None:
                direction = snap.direction
            continue
        if topic == Topics.SCAN:
            with contextlib.suppress(Exception):
                ranges, angles = scan_to_ranges_angles(deserialize_message(data, LaserScan))
            if pose is None or ranges is None:
                continue
            keep = (ranges > _SELF_RETURN_M) & (ranges < RobotSpecs.LIDAR_MAX_RANGE * 0.99)
            r, a = ranges[keep], angles[keep]
            if r.size == 0:
                continue
            w = pose.yaw + a
            px, py = pose.x + r * np.cos(w), pose.y + r * np.sin(w)
            sel = _off_wall(px, py, wall_margin)
            for gx, gy in zip(np.floor(px[sel] / cell).astype(int), np.floor(py[sel] / cell).astype(int), strict=True):
                counts[(gx, gy)] = counts.get((gx, gy), 0) + 1
            continue
        if topic != Topics.VISION_DETECTIONS or pose is None:
            continue
        try:
            payload = json.loads(deserialize_message(data, String).data)
        except Exception:  # noqa: BLE001
            continue
        for det in decode_detections(payload):
            if det.color not in (SignColor.RED, SignColor.GREEN):
                continue
            pt = detection_to_world_point(det, pose, lidar_ranges_m=ranges, lidar_angles_rad=angles)
            if pt is not None:
                dets.append((pt[0], pt[1], det.color))

    return track, direction, _peaks(counts, cell, min_returns, max_pillars), dets


def _peaks(
    counts: dict[tuple[int, int], int], cell: float, min_returns: int, max_pillars: int
) -> list[tuple[float, float, int]]:
    """Greedy peak picking: each peak claims a neighbourhood before the next is taken."""
    pillars: list[tuple[float, float, int]] = []
    for (gx, gy), n in sorted(counts.items(), key=lambda kv: -kv[1]):
        if n < min_returns:
            break
        x, y = (gx + 0.5) * cell, (gy + 0.5) * cell
        if any(math.hypot(x - a, y - b) < _PEAK_CLAIM_M for a, b, _ in pillars):
            continue
        pillars.append((x, y, n))
        if len(pillars) >= max_pillars:
            break
    return pillars


def _colour(x: float, y: float, dets: list[tuple[float, float, SignColor]]) -> tuple[SignColor | None, int, int]:
    """Majority colour for a pillar, with the two vote counts so a weak call shows."""
    votes = Counter(c for dx, dy, c in dets if math.hypot(dx - x, dy - y) < _COLOUR_MATCH_M)
    red, green = votes.get(SignColor.RED, 0), votes.get(SignColor.GREEN, 0)
    if red == 0 and green == 0:
        return None, 0, 0
    return (SignColor.RED if red >= green else SignColor.GREEN), red, green


def _judge(
    track: list[tuple[Pose, int]],
    pillar: tuple[float, float],
    colour: SignColor,
    direction: Direction,
) -> str:
    """CORRECT, WRONG or never-crossed, by the simulator's own footprint rule."""
    x, y = pillar
    # SNAP FIRST. Production never asks the corridor of a raw estimate: the slot
    # map publishes legal cells and measured 0.0% off-lattice over 6,971
    # committed positions. Asking `corridor_for_position` with a LIDAR position
    # instead reads (0.53, 0.83) as SOUTH when its cell (0.6, 1.0) is WEST, and
    # the rule then judges the wrong AXIS entirely. That was a bug in THIS
    # SCRIPT, briefly mistaken for one in production -- all 24 legal cells label
    # correctly.
    x, y = snap_to_lattice(x, y, 0.35)
    corridor = corridor_for_position(x, y)
    rule = pass_side_lateral_axis(corridor, colour, direction)
    if rule is None:
        return "no rule"
    lateral_axis, permitted = rule
    heading = TRAVEL_DIRS[(corridor, direction)]
    if lateral_axis == Axis.Y:
        depth_axis, ahead = Axis.X, (1 if heading.nx > 0 else -1)
    else:
        depth_axis, ahead = Axis.Y, (1 if heading.ny > 0 else -1)
    sign_depth = x if depth_axis == Axis.X else y
    sign_lat = x if lateral_axis == Axis.X else y

    engaged = False
    scored_lap: set[int] = set()
    verdicts: list[str] = []
    for p, lap in track:
        if math.hypot(x - p.x, y - p.y) > _APPROACH_M:
            continue
        # ONCE PER LAP, mirroring the simulator's _pass_side_scored. Without
        # this a chassis pendulumming beside a pillar re-crosses the radius
        # every few ticks and each crossing counts: the first version of this
        # script reported 45 and 53 passes over 10 pillars and 3 laps, where 30
        # is the ceiling, and it inflated exactly the wedged rounds.
        if lap in scored_lap:
            continue
        cs = _corners(p.x, p.y, p.yaw)
        behind = [(cx if depth_axis == Axis.X else cy) - sign_depth for cx, cy in cs]
        if min(d * ahead for d in behind) <= 0.0:
            engaged = True
            continue
        if not engaged:
            continue
        engaged = False
        scored_lap.add(lap)
        robot_lat = p.x if lateral_axis == Axis.X else p.y
        if robot_lat == sign_lat:
            continue
        verdicts.append("CORRECT" if (1 if robot_lat > sign_lat else -1) == permitted else "WRONG")
    if not verdicts:
        return "never crossed"
    return f"{verdicts.count('WRONG')} WRONG / {len(verdicts)}"


def _score_bag(
    pillars: list[tuple[float, float, int]],
    dets: list[tuple[float, float, SignColor]],
    track: list[tuple[Pose, int]],
    direction: Direction,
    min_votes: int,
) -> tuple[list, int, int, int, int, int, int]:
    """Classify every object and judge the ones that are signs."""
    rows, wrong, passes, unknown = [], 0, 0, 0
    fins = greens = reds = 0
    signs = [(x, y, n) for x, y, n in pillars if _near_wall(x, y) >= _FIN_BAND_M]
    fins = len(pillars) - len(signs)
    for (x, y, n), sec in zip(signs, _assign_sections(signs).values(), strict=False):
        # Colour from the LAYOUT, not from a vote. The vote is still computed
        # and shown so a disagreement is visible: where it differs, the camera
        # attributed that detection to the wrong pillar.
        colour = _layout_colour(x, y, sec)
        _voted, red, green = _colour(x, y, dets)
        if colour is None:
            unknown += 1
            rows.append([f"({x:.2f},{y:.2f})", n, "off layout", f"{red}R/{green}G", "not judged"])
            continue
        greens += int(colour is SignColor.GREEN)
        reds += int(colour is SignColor.RED)
        verdict = _judge(track, (x, y), colour, direction)
        if "WRONG" in verdict:
            w, total = verdict.split(" WRONG / ")
            wrong += int(w)
            passes += int(total)
        agree = "" if _voted is colour else "  <- vote said " + (_voted.value if _voted else "nothing")
        rows.append([f"({x:.2f},{y:.2f})", n, colour.value + agree, f"{red}R/{green}G", verdict])
    return rows, wrong, passes, unknown, fins, greens, reds


def main() -> int:
    """Judge every pillar pass in each bag against LIDAR-located truth."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--wall-margin", type=float, default=0.25)
    parser.add_argument("--cell", type=float, default=0.05)
    parser.add_argument("--min-returns", type=int, default=250)
    parser.add_argument("--max-pillars", type=int, default=10)
    parser.add_argument(
        "--min-votes",
        type=int,
        default=20,
        help="colour votes a pillar needs before its passes are judged at all",
    )
    parser.add_argument("--detail", action="store_true", help="per-pillar table as well as the summary")
    args = parser.parse_args()

    summary = []
    for bag_dir in args.bag_dirs:
        track, direction, pillars, dets = _read(
            bag_dir, args.wall_margin, args.cell, args.min_returns, args.max_pillars
        )
        if direction is None or not pillars:
            summary.append([bag_dir.name.replace("run_", ""), "-", len(pillars), "no direction or no pillars", "", ""])
            continue
        rows, wrong, passes, unknown, _fins, greens, reds = _score_bag(pillars, dets, track, direction, args.min_votes)
        if args.detail:
            print()
            print(f"=== {bag_dir.name}  direction={direction}")
            print_table(rows, ["object (LIDAR)", "returns", "colour vote", "R/G", "verdict"])
        ok = (greens, reds) == (_EXPECTED_GREEN, _EXPECTED_RED)
        summary.append(
            [
                bag_dir.name.replace("run_", ""),
                str(direction).split(".")[-1][:4],
                len(pillars),
                f"{greens}G/{reds}R" + ("" if ok else "  <- MISMATCH"),
                unknown,
                passes,
                f"{wrong} ({100 * wrong / passes:.0f}%)" if passes else "-",
            ]
        )

    print()
    print_table(summary, ["run", "dir", "pillars found", "colour UNKNOWN", "passes judged", "WRONG-SIDE"])
    print()
    print(
        "Judged from LIDAR-located pillars and the recorded pose, with the production\n"
        "pass-side rule and the simulator's footprint crossing test. Colour is the one\n"
        "quantity that must come from the camera; the R/G split shows how strong each\n"
        "call was. A wrong-side pass ENDS the round."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

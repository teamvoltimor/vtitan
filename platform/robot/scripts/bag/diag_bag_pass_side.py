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
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from shared.domain.enums import Axis, Direction  # noqa: E402
from shared.domain.models import Pose, SignColor, Waypoint  # noqa: E402

from scripts.common.bag_io import create_bags_parser, settled_direction  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
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


def _load(bag_dir: Path):  # noqa: ANN202
    """Read nav_debug rows and detection frames from one bag."""
    import json  # noqa: PLC0415

    from rclpy.serialization import deserialize_message  # noqa: PLC0415
    from std_msgs.msg import String  # noqa: PLC0415

    from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader  # noqa: PLC0415

    reader = open_reader(bag_dir)
    t0 = None
    rows, frames = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.NAV_DEBUG:
            rows.append((rel, decode_nav_debug(data)))
        elif topic == "/vision/detections":
            frames.append((rel, json.loads(deserialize_message(data, String).data) or []))
    return rows, frames


def _detections(payload):  # noqa: ANN001, ANN202
    """Rebuild typed detections from the wire payload."""
    from shared.domain.models import Detection  # noqa: PLC0415

    out = []
    for d in payload:
        try:
            colour = SignColor(d["class_name"])
        except (KeyError, ValueError):
            continue
        bbox = d.get("bbox")
        if not bbox or len(bbox) != 4:
            continue
        out.append(
            Detection(
                class_name=colour,
                confidence=float(d.get("confidence", 0.0)),
                bbox=tuple(float(v) for v in bbox),
                x=float(d.get("x", 0.0)),
                y=float(d.get("y", 0.0)),
                width=float(d.get("width", 0.0)),
                height=float(d.get("height", 0.0)),
                area=float(d.get("area", 0.0)),
            )
        )
    return out


def _passes(run: str, rows, frames) -> list[Pass]:  # noqa: ANN001
    """Replay the router, then judge each pillar against the SHIPPED rule.

    Legality is evaluated in the WORLD frame with ``pass_side_lateral_axis``,
    not from a restatement of the rule: it returns the axis and the sign the
    deformed waypoint must take relative to the pillar, so the same call decides
    what was REQUIRED, what the router COMMANDED (the deformed waypoint it
    actually produced) and what the chassis ACHIEVED (where it really went).
    Those three separate a routing error from an execution one.
    """
    tuning = get_tuning(None)
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)

    frame_i = 0
    best: dict[tuple[float, float], tuple] = {}

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            obs.extend(
                o
                for det in _detections(frames[frame_i][1])
                if (o := detection_to_observation(det, pose, tuning)) is not None
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
        committed = router.committed_sign_position
        if committed is None:
            continue
        rng = math.hypot(committed.x - d.pose_x, committed.y - d.pose_y)
        key = (round(committed.x, 1), round(committed.y, 1))
        if key in best and best[key][0] <= rng:
            continue
        colour = next(
            (s.color for s in router.signs if abs(s.x - committed.x) < 1e-9 and abs(s.y - committed.y) < 1e-9),
            SignColor.UNKNOWN,
        )
        rule = pass_side_lateral_axis(d.current_corridor, colour, direction)
        best[key] = (rng, colour, str(d.current_corridor), rule, committed, (d.pose_x, d.pose_y), deformed)

    out: list[Pass] = []
    for rng, colour, corridor, rule, sign_pos, robot, deformed in best.values():
        if rule is None:
            continue
        axis, want = rule
        idx = 0 if axis is Axis.X else 1
        sign_axis = sign_pos.x if idx == 0 else sign_pos.y
        achieved_delta = robot[idx] - sign_axis
        commanded_delta = deformed[idx] - sign_axis
        out.append(
            Pass(
                run=run,
                colour=colour,
                corridor=corridor,
                commanded=(1 if commanded_delta > 0 else -1) * want,
                achieved=(1 if achieved_delta > 0 else -1) * want,
                lateral_m=abs(achieved_delta),
            )
        )
    return out


def main() -> None:
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()

    rows_out = []
    tally = {"routing": 0, "execution": 0, "ok": 0}
    for bag in args.bag_dirs:
        rows, frames = _load(Path(bag))
        direction = settled_direction(rows)
        run = Path(bag).name.replace("run_", "")
        for p in _passes(run, rows, frames):
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
    if not rows_out:
        print("No sign passes reconstructed from these bags.")
        return
    print("== PASS SIDE, required vs commanded vs achieved")
    print_table(rows_out, ["run", "direction", "corridor", "colour", "clearance m", "verdict"])
    print()
    print(f"  commanded the WRONG side (routing):        {tally['routing']}")
    print(f"  commanded right, chassis went wrong (exec): {tally['execution']}")
    print(f"  correct:                                    {tally['ok']}")


if __name__ == "__main__":
    main()

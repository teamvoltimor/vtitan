r"""If the bay-exit sightings were ingested, what map would the router be handed?

Operator question: *"the robot throws away everything it sees from inside the
pocket -- if we kept it, would it know about the first pillar, or would it just
invent pillars?"*

``ingest_during_bay_exit`` ships OFF for exactly one reason: 37.3% of the
observations accepted during a clockwise exit land on the pillar the round later
commits to, and NOTHING was known about the other 62.7%. A map that invents
pillars is not an improvement on an empty one, and the corpus cannot referee the
question because its scenarios start OUTSIDE the bay, so the branch never runs
there. This script is the referee.

WHAT IT DOES. It rebuilds the production buffer off the bag -- the same
``detection_body_frame`` the gateway uses, the same RED/GREEN filter and the same
``settle_ticks`` budget ``_buffer_creep_sightings`` applies -- then replays it
through a FRESH ``SignRouter`` the way ``CoreNavigator.replay_sign_observations``
does: tick by tick, because the map needs ``MIN_HITS`` confirmations ACROSS ticks
before it publishes anything, and one batch would count once.

BOTH HEADING ARMS ARE REPLAYED, never one. The correction production applies is
``heading_delta``, which is exactly 0 or pi (it comes from ``TRAVEL_DIRS``), so
printing both arms costs nothing and buys a two-sided control: a replay that
looks good under BOTH arms is measuring something other than the pillars.

TRUTH, AND ITS LIMIT. "A real pillar" means a position the round ITSELF
committed to later, read off ``committed_sign_x_m``/``_y_m``. That is the
robot's own belief, not ground truth: a round that believed a ghost all the way
round scores that ghost as real here. It is still the right ruler, because the
question is whether the replayed map agrees with the map the round went on to
build and drive by -- and unlike a hand-placed layout it is recorded per round.

THREE MORE LIMITS, DECLARED:

* The poses are the ones the robot BELIEVED, and the pocket is where the
  localizer is worst. Junk caused by a bad pose and junk caused by the buffer
  are not separable here; both are junk the flag would ship.
* ``slot_sign_map`` is pinned true, so every published position IS a legal cell
  and "off-lattice" is 0 BY CONSTRUCTION. The junk measure is therefore WHICH
  cell, never whether it is a cell.
* It scores the MAP, never the outcome. The alternative round was never driven.

ONE THING IT IS LOOKING FOR SPECIFICALLY. The buffered path goes through
``get_sign_sightings``, which does NOT apply the pillar-aspect test that
``detection_to_observation`` uses to keep the magenta parking barrier -- which
reads RED under motion blur -- from seeding a sign. The barrier is in view from
inside the pocket, so the share of buffered samples that test would have
rejected is printed as its own column.

Usage::

    VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
    PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_bay_ingest_junk.py \
        ../../data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from shared.domain.enums import Axis, Direction, NavigatorPhase
from shared.domain.models import SignColor, TrafficSignObservation

from scripts.common.bag_io import create_bags_parser, decode_detections, read_vision_rows, settled_direction
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning
from src.navigation.corridor_estimator import section_from_heading
from src.navigation.planning.sign_discovery import detection_body_frame
from src.navigation.planning.sign_router import SignRouter
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.utils import wrap_angle

ARMS = (("delta=0", 0.0), ("delta=pi", math.pi))
"""The only two corrections production can apply; ``TRAVEL_DIRS`` admits no third."""

SAME_PILLAR_M = 0.10
"""Two believed positions within this are one pillar. Half the closest legal pair."""

PHYSICAL_MAX_SIGNS = 8
"""Two per section over four sections. The rulebook cap, not a tuning value."""


@dataclass(slots=True)
class Buffered:
    """One sighting as ``_buffer_creep_sightings`` would have kept it."""

    tick: int
    pose_x: float
    pose_y: float
    yaw: float
    color: SignColor
    range_m: float
    bearing_rad: float
    confidence: float
    barrier_shaped: bool


@dataclass(slots=True)
class RunResult:
    """One round's replay, both arms."""

    run: str
    direction: Direction | None
    exit_s: float = 0.0
    ticks: int = 0
    samples: int = 0
    barrier_shaped: int = 0
    target: tuple[float, float] | None = None
    believed: list[tuple[float, float]] = field(default_factory=list)
    aimed_side: int | None = None
    arms: dict[str, tuple[int, bool, int, str]] = field(default_factory=dict)


def _bay_window(rows) -> tuple[float, float] | None:  # noqa: ANN001
    """First and last ``/nav_debug`` time stamped BAY_EXIT, or None."""
    times = [rel for rel, d in rows if d.phase == NavigatorPhase.BAY_EXIT]
    return (times[0], times[-1]) if times else None


def _buffer(rows, frames, window, budget: int) -> tuple[list[Buffered], int]:  # noqa: ANN001
    """Rebuild the production buffer over the exit window.

    Walks NAV TICKS, not camera frames, and refolds the latest frame on every
    tick: ``_latest_detections`` is never cleared, so the gateway hands the same
    boxes back until a new frame lands, and that repetition across ticks is
    precisely what lets a track reach ``MIN_HITS``. Replaying one frame per tick
    would starve the map of the evidence it really got.
    """
    tuning = get_tuning(None)
    aspect_max = tuning.sign_discovery.max_pillar_aspect
    t_from, t_to = window
    out: list[Buffered] = []
    frame_i = 0
    latest: list[dict] | None = None
    tick = 0
    for rel, d in rows:
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            latest = frames[frame_i][1]
            frame_i += 1
        if rel < t_from or rel > t_to:
            continue
        if latest is None or d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        kept = []
        for det in decode_detections(latest):
            if det.color not in (SignColor.RED, SignColor.GREEN):
                continue
            body = detection_body_frame(det)
            if body is None:
                continue
            bbox = det.as_bbox()
            height = bbox.y_max - bbox.y_min
            width = bbox.x_max - bbox.x_min
            kept.append((det, body, height > 0 and width / height > aspect_max))
        if not kept:
            continue
        tick += 1
        for det, body, barrier_shaped in kept:
            out.append(
                Buffered(
                    tick=tick,
                    pose_x=d.pose_x,
                    pose_y=d.pose_y,
                    yaw=d.pose_yaw,
                    color=det.color,
                    range_m=body[0],
                    bearing_rad=body[1],
                    confidence=det.confidence,
                    barrier_shaped=barrier_shaped,
                )
            )
    # Same clip as production: the OLDEST go, because they were taken deepest
    # in the pocket where the camera sees least of the mat.
    if len(out) > budget:
        out = out[len(out) - budget :]
    return out, tick


def _replay(samples: list[Buffered], delta: float, direction: Direction | None, yaw_end: float):
    """Feed the frame-corrected buffer to a fresh router, tick by tick."""
    tuning = get_tuning(None)
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    corridor = section_from_heading(wrap_angle(yaw_end + delta), direction) if direction else None
    by_tick: dict[int, list[TrafficSignObservation]] = {}
    for s in samples:
        yaw = wrap_angle(s.yaw + delta)
        by_tick.setdefault(s.tick, []).append(
            TrafficSignObservation(
                world_x_m=s.pose_x + s.range_m * math.cos(yaw + s.bearing_rad),
                world_y_m=s.pose_y + s.range_m * math.sin(yaw + s.bearing_rad),
                color=s.color,
                confidence=s.confidence,
                detected_at_timestamp=0.0,
            )
        )
    for tick in sorted(by_tick):
        observations = by_tick[tick]
        anchor = (observations[0].world_x_m, observations[0].world_y_m)
        router.deform_waypoint(
            waypoint=anchor,
            robot_pos=anchor,
            robot_yaw=0.0,
            corridor=corridor,
            observations=observations,
            lidar_proposals=None,
        )
    return router.signs


def _distinct(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Collapse repeats of the same believed position."""
    out: list[tuple[float, float]] = []
    for p in points:
        if all(math.dist(p, q) > SAME_PILLAR_M for q in out):
            out.append(p)
    return out


def _aimed_side(rows, t_end: float) -> tuple[tuple[float, float] | None, int | None]:  # noqa: ANN001
    """The first pillar committed to after the exit, and which side the round aimed.

    The side is the RAW sign of the deformed target's offset from the believed
    pillar along the pass-side axis -- raw, not normalised against either
    colour, so the comparison downstream is against a multiplier of the same
    kind. It is the round's OWN verdict on the colour -- no colour is recorded
    in the bag -- so it can only be compared with a replayed colour, never used
    as truth on its own.
    """
    for rel, d in rows:
        if rel <= t_end:
            continue
        if d.committed_sign_x_m is None or d.committed_sign_y_m is None:
            continue
        target = (d.committed_sign_x_m, d.committed_sign_y_m)
        if d.sign_target_x_m is None or d.sign_target_y_m is None:
            return target, None
        corridor = corridor_for_position(*target)
        if corridor is None:
            return target, None
        # Either colour names the same AXIS -- the table's two multipliers are
        # negations on one axis -- so RED is asked only for the axis here, never
        # for its sign.
        rule = pass_side_lateral_axis(corridor, SignColor.RED, d.direction)
        if rule is None:
            return target, None
        offset = (d.sign_target_x_m - target[0]) if rule[0] == Axis.X else (d.sign_target_y_m - target[1])
        if abs(offset) < 1e-6:
            return target, None
        return target, 1 if offset > 0 else -1
    return None, None


def _colour_verdict(track, target: tuple[float, float], aimed: int | None, direction: Direction | None) -> str:  # noqa: ANN001
    """Does the replayed track's colour demand the side the round aimed at?

    Both sides of the comparison are raw axis signs: ``aimed`` is the sign of
    the recorded deformation, ``rule[1]`` the multiplier this colour demands on
    the same axis. A disagreement means the replayed map would have sent the
    robot round the other side of the pillar the round actually routed.
    """
    if aimed is None or direction is None:
        return "-"
    corridor = corridor_for_position(*target)
    if corridor is None:
        return "-"
    rule = pass_side_lateral_axis(corridor, track.color, direction)
    if rule is None:
        return "-"
    return "agree" if (rule[1] > 0) == (aimed > 0) else "WRONG"


def _run_row(res: RunResult) -> list[object]:
    """One printed line per round, both arms side by side."""
    row: list[object] = [
        res.run,
        "-" if res.direction is None else res.direction.value[:4],
        f"{res.exit_s:.1f}",
        res.ticks,
        res.samples,
        f"{100 * res.barrier_shaped / max(res.samples, 1):.0f}%",
        len(res.believed),
    ]
    for name, _ in ARMS:
        n, on_target, junk, colour = res.arms[name]
        row += [n, "YES" if on_target else "no", junk, colour]
    return row


def _measure(bag_dir: Path, budget: int, radius: float) -> RunResult | None:
    """Everything one bag has to say, or None when it has no bay exit to score."""
    rows, frames = read_vision_rows(bag_dir)
    window = _bay_window(rows)
    if window is None or not frames:
        return None
    direction = settled_direction(rows)
    samples, ticks = _buffer(rows, frames, window, budget)
    yaw_end = next((d.pose_yaw for rel, d in reversed(rows) if rel <= window[1] and d.pose_yaw is not None), 0.0)
    target, aimed = _aimed_side(rows, window[1])
    believed = _distinct([(d.committed_sign_x_m, d.committed_sign_y_m) for _, d in rows if d.committed_sign_x_m is not None])
    res = RunResult(
        run=bag_dir.name.replace("run_2026", ""),
        direction=direction,
        exit_s=window[1] - window[0],
        ticks=ticks,
        samples=len(samples),
        barrier_shaped=sum(1 for s in samples if s.barrier_shaped),
        target=target,
        believed=believed,
        aimed_side=aimed,
    )
    for name, delta in ARMS:
        tracks = _replay(samples, delta, direction, yaw_end) if samples else []
        on_target = target is not None and any(math.dist((t.x, t.y), target) <= radius for t in tracks)
        on_real = sum(1 for t in tracks if any(math.dist((t.x, t.y), p) <= radius for p in believed))
        colour = "-"
        if target is not None and tracks:
            nearest = min(tracks, key=lambda t: math.dist((t.x, t.y), target))
            if math.dist((nearest.x, nearest.y), target) <= radius:
                colour = _colour_verdict(nearest, target, aimed, direction)
        res.arms[name] = (len(tracks), on_target, len(tracks) - on_real, colour)
    return res


def main() -> int:
    parser = create_bags_parser(__doc__.split("\n\n")[0])
    parser.add_argument("--radius", type=float, default=0.35, help="how near a track must land to count as ON a pillar")
    args = parser.parse_args()

    tuning = get_tuning(None)
    budget = tuning.sign_router.settle_ticks
    print("== TUNING IN FORCE")
    print(f"   sign_router.settle_ticks         = {budget}   (the buffer's own cap)")
    print(f"   sign_router.slot_sign_map        = {tuning.sign_router.slot_sign_map}   (true: a published position IS a legal cell)")
    print(f"   sign_discovery.max_pillar_aspect = {tuning.sign_discovery.max_pillar_aspect}   (the test the buffered path SKIPS)")
    print(f"   on-pillar radius                 = {args.radius} m")

    results: list[RunResult] = []
    for bag_dir in args.bag_dirs:
        if not (bag_dir / "metadata.yaml").exists():
            continue
        try:
            res = _measure(bag_dir, budget, args.radius)
        except Exception as exc:  # noqa: BLE001 - a truncated recording is not a finding
            print(f"!! {bag_dir.name}: {type(exc).__name__}: {exc}")
            continue
        if res is not None:
            results.append(res)

    if not results:
        print("\nno bag carried a BAY_EXIT phase and a vision topic -- nothing below means anything")
        return 0

    print(f"\n== REPLAYED MAP AT HAND-OVER ({len(results)} rounds with a bay exit)")
    print_table(
        [_run_row(r) for r in results],
        ["run", "dir", "exit s", "ticks", "samples", "barrier-shaped", "believed later"]
        + [f"{name} {c}" for name, _ in ARMS for c in ("tracks", "on 1st", "junk", "colour")],
    )

    print("\n== BY ARM AND DIRECTION")
    summary = []
    for name, _ in ARMS:
        for direction in (Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE, None):
            group = [r for r in results if direction is None or r.direction == direction]
            if not group:
                continue
            tracks = sum(r.arms[name][0] for r in group)
            junk = sum(r.arms[name][2] for r in group)
            summary.append([
                name,
                "all" if direction is None else direction.value[:4],
                len(group),
                sum(1 for r in group if r.arms[name][1]),
                tracks,
                junk,
                f"{100 * junk / max(tracks, 1):.0f}%",
                sum(1 for r in group if r.arms[name][3] == "WRONG"),
                sum(1 for r in group if r.arms[name][0] > PHYSICAL_MAX_SIGNS),
            ])
    print_table(
        summary,
        ["arm", "dir", "rounds", "1st pillar found", "tracks", "junk", "junk share", "colour WRONG", "rounds > 8"],
    )
    print("\nA map is worth shipping only if it finds the first pillar in most rounds AND")
    print("its junk share is low. Finding it inside a pile of ghosts is not finding it:")
    print("the router steers around every track it believes, not just the right one.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

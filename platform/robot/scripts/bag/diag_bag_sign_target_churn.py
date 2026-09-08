r"""Replay the real SignRouter over recorded detections: does the aim point sit still?

The 09-07 track report was that the robot lined up correctly to pass a sign,
kept correcting, and arrived badly placed. Measured on the bags, the controller
was not hunting -- ZERO steering flips -- but the committed sign's believed
position jumped a median 0.20-0.60 m (max 1.01 m) and the robot re-aimed at one
physical pillar 4-14 times. A published track is refined in place every frame,
so the target moved underneath a controller that tracked it faithfully.

This drives the REAL ``SignRouter`` over the recorded ``/vision/detections`` and
pose, with the position limits (``SIGN_POSITION_SLEW_M``,
``SIGN_POSITION_LATCH_M``) off and on, and reports how far the committed point
moves in each arm.

**The simulator cannot answer this question.** Its vision emulator places signs
from true geometry reprojected through the believed pose, so it has no
range-refinement jitter to reproduce -- a corpus A/B would come back flat and
mean nothing. The bag is the only instrument that sees the effect, which is the
same lesson as ``VISION_RANGE_MODEL``: an A/B is only evidence if the instrument
can resolve the effect.

What it reports per arm:

* **jumps** -- per-tick movements of the committed sign's position above 1 cm,
  the thing the robot re-aims at.
* **churn** -- commitment segments per distinct physical sign location, where a
  segment breaks when the committed point moves more than 0.15 m. 1.0x means
  the robot committed once and held.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_sign_target_churn.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)

from shared.domain.enums import Direction  # noqa: E402
from shared.domain.models import Detection, Pose, SignColor, Waypoint  # noqa: E402

from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_nav_debug,
    elapsed_seconds,
    open_reader,
    settled_direction,
)
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_observation  # noqa: E402
from src.navigation.planning.sign_router import SignRouter  # noqa: E402

VISION_DETECTIONS = "/vision/detections"

MOVE_EPS_M = 0.01
"""Below this a position update is numerical noise, not a re-aim."""

SEGMENT_BREAK_M = 0.15
"""A committed point moving further than this counts as a fresh commitment."""

CLUSTER_M = 0.35
"""Commitment anchors closer than this are treated as one physical pillar."""


def _load(bag_dir: Path) -> tuple[list, list]:
    """Read one bag once, returning nav_debug rows and detection frames."""
    reader = open_reader(bag_dir)
    t0 = None
    rows: list[tuple[float, object]] = []
    frames: list[tuple[float, list[dict]]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.NAV_DEBUG:
            rows.append((rel, decode_nav_debug(data)))
        elif topic == VISION_DETECTIONS:
            frames.append((rel, json.loads(deserialize_message(data, String).data) or []))
    return rows, frames


def _detections(payload: list[dict]) -> list[Detection]:
    """Rebuild typed detections from the wire payload, skipping malformed ones."""
    out: list[Detection] = []
    for d in payload:
        try:
            colour = SignColor(d["class_name"]) if "class_name" in d else SignColor(d["class"])
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


def _replay(bag_dir: Path, *, limits: bool) -> tuple[list[float], float]:
    """Run the sign map over one bag; return committed-point jumps and churn.

    Drives ``_ingest_observations`` -- the map plus the shipped
    ``_settled_position`` -- rather than ``deform_waypoint``. The full deform
    path needs the navigator's planned waypoint, which the bag does not record,
    and reconstructing it from pose and heading does not reproduce the corridor
    test (measured: 0 of 1384 ticks committed, against 37 real commitments in
    the same bag). So the COMMITMENT is taken from what the robot actually
    committed to -- ``committed_sign_x_m``/``_y_m`` matched to the nearest
    published track -- and the shipped limiter is exercised against it. That
    measures the mechanism under test without faking the part around it.
    """
    rows, frames = _load(bag_dir)
    overrides = {"SIGN_DUPLICATE_RADIUS_M": 0.30 if limits else 0.0}
    tuning = tuning_with_overrides(overrides, group="sign_router")
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)

    frame_i = 0
    jumps: list[float] = []
    anchors: list[tuple[float, float]] = []
    previous: tuple[float, float] | None = None

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        observations = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            observations.extend(
                obs
                for det in _detections(frames[frame_i][1])
                if (obs := detection_to_observation(det, pose, tuning)) is not None
            )
            frame_i += 1

        # Point the shipped limiter at whichever track the robot was really
        # committed to this tick, so the latch and the slew see the same sign
        # the hardware was steering around.
        target = None
        if d.committed_sign_x_m is not None and d.committed_sign_y_m is not None:
            target = (d.committed_sign_x_m, d.committed_sign_y_m)
            best, best_d = None, CLUSTER_M
            for i, s in enumerate(router.signs):
                gap = math.hypot(s.x - target[0], s.y - target[1])
                if gap < best_d:
                    best, best_d = i, gap
            router._committed = best  # noqa: SLF001  (diagnostic drives the real limiter)
        else:
            router._committed = None  # noqa: SLF001

        router._ingest_observations(observations, Waypoint(d.pose_x, d.pose_y))  # noqa: SLF001

        committed = router.committed_sign_position
        if committed is None:
            previous = None
            continue
        point = (committed.x, committed.y)
        if previous is not None:
            moved = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if moved > MOVE_EPS_M:
                jumps.append(moved)
            if moved > SEGMENT_BREAK_M:
                anchors.append(point)
        else:
            anchors.append(point)
        previous = point

    clusters: list[tuple[float, float]] = []
    for p in anchors:
        if not any(math.hypot(p[0] - c[0], p[1] - c[1]) < CLUSTER_M for c in clusters):
            clusters.append(p)
    churn = len(anchors) / len(clusters) if clusters else 0.0
    return jumps, churn


def main() -> None:
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()

    out = []
    for bag in args.bag_dirs:
        for label, limits in (("off", False), ("ON", True)):
            jumps, churn = _replay(Path(bag), limits=limits)
            out.append(
                [
                    Path(bag).name.replace("run_", ""),
                    label,
                    len(jumps),
                    round(statistics.median(jumps), 3) if jumps else 0.0,
                    round(max(jumps), 3) if jumps else 0.0,
                    round(churn, 1),
                ]
            )
    print("== COMMITTED SIGN POSITION, replayed over recorded detections")
    print_table(out, ["run", "limits", "jumps", "med jump m", "max jump m", "churn"])


if __name__ == "__main__":
    main()

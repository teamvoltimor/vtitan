r"""Bag-replay pass-side judgement, shared by the diag_bag_* pass diagnostics.

``diag_bag_pass_side`` reconstructed every sign pass from a recorded bag and
judged it against the shipped rule; ``diag_bag_exec_failures``,
``diag_bag_pass_geometry``, ``diag_bag_pass_pairs``,
``diag_bag_pass_side_inner_outer``, ``diag_bag_side_correction_outcome`` and
``diag_bag_pair_crossing_*`` all imported its private ``_load``/``_passes`` to
avoid replaying the router themselves. That hidden library is the reason this
module exists: the replay and the ``Pass`` record are one API, and a change to
the judgement must not silently move six diagnostics.

Legality is evaluated in the WORLD frame with ``pass_side_lateral_axis``, not
from a restatement of the rule: it returns the axis and the sign the deformed
waypoint must take relative to the pillar, so the same call decides what was
REQUIRED, what the router COMMANDED (the deformed waypoint it actually
produced) and what the chassis ACHIEVED (where it really went). Those three
separate a routing error from an execution one.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from typing import TYPE_CHECKING

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.domain.enums import Axis, Direction
from shared.domain.models import Pose, SignColor

from scripts.common.bag_io import decode_detections, scan_to_ranges_angles, settled_direction
from scripts.common.stats import nearest_by_time
from src.navigation.planning.sign_discovery import detection_to_observation
from src.navigation.planning.sign_router import SignRouter
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.models import NavigatorDebugSnapshot

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
    taking the wheel). The corrections themselves often fail to reach the legal
    side or to avoid contact, and that is a claim about ONE manoeuvre type and
    needs the type to test; see
    ``adr:0059-pass-side-travel-relative-and-scorer-independence``."""

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


def collect_passes(
    run: str,
    rows: list[tuple[float, NavigatorDebugSnapshot]],
    frames: list[tuple[float, list[dict]]],
    scans: list[tuple[float, object]],
    tuning: NavigationTuning,
) -> tuple[list[Pass], int]:
    """Replay the router, then judge each pillar against the SHIPPED rule.

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
        # Judge with the SIGN's settled corridor, which is the one the router
        # itself keys the rule off (``router.py:528`` reads
        # ``self._sign_corridors[index]``). ``d.current_corridor`` is the
        # ROBOT's, and the two legitimately disagree at corners -- exactly where
        # the failures live. The robot's corridor reports far more routing errors
        # than the sign's; see
        # ``adr:0064-corridor-by-depth-and-clearance-budget``. The trap is stated
        # at the top of diag_bag_pass_side's docstring and its verdict line used
        # to commit it anyway.
        colour, sign_corridor = next(
            (
                (spec.color, section)
                for spec, section in router.lane_specs
                if abs(spec.x - committed.x) < 1e-9 and abs(spec.y - committed.y) < 1e-9
            ),
            (SignColor.UNKNOWN, d.current_corridor),
        )
        rule = pass_side_lateral_axis(sign_corridor, colour, direction)
        best[key] = (rng, colour, str(sign_corridor), rule, committed, (d.pose_x, d.pose_y), deformed)

    out: list[Pass] = []
    for key, (_rng, colour, corridor, rule, sign_pos, robot, deformed) in best.items():
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

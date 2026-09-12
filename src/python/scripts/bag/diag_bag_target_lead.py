r"""Is the steering TARGET ahead of the robot along the loop, or behind it?

The 2026-09-11 evening rounds all turned around mid-lap and drove back the way
they came, at POSITIVE commanded speed, with ``waypoint_index`` frozen and
every backward-index guard reading clean.

Hypothesis under test: ``WaypointController.select_target_point`` searches
FORWARD from ``waypoint_index`` around the whole closed loop and returns the
first candidate that is (a) ahead of the chassis in its local frame and (b) at
least ``lookahead_distance`` away. On a closed loop, once the chassis has
rotated toward the way it came, the waypoints just after the index read BEHIND
the chassis and get skipped, and the search keeps walking until it reaches the
stretch of path the robot ALREADY DROVE -- which is in front of the chassis and
therefore accepted. Pure pursuit then tracks the loop backwards, happily, with
a small heading error and a small crosstrack.

Measured here WITHOUT reconstructing the path, so no corridor-width/rotation
belief is needed: bearing about the track centre is a monotone proxy for
along-path progress on this circuit (the same measure
``diag_bag_reverse_lap.py`` uses).

Per tick:

* ``theta_robot``  = bearing of the pose about the track centre
* ``theta_target`` = bearing of the logged ``steer_target`` about the same
* ``lead``         = wrap(theta_target - theta_robot) * travel_sign, in
  degrees. POSITIVE = the target is ahead along the intended lap direction;
  NEGATIVE = the target sits on path the robot has already driven.

``travel_sign`` is the direction the PLANNED PATH runs, derived from the bag
itself: the net bearing change accumulated across exactly those ticks where
``waypoint_index`` ADVANCED. It is never taken from the direction estimator
(itself a suspect in this family of failures), and never from the early
trajectory (a start manoeuvre can sweep the bearing the wrong way and flip the
whole measurement -- it did, on run_20260911_172543, during development of this
script). Index-advancing ticks are by construction the stretches where the
robot really was making along-path progress, so they define the intended
direction even on a run that spends most of its time going the wrong way. The
adopted ``direction`` is printed alongside purely as a cross-check.

The reversal window is found the established way: the deepest DRAWDOWN from the
running maximum of signed unwrapped progress. Instantaneous wrong-way arcs read
zero on these runs; only cumulative drawdown sees the reversal.

Stats are reported inside the drawdown window and outside it, so a clean
control run can be carried through the same query.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_target_lead.py data/live/runs/run_*
    pixi run -e dev python scripts/bag/diag_bag_target_lead.py RUN --trace
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import TrackDimensions

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows, posed_rows

CENTER = (TrackDimensions.CENTER_COORD, TrackDimensions.CENTER_COORD)

_MIN_RADIUS_M = 0.30
"""Bearing about a point is undefined at the point; drop ticks nearer than this."""

_MAX_INDEX_STEP = 4
"""Largest ``waypoint_index`` jump still read as a forward advance rather than a re-seek."""

_TRACE_STEP_S = 1.0
"""Trace rows are thinned to roughly one per this many seconds."""

_FAR_TARGET_M = 1.0
"""A target this far out is several times any lookahead the search ever asks for."""

_MIN_USABLE_TICKS = 10
"""Below this a run has nothing to say and is reported as such, not analysed."""


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _unwrap(series: list[float]) -> list[float]:
    out: list[float] = []
    turns = 0.0
    prev: float | None = None
    for raw in series:
        if prev is not None:
            d = raw - prev
            if d > math.pi:
                turns -= 2 * math.pi
            elif d < -math.pi:
                turns += 2 * math.pi
        prev = raw
        out.append(raw + turns)
    return out


class Tick:
    """One usable control tick, reduced to the quantities this diagnostic compares."""

    __slots__ = (
        "angle_err_deg",
        "crosstrack",
        "lead",
        "phase",
        "speed",
        "t",
        "tgt_dist",
        "tgt_theta",
        "theta",
        "wp",
    )

    def __init__(
        self,
        t: float,
        theta: float,
        lead: float,
        angle_err_deg: float | None,
        crosstrack: float | None,
        speed: float | None,
        wp: int | None,
        phase: str,
        tgt_theta: float,
        tgt_dist: float,
    ) -> None:
        self.t = t
        self.theta = theta
        self.lead = lead
        self.angle_err_deg = angle_err_deg
        self.crosstrack = crosstrack
        self.speed = speed
        self.wp = wp
        self.phase = phase
        self.tgt_theta = tgt_theta
        self.tgt_dist = tgt_dist


def _path_sign(posed: list, raw_theta: list[float]) -> tuple[float, int]:
    """Which way the bearing moves when ``waypoint_index`` ADVANCES, and on how many ticks.

    This is the planned path's own orientation about the track centre. Ticks
    where the index did not advance (frozen, or knocked back by a re-seek) say
    nothing about it and are excluded.
    """
    total = 0.0
    n_used = 0
    span = max((s.waypoint_index for _, s in posed if s.waypoint_index is not None), default=0) + 1
    for i in range(1, len(posed)):
        prev_wp, wp = posed[i - 1][1].waypoint_index, posed[i][1].waypoint_index
        if prev_wp is None or wp is None or span <= 1:
            continue
        step = (wp - prev_wp) % span
        if not 1 <= step <= _MAX_INDEX_STEP:
            continue
        total += _wrap(raw_theta[i] - raw_theta[i - 1])
        n_used += 1
    return (1.0 if total >= 0 else -1.0), n_used


def _ticks(bag_dir: Path) -> tuple[list[Tick], int, int, int, str]:
    """Per-tick lead series, the path sign, ticks without a target, sign-evidence ticks, adopted direction."""
    rows, _ = load_nav_debug_rows(bag_dir)
    posed = [
        (t, s) for t, s in posed_rows(rows) if math.hypot(s.pose_x - CENTER[0], s.pose_y - CENTER[1]) >= _MIN_RADIUS_M
    ]
    if not posed:
        return [], 0, 0, 0, "none"
    raw_theta = [math.atan2(s.pose_y - CENTER[1], s.pose_x - CENTER[0]) for _, s in posed]
    unwrapped = _unwrap(raw_theta)
    sign, sign_ticks = _path_sign(posed, raw_theta)
    adopted = next((str(s.direction) for _, s in posed if s.direction), "none")

    out: list[Tick] = []
    missing = 0
    for (t, s), u in zip(posed, unwrapped, strict=True):
        tx, ty = s.steer_target_x, s.steer_target_y
        if not isinstance(tx, (int, float)) or not isinstance(ty, (int, float)):
            missing += 1
            continue
        tgt_theta = math.atan2(ty - CENTER[1], tx - CENTER[0])
        robot_theta = math.atan2(s.pose_y - CENTER[1], s.pose_x - CENTER[0])
        out.append(
            Tick(
                t=t,
                theta=u * sign,
                lead=math.degrees(_wrap(tgt_theta - robot_theta)) * sign,
                angle_err_deg=(math.degrees(s.angle_error_rad) if isinstance(s.angle_error_rad, float) else None),
                crosstrack=s.crosstrack_error_m,
                speed=s.commanded_speed_mps,
                wp=s.waypoint_index,
                phase=str(s.phase),
                tgt_theta=tgt_theta,
                tgt_dist=math.hypot(tx - s.pose_x, ty - s.pose_y),
            )
        )
    return out, int(sign), missing, sign_ticks, adopted


def _drawdown_window(ticks: list[Tick]) -> tuple[int, int, float]:
    """Indices (peak, trough) of the deepest drawdown from the running max, plus its size in degrees."""
    cur_peak_i = 0
    best = 0.0
    out = (0, 0, 0.0)
    for i, tk in enumerate(ticks):
        if tk.theta > ticks[cur_peak_i].theta:
            cur_peak_i = i
        drawdown = ticks[cur_peak_i].theta - tk.theta
        if drawdown > best:
            best = drawdown
            out = (cur_peak_i, i, math.degrees(drawdown))
    return out


def _fmt(vals: list[float]) -> str:
    if not vals:
        return "n=0"
    ordered = sorted(vals)
    last = len(ordered) - 1
    return (
        f"n={len(ordered)} p10={ordered[last // 10]:+.2f} p50={statistics.median(ordered):+.2f} "
        f"p90={ordered[min(last, 9 * len(ordered) // 10)]:+.2f} "
        f"min={ordered[0]:+.2f} max={ordered[last]:+.2f}"
    )


def _group_stats(label: str, grp: list[Tick]) -> None:
    leads = [x.lead for x in grp]
    behind = sum(1 for v in leads if v < 0)
    pct = 100 * behind / len(leads) if leads else 0.0
    print(f"  {label} lead_deg    {_fmt(leads)}  behind={behind}/{len(leads)} ({pct:.0f}%)")
    print(f"  {label} |angle_err| {_fmt([abs(x.angle_err_deg) for x in grp if x.angle_err_deg is not None])}")
    print(f"  {label} crosstrack  {_fmt([x.crosstrack for x in grp if isinstance(x.crosstrack, float)])}")
    print(f"  {label} speed_mps   {_fmt([x.speed for x in grp if isinstance(x.speed, float)])}")
    print(f"  {label} target_dist {_fmt([x.tgt_dist for x in grp])}")
    wps = [x.wp for x in grp if x.wp is not None]
    if wps:
        print(f"  {label} wp_index    first={wps[0]} last={wps[-1]} distinct={len(set(wps))}")


def _trace(inside: list[Tick]) -> None:
    print(f"  {'t':>7} {'lead':>7} {'wp':>5} {'aerr':>7} {'xtrk':>6} {'spd':>6}  phase")
    last = -math.inf
    for x in inside:
        if x.t - last < _TRACE_STEP_S:
            continue
        last = x.t
        aerr = f"{x.angle_err_deg:+.1f}" if x.angle_err_deg is not None else "-"
        xtrk = f"{x.crosstrack:.2f}" if isinstance(x.crosstrack, float) else "-"
        spd = f"{x.speed:.2f}" if isinstance(x.speed, float) else "-"
        print(f"  {x.t:7.1f} {x.lead:+7.1f} {x.wp!s:>5} {aerr:>7} {xtrk:>6} {spd:>6}  {x.phase}")


def _report(bag_dir: Path, trace: bool) -> None:
    ticks, sign, missing, sign_ticks, adopted = _ticks(bag_dir)
    print(f"\n=== {bag_dir.name} ===")
    if len(ticks) < _MIN_USABLE_TICKS:
        print(f"  too few usable ticks ({len(ticks)}); ticks without a logged target={missing}")
        return
    lo, hi, dd_deg = _drawdown_window(ticks)
    print(
        f"  path_sign={sign:+d} (from {sign_ticks} index-advancing ticks; adopted direction={adopted})  "
        f"ticks={len(ticks)}  no-target ticks={missing}  "
        f"deepest drawdown {dd_deg:.0f} deg over {ticks[hi].t - ticks[lo].t:.1f}s "
        f"(t={ticks[lo].t:.1f}..{ticks[hi].t:.1f})"
    )
    far = [x for x in ticks if x.tgt_dist > _FAR_TARGET_M]
    far_behind = sum(1 for x in far if x.lead < 0)
    print(
        f"  whole run: target_dist > {_FAR_TARGET_M:.1f} m on {len(far)}/{len(ticks)} ticks "
        f"({100 * len(far) / len(ticks):.1f}%), of which {far_behind} are BEHIND along the path"
    )
    inside = ticks[lo : hi + 1]
    outside = ticks[:lo] + ticks[hi + 1 :]
    _group_stats("IN-window ", inside)
    _group_stats("OUT-window", outside)
    tgt_unwrapped = [v * sign for v in _unwrap([x.tgt_theta for x in inside])]
    print(
        f"  IN-window  target swept {math.degrees(tgt_unwrapped[-1] - tgt_unwrapped[0]):+.0f} deg about the centre "
        f"(robot swept {math.degrees(ticks[hi].theta - ticks[lo].theta):+.0f} deg)"
    )
    if trace:
        _trace(inside)


def main() -> None:
    """Parse arguments and report every bag named on the command line."""
    parser = create_bags_parser("Signed along-loop lead of the steering target relative to the robot.")
    parser.add_argument("--trace", action="store_true", help="Print a thinned per-tick trace inside the window")
    args = parser.parse_args()
    for bag_dir in args.bag_dirs:
        try:
            _report(Path(bag_dir), args.trace)
        except Exception as exc:  # noqa: BLE001
            print(f"\n=== {Path(bag_dir).name} === FAILED: {type(exc).__name__}: {exc}")


if __name__ == "__main__":
    main()

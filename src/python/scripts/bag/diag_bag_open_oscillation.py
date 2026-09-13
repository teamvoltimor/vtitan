"""Is the Open zig-zag a CONTROL limit cycle, a PATH that moves, or a corner the chassis runs wide?

The 2026-09-12 Open rounds ran on a track whose inner walls formed a 1 m x 1 m
square, so the corridor is ~1.00 m. The car finished 3/3 laps in every round and
visibly snaked down the straights. This script characterises that snake and
attributes it.

SIX naive reads it exists to defeat -- each one was tried and failed first:

1. "``crosstrack_error_m`` oscillates, count its zero crossings."  It has none,
   ever. ``track_geometry.cross_track_error`` returns
   ``project_onto_path(...).distance_m``, UNSIGNED, and its own docstring says
   so. Sign changes on it are identically zero and mean nothing. The sign is
   RECONSTRUCTED here (see 3) and the reconstruction is VALIDATED against the
   published magnitude before any of it is believed.

2. "Mean |crosstrack| is 0.13-0.18 m, so it drifts."  A mean cannot separate a
   steady bias (parked off-centre, harmless) from an oscillation (crossing the
   centreline repeatedly, expensive). The discriminating statistics are the
   ZERO-CROSSING RATE and the spatial WAVELENGTH, measured against PATH
   DISTANCE and not time -- a slower car oscillating at the same spatial
   wavelength shows a lower temporal frequency and looks cured when nothing
   changed. Both are reported, plus the temporal period, so the two views can
   be reconciled instead of one silently standing in for the other.

3. "Segment the straights with ``path_turn_ahead_rad < 0.10``."  That leaves
   almost nothing: the corner preview is 0.80 m (``corner_preview_distance_m``)
   and a straight on this track is barely longer, so the preview is armed over
   most of the lap and the "straight" set collapses to a handful of ticks -- and
   with it the travel axis, which then comes out of a garbage chord. Straights
   are segmented on HEADING instead: contiguous ticks whose ``pose_yaw`` is
   within 0.25 rad of an axis. The lateral coordinate is then a WORLD axis, and
   the path's own lateral is the median of ``steer_target`` over the segment
   (target points lie on the planned path; the Open Challenge has no sign
   router, so no deformation can move them).

4. "The car wobbles" does not say WHICH loop wobbles. The same projection
   separates them: on a straight the planned path's lateral coordinate is
   CONSTANT, so a target residual comparable to the pose residual means
   target/path selection is the oscillator, while a target residual near zero
   means the path is stationary and the chassis is limit-cycling on it.

5. "Differentiate the gyro for the achieved turn radius."  ``/imu/data``
   ``angular_velocity`` is IDENTICALLY ZERO in these bags -- the BNO08x runs in
   UART-RVC mode, which publishes orientation only. A radius computed that way
   comes out infinite and nothing raises. Differentiating ``pose_yaw`` instead
   does not rescue it either: per-tick pose yaw noise puts |yaw rate| at p50
   0.35 rad/s on a STRAIGHT with the wheel centred, which does not separate from
   0.42 rad/s in a corner. So the achieved radius is measured here over a WHOLE
   CORNER instead: total heading change across the corner is ~1.57 rad, far
   above the noise, and ``R_achieved = path_length / |total heading change|``.
   That is a real measurement of what the chassis traced at race speed, and it
   needs neither the gyro nor a per-tick derivative. It inherits the ~10%
   over-read of bag pose path length, which is stated wherever it is printed.

6. "``lookahead_distance_m`` is a constant", and "the speed ladder is
   ``motion/speed.toml``".  Neither holds.
   ``WaypointController.select_lookahead`` ramps the lookahead between
   ``lookahead_long`` and ``lookahead_short`` on
   ``crosstrack / effective_transition`` and on
   ``path_turn_ahead_rad / corner_turn_threshold_rad``, where
   ``effective_transition`` is ``min(lookahead_transition, wall budget)`` and the
   budget is derived from the path's distance to the MAT EDGE -- a function of
   corridor width, so a width change silently re-scales that ramp.
   ``navigation/motion/speed.toml`` reads ``max_mps = 0.156``, but the motor
   profile ``profiles/rev-hd-hex-motor-6000rpm/motion/speed.toml`` overrides the
   whole Open ladder to 0.26/0.38/0.50 with a 0.55 cap. Occupancy of every rung
   is counted so the report names the ladder in force rather than the file that
   looks authoritative.

The planned corner radius is reconstructed too, by CIRCLE-FITTING the recorded
``steer_target`` points through each corner rather than trusting
``waypoints.toml``'s ``arc_radius = 0.45``: ``corner_arc_radius`` returns
``min(max_radius, max(w_entry, w_exit)/2 - center_bias_m)``, so on a 1.00 m
corridor with ``wide_center_bias_m = 0.10`` the ceiling never binds.

``R_min = 0.053 + 1.86 * v`` is also printed for reference, but it is an
EXTRAPOLATED fit above Obstacles speeds and every figure derived from it is
labelled as such. The achieved-radius measurement above is what carries weight.

Usage:
    VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' PYTHONPATH=. \
        pixi run -e dev python scripts/bag/diag_bag_open_oscillation.py \
        ../../data/live/runs/run_20260912_074544 ... --label WIDE
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows

# Shipped values, quoted only to LABEL the measurement. This script never loads
# tuning: hard-coding keeps the report explicit about what it assumed, and each
# is printed next to what the bag actually shows.
CORNER_TURN_THRESHOLD_RAD = 0.35  # config/navigation/motion/pursuit.toml
LOOKAHEAD_SHORT = 0.16  # pursuit.toml lookahead_short
LOOKAHEAD_LONG_OPEN = 0.24  # pursuit.toml open_lookahead_long (Open only)

HEADING_TOLERANCE_RAD = 0.15
"""How far from an axis a tick may sit and still count as "on a straight".

0.15 rad is 8.6 deg. Every tick inside the tolerance at either END of a corner
arc is arc that leaks into the straight, so the tolerance sets how much curvature
contaminates the straight statistics AND how much of the 90 deg sweep the corner
window keeps -- measured here as a corner heading change of 1.27 rad against the
geometric 1.57. Tighter is cleaner on both counts and costs only straight
coverage; 0.25 rad was tried and left 28 deg of arc inside the straights."""

MIN_STRAIGHT_M = 0.50

_FIT_RMS_MAX = 0.025
"""Circle-fit residual above which a corner's planned radius is not believed."""

_GAP_S = 0.30
"""Time gap above which the path-distance chain and every segment is broken.

The control loop runs at ~20 Hz, so 0.30 s is six missed ticks -- long enough
that a latched manoeuvre or an escape qualifies and a one-tick branch change
does not."""

# profiles/rev-hd-hex-motor-6000rpm/motion/speed.toml, Open ladder plus creep.
SPEED_RUNGS = {
    "min .050": 0.0499,
    "creep .152": 0.1521,
    "oSlow .26": 0.26,
    "oMed .38": 0.38,
    "oFast .50": 0.50,
}

R_MIN_INTERCEPT = 0.053
R_MIN_SLOPE = 1.86


def r_min_for(speed_mps: float) -> float:
    """Tightest circle the chassis traces at ``speed_mps``. EXTRAPOLATED at Open speeds."""
    return R_MIN_INTERCEPT + R_MIN_SLOPE * abs(speed_mps)


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _p(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round(q * (len(ordered) - 1)))))
    return ordered[idx]


def _med(values: list[float]) -> float:
    clean = [v for v in values if v is not None and not math.isnan(v)]
    return statistics.median(clean) if clean else float("nan")


def _mean(values: list[float]) -> float:
    clean = [v for v in values if v is not None and not math.isnan(v)]
    return statistics.fmean(clean) if clean else float("nan")


class Tick:
    """One path-tracking tick with everything this analysis needs."""

    __slots__ = (
        "ang",
        "clr",
        "clr_speed",
        "dev",
        "hdg_speed",
        "look",
        "newseg",
        "quad",
        "s",
        "speed",
        "steer",
        "t",
        "turn",
        "tx",
        "ty",
        "wp",
        "x",
        "xt",
        "y",
        "yaw",
    )

    def __init__(self, **kw: object) -> None:
        for k, v in kw.items():
            setattr(self, k, v)


def load_ticks(bag_dir: Path) -> tuple[list[Tick], dict[str, object]]:
    """Every tick carrying a POSE, plus whole-run context.

    The filter is the pose, NOT ``crosstrack_error_m``, and that distinction is
    load-bearing. ``crosstrack_error_m`` is published only on the
    ``normal_drive`` branch; the navigator drops to ``waypoint_reached`` for a
    tick every time it retires a waypoint, which on this track is ~120 times a
    round. Keying on the crosstrack therefore chops the run into ~120 fragments
    of a few ticks each, every one of them too short to define a travel axis or
    a wavelength -- measured here as 1 usable straight segment of 0.59 m out of
    a 20 m round before the fix. Those ticks have a perfectly good pose, so they
    are kept and only the per-branch fields go None.

    A segment is broken on a TIME gap instead (``_GAP_S``), which is what a
    latched manoeuvre or an escape actually looks like, rather than on a
    one-tick branch change that the chassis drove straight through.
    """
    rows, _ = load_nav_debug_rows(bag_dir)
    ticks: list[Tick] = []
    laps = 0
    lap_stamps: list[float] = []
    last_lap = 0
    maneuver_ticks = 0
    total_ticks = 0
    escapes = 0
    widths: list[float] = []
    start_widths: list[float] = []
    all_speeds: list[float] = []
    duration = rows[-1][0] if rows else 0.0
    prev: Tick | None = None
    s_acc = 0.0

    for t, snap in rows:
        total_ticks += 1
        escapes = max(escapes, snap.escape_count or 0)
        if snap.commanded_speed_mps is not None:
            all_speeds.append(snap.commanded_speed_mps)
        if snap.corridor_width_belief_m is not None:
            widths.append(snap.corridor_width_belief_m)
        if snap.start_measured_corridor_width_m is not None:
            start_widths.append(snap.start_measured_corridor_width_m)
        if snap.active_maneuver_type is not None:
            maneuver_ticks += 1
        lap = snap.laps_completed or 0
        if lap > last_lap:
            lap_stamps.append(t)
            last_lap = lap
        laps = max(laps, lap)

        if snap.pose_x is None or snap.pose_y is None or snap.pose_yaw is None:
            prev = None
            continue
        new = prev is None or (t - prev.t) > _GAP_S
        if not new:
            s_acc += math.hypot(snap.pose_x - prev.x, snap.pose_y - prev.y)
        tick = Tick(
            t=t,
            s=s_acc,
            x=snap.pose_x,
            y=snap.pose_y,
            yaw=snap.pose_yaw,
            quad=0,
            dev=0.0,
            xt=snap.crosstrack_error_m,
            look=snap.lookahead_distance_m,
            turn=abs(snap.path_turn_ahead_rad) if snap.path_turn_ahead_rad is not None else None,
            tx=snap.steer_target_x,
            ty=snap.steer_target_y,
            ang=snap.angle_error_rad,
            steer=snap.commanded_steering_norm,
            speed=snap.commanded_speed_mps,
            clr=snap.forward_clearance_m,
            clr_speed=snap.clearance_speed_mps,
            hdg_speed=snap.heading_speed_mps,
            wp=snap.waypoint_index,
            newseg=new,
        )
        ticks.append(tick)
        prev = tick

    _assign_axis(ticks)
    return ticks, {
        "laps": laps,
        "duration": duration,
        "lap_times": [b - a for a, b in zip([0.0, *lap_stamps], lap_stamps, strict=False)],
        "maneuver_frac": maneuver_ticks / max(total_ticks, 1),
        "escapes": float(escapes),
        "total_ticks": total_ticks,
        "width_belief": _med(widths),
        "width_belief_min": min(widths) if widths else float("nan"),
        "width_belief_max": max(widths) if widths else float("nan"),
        "start_width": _med(start_widths),
        "all_speeds": all_speeds,
    }


def _assign_axis(ticks: list[Tick], half_window: int = 3) -> None:
    """Fill ``quad``/``dev`` from a lightly SMOOTHED heading.

    Smoothed, because the axis assignment is a threshold on the heading and the
    raw ``pose_yaw`` crosses a quadrant boundary on single noisy ticks, which
    shatters a straight and randomises where a corner window starts. +/-3 ticks
    is 0.3 s against a measured oscillation period of 4-6 s, so the snake this
    script is measuring passes through untouched -- it is the per-tick localizer
    jitter that does not. ``yaw`` itself is left RAW: the corner's total heading
    change is a difference of endpoints, and smoothing endpoints would bias it.
    """
    for i, tk in enumerate(ticks):
        lo, hi = max(0, i - half_window), min(len(ticks), i + half_window + 1)
        block = ticks[lo:hi] if not any(x.newseg for x in ticks[lo + 1 : hi]) else [tk]
        sin_sum = sum(math.sin(w.yaw) for w in block)
        cos_sum = sum(math.cos(w.yaw) for w in block)
        smooth = math.atan2(sin_sum, cos_sum)
        quad = int(round(smooth / (math.pi / 2.0))) % 4
        tk.quad = quad
        tk.dev = wrap(smooth - quad * (math.pi / 2.0))


def circumradius(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    """Radius of the circle through three points; inf when they are collinear."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if abs(area2) < 1e-12:
        return float("inf")
    la = math.hypot(bx - cx, by - cy)
    lb = math.hypot(ax - cx, ay - cy)
    lc = math.hypot(ax - bx, ay - by)
    return la * lb * lc / (2.0 * abs(area2))


def planned_path_radius(ticks: list[Tick], span: int = 1) -> dict[str, float]:
    """The radius the PLANNED PATH actually asks for, read off the waypoint ring.

    ``steer_target`` is emitted in path order as the robot drives, so
    de-duplicating consecutive targets recovers the planned polyline in order
    without any circle fit and without needing to know where a corner is. The
    local radius is then the circumradius of three points spaced ``span``
    waypoints apart: near-infinite on a straight, the arc radius on a corner.

    ``span`` is 1, i.e. CONSECUTIVE waypoints, and that is not a detail.
    ``waypoints.toml`` ships ``num_intermediate_arc_points = 3``, so a 90 deg
    arc is a polyline of only five points about 22.5 deg apart. Three
    consecutive points on a circle reproduce that circle exactly; reach three
    waypoints either side (span 3, tried first) and the outer two land on the
    adjoining straights, which inflated the answer to 0.45-0.49 m.

    This replaces fitting a circle to the targets inside a corner WINDOW, which
    was tried first and is biased: the window's first and last targets sit on
    the adjoining straights, and a circle through arc-plus-tails fits with a
    small residual at a radius 25-45% too large (0.49-0.55 measured against a
    0.38 geometric expectation). Ordering the ring removes the window from the
    question entirely.

    The reported corner radius is the 10th percentile of the local radius --
    the arc is a minority of the ring (a 0.40 m arc is 0.63 m of a 1.63 m side),
    so the low tail is the arc and the median would be the straight.
    """
    ordered: list[tuple[float, float]] = []
    for tk in ticks:
        if tk.tx is None or tk.ty is None:
            continue
        pt = (tk.tx, tk.ty)
        if ordered and math.hypot(pt[0] - ordered[-1][0], pt[1] - ordered[-1][1]) < 1e-6:
            continue
        if ordered and math.hypot(pt[0] - ordered[-1][0], pt[1] - ordered[-1][1]) > 0.40:
            ordered = [pt]  # a re-seek jumped the ring; start a fresh run
            continue
        ordered.append(pt)
    radii: list[float] = []
    spacing: list[float] = []
    for i in range(span, len(ordered) - span):
        r = circumradius(ordered[i - span], ordered[i], ordered[i + span])
        if math.isfinite(r) and r < 20.0:
            radii.append(r)
        spacing.append(math.hypot(ordered[i][0] - ordered[i - 1][0], ordered[i][1] - ordered[i - 1][1]))
    return {
        "waypoints_seen": float(len(ordered)),
        "spacing_med": _med(spacing),
        "radius_p10": _p(radii, 0.10) if radii else float("nan"),
        "radius_p25": _p(radii, 0.25) if radii else float("nan"),
        "radius_med": _med(radii),
    }


def straight_segments(ticks: list[Tick]) -> list[list[Tick]]:
    """The stretches the chassis drove along one track axis, trimmed to the straight.

    Two-stage, and the second stage is the point. A run of constant heading
    QUADRANT already isolates one side of the loop, but it also carries half a
    corner arc at each end, which would put arc curvature into the straight's
    oscillation statistics. So each quadrant run is then TRIMMED to its first
    and last tick within ``HEADING_TOLERANCE_RAD`` of the axis.

    Trimming rather than filtering is deliberate: the snake itself swings the
    heading, and on these runs it exceeds the tolerance mid-straight often
    enough that a plain filter shatters a 1 m straight into fragments too short
    to carry a wavelength -- 4.5 m of usable straight out of 12 m driven, and 1
    corner detected out of 12. An interior excursion is the SIGNAL, not a
    segment boundary.
    """
    out: list[list[Tick]] = []
    run: list[Tick] = []

    def flush(block: list[Tick]) -> None:
        inside = [i for i, tk in enumerate(block) if abs(tk.dev) < HEADING_TOLERANCE_RAD]
        if not inside:
            return
        seg = block[inside[0] : inside[-1] + 1]
        if len(seg) > 3 and seg[-1].s - seg[0].s >= MIN_STRAIGHT_M:
            out.append(seg)

    for i, tk in enumerate(ticks):
        if i > 0 and (tk.newseg or tk.quad != ticks[i - 1].quad):
            flush(run)
            run = []
        run.append(tk)
    flush(run)
    return out


def axis_vectors(quad: int) -> tuple[tuple[float, float], tuple[float, float]]:
    """Unit travel direction and its left normal for a track axis."""
    theta = quad * (math.pi / 2.0)
    u = (math.cos(theta), math.sin(theta))
    return u, (-u[1], u[0])


def signed_crosstrack(
    segments: list[list[Tick]],
) -> tuple[list[list[tuple[float, float, float, float]]], dict[str, float]]:
    """Signed crosstrack per straight, its target residual, and the validation.

    Lateral coordinates are taken along the segment's own WORLD axis normal. The
    planned path's lateral is the MEDIAN of ``steer_target`` over the segment --
    the targets are waypoints of the planned path, so their median is the path's
    lateral position, and the median rather than the mean because the last few
    targets of a straight already sit on the corner arc.

    The validation is the point of the function, not a footnote:
    ``| pose_lateral - path_lateral |`` is compared tick by tick against the
    published ``crosstrack_error_m``. A high correlation and a small median
    absolute difference is what licenses reading zero crossings off the
    reconstruction. A poor fit means the reference is wrong and every
    oscillation number downstream has to be thrown away rather than believed.

    Returns per segment ``(s, signed_xt, target_residual, t)`` rows plus the
    validation statistics.
    """
    out: list[list[tuple[float, float, float, float]]] = []
    recon: list[float] = []
    published: list[float] = []
    for seg in segments:
        _u, n = axis_vectors(seg[0].quad)
        tgt_lat = [(tk.tx * n[0] + tk.ty * n[1]) if tk.tx is not None and tk.ty is not None else None for tk in seg]
        have = [v for v in tgt_lat if v is not None]
        if len(have) < 5:
            continue
        path_lat = statistics.median(have)
        rows: list[tuple[float, float, float, float]] = []
        for tk, p in zip(seg, tgt_lat, strict=False):
            q = tk.x * n[0] + tk.y * n[1]
            rows.append((tk.s, q - path_lat, (p - path_lat) if p is not None else float("nan"), tk.t))
            if tk.xt is not None:
                recon.append(abs(q - path_lat))
                published.append(tk.xt)
        out.append(rows)

    stats: dict[str, float] = {"n": float(len(recon))}
    if len(recon) > 30:
        mr, mp = statistics.fmean(recon), statistics.fmean(published)
        num = sum((a - mr) * (b - mp) for a, b in zip(recon, published, strict=False))
        dr = math.sqrt(sum((a - mr) ** 2 for a in recon))
        dp = math.sqrt(sum((b - mp) ** 2 for b in published))
        stats["corr"] = num / (dr * dp) if dr > 0 and dp > 0 else float("nan")
        stats["med_abs_diff"] = statistics.median([abs(a - b) for a, b in zip(recon, published, strict=False)])
        stats["recon_mean"] = mr
        stats["published_mean"] = mp
    return out, stats


def oscillation_of(series: list[list[tuple[float, float, float]]]) -> dict[str, float]:
    """Zero crossings, wavelength, period and peak-to-peak of a signed signal.

    Each row is ``(s, value, t)``. Crossings, path length and elapsed time
    accumulate PER SEGMENT, so a stretch the robot never drove contributes to
    none of them. Wavelength is twice the MEDIAN half-cycle: a full cycle is two
    crossings, and the median resists the one enormous half-cycle a corner entry
    contributes. Peak-to-peak is measured between the extrema of CONSECUTIVE
    half-cycles (one full swing) rather than as a global max-minus-min, which a
    single excursion would otherwise set for an entire run.
    """
    crossings = 0
    length = 0.0
    elapsed = 0.0
    half_lengths: list[float] = []
    half_times: list[float] = []
    extrema: list[float] = []
    swings: list[float] = []
    samples: list[float] = []

    for seg in series:
        if len(seg) < 3:
            continue
        length += seg[-1][0] - seg[0][0]
        elapsed += seg[-1][2] - seg[0][2]
        cross_s: list[float] = []
        cross_t: list[float] = []
        cur_ext = seg[0][1]
        seg_ext: list[float] = []
        for (_sa, va, _ta), (sb, vb, tb) in zip(seg, seg[1:], strict=False):
            samples.append(vb)
            if va * vb < 0.0:
                crossings += 1
                cross_s.append(sb)
                cross_t.append(tb)
                seg_ext.append(cur_ext)
                cur_ext = vb
            elif abs(vb) > abs(cur_ext):
                cur_ext = vb
        seg_ext.append(cur_ext)
        half_lengths.extend(b - a for a, b in zip(cross_s, cross_s[1:], strict=False))
        half_times.extend(b - a for a, b in zip(cross_t, cross_t[1:], strict=False))
        extrema.extend(abs(e) for e in seg_ext)
        swings.extend(abs(a) + abs(b) for a, b in zip(seg_ext, seg_ext[1:], strict=False))

    return {
        "path_m": length,
        "time_s": elapsed,
        "crossings": float(crossings),
        "cross_per_m": crossings / length if length > 0 else float("nan"),
        "cross_per_s": crossings / elapsed if elapsed > 0 else float("nan"),
        "wavelength_m": 2.0 * statistics.median(half_lengths) if half_lengths else float("nan"),
        "wl_p25": 2.0 * _p(half_lengths, 0.25) if half_lengths else float("nan"),
        "wl_p75": 2.0 * _p(half_lengths, 0.75) if half_lengths else float("nan"),
        "period_s": 2.0 * statistics.median(half_times) if half_times else float("nan"),
        "ptp_med": statistics.median(swings) if swings else float("nan"),
        "ptp_p90": _p(swings, 0.90) if swings else float("nan"),
        "peak_med": statistics.median(extrema) if extrema else float("nan"),
        "abs_mean": statistics.fmean(abs(v) for v in samples) if samples else float("nan"),
        "rms": math.sqrt(statistics.fmean([v * v for v in samples])) if samples else float("nan"),
        "n": float(len(samples)),
    }


def lagged_corr(pairs: list[list[tuple[float, float]]], max_lag: int = 30) -> tuple[int, float, float]:
    """Best lag (ticks) of the second signal behind the first, its corr, and corr at lag 0.

    A tracker merely correcting has its strongest correlation at a SMALL lag and
    a sign opposing the error. A limit cycle puts the peak near a QUARTER
    PERIOD, because the wheel is in quadrature with the error instead of against
    it -- that phase, not the magnitude, is the discriminator.
    """
    best_lag, best_corr, zero_corr = 0, 0.0, 0.0
    for lag in range(-max_lag, max_lag + 1):
        xs: list[float] = []
        ys: list[float] = []
        for seg in pairs:
            for i in range(len(seg)):
                j = i + lag
                if 0 <= j < len(seg):
                    xs.append(seg[i][0])
                    ys.append(seg[j][1])
        if len(xs) < 50:
            continue
        mx, my = statistics.fmean(xs), statistics.fmean(ys)
        num = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=False))
        dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
        dy = math.sqrt(sum((y - my) ** 2 for y in ys))
        corr = num / (dx * dy) if dx > 0 and dy > 0 else 0.0
        if lag == 0:
            zero_corr = corr
        if abs(corr) > abs(best_corr):
            best_lag, best_corr = lag, corr
    return best_lag, best_corr, zero_corr


def fit_circle(points: list[tuple[float, float]]) -> tuple[float, float]:
    """Kasa algebraic circle fit; ``(radius, rms residual)``, or NaNs if degenerate.

    Run on the recorded ``steer_target`` points through a corner, which are
    waypoints of the PLANNED path -- so this reads the radius the planner
    actually laid down rather than the ``arc_radius`` ceiling the config
    advertises. The two differ whenever ``corner_arc_radius``'s width term binds.

    The RMS residual is returned with the radius and is not decoration: a
    partial arc with a few straight-segment points on the end fits a circle
    perfectly happily and returns a radius that is an artefact of the fit. The
    caller rejects on the residual instead of believing every number this
    produces.
    """
    n = len(points)
    if n < 8:
        return float("nan"), float("nan")
    mx = statistics.fmean(p[0] for p in points)
    my = statistics.fmean(p[1] for p in points)
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for px, py in points:
        u, v = px - mx, py - my
        suu += u * u
        svv += v * v
        suv += u * v
        suuu += u * u * u
        svvv += v * v * v
        suvv += u * v * v
        svuu += v * u * u
    det = 2.0 * (suu * svv - suv * suv)
    if abs(det) < 1e-12:
        return float("nan"), float("nan")
    uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / det
    vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / det
    radius = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    rms = math.sqrt(statistics.fmean([(math.hypot(px - mx - uc, py - my - vc) - radius) ** 2 for px, py in points]))
    return radius, rms


def corner_windows(ticks: list[Tick], straights: list[list[Tick]]) -> dict[str, float]:
    """What each corner ASKED for and what the chassis actually traced.

    A corner is the stretch between the end of one straight and the start of the
    next, when the axis quadrant advanced by one. The achieved radius is
    ``path_length / |total heading change|`` over that whole window: the heading
    change is ~1.57 rad, orders above the per-tick pose-yaw noise that makes a
    differentiated yaw rate useless here, so this is a real measurement rather
    than a derivative of noise. It inherits the bag pose path length's ~10%
    over-read, which biases it HIGH -- i.e. against the finding it is used to
    test, so a positive result is conservative.

    The planned radius over the same window comes from circle-fitting the
    ``steer_target`` points, which are on the planned path.
    """
    achieved: list[float] = []
    planned: list[float] = []
    ratio: list[float] = []
    speeds: list[float] = []
    entry_speeds: list[float] = []
    extrap: list[float] = []
    peak_xt: list[float] = []
    peak_steer: list[float] = []
    dyaws: list[float] = []

    pos = {id(tk): i for i, tk in enumerate(ticks)}
    for a, b in zip(straights, straights[1:], strict=False):
        i0, i1 = pos[id(a[-1])], pos[id(b[0])]
        if i1 <= i0:
            continue
        window = ticks[i0 : i1 + 1]
        if any(tk.newseg for tk in window[1:]):
            continue
        dyaw = sum(wrap(y.yaw - x.yaw) for x, y in zip(window, window[1:], strict=False))
        if abs(dyaw) < 0.9 or abs(dyaw) > 2.0:
            continue
        length = window[-1].s - window[0].s
        if length < 0.15 or length > 2.0:
            continue
        r_ach = length / abs(dyaw)
        r_plan, fit_rms = fit_circle([(tk.tx, tk.ty) for tk in window if tk.tx is not None and tk.ty is not None])
        v_mean = _mean([tk.speed for tk in window if tk.speed is not None])
        achieved.append(r_ach)
        dyaws.append(abs(dyaw))
        speeds.append(v_mean)
        entry_speeds.append(window[0].speed if window[0].speed is not None else float("nan"))
        xts = [tk.xt for tk in window if tk.xt is not None]
        sts = [abs(tk.steer) for tk in window if tk.steer is not None]
        peak_xt.append(max(xts) if xts else float("nan"))
        peak_steer.append(max(sts) if sts else float("nan"))
        if not math.isnan(r_plan) and 0.05 < r_plan < 2.0 and fit_rms < _FIT_RMS_MAX:
            planned.append(r_plan)
            ratio.append(r_ach / r_plan)
            extrap.append(r_min_for(v_mean) / r_plan)

    return {
        "corners": float(len(achieved)),
        "r_achieved_med": _med(achieved),
        "r_achieved_p25": _p(achieved, 0.25) if achieved else float("nan"),
        "r_achieved_p75": _p(achieved, 0.75) if achieved else float("nan"),
        "r_planned_med": _med(planned),
        "r_ratio_med": _med(ratio),
        "corner_speed_med": _med(speeds),
        "entry_speed_med": _med(entry_speeds),
        "rmin_over_planned": _med(extrap),
        "peak_xt_med": _med(peak_xt),
        "peak_steer_med": _med(peak_steer),
        "dyaw_med": _med(dyaws),
    }


def exit_profile(
    straights: list[list[Tick]], span_m: float = 2.0, bin_m: float = 0.20
) -> list[tuple[float, float, int]]:
    """Unsigned crosstrack binned by distance after a straight BEGINS (i.e. after a corner).

    An overshoot that DECAYS along the straight is a corner the chassis ran
    wide out of; a flat profile is a free-running limit cycle the corner merely
    happens to precede. Nothing else in the bag separates those two.
    """
    bins: dict[int, list[float]] = {}
    for seg in straights:
        s0 = seg[0].s
        for tk in seg:
            d = tk.s - s0
            if d <= span_m and tk.xt is not None:
                bins.setdefault(int(d / bin_m), []).append(tk.xt)
    return [(k * bin_m, statistics.fmean(v), len(v)) for k, v in sorted(bins.items()) if len(v) >= 5]


def sign_runs(series: list[list[tuple[float, float, float]]]) -> dict[str, float]:
    """Mean/median length in TICKS of a constant-sign run.

    White noise has no runs: a signal whose sign persists for tens of ticks is
    structured whatever its magnitude, which is the one statistic that survives
    the pose noise floor. Reported alongside the distance-domain wavelength so
    the temporal and spatial views can be reconciled.
    """
    lengths: list[int] = []
    for seg in series:
        run = 0
        prev_sign = 0
        for _s, v, _t in seg:
            sign = 1 if v > 0 else (-1 if v < 0 else prev_sign)
            if sign == prev_sign or prev_sign == 0:
                run += 1
            else:
                lengths.append(run)
                run = 1
            prev_sign = sign
        if run > 0:
            lengths.append(run)
    return {
        "mean_ticks": _mean([float(v) for v in lengths]),
        "med_ticks": _med([float(v) for v in lengths]),
        "runs": float(len(lengths)),
    }


def speed_by_turn(ticks: list[Tick]) -> list[tuple[str, float, float, float, int]]:
    """Commanded speed, forward clearance and lookahead, binned by the upcoming turn.

    The clearance ladder in ``motion/clearance.toml`` is keyed on FORWARD
    CLEARANCE, not on path curvature, so this says whether a corner actually
    gets a speed cut or whether a wide corridor keeps clearance high until the
    corner is already underway.
    """
    edges = [(0.0, 0.10), (0.10, 0.35), (0.35, 0.70), (0.70, 9.0)]
    names = ["turn<0.10", "0.10-0.35", "0.35-0.70", ">0.70"]
    out: list[tuple[str, float, float, float, int]] = []
    for name, (lo, hi) in zip(names, edges, strict=False):
        sel = [tk for tk in ticks if tk.turn is not None and lo <= tk.turn < hi]
        if not sel:
            continue
        out.append(
            (
                name,
                _med([tk.speed for tk in sel if tk.speed is not None]),
                _med([tk.clr for tk in sel if tk.clr is not None]),
                _med([tk.look for tk in sel if tk.look is not None]),
                len(sel),
            )
        )
    return out


def analyse(bag_dir: Path) -> dict[str, object]:
    ticks, ctx = load_ticks(bag_dir)
    if len(ticks) < 50:
        return {"run": bag_dir.name, "error": f"only {len(ticks)} usable ticks"}

    straights = straight_segments(ticks)
    if not straights:
        return {"run": bag_dir.name, "error": "no straight segment found"}
    rows, validation = signed_crosstrack(straights)

    signed = [[(s, v, t) for s, v, _r, t in seg] for seg in rows]
    tgtres = [[(s, r, t) for s, _v, r, t in seg if not math.isnan(r)] for seg in rows]
    steer_series = [[(tk.s, tk.steer, tk.t) for tk in seg if tk.steer is not None] for seg in straights]
    dev_series = [[(tk.s, tk.dev, tk.t) for tk in seg] for seg in straights]

    osc = oscillation_of(signed)
    tgt = oscillation_of(tgtres)
    steer = oscillation_of(steer_series)
    dev = oscillation_of(dev_series)

    xt_steer_pairs = [
        [(v, tk.steer) for tk, (_s, v, _r, _t) in zip(seg, rr, strict=False) if tk.steer is not None]
        for seg, rr in zip(straights, rows, strict=False)
    ]

    looks = [tk.look for tk in ticks if tk.look is not None]
    looks_str = [tk.look for seg in straights for tk in seg if tk.look is not None]
    speeds_all = ctx["all_speeds"]
    rungs = {
        name: sum(1 for v in speeds_all if abs(v - target) < 0.005) / max(len(speeds_all), 1)
        for name, target in SPEED_RUNGS.items()
    }
    binds_heading = sum(
        1
        for tk in ticks
        if tk.hdg_speed is not None and tk.clr_speed is not None and tk.hdg_speed < tk.clr_speed - 1e-9
    )
    wp_back = sum(
        1
        for a, b in zip(ticks, ticks[1:], strict=False)
        if a.wp is not None and b.wp is not None and b.wp < a.wp - 1 and not b.newseg
    )

    return {
        "run": bag_dir.name,
        "laps": float(ctx["laps"]),
        "duration": ctx["duration"],
        "lap_times": ctx["lap_times"],
        "maneuver_frac": ctx["maneuver_frac"],
        "escapes": ctx["escapes"],
        "width_belief": ctx["width_belief"],
        "width_min": ctx["width_belief_min"],
        "width_max": ctx["width_belief_max"],
        "start_width": ctx["start_width"],
        "path_m": ticks[-1].s,
        "validation": validation,
        "osc": osc,
        "tgt": tgt,
        "steer": steer,
        "dev": dev,
        "sign_runs_xt": sign_runs(signed),
        "sign_runs_steer": sign_runs(steer_series),
        "lag": lagged_corr(xt_steer_pairs),
        "straight_count": float(len(straights)),
        "straight_path_m": sum(seg[-1].s - seg[0].s for seg in straights),
        "xt_straight_mean": _mean([tk.xt for seg in straights for tk in seg if tk.xt is not None]),
        "ang_abs_straight": _med([abs(tk.ang) for seg in straights for tk in seg if tk.ang is not None]),
        "look_med": _med(looks),
        "look_straight": _med(looks_str),
        "look_frac_long": sum(1 for v in looks if v >= LOOKAHEAD_LONG_OPEN - 1e-6) / len(looks),
        "look_frac_ramp": sum(1 for v in looks if LOOKAHEAD_SHORT + 1e-6 < v < LOOKAHEAD_LONG_OPEN - 1e-6) / len(looks),
        "look_frac_short": sum(1 for v in looks if v <= LOOKAHEAD_SHORT + 1e-6) / len(looks),
        "look_straight_frac_short": (
            sum(1 for v in looks_str if v <= LOOKAHEAD_SHORT + 1e-6) / len(looks_str) if looks_str else float("nan")
        ),
        "rungs": rungs,
        "speed_straight": _med([tk.speed for seg in straights for tk in seg if tk.speed is not None]),
        "heading_binds": binds_heading / len(ticks),
        "wp_backward": float(wp_back),
        "corner": corner_windows(ticks, straights),
        "ring": planned_path_radius(ticks),
        "exit_profile": exit_profile(straights),
        "speed_by_turn": speed_by_turn(ticks),
    }


def print_report(results: list[dict[str, object]], label: str) -> None:
    ok = [r for r in results if "error" not in r]
    for r in results:
        if "error" in r:
            print(f"  {r['run']:<24} SKIPPED: {r['error']}")
    if not ok:
        print("no usable bags")
        return

    print()
    print(f"=== {label}: RECONSTRUCTION CONTROL (|signed| vs published crosstrack_error_m) ===")
    print("A low corr here INVALIDATES every oscillation number below it.")
    print(f"{'run':<24}{'nTicks':>8}{'corr':>8}{'medDiff':>9}{'reconMean':>11}{'pubMean':>9}{'width':>8}")
    for r in ok:
        v = r["validation"]
        print(
            f"{r['run']:<24}{int(v.get('n', 0)):>8}{v.get('corr', float('nan')):>8.3f}"
            f"{v.get('med_abs_diff', float('nan')):>9.4f}{v.get('recon_mean', float('nan')):>11.4f}"
            f"{v.get('published_mean', float('nan')):>9.4f}{r['width_belief']:>8.3f}"
        )

    print()
    print(f"=== {label}: SIGNED crosstrack oscillation on HEADING-DEFINED straights ===")
    head = (
        f"{'run':<24}{'dur':>7}{'strM':>7}{'seg':>5}"
        f"{'xc/m':>7}{'xc/s':>7}{'lambda':>8}{'per_s':>7}{'ptp':>7}{'ptp90':>7}{'rms':>7}{'|xt|pub':>8}"
        f"{'runTk':>7}{'stRunTk':>8}"
    )
    print(head)
    print("-" * len(head))
    for r in ok:
        o = r["osc"]
        print(
            f"{r['run']:<24}{r['duration']:>7.1f}{r['straight_path_m']:>7.1f}{int(r['straight_count']):>5}"
            f"{o['cross_per_m']:>7.2f}{o['cross_per_s']:>7.2f}{o['wavelength_m']:>8.2f}{o['period_s']:>7.2f}"
            f"{o['ptp_med']:>7.3f}{o['ptp_p90']:>7.3f}{o['rms']:>7.3f}{r['xt_straight_mean']:>8.3f}"
            f"{r['sign_runs_xt']['mean_ticks']:>7.1f}{r['sign_runs_steer']['mean_ticks']:>8.1f}"
        )

    print()
    print(f"=== {label}: PATH or CHASSIS? target lateral residual vs signed pose offset ===")
    head2 = (
        f"{'run':<24}{'tgtRms':>8}{'tgtXc/m':>9}{'poseRms':>9}{'ratio':>7}"
        f"{'stXc/m':>8}{'stLam':>7}{'stPtp':>7}{'devRms':>8}{'lagTk':>7}{'corrLag':>8}{'corr0':>7}{'wpBack':>7}"
    )
    print(head2)
    print("-" * len(head2))
    for r in ok:
        o, t, st, dv = r["osc"], r["tgt"], r["steer"], r["dev"]
        lag, corr, zero = r["lag"]
        ratio = t["rms"] / o["rms"] if o["rms"] and o["rms"] > 0 else float("nan")
        print(
            f"{r['run']:<24}{t['rms']:>8.4f}{t['cross_per_m']:>9.2f}{o['rms']:>9.4f}{ratio:>7.2f}"
            f"{st['cross_per_m']:>8.2f}{st['wavelength_m']:>7.2f}{st['ptp_med']:>7.3f}{dv['rms']:>8.4f}"
            f"{lag:>7d}{corr:>8.2f}{zero:>7.2f}{int(r['wp_backward']):>7}"
        )

    print()
    print(f"=== {label}: CORNER -- what the path asked vs what the chassis traced ===")
    print("R_ach = corner path length / total heading change. Path length over-reads ~10% (biases R_ach HIGH).")
    head3 = (
        f"{'run':<24}{'nCor':>6}{'dyaw':>7}{'Rring':>8}{'R_ach':>8}{'ach/ring':>9}"
        f"{'Vcor':>7}{'Vent':>7}{'Rmin/R*':>9}{'pkXt':>7}{'pkSt':>7}"
    )
    print(head3)
    print("-" * len(head3))
    for r in ok:
        c = r["corner"]
        print(
            f"{r['run']:<24}{int(c['corners']):>6}{c['dyaw_med']:>7.2f}{r['ring']['radius_p10']:>8.3f}"
            f"{c['r_achieved_med']:>8.3f}{c['r_achieved_med'] / r['ring']['radius_p10']:>9.2f}{c['corner_speed_med']:>7.3f}"
            f"{c['entry_speed_med']:>7.3f}{c['rmin_over_planned']:>9.2f}{c['peak_xt_med']:>7.3f}"
            f"{c['peak_steer_med']:>7.3f}"
        )
    print("* Rmin/R uses the EXTRAPOLATED fit R_min = 0.053 + 1.86 v. Not a measurement.")

    print()
    print(f"=== {label}: LOOKAHEAD and SPEED ===")
    head4 = (
        f"{'run':<24}{'lkAll':>7}{'lkStr':>7}{'%long':>7}{'%ramp':>7}{'%shrt':>7}{'%shrtStr':>9}"
        f"{'spdStr':>8}{'hdg%':>7}{'man%':>6}{'esc':>5}{'wMin':>7}{'wMax':>7}"
    )
    print(head4)
    print("-" * len(head4))
    for r in ok:
        print(
            f"{r['run']:<24}{r['look_med']:>7.3f}{r['look_straight']:>7.3f}"
            f"{r['look_frac_long'] * 100:>7.1f}{r['look_frac_ramp'] * 100:>7.1f}{r['look_frac_short'] * 100:>7.1f}"
            f"{r['look_straight_frac_short'] * 100:>9.1f}{r['speed_straight']:>8.3f}"
            f"{r['heading_binds'] * 100:>7.1f}{r['maneuver_frac'] * 100:>6.1f}{int(r['escapes']):>5}"
            f"{r['width_min']:>7.3f}{r['width_max']:>7.3f}"
        )

    print()
    print(f"=== {label}: SPEED RUNG OCCUPANCY (% of all published ticks) ===")
    names = list(SPEED_RUNGS)
    print(f"{'run':<24}" + "".join(f"{n:>12}" for n in names) + f"{'other':>9}")
    for r in ok:
        rr = r["rungs"]
        print(
            f"{r['run']:<24}"
            + "".join(f"{rr[n] * 100:>12.1f}" for n in names)
            + f"{(1 - sum(rr.values())) * 100:>9.1f}"
        )

    print()
    print(f"=== {label}: pooled over {len(ok)} bags (median of per-bag values) ===")

    def m(path: str) -> float:
        vals = []
        for r in ok:
            node: object = r
            for key in path.split("."):
                if isinstance(node, dict):
                    node = node.get(key, float("nan"))
            if isinstance(node, (int, float)) and not (isinstance(node, float) and math.isnan(node)):
                vals.append(float(node))
        return statistics.median(vals) if vals else float("nan")

    lines: list[tuple[str, float | None]] = [
        ("-- reconstruction control --", None),
        ("corr(|signed|, published |xt|)", m("validation.corr")),
        ("median |difference| (m)", m("validation.med_abs_diff")),
        ("straight path covered per run (m)", m("straight_path_m")),
        ("straight segments per run", m("straight_count")),
        ("-- oscillation on straights --", None),
        ("signed xt zero crossings / m", m("osc.cross_per_m")),
        ("signed xt zero crossings / s", m("osc.cross_per_s")),
        ("signed xt wavelength (m/cycle)", m("osc.wavelength_m")),
        ("signed xt wavelength p25 (m)", m("osc.wl_p25")),
        ("signed xt wavelength p75 (m)", m("osc.wl_p75")),
        ("signed xt period (s/cycle)", m("osc.period_s")),
        ("signed xt peak-to-peak median (m)", m("osc.ptp_med")),
        ("signed xt peak-to-peak p90 (m)", m("osc.ptp_p90")),
        ("signed xt rms (m)", m("osc.rms")),
        ("published |xt| mean, straights (m)", m("xt_straight_mean")),
        ("sign-run length, xt (ticks)", m("sign_runs_xt.mean_ticks")),
        ("|angle_error| on straights (rad)", m("ang_abs_straight")),
        ("heading deviation rms (rad)", m("dev.rms")),
        ("-- the wheel --", None),
        ("steering zero crossings / m", m("steer.cross_per_m")),
        ("steering wavelength (m/cycle)", m("steer.wavelength_m")),
        ("steering period (s/cycle)", m("steer.period_s")),
        ("steering peak-to-peak (norm)", m("steer.ptp_med")),
        ("sign-run length, steering (ticks)", m("sign_runs_steer.mean_ticks")),
        ("-- path or chassis --", None),
        ("target lateral residual rms (m)", m("tgt.rms")),
        ("target residual zero crossings / m", m("tgt.cross_per_m")),
        ("waypoint backward jumps per run", m("wp_backward")),
        ("-- lookahead --", None),
        ("lookahead median, all (m)", m("look_med")),
        ("lookahead median, straights (m)", m("look_straight")),
        ("% ticks at long lookahead 0.24", m("look_frac_long") * 100.0),
        ("% ticks mid-ramp", m("look_frac_ramp") * 100.0),
        ("% ticks at short lookahead 0.16", m("look_frac_short") * 100.0),
        ("% straight ticks at short lookahead", m("look_straight_frac_short") * 100.0),
        ("-- corner --", None),
        ("corners measured per run", m("corner.corners")),
        ("planned corner radius, RING p10 (m)", m("ring.radius_p10")),
        ("planned corner radius, RING p25 (m)", m("ring.radius_p25")),
        ("planned path radius, RING median (m)", m("ring.radius_med")),
        ("waypoint spacing on the ring (m)", m("ring.spacing_med")),
        ("planned corner radius, window fit (m)", m("corner.r_planned_med")),
        ("ACHIEVED corner radius (m)", m("corner.r_achieved_med")),
        ("achieved / planned", m("corner.r_ratio_med")),
        ("corner mean speed (m/s)", m("corner.corner_speed_med")),
        ("corner entry speed (m/s)", m("corner.entry_speed_med")),
        ("R_min(v)/R_planned [EXTRAPOLATED]", m("corner.rmin_over_planned")),
        ("peak |xt| in a corner (m)", m("corner.peak_xt_med")),
        ("peak |steer| in a corner", m("corner.peak_steer_med")),
        ("-- run --", None),
        ("speed median, straights (m/s)", m("speed_straight")),
        ("% ticks heading-limited", m("heading_binds") * 100.0),
        ("corridor width belief (m)", m("width_belief")),
        ("start-measured corridor width (m)", m("start_width")),
        ("% ticks in a manoeuvre", m("maneuver_frac") * 100.0),
        ("escapes per run", m("escapes")),
        ("duration (s)", m("duration")),
        ("driven path (m) [over-reads ~10%]", m("path_m")),
    ]
    for name, value in lines:
        if value is None:
            print(f"  {name}")
        else:
            print(f"  {name:<42}{value:>10.4f}")

    ratio = m("tgt.rms") / m("osc.rms") if m("osc.rms") else float("nan")
    print(f"  {'target/pose rms ratio':<42}{ratio:>10.4f}")
    laps = [lt for r in ok for lt in r["lap_times"]]
    if laps:
        print(f"  {'lap time (s), median of ' + str(len(laps)):<42}{statistics.median(laps):>10.2f}")
    print(f"  {'xt->steer best lag (ticks)':<42}{statistics.median([r['lag'][0] for r in ok]):>10.1f}")
    print(f"  {'xt->steer corr at best lag':<42}{statistics.median([r['lag'][1] for r in ok]):>10.3f}")
    print(f"  {'xt->steer corr at lag 0':<42}{statistics.median([r['lag'][2] for r in ok]):>10.3f}")

    print()
    print(f"=== {label}: STRAIGHT-ENTRY PROFILE (mean published |xt| vs metres after the corner) ===")
    all_bins: dict[float, list[float]] = {}
    for r in ok:
        for d, v, _n in r["exit_profile"]:
            all_bins.setdefault(round(d, 2), []).append(v)
    for d in sorted(all_bins):
        print(f"  {d:>5.2f} m   {statistics.fmean(all_bins[d]):>7.4f}   (bags {len(all_bins[d])})")

    print()
    print(f"=== {label}: SPEED / CLEARANCE / LOOKAHEAD vs path_turn_ahead_rad ===")
    buckets: dict[str, list[tuple[float, float, float, int]]] = {}
    for r in ok:
        for name, spd, clr, look, n in r["speed_by_turn"]:
            buckets.setdefault(name, []).append((spd, clr, look, n))
    print(f"{'bucket':<12}{'speed':>9}{'clearance':>11}{'lookahead':>11}{'ticks':>9}")
    for name in ["turn<0.10", "0.10-0.35", "0.35-0.70", ">0.70"]:
        if name not in buckets:
            continue
        b = buckets[name]
        print(
            f"{name:<12}{_med([x[0] for x in b]):>9.3f}{_med([x[1] for x in b]):>11.3f}"
            f"{_med([x[2] for x in b]):>11.3f}{sum(x[3] for x in b):>9}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_dirs", type=Path, nargs="+")
    parser.add_argument("--label", default="SET")
    args = parser.parse_args()

    results: list[dict[str, object]] = []
    for bag in args.bag_dirs:
        try:
            results.append(analyse(bag))
        except Exception as exc:
            results.append({"run": bag.name, "error": repr(exc)})
    print_report(results, args.label)


if __name__ == "__main__":
    main()

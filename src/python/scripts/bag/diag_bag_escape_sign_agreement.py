r"""Does the escape push toward the side the ROUTER wanted to pass on?

Operator report 2026-09-12, after 32 Obstacles rounds on a 1.00 m track: *"the
escape angle has the wrong sign -- when the car escapes next to a pillar it is
trying to pass, the escape drives it away from the pass it needed."*

``diag_bag_escape_intent.py`` already answers the low-assumption half of this:
it compares the escape's steering against what the robot was commanding one
tick earlier. That test cannot distinguish "the escape fought the plan" from
"the plan was already pointed the wrong way" -- both read as AGREES/OPPOSES on
the same axis. This script asks the question the operator actually asked, which
needs the ROUTER's own intent rather than the last command: which side of the
COMMITTED pillar did the router aim past, and did the escape move the chassis
toward that side or away from it?

SIGN CONVENTION, stated once and used throughout:

* ``maneuver_steering`` / ``commanded_steering_norm``: **+1 is full LEFT lock**.
  Positive steering yaws the chassis counter-clockwise while travelling
  FORWARD. This is the stack-wide convention (see the LEFT/RIGHT branches of
  ``CollisionAvoidanceController.compute_escape_maneuver``).
* Ackermann REVERSE flips the yaw response, so the side the NOSE swings toward
  is ``sign(steering) * sign(speed)``. Called ``nose`` below: **+1 = the nose
  swings to the robot's LEFT**. Every reactive K-turn reverses
  (``escape.rev_speed = -0.20``), so its ``nose`` is the NEGATED steering sign.
* The frame for every lateral measurement is the ENTRY frame: the pose on the
  last tick before the manoeuvre latched. It is held FIXED across the episode
  precisely because a K-turn rotates the chassis -- measuring "after" in the
  post-escape body frame would let the rotation masquerade as translation.
  ``lat(p) = -(p.x-x0)*sin(yaw0) + (p.y-y0)*cos(yaw0)``, positive to the LEFT.
* Wanted pass side ``W = sign( lat(sign_target) - lat(committed_sign) )``. The
  router's own deformed aim point against the pillar it is routing around, so
  ``W=+1`` means "the router is aiming to pass on the pillar's LEFT". This is
  the router's decision, not a replay's re-derivation of the rulebook.
* Alignment ``A(t) = W * ( lat(robot(t)) - lat(committed_sign) )``. Positive =
  the chassis is on the side of the pillar the router wants to be on. The
  deliverable is ``dA = A(exit) - A(entry)``: **positive means the escape left
  the robot BETTER placed for the pass it was already committed to.**

CONTROLS, because a sign agreement is easy to fake:

* Episodes with no committed sign are counted and reported SEPARATELY, never
  folded in. They are the base rate that says whether the committed subset is
  special at all.
* ``committed_sign_x_m`` is only published on the ``normal_drive`` and
  ``escape_triggered`` phases (``_handle_stuck_escape`` builds a fresh
  snapshot, and a latched ``active_maneuver`` tick rebuilds too). So a STRICT
  count (the latch tick itself carried it) is printed alongside a WINDOWED one
  (the last committed sign within ``--commit-window`` ticks). If the two
  diverge wildly the windowed number is inheriting a stale belief and should
  not be trusted.
* ``nose`` agreement and the MEASURED lateral displacement are reported
  separately. They are not the same claim: a reverse that swings the nose left
  can still translate the body right, and if they disagree the steering sign is
  not the lever.
* ``dA`` is split BY nose-agreement. If agreeing episodes gain and disagreeing
  ones lose, flipping the sign is supported. If both lose, the sign is not the
  defect and the report must say so.

Usage::

    VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
    PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_escape_sign_agreement.py \
        ../../data/live/runs/run_20260912_09*
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows
from scripts.common.stats import fmean
from scripts.common.tables import print_table

# The rulebook track: a 3.0 m square with a 1.0 x 1.0 m inner island spanning
# [1, 2] on both axes. A pose outside the island's span on BOTH axes is in a
# corner; outside on exactly one is on a straight. Derived, not eyeballed.
_ISLAND_LO = 1.0
_ISLAND_HI = 2.0


def _sign(v: float | None, dead: float = 1e-6) -> int:
    if v is None:
        return 0
    return 1 if v > dead else (-1 if v < -dead else 0)


def _zone(x: float, y: float) -> str:
    """``corner`` or ``straight`` from the island span, or ``off-track``."""
    off_x = x < _ISLAND_LO or x > _ISLAND_HI
    off_y = y < _ISLAND_LO or y > _ISLAND_HI
    if off_x and off_y:
        return "corner"
    if off_x or off_y:
        return "straight"
    return "off-track"


@dataclass(slots=True)
class Episode:
    """One latched manoeuvre, entry to exit."""

    run: str
    kind: str
    t_entry: float
    t_exit: float
    corridor: str
    zone: str
    lap: int
    steer_mean: float
    speed_mean: float
    nose: int
    committed_strict: bool
    committed_windowed: bool
    wanted_side: int = 0
    align_before: float | None = None
    align_after: float | None = None
    lateral_move: float | None = None
    along_move: float | None = None
    sign_dist_before: float | None = None
    sign_dist_after: float | None = None
    pose_entry: tuple[float, float, float] | None = None
    sign_x_m: float | None = None
    sign_y_m: float | None = None
    trigger_to_sign_m: float | None = None
    trigger_range_m: float | None = None
    trigger_angle_rad: float | None = None

    @property
    def duration(self) -> float:
        """Wall-clock seconds the manoeuvre stayed latched."""
        return self.t_exit - self.t_entry

    @property
    def d_align(self) -> float | None:
        """``A(exit) - A(entry)``, or None when no committed sign anchors it."""
        if self.align_before is None or self.align_after is None:
            return None
        return self.align_after - self.align_before


def _collect(bag_dir: Path, commit_window: int, settle: int) -> tuple[list[Episode], dict]:
    """Every latched-manoeuvre episode in one bag, plus run-level counters."""
    rows, _ = load_nav_debug_rows(bag_dir)
    run = bag_dir.name
    episodes: list[Episode] = []
    meta = {
        "run": run,
        "ticks": len(rows),
        "duration": rows[-1][0] if rows else 0.0,
        "laps": max((s.laps_completed for _, s in rows), default=0),
        "direction": next((s.direction.value for _, s in rows if s.direction), "?"),
        "escape_count_peak": max((s.escape_count or 0 for _, s in rows), default=0),
        "committed_ticks": sum(1 for _, s in rows if s.committed_sign_x_m is not None),
        "posed_ticks": sum(1 for _, s in rows if s.pose_x is not None),
    }

    # A ring of the most recent ticks that PUBLISHED a committed sign, so a
    # latch tick that rebuilt its snapshot can still be attributed.
    last_commit: tuple[int, float, float] | None = None  # (tick index, x, y)
    open_ep: dict | None = None
    posed = [(i, t, s) for i, (t, s) in enumerate(rows) if s.pose_x is not None and s.pose_y is not None]
    by_index = {i: (t, s) for i, t, s in posed}

    for i, t, s in posed:
        if s.committed_sign_x_m is not None and s.committed_sign_y_m is not None:
            last_commit = (i, s.committed_sign_x_m, s.committed_sign_y_m)

        latched = s.active_maneuver_type is not None
        if latched and open_ep is None:
            # Entry frame is the last posed tick BEFORE the latch.
            prev = next((by_index[j] for j in range(i - 1, -1, -1) if j in by_index), None)
            frame = prev[1] if prev else s
            commit_strict = s.committed_sign_x_m is not None
            commit_win = last_commit is not None and (i - last_commit[0]) <= commit_window
            open_ep = {
                "t_entry": t,
                "x0": frame.pose_x,
                "y0": frame.pose_y,
                "yaw0": frame.pose_yaw if frame.pose_yaw is not None else 0.0,
                "kind": s.active_maneuver_type.value,
                "corridor": s.current_corridor.value if s.current_corridor else "?",
                "lap": s.laps_completed,
                "steer": [],
                "speed": [],
                "commit_strict": commit_strict,
                "commit_win": commit_win,
                "sign": (last_commit[1], last_commit[2]) if commit_win and last_commit else None,
                "target": (s.sign_target_x_m, s.sign_target_y_m)
                if s.sign_target_x_m is not None and s.sign_target_y_m is not None
                else None,
                # The ray the escape verdict was minimised over, as recorded on
                # the latch tick. Projected to a world point below, it says
                # whether the escape was running from the pillar the router had
                # a plan for (a mask leak) or from something else (a wall).
                "trig": (s.escape_trigger_angle_rad, s.escape_trigger_range_m)
                if s.escape_trigger_angle_rad is not None and s.escape_trigger_range_m is not None
                else None,
                "trig_pose": (s.pose_x, s.pose_y, s.pose_yaw if s.pose_yaw is not None else 0.0),
                "exit_i": i,
            }
        if latched and open_ep is not None:
            if s.maneuver_steering is not None:
                open_ep["steer"].append(s.maneuver_steering)
            if s.maneuver_speed_mps is not None:
                open_ep["speed"].append(s.maneuver_speed_mps)
            open_ep["exit_i"] = i
            open_ep["t_exit"] = t
        if not latched and open_ep is not None:
            episodes.append(_close(run, open_ep, by_index, posed, settle))
            open_ep = None

    if open_ep is not None:
        episodes.append(_close(run, open_ep, by_index, posed, settle))
    return episodes, meta


def _close(run: str, ep: dict, by_index: dict, posed: list, settle: int) -> Episode:
    """Finish an episode: measure the after-pose and the alignment delta."""
    x0, y0, yaw0 = ep["x0"], ep["y0"], ep["yaw0"]
    cos0, sin0 = math.cos(yaw0), math.sin(yaw0)

    def lat(px: float, py: float) -> float:
        return -(px - x0) * sin0 + (py - y0) * cos0

    def along(px: float, py: float) -> float:
        return (px - x0) * cos0 + (py - y0) * sin0

    steer = ep["steer"]
    speed = ep["speed"]
    steer_mean = fmean(steer) if steer else 0.0
    speed_mean = fmean(speed) if speed else 0.0
    nose = _sign(steer_mean) * _sign(speed_mean)

    # The after-pose: `settle` posed ticks past the exit, so the measurement
    # covers the manoeuvre AND the first moments of the recovery rather than
    # the instant the latch dropped (where the chassis is still mid-swing).
    order = [i for i, _, _ in posed]
    try:
        k = order.index(ep["exit_i"])
    except ValueError:
        k = len(order) - 1
    after_i = order[min(k + settle, len(order) - 1)]
    _, s_after = by_index[after_i]

    out = Episode(
        run=run,
        kind=ep["kind"],
        t_entry=ep["t_entry"],
        t_exit=ep.get("t_exit", ep["t_entry"]),
        corridor=ep["corridor"],
        zone=_zone(x0, y0),
        lap=ep["lap"],
        steer_mean=steer_mean,
        speed_mean=speed_mean,
        nose=nose,
        committed_strict=ep["commit_strict"],
        committed_windowed=ep["commit_win"],
    )
    out.pose_entry = (x0, y0, yaw0)
    out.lateral_move = lat(s_after.pose_x, s_after.pose_y)
    out.along_move = along(s_after.pose_x, s_after.pose_y)

    sign = ep["sign"]
    target = ep["target"]
    trig = ep.get("trig")
    if trig is not None:
        out.trigger_angle_rad, out.trigger_range_m = trig
    if sign is None or target is None:
        return out
    sx, sy = sign
    if trig is not None:
        tang, trng = trig
        px, py, pyaw = ep["trig_pose"]
        bearing = pyaw + tang
        out.trigger_to_sign_m = math.hypot(
            px + trng * math.cos(bearing) - sx, py + trng * math.sin(bearing) - sy
        )
    w = _sign(lat(target[0], target[1]) - lat(sx, sy))
    if w == 0:
        return out
    out.sign_x_m, out.sign_y_m = sx, sy
    out.wanted_side = w
    out.align_before = w * (0.0 - lat(sx, sy))  # robot is the frame origin
    out.align_after = w * (lat(s_after.pose_x, s_after.pose_y) - lat(sx, sy))
    out.sign_dist_before = math.hypot(sx - x0, sy - y0)
    out.sign_dist_after = math.hypot(sx - s_after.pose_x, sy - s_after.pose_y)
    return out


def _fmt(values: list[float]) -> str:
    """``n/mean/p50`` one-liner for a sample, or ``n=0`` when it is empty."""
    if not values:
        return "n=0"
    values = sorted(values)
    p50 = values[len(values) // 2]
    return f"n={len(values):3} mean={fmean(values):+.3f} p50={p50:+.3f}"


def main() -> int:  # noqa: C901, PLR0912, PLR0915
    """Inventory every latched escape, then score it against the router's aim."""
    parser = create_bags_parser(__doc__)
    parser.add_argument(
        "--commit-window", type=int, default=8,
        help="posed ticks back a committed sign may be inherited from when the latch tick rebuilt its snapshot",
    )
    parser.add_argument(
        "--settle", type=int, default=4,
        help="posed ticks past the latch drop at which the AFTER pose is measured",
    )
    parser.add_argument("--label", default="", help="tag printed in the header, e.g. 'obstacles 09-12'")
    args = parser.parse_args()

    all_eps: list[Episode] = []
    metas: list[dict] = []
    for bag_dir in args.bag_dirs:
        try:
            eps, meta = _collect(Path(bag_dir), args.commit_window, args.settle)
        except Exception as exc:  # noqa: BLE001 - a corrupt bag must not kill the corpus
            print(f"!! {Path(bag_dir).name}: {type(exc).__name__}: {exc}")
            continue
        all_eps.extend(eps)
        meta["episodes"] = len(eps)
        metas.append(meta)

    if not metas:
        print("no bags read -- nothing below means anything")
        return 0

    tag = f"  [{args.label}]" if args.label else ""
    print(f"== ESCAPE INVENTORY{tag}   {len(metas)} runs, {len(all_eps)} latched episodes")
    print("   <- if episodes is near zero, every number after this is empty")
    table = []
    for m in sorted(metas, key=lambda d: d["run"]):
        eps = [e for e in all_eps if e.run == m["run"]]
        table.append([
            m["run"].replace("run_", ""),
            m["direction"][:4],
            f"{m['duration']:.0f}",
            m["laps"],
            len(eps),
            f"{sum(e.duration for e in eps):.1f}",
            sum(1 for e in eps if e.zone == "corner"),
            sum(1 for e in eps if e.zone == "straight"),
            sum(1 for e in eps if e.committed_strict),
            sum(1 for e in eps if e.committed_windowed),
            f"{100 * m['committed_ticks'] / max(m['posed_ticks'], 1):.0f}%",
        ])
    print_table(
        table,
        ["run", "dir", "dur s", "laps", "eps", "esc s", "corner", "straight",
         "cmt(strict)", "cmt(win)", "committed ticks"],
    )

    total_dur = sum(m["duration"] for m in metas)
    esc_dur = sum(e.duration for e in all_eps)
    print(f"\n   escapes per minute of driving : {60 * len(all_eps) / max(total_dur, 1e-9):.2f}")
    print(f"   share of wall-clock in a latched manoeuvre : {100 * esc_dur / max(total_dur, 1e-9):.1f}%")

    print("\n== BY MANOEUVRE KIND")
    kinds = Counter(e.kind for e in all_eps)
    ktable = []
    for k, n in kinds.most_common():
        ks = [e for e in all_eps if e.kind == k]
        ktable.append([
            k, n, f"{100 * n / len(all_eps):.0f}%",
            f"{sum(e.duration for e in ks):.1f}",
            f"{100 * sum(e.duration for e in ks) / max(esc_dur, 1e-9):.0f}%",
            f"{fmean([e.duration for e in ks]):.2f}",
            f"{fmean([e.speed_mean for e in ks]):+.2f}",
            sum(1 for e in ks if e.committed_windowed),
        ])
    print_table(ktable, ["kind", "episodes", "share", "total s", "share of esc s", "mean s", "mean speed", "committed"])

    print("\n== WHERE (zone x corridor)")
    zt = []
    for z in ("corner", "straight", "off-track"):
        zs = [e for e in all_eps if e.zone == z]
        if not zs:
            continue
        per_corr = Counter(e.corridor for e in zs)
        zt.append([z, len(zs), f"{100 * len(zs) / len(all_eps):.0f}%",
                   " ".join(f"{c}:{n}" for c, n in per_corr.most_common())])
    print_table(zt, ["zone", "episodes", "share", "by corridor"])

    # ---------------- the core deliverable ----------------
    usable = [e for e in all_eps if e.d_align is not None and e.nose != 0]
    print(f"\n== SIGN AGREEMENT   usable {len(usable)} of {len(all_eps)} episodes")
    print("   usable = a committed sign AND a router aim point AND a non-zero nose-swing side")
    print("   <- if usable is near zero, nothing below means anything")
    if not usable:
        uncommitted = [e for e in all_eps if not e.committed_windowed]
        print(f"   (episodes with NO committed sign at all: {len(uncommitted)})")
        return 0

    # STEERING is the control variable, not the nose. Both forward and reverse
    # curve the BODY toward the steered side (the reverse only flips which way
    # the NOSE points while it does), so the sign that decides where the chassis
    # ends up is the raw steering sign. The nose split is printed after it as
    # the cross-check, and the two must be mirror images on a reversing escape.
    agree = [e for e in usable if _sign(e.steer_mean) == e.wanted_side]
    oppose = [e for e in usable if _sign(e.steer_mean) != e.wanted_side]
    print(f"\n  STEERING sign matches the side the router wanted : {len(agree):4}  ({100 * len(agree) / len(usable):.0f}%)")
    print(f"  STEERING sign is the opposite                    : {len(oppose):4}  ({100 * len(oppose) / len(usable):.0f}%)")
    nose_agree = sum(1 for e in usable if e.nose == e.wanted_side)
    print(f"  (cross-check, NOSE-swing side matches            : {nose_agree:4}"
          f"  ({100 * nose_agree / len(usable):.0f}%) -- the mirror, since every escape here reverses)")
    print("   <- near 50/50 means the escape is INDIFFERENT to the router, which is")
    print("      neither 'helps' nor 'fights' and must not be read as either")

    print("\n  did the escape leave the robot better placed?  dA = A(exit) - A(entry), metres")
    print("  positive = ended up further onto the side the router wanted")
    dt = []
    for label, group in (("steer AGREES", agree), ("steer OPPOSES", oppose), ("ALL", usable)):
        deltas = [e.d_align for e in group if e.d_align is not None]
        if not deltas:
            continue
        better = sum(1 for d in deltas if d > 0.0)
        dt.append([label, len(deltas), f"{fmean(deltas):+.3f}",
                   f"{sorted(deltas)[len(deltas) // 2]:+.3f}",
                   f"{better}/{len(deltas)} ({100 * better / len(deltas):.0f}%)"])
    print_table(dt, ["group", "n", "mean dA", "p50 dA", "improved"])

    print("\n  alignment BEFORE the escape (was the robot already on the right side?)")
    for label, group in (("steer AGREES", agree), ("steer OPPOSES", oppose)):
        befores = [e.align_before for e in group if e.align_before is not None]
        onside = sum(1 for b in befores if b > 0.0)
        print(f"    {label:13} {_fmt(befores)}   already on the wanted side: {onside}/{len(befores)}")

    print("\n  SIDE FLIPS -- the escape carried the chassis ACROSS the pillar's axis")
    ft = []
    for label, group in (("steer AGREES", agree), ("steer OPPOSES", oppose), ("ALL", usable)):
        ok = [e for e in group if e.align_before is not None and e.align_after is not None]
        lost = sum(1 for e in ok if e.align_before > 0.0 >= e.align_after)
        won = sum(1 for e in ok if e.align_before <= 0.0 < e.align_after)
        ft.append([label, len(ok), f"{lost} ({100 * lost / max(len(ok), 1):.0f}%)",
                   f"{won} ({100 * won / max(len(ok), 1):.0f}%)"])
    print_table(ft, ["group", "n", "correct -> WRONG side", "wrong -> correct side"])
    print("   <- 'correct -> WRONG side' is the rule-9.18 exposure: a pass that was")
    print("      going to be legal is not any more when the escape lets go")

    print("\n  WHICH SIDE the escape picked, against travel direction")
    print("  (_k_turn_steer_sign falls back to CW -> -1, CCW -> +1 when LIDAR ties)")
    st = []
    for k in sorted({e.kind for e in usable}):
        ks = [e for e in usable if e.kind == k]
        left = sum(1 for e in ks if _sign(e.steer_mean) > 0)
        right = sum(1 for e in ks if _sign(e.steer_mean) < 0)
        wl = sum(1 for e in ks if e.wanted_side > 0)
        st.append([k, len(ks), f"{left} ({100 * left / max(len(ks), 1):.0f}%)", right,
                   f"{wl} ({100 * wl / max(len(ks), 1):.0f}%)"])
    print_table(st, ["kind", "n", "steered LEFT", "steered RIGHT", "router wanted LEFT"])
    print("   <- if 'steered LEFT' is pinned near 0% or 100% while 'router wanted LEFT'")
    print("      sits near 50%, the side is a standing bias, not a reading")

    print("\n  MEASURED lateral displacement vs the wanted side (is steering even the lever?)")
    moved_right_way = sum(
        1 for e in usable
        if e.lateral_move is not None and _sign(e.lateral_move) == e.wanted_side
    )
    print(f"    body translated toward the wanted side: {moved_right_way}/{len(usable)}"
          f" ({100 * moved_right_way / len(usable):.0f}%)")
    agree_move = sum(
        1 for e in usable
        if e.lateral_move is not None and _sign(e.lateral_move) == e.nose
    )
    print(f"    body translated toward the NOSE side  : {agree_move}/{len(usable)}"
          f" ({100 * agree_move / len(usable):.0f}%)")
    print("   <- if the second number is near 50%, the steering SIGN does not control")
    print("      where the body ends up and flipping it cannot be the fix")

    print("\n  WHAT the escape was running from -- trigger ray projected to a world point")
    print("  (the escape mask is supposed to withhold the committed pillar's returns)")
    trig = [e.trigger_to_sign_m for e in usable if e.trigger_to_sign_m is not None]
    if trig:
        near = sum(1 for v in trig if v <= 0.15)
        mid = sum(1 for v in trig if 0.15 < v <= 0.30)
        print(f"    trigger ray landed WITHIN 0.15 m of the committed pillar: {near}/{len(trig)}"
              f" ({100 * near / len(trig):.0f}%)  <- mask LEAK: the escape is running from the sign")
        print(f"    0.15-0.30 m: {mid}/{len(trig)} ({100 * mid / len(trig):.0f}%)"
              f"    beyond 0.30 m: {len(trig) - near - mid}/{len(trig)}"
              f" ({100 * (len(trig) - near - mid) / len(trig):.0f}%)  <- a wall or an unmapped object")
        print(f"    trigger-to-sign distance: {_fmt(trig)}")
        rng = [e.trigger_range_m for e in usable if e.trigger_range_m is not None]
        print(f"    trigger RANGE from the chassis: {_fmt(rng)}"
              "   (min_valid_range_m = 0.044, obstacles contact_dist = 0.04)")
    else:
        print("    no trigger ray recorded on any latch tick -- this section is empty")

    print("\n  range to the committed pillar, before -> after")
    db = [e.sign_dist_before for e in usable if e.sign_dist_before is not None]
    da = [e.sign_dist_after for e in usable if e.sign_dist_after is not None]
    print(f"    before {_fmt(db)}")
    print(f"    after  {_fmt(da)}")

    print("\n  by manoeuvre kind, and by zone")
    kt = []
    for k in sorted({e.kind for e in usable}):
        for z in ("corner", "straight"):
            ks = [e for e in usable if e.kind == k and e.zone == z]
            if not ks:
                continue
            a = sum(1 for e in ks if _sign(e.steer_mean) == e.wanted_side)
            deltas = [e.d_align for e in ks if e.d_align is not None]
            kt.append([k, z, len(ks), f"{a}/{len(ks)} ({100 * a / len(ks):.0f}%)",
                       f"{fmean(deltas):+.3f}" if deltas else "-"])
    print_table(kt, ["kind", "zone", "usable", "steer agrees", "mean dA"])

    # ---------------- the control ----------------
    print("\n== CONTROL: episodes with NO committed sign")
    nocmt = [e for e in all_eps if not e.committed_windowed]
    print(f"  {len(nocmt)} of {len(all_eps)} episodes ({100 * len(nocmt) / len(all_eps):.0f}%) fired with no sign committed")
    print(f"  strict (latch tick itself carried the sign): "
          f"{sum(1 for e in all_eps if e.committed_strict)} of {len(all_eps)}")
    print("   <- a large gap between strict and windowed means the windowed attribution")
    print("      is inheriting a stale belief; prefer the strict column then")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

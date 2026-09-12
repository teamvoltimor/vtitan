r"""The APPROACH that ends in contact, against the approach that does not.

Every session so far has looked at the contact RECOVERY -- the
``already_touching`` reverse branch of ``compute_escape_maneuver``'s
SIDE_CORRECTION. Nobody has asked how the chassis got to a pillar's flank in
the first place. This script traces the run-up.

The unit of analysis is a COMMITMENT EPISODE: a contiguous stretch of ticks on
which ``committed_sign_x_m``/``_y_m`` name the same pillar (jumps beyond
``SAME_SIGN_M`` start a new episode). Each episode is classified:

* **CONTACT** -- at least one tick inside it latched a SIDE_CORRECTION with a
  NEGATIVE ``maneuver_speed_mps``. That negative speed is the discriminator:
  the creeping (non-touching) side correction runs at ``SIDE_CORRECTION_SPEED``
  forward, the ``already_touching`` branch swaps in ``ESCAPE_REV_SPEED``.
* **CLEAN** -- it did not. This is the WITHIN-RUN CONTROL. A profile that
  looks the same on both explains nothing.

For each episode the run-up is resampled by ALONG-RANGE rather than by time:
``along`` is the forward component of (pillar - robot) in the robot frame, so
1.40 m is the router's activation distance and 0.0 m is abeam. ``across`` is
the perpendicular component -- the lateral separation that decides contact.
Reading the profile against ``along`` instead of against the clock makes the
CONTACT and CLEAN curves directly comparable however fast each was driven.

WHAT THE NUMBERS SHOULD BE IF EVERYTHING IS HEALTHY
---------------------------------------------------
Chassis half-width 0.097 m, sign half-width 0.025 m, so bodies overlap below
``CONTACT_GAP_M`` 0.122 m centre-to-centre. ``sign_clearance_margin_m`` 0.10
puts the COMMANDED lateral at 0.222 m. In a 1.0 m corridor with the pillar on
the centreline the chassis centre may sit at most 0.403 m from it before the
far wall touches, so a healthy profile rises to ~0.222 m by the time ``along``
falls under the 0.9 m lane ramp and HOLDS there to abeam. A profile that peaks
above 0.222 and then falls under 0.122 was DIVERTED; one that never reaches
0.222 was never going to clear.

ATTRIBUTION
-----------
Every tick of the run-up carries a d|across| and a latched-manoeuvre flag, so
the lateral the episode lost is split into what it lost while an escape owned
the steering and what it lost while the router did. That is the test of whether
the contact recovery (A) and the opposing manoeuvre on an already-legal pass
(B) are one mechanism: if the lateral is lost on manoeuvre ticks in both, they
are.

TRIGGER PROVENANCE
------------------
``escape_trigger_angle_rad``/``_range_m`` are recorded from the MASKED scan
(``navigator.py`` computes ``escape_trigger`` from ``escape_ranges``), so
projecting that ray into the world and measuring its distance to the committed
pillar is a direct test of the 2026-09-11 escape-mask repair (``4fc0fbab``). A
trigger sitting ON the committed pillar means the mask did not withhold it.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_contact_approach.py \
        data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (models <-> enums import cycle)
from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows  # noqa: E402
from scripts.common.stats import percentile  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants.track import TrackDimensions  # noqa: E402
from shared.domain.enums import ManeuverType  # noqa: E402

CONTACT_GAP_M = 0.122
"""Chassis half-width + sign half-width: below this the bodies overlap."""

COMMANDED_LATERAL_M = 0.222
"""CONTACT_GAP_M + sign_clearance_margin_m (0.10), the router's ask."""

FAR_WALL_LIMIT_M = 0.403
"""Chassis centre to a centreline pillar with the far wall just touching."""

SAME_SIGN_M = 0.35
"""A committed belief that jumps further than this is a DIFFERENT pillar."""

ALONG_BINS = (1.40, 1.20, 1.00, 0.80, 0.60, 0.45, 0.30, 0.20, 0.10)
"""Along-range gates the run-up is resampled at, activation distance first."""


def _num(v) -> bool:  # noqa: ANN001
    return isinstance(v, (int, float))


@dataclass
class Tick:
    t: float
    x: float
    y: float
    yaw: float
    along: float
    across: float
    abs_across: float
    maneuver: str | None
    man_speed: float | None
    man_steer: float | None
    contact: bool
    """SIDE_CORRECTION latched with a negative speed: the already_touching branch."""
    speed: float | None
    fwd_clear: float | None
    trig_ang: float | None
    trig_rng: float | None


@dataclass
class Episode:
    run: str
    sign_x: float
    sign_y: float
    ticks: list[Tick] = field(default_factory=list)

    @property
    def contact(self) -> bool:
        return any(t.contact for t in self.ticks)

    @property
    def first_contact_i(self) -> int | None:
        for i, t in enumerate(self.ticks):
            if t.contact:
                return i
        return None

    def runup(self) -> list[Tick]:
        """Ticks before the first contact, or the whole episode when clean."""
        i = self.first_contact_i
        return self.ticks[:i] if i is not None else list(self.ticks)


def _episodes(run: str, rows) -> list[Episode]:  # noqa: ANN001
    out: list[Episode] = []
    cur: Episode | None = None
    for t, s in rows:
        ok = (
            _num(s.committed_sign_x_m)
            and _num(s.committed_sign_y_m)
            and _num(s.pose_x)
            and _num(s.pose_y)
            and _num(s.pose_yaw)
        )
        if not ok:
            cur = None
            continue
        sx, sy = float(s.committed_sign_x_m), float(s.committed_sign_y_m)
        if cur is None or math.hypot(sx - cur.sign_x, sy - cur.sign_y) > SAME_SIGN_M:
            cur = Episode(run=run, sign_x=sx, sign_y=sy)
            out.append(cur)
        # Keep the belief fresh; the episode is named by its first sighting but
        # the geometry is measured against what the router held THIS tick.
        cur.sign_x, cur.sign_y = sx, sy
        rx, ry, yaw = float(s.pose_x), float(s.pose_y), float(s.pose_yaw)
        dx, dy = sx - rx, sy - ry
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        across = -dx * math.sin(yaw) + dy * math.cos(yaw)
        man = s.active_maneuver_type
        man_s = str(man.value) if isinstance(man, ManeuverType) else (str(man) if man else None)
        mspd = s.maneuver_speed_mps if _num(s.maneuver_speed_mps) else None
        cur.ticks.append(
            Tick(
                t=t,
                x=rx,
                y=ry,
                yaw=yaw,
                along=along,
                across=across,
                abs_across=abs(across),
                maneuver=man_s,
                man_speed=mspd,
                man_steer=s.maneuver_steering if _num(s.maneuver_steering) else None,
                contact=bool(man_s == ManeuverType.SIDE_CORRECTION.value and mspd is not None and mspd < 0.0),
                speed=s.commanded_speed_mps if _num(s.commanded_speed_mps) else None,
                fwd_clear=s.forward_clearance_m if _num(s.forward_clearance_m) else None,
                trig_ang=s.escape_trigger_angle_rad if _num(s.escape_trigger_angle_rad) else None,
                trig_rng=s.escape_trigger_range_m if _num(s.escape_trigger_range_m) else None,
            )
        )
    return out


def _profile(ep: Episode) -> dict[float, float]:
    """|across| on the LAST genuine crossing of each ALONG_BINS gate.

    A gate the episode never crossed -- because the router only committed
    INSIDE it -- is reported absent rather than back-filled with the entry
    tick. Back-filling would make a late commitment look like a wide approach
    that simply held, which is the opposite of what it is.
    """
    out: dict[float, float] = {}
    ticks = ep.runup()
    for gate in ALONG_BINS:
        crossings = [
            ticks[i] for i in range(1, len(ticks)) if ticks[i - 1].along > gate >= ticks[i].along
        ]
        if crossings:
            out[gate] = crossings[-1].abs_across
    return out


def _attribute(ep: Episode, whole: bool = False) -> tuple[float, float, int, int]:
    """Split |across| change into manoeuvre-owned and router-owned metres.

    ``whole=False`` reads the RUN-UP only (before any contact), which is what
    "how did it get there" asks. ``whole=True`` reads the entire commitment,
    which is what "who spends the lateral on a pass" asks -- and the contact
    recovery itself lives only in the second.

    Returns (lost_under_manoeuvre, lost_under_router, man_ticks, free_ticks),
    where "lost" is the summed NEGATIVE d|across| over closing ticks.
    """
    src = ep.ticks if whole else ep.runup()
    ticks = [t for t in src if t.along > 0.0]
    lost_man = lost_free = 0.0
    n_man = n_free = 0
    for a, b in zip(ticks, ticks[1:]):
        d = b.abs_across - a.abs_across
        if b.maneuver is not None:
            n_man += 1
            if d < 0:
                lost_man += -d
        else:
            n_free += 1
            if d < 0:
                lost_free += -d
    return lost_man, lost_free, n_man, n_free


def _wall_dist(x: float, y: float) -> float:
    """Distance from a world point to the nearest TRACK WALL face.

    The outer boundary is the mat edge at MIN_COORD/MAX_COORD; the inner one is
    the CORNER_MIN..CORNER_MAX square. Both are fixed by the rules, so this
    needs no ground truth -- only the same pose every other field here is
    already measured against.
    """
    outer = min(
        x - TrackDimensions.MIN_COORD,
        TrackDimensions.MAX_COORD - x,
        y - TrackDimensions.MIN_COORD,
        TrackDimensions.MAX_COORD - y,
    )
    lo, hi = TrackDimensions.CORNER_MIN, TrackDimensions.CORNER_MAX
    dx = max(lo - x, 0.0, x - hi)
    dy = max(lo - y, 0.0, y - hi)
    inner = math.hypot(dx, dy) if (dx or dy) else -min(x - lo, hi - x, y - lo, hi - y)
    return min(abs(outer), abs(inner))


def _trigger_world(tk: Tick) -> tuple[float, float] | None:
    if tk.trig_ang is None or tk.trig_rng is None or not math.isfinite(tk.trig_rng):
        return None
    b = tk.yaw + tk.trig_ang
    return (tk.x + tk.trig_rng * math.cos(b), tk.y + tk.trig_rng * math.sin(b))


def _p(vals, q) -> float:  # noqa: ANN001
    return percentile(vals, q) if vals else math.nan


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--link-window", type=float, default=6.0, help="seconds a contact recovery may precede a wrong-side verdict and still count")
    parser.add_argument("--min-ticks", type=int, default=5, help="drop episodes shorter than this")
    parser.add_argument("--detail", type=int, default=0, help="print per-episode rows for the first N contact episodes per run")
    args = parser.parse_args()

    all_eps: list[Episode] = []
    run_rows: dict[str, list] = {}
    per_run: list[list[object]] = []

    for bag_dir in args.bag_dirs:
        run = bag_dir.name
        try:
            rows, _ = load_nav_debug_rows(bag_dir)
        except Exception as exc:  # noqa: BLE001
            print(f"!! {run}: FAILED to read ({exc})")
            continue
        run_rows[run] = rows
        eps = [e for e in _episodes(run, rows) if len(e.ticks) >= args.min_ticks]
        all_eps.extend(eps)
        drive = [
            s
            for _, s in rows
            if _num(s.commanded_speed_mps) and abs(float(s.commanded_speed_mps)) > 1e-3
        ]
        contact_ticks = sum(
            1
            for _, s in rows
            if s.active_maneuver_type == ManeuverType.SIDE_CORRECTION
            and _num(s.maneuver_speed_mps)
            and float(s.maneuver_speed_mps) < 0.0
        )
        con = [e for e in eps if e.contact]
        per_run.append(
            [
                run,
                len(rows),
                len(drive),
                contact_ticks,
                f"{100.0 * contact_ticks / max(len(drive), 1):.1f}%",
                len(eps),
                len(con),
                f"{100.0 * len(con) / max(len(eps), 1):.0f}%",
            ]
        )

    print("\n## Per run: contact-recovery load and commitment episodes\n")
    print_table(
        per_run,
        ["run", "ticks", "driving", "contactTk", "%drv", "episodes", "contact eps", "%eps"],
    )

    con = [e for e in all_eps if e.contact]
    cln = [e for e in all_eps if not e.contact]

    print(f"\n## Approach profile: |across| (m) by along-range gate  [contact n={len(con)}, clean n={len(cln)}]")
    print(f"   healthy = rises to {COMMANDED_LATERAL_M:.3f}; contact below {CONTACT_GAP_M:.3f}; far wall at {FAR_WALL_LIMIT_M:.3f}\n")
    prof_c = [_profile(e) for e in con]
    prof_k = [_profile(e) for e in cln]
    rows_p: list[list[object]] = []
    for gate in ALONG_BINS:
        cv = [p[gate] for p in prof_c if gate in p]
        kv = [p[gate] for p in prof_k if gate in p]
        rows_p.append(
            [
                f"{gate:.2f}",
                len(cv),
                f"{_p(cv, 0.1):.3f}",
                f"{_p(cv, 0.5):.3f}",
                f"{_p(cv, 0.9):.3f}",
                len(kv),
                f"{_p(kv, 0.1):.3f}",
                f"{_p(kv, 0.5):.3f}",
                f"{_p(kv, 0.9):.3f}",
                f"{_p(cv, 0.5) - _p(kv, 0.5):+.3f}",
            ]
        )
    print_table(
        rows_p,
        ["along", "nC", "C p10", "C p50", "C p90", "nK", "K p10", "K p50", "K p90", "dp50"],
    )

    print("\n## Monotone or diverted?  (run-up only, along>0)\n")
    rows_m: list[list[object]] = []
    for label, eps in (("contact", con), ("clean", cln)):
        peaks, finals, drops, at_alongs, ever_ok, never_ok = [], [], [], [], 0, 0
        for e in eps:
            tk = [t for t in e.runup() if 0.0 < t.along <= 1.40]
            if len(tk) < 3:
                continue
            peak = max(t.abs_across for t in tk)
            peak_i = max(range(len(tk)), key=lambda i: tk[i].abs_across)
            final = tk[-1].abs_across
            peaks.append(peak)
            finals.append(final)
            drops.append(peak - final)
            at_alongs.append(tk[peak_i].along)
            if peak >= COMMANDED_LATERAL_M:
                ever_ok += 1
            else:
                never_ok += 1
        n = ever_ok + never_ok
        rows_m.append(
            [
                label,
                n,
                f"{_p(peaks, 0.5):.3f}",
                f"{_p(at_alongs, 0.5):.2f}",
                f"{_p(finals, 0.5):.3f}",
                f"{_p(drops, 0.5):.3f}",
                f"{_p(drops, 0.9):.3f}",
                f"{100.0 * ever_ok / max(n, 1):.0f}%",
            ]
        )
    print_table(
        rows_m,
        ["episodes", "n", "peak p50", "peak@along", "final p50", "drop p50", "drop p90", "ever reached 0.222"],
    )

    print("\n## Who spent the lateral: manoeuvre-owned vs router-owned d|across|\n")
    rows_a: list[list[object]] = []
    for label, eps in (("contact", con), ("clean", cln)):
        lm, lf, nm, nf, share = [], [], [], [], []
        for e in eps:
            a, b, c, d = _attribute(e)
            if c + d < 3:
                continue
            lm.append(a)
            lf.append(b)
            nm.append(c)
            nf.append(d)
            share.append(c / max(c + d, 1))
        rows_a.append(
            [
                label,
                len(lm),
                f"{_p(share, 0.5) * 100:.0f}%",
                f"{_p(lm, 0.5):.3f}",
                f"{_p(lf, 0.5):.3f}",
                f"{sum(lm) / max(sum(lm) + sum(lf), 1e-9) * 100:.0f}%",
                f"{_p([a / max(c, 1) for a, c in zip(lm, nm)], 0.5) * 1000:.2f}",
                f"{_p([b / max(d, 1) for b, d in zip(lf, nf)], 0.5) * 1000:.2f}",
            ]
        )
    print_table(
        rows_a,
        ["episodes", "n", "man tick share", "lost|man p50", "lost|router p50", "man share of loss", "mm/man tick", "mm/router tick"],
    )

    print("\n## Same split over the WHOLE commitment (the contact recovery included)\n")
    rows_w: list[list[object]] = []
    for label, eps in (("contact", con), ("clean", cln)):
        lm, lf, nm, nf, share = [], [], [], [], []
        for e in eps:
            a, b, c, d = _attribute(e, whole=True)
            if c + d < 3:
                continue
            lm.append(a)
            lf.append(b)
            nm.append(c)
            nf.append(d)
            share.append(c / max(c + d, 1))
        rows_w.append([
            label, len(lm), f"{_p(share, 0.5) * 100:.0f}%",
            f"{_p(lm, 0.5):.3f}", f"{_p(lf, 0.5):.3f}",
            f"{sum(lm) / max(sum(lm) + sum(lf), 1e-9) * 100:.0f}%",
            f"{_p([a / max(c, 1) for a, c in zip(lm, nm)], 0.5) * 1000:.2f}",
            f"{_p([b / max(d, 1) for b, d in zip(lf, nf)], 0.5) * 1000:.2f}",
        ])
    print_table(rows_w, ["episodes", "n", "man tick share", "lost|man p50", "lost|router p50", "man share of loss", "mm/man tick", "mm/router tick"])

    print("\n## When did the router COMMIT, and how much room was left\n")
    rows_c: list[list[object]] = []
    for label, eps in (("contact", con), ("clean", cln)):
        along0, across0, rng0, late = [], [], [], 0
        for e in eps:
            t0 = e.ticks[0]
            along0.append(t0.along)
            across0.append(t0.abs_across)
            rng0.append(math.hypot(e.sign_x - t0.x, e.sign_y - t0.y))
            if t0.along < 0.9:
                late += 1
        rows_c.append([
            label, len(eps),
            f"{_p(along0, 0.1):.2f}", f"{_p(along0, 0.5):.2f}", f"{_p(along0, 0.9):.2f}",
            f"{_p(rng0, 0.5):.2f}", f"{_p(across0, 0.5):.3f}",
            f"{100.0 * late / max(len(eps), 1):.0f}%",
        ])
    print_table(rows_c, ["episodes", "n", "commit along p10", "p50", "p90", "commit range p50", "commit |across| p50", "inside the 0.9 m ramp"])

    print("\n## WALL or PILLAR? distance from the escape trigger to the nearest TRACK WALL\n")
    rows_wp: list[list[object]] = []
    for label, only_contact in (("contact ticks", True), ("any latched tick", False)):
        dw, ds, wall_n, sign_n, both_n, neither_n = [], [], 0, 0, 0, 0
        for e in all_eps:
            for tk in e.ticks:
                if only_contact and not tk.contact:
                    continue
                if not only_contact and tk.maneuver is None:
                    continue
                w = _trigger_world(tk)
                if w is None:
                    continue
                a = _wall_dist(w[0], w[1])
                b = math.hypot(w[0] - e.sign_x, w[1] - e.sign_y)
                dw.append(a)
                ds.append(b)
                near_w, near_s = a <= 0.10, b <= 0.25
                if near_w and near_s:
                    both_n += 1
                elif near_w:
                    wall_n += 1
                elif near_s:
                    sign_n += 1
                else:
                    neither_n += 1
        n = wall_n + sign_n + both_n + neither_n
        rows_wp.append([
            label, n,
            f"{_p(dw, 0.5):.3f}", f"{_p(ds, 0.5):.3f}",
            f"{100.0 * wall_n / max(n, 1):.0f}%",
            f"{100.0 * sign_n / max(n, 1):.0f}%",
            f"{100.0 * both_n / max(n, 1):.0f}%",
            f"{100.0 * neither_n / max(n, 1):.0f}%",
        ])
    print_table(rows_wp, ["sample", "n", "d(trig,wall) p50", "d(trig,sign) p50", "wall only", "pillar only", "ambiguous", "neither"])

    print("\n## A vs B: does a contact recovery PRECEDE the robot's own wrong-side verdict?\n")
    print("   wrong_side_pass_count is the ROUTER's verdict, incremented when it retires a"
          " sign it judged passed on the forbidden side. No replay, no ground truth.")
    rows_ab: list[list[object]] = []
    for bag_dir in args.bag_dirs:
        run = bag_dir.name
        rows = run_rows.get(run)
        if rows is None:
            continue
        contact_t = [
            t
            for t, s in rows
            if s.active_maneuver_type == ManeuverType.SIDE_CORRECTION
            and _num(s.maneuver_speed_mps)
            and float(s.maneuver_speed_mps) < 0.0
        ]
        bumps: list[float] = []
        prev: int | None = None
        for t, s in rows:
            v = s.wrong_side_pass_count
            if not isinstance(v, int):
                continue
            if prev is not None and v > prev:
                bumps.append(t)
            prev = v
        hit = sum(1 for b in bumps if any(0.0 <= b - c <= args.link_window for c in contact_t))
        # Control: the same test at a matched set of RANDOM times. If contact
        # recoveries are simply dense, any instant would "precede" one, and the
        # hit rate above would mean nothing.
        rng_ = random.Random(7)
        span = rows[-1][0] if rows else 0.0
        fake = [rng_.uniform(0.0, span) for _ in range(max(len(bumps), 50))]
        base_hit = sum(1 for b in fake if any(0.0 <= b - c <= args.link_window for c in contact_t))
        rows_ab.append([
            run, len(bumps), len(contact_t),
            f"{100.0 * hit / max(len(bumps), 1):.0f}%",
            f"{100.0 * base_hit / max(len(fake), 1):.0f}%",
        ])
    print_table(rows_ab, ["run", "wrong-side bumps", "contact ticks", "preceded by contact", "chance baseline"])

    print("\n## Does the contact recovery push the chassis to the WRONG side?\n")
    rows_sd: list[list[object]] = []
    flips = held = 0
    d_signed: list[float] = []
    for e in con:
        i = e.first_contact_i
        if i is None or i == 0:
            continue
        before = e.ticks[i - 1].across
        after = e.ticks[-1].across
        d_signed.append(abs(after) - abs(before))
        if before * after < 0:
            flips += 1
        else:
            held += 1
    rows_sd.append([
        flips + held,
        f"{100.0 * flips / max(flips + held, 1):.0f}%",
        f"{_p(d_signed, 0.1):+.3f}",
        f"{_p(d_signed, 0.5):+.3f}",
        f"{_p(d_signed, 0.9):+.3f}",
    ])
    print_table(rows_sd, ["contact episodes", "side FLIPPED across the recovery", "d|across| p10", "p50", "p90"])

    print("\n## Belief CHURN: how far the committed pillar moves between consecutive ticks\n")
    rows_ch: list[list[object]] = []
    for bag_dir in args.bag_dirs:
        run = bag_dir.name
        rows = run_rows.get(run)
        if rows is None:
            continue
        jumps: list[float] = []
        last: tuple[float, float] | None = None
        big = 0
        for _, s in rows:
            if not (_num(s.committed_sign_x_m) and _num(s.committed_sign_y_m)):
                last = None
                continue
            cur = (float(s.committed_sign_x_m), float(s.committed_sign_y_m))
            if last is not None:
                d = math.hypot(cur[0] - last[0], cur[1] - last[1])
                jumps.append(d)
                if d > SAME_SIGN_M:
                    big += 1
            last = cur
        moved = [j for j in jumps if j > 1e-6]
        rows_ch.append([
            run, len(jumps),
            f"{100.0 * len(moved) / max(len(jumps), 1):.0f}%",
            f"{_p(moved, 0.5):.3f}", f"{_p(moved, 0.9):.3f}",
            sum(1 for j in jumps if j > 0.10),
            big,
        ])
    print_table(rows_ch, ["run", "committed ticks", "belief moved", "move p50", "p90", ">0.10 m", ">0.35 m (new pillar)"])

    print("\n## WHERE on the track does contact happen? (corner = both coords outside the inner square)\n")
    rows_loc: list[list[object]] = []
    lo, hi = TrackDimensions.CORNER_MIN, TrackDimensions.CORNER_MAX
    for label, eps in (("contact", con), ("clean", cln)):
        corner = straight = sign_corner = 0
        spd: list[float] = []
        for e in eps:
            i = e.first_contact_i
            tk = e.ticks[i if i is not None else -1]
            in_x = lo <= tk.x <= hi
            in_y = lo <= tk.y <= hi
            if not (lo <= e.sign_x <= hi) and not (lo <= e.sign_y <= hi):
                sign_corner += 1
            if not in_x and not in_y:
                corner += 1
            else:
                straight += 1
            if tk.speed is not None:
                spd.append(abs(tk.speed))
        n = corner + straight
        rows_loc.append([
            label, n,
            f"{100.0 * corner / max(n, 1):.0f}%",
            f"{100.0 * straight / max(n, 1):.0f}%",
            f"{100.0 * sign_corner / max(n, 1):.0f}%",
            f"{_p(spd, 0.5):.3f}",
        ])
    print_table(rows_loc, ["episodes", "n", "robot in a CORNER", "on a straight", "BELIEF in a corner", "speed p50 m/s"])

    print("\n## Escape-trigger provenance on contact ticks (tests the 4fc0fbab mask)\n")
    rows_t: list[list[object]] = []
    for label, only_contact in (("contact ticks", True), ("any latched tick", False)):
        d_sign, ranges, on_sign = [], [], 0
        n = 0
        for e in all_eps:
            for tk in e.ticks:
                if only_contact and not tk.contact:
                    continue
                if not only_contact and tk.maneuver is None:
                    continue
                w = _trigger_world(tk)
                if w is None:
                    continue
                n += 1
                d = math.hypot(w[0] - e.sign_x, w[1] - e.sign_y)
                d_sign.append(d)
                ranges.append(tk.trig_rng)
                if d <= 0.12:
                    on_sign += 1
        rows_t.append(
            [
                label,
                n,
                f"{_p(d_sign, 0.1):.3f}",
                f"{_p(d_sign, 0.5):.3f}",
                f"{_p(d_sign, 0.9):.3f}",
                f"{100.0 * on_sign / max(n, 1):.1f}%",
                f"{_p(ranges, 0.5):.3f}",
            ]
        )
    print_table(
        rows_t,
        ["sample", "n", "d(trig,sign) p10", "p50", "p90", "<=0.12 m (mask miss)", "trig range p50"],
    )

    print("\n## Robot-to-belief range at the moment of contact engagement\n")
    rng: list[float] = []
    fwd: list[float] = []
    across_at: list[float] = []
    steer: list[float] = []
    for e in con:
        i = e.first_contact_i
        if i is None:
            continue
        tk = e.ticks[i]
        rng.append(math.hypot(e.sign_x - tk.x, e.sign_y - tk.y))
        across_at.append(tk.abs_across)
        if tk.fwd_clear is not None:
            fwd.append(tk.fwd_clear)
        if tk.man_steer is not None:
            steer.append(tk.man_steer)
    print_table(
        [
            [
                len(rng),
                f"{_p(rng, 0.1):.3f}",
                f"{_p(rng, 0.5):.3f}",
                f"{_p(rng, 0.9):.3f}",
                f"{_p(across_at, 0.5):.3f}",
                f"{_p(fwd, 0.5):.3f}",
                f"{_p(steer, 0.5):+.3f}",
            ]
        ],
        ["engagements", "range p10", "p50", "p90", "|across| p50", "fwd clear p50", "man steer p50"],
    )

    if args.detail:
        print("\n## Per-episode detail (contact episodes)\n")
        seen: dict[str, int] = {}
        rows_d: list[list[object]] = []
        for e in con:
            k = seen.get(e.run, 0)
            if k >= args.detail:
                continue
            seen[e.run] = k + 1
            p = _profile(e)
            lm, lf, nm, nf = _attribute(e)
            rows_d.append(
                [
                    e.run[-6:],
                    f"{e.ticks[0].t:.1f}",
                    len(e.ticks),
                    f"({e.sign_x:.2f},{e.sign_y:.2f})",
                    " ".join(f"{p.get(g, float('nan')):.2f}" for g in (1.40, 1.00, 0.60, 0.30, 0.10)),
                    f"{lm:.3f}",
                    f"{lf:.3f}",
                    f"{nm}/{nm + nf}",
                ]
            )
        print_table(
            rows_d,
            ["run", "t0", "ticks", "belief", "|across| @1.4/1.0/0.6/0.3/0.1", "lost|man", "lost|rtr", "man ticks"],
        )


if __name__ == "__main__":
    main()

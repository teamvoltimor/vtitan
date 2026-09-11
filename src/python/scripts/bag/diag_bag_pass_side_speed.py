r"""Does the SPEED carried at commitment separate the pass-side outcome?

Wraps ``diag_bag_pass_side`` WITHOUT re-deriving its rule: it imports ``_passes``
and reuses the shipped three-way verdict verbatim (routing / execution / ok).
What it adds is the per-pass CONTEXT at the commit tick, recovered through a
side-channel tap on ``SignRouter.deform_waypoint`` so ``_passes`` itself runs
unmodified: commanded speed, forward clearance, risk, path turn ahead, and which
of the two speed limiters (heading crawl vs clearance) was binding.

The hypothesis under test: ``MIN_TURN_RADIUS_M`` is a speed curve
R = 0.053 + 1.86 v, so the lateral displacement available over the commit range
s is about s^2 / (2 R). At cruise (0.26 m/s) that is 0.206 m against a ~0.16 m
cross; at 0.15 m/s it is 0.333 m. If the mechanism is real, execution failures
concentrate at high commit speed.

Controls carried, because a null is unreadable without them:

* the pooled three-way tally is printed so it can be diffed against the shipped
  script's own output over the same bags;
* the speed recovered through the tap is compared against ``Pass.commit_speed_mps``
  which ``_passes`` recorded independently -- they must agree on ~100% of passes
  or the tap is mis-aligned and nothing downstream means anything;
* the known-present crossing-vs-holding separation is recomputed, and printed
  first, so a pipeline that cannot see a real effect is visible as such.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_pass_side_speed.py \
        data/live/runs/run_2026090[6-9]* data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402
from scripts.bag import diag_bag_pass_side as dps  # noqa: E402
from scripts.common.bag_io import create_bags_parser  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.sign_router import SignRouter  # noqa: E402

TURN_R_INTERCEPT = 0.053
TURN_R_SLOPE = 1.86
"""R = 0.053 + 1.86 v, measured on this chassis."""

SPEED_BANDS = ((0.0, 0.18), (0.18, 0.24), (0.24, 99.0))
RANGE_BANDS = ((0.0, 0.40), (0.40, 0.55), (0.55, 99.0))

_TAP: list[tuple[float, float, float, tuple[float, float] | None]] = []


class _TapRouter(SignRouter):
    """A SignRouter that records which sign it was committed to, per tick.

    Behaviourally identical to the shipped router -- it only appends to a
    module-level log after delegating -- so ``_passes`` sees exactly the replay
    it would have seen, and the verdicts are the shipped ones.
    """

    def deform_waypoint(self, *args, **kwargs):  # noqa: ANN002,ANN003,ANN201
        out = super().deform_waypoint(*args, **kwargs)
        pose_xy = args[1] if len(args) > 1 else kwargs["robot_pos"]
        yaw = args[2] if len(args) > 2 else kwargs["robot_yaw"]
        c = self.committed_sign_position
        _TAP.append(
            (pose_xy[0], pose_xy[1], yaw, None if c is None else (round(c.x, 1), round(c.y, 1)))
        )
        return out


def _chi2_p(table: list[list[int]]) -> tuple[float, float, int]:
    """Pearson chi-square on a contingency table -> (chi2, p, dof)."""
    rows = len(table)
    cols = len(table[0])
    total = sum(sum(r) for r in table)
    if total == 0:
        return (0.0, 1.0, 0)
    row_s = [sum(r) for r in table]
    col_s = [sum(table[i][j] for i in range(rows)) for j in range(cols)]
    chi2 = 0.0
    for i in range(rows):
        for j in range(cols):
            exp = row_s[i] * col_s[j] / total
            if exp > 0:
                chi2 += (table[i][j] - exp) ** 2 / exp
    dof = (rows - 1) * (cols - 1)
    return (chi2, _chi2_sf(chi2, dof), dof)


def _chi2_sf(x: float, k: int) -> float:
    """Upper tail of the chi-square distribution, via the regularised gamma Q."""
    if k <= 0 or x <= 0:
        return 1.0
    a, xx = k / 2.0, x / 2.0
    if xx < a + 1.0:  # series for P, then Q = 1 - P
        term = 1.0 / a
        total = term
        n = 1
        while n < 500:
            term *= xx / (a + n)
            total += term
            if abs(term) < abs(total) * 1e-14:
                break
            n += 1
        return 1.0 - total * math.exp(-xx + a * math.log(xx) - math.lgamma(a))
    # Lentz continued fraction for Q
    tiny = 1e-300
    b = xx + 1.0 - a
    c = 1.0 / tiny
    d = 1.0 / b
    h = d
    for i in range(1, 500):
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        if abs(d) < tiny:
            d = tiny
        c = b + an / c
        if abs(c) < tiny:
            c = tiny
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < 1e-14:
            break
    return h * math.exp(-xx + a * math.log(xx) - math.lgamma(a))


def _verdict(p) -> str:  # noqa: ANN001
    """The SHIPPED three-way verdict, copied from diag_bag_pass_side.main()."""
    if p.commanded < 0:
        return "routing"
    if p.achieved < 0:
        return "execution"
    return "ok"


def _band(value: float, bands) -> int:  # noqa: ANN001
    for i, (lo, hi) in enumerate(bands):
        if lo <= value < hi:
            return i
    return len(bands) - 1


def _rate_table(records, key_fn, labels, title) -> None:  # noqa: ANN001
    """Print exec-vs-ok by bucket, with a chi-square over the 2xN table."""
    buckets: dict[int, Counter] = {}
    for r in records:
        buckets.setdefault(key_fn(r), Counter())[r["verdict"]] += 1
    print(f"  {title}")
    print(f"    {'bucket':>18} {'n':>5} {'exec':>6} {'ok':>6} {'exec rate':>10}")
    table = [[], []]
    for i in sorted(buckets):
        b = buckets[i]
        n = b["execution"] + b["ok"]
        if n == 0:
            continue
        table[0].append(b["execution"])
        table[1].append(b["ok"])
        print(
            f"    {labels(i):>18} {n:5d} {b['execution']:6d} {b['ok']:6d}"
            f" {100 * b['execution'] / n:9.1f}%"
        )
    if len(table[0]) > 1:
        chi2, p, dof = _chi2_p(table)
        print(f"    chi2={chi2:.2f} dof={dof} p={p:.4f}")
    print()


def main() -> None:
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()
    tuning = get_tuning(None)

    dps.SignRouter = _TapRouter  # the tap, installed where _passes constructs it

    records: list[dict] = []
    skipped: list[str] = []
    peaks: list[int] = []
    bags_with_passes = 0
    tap_mismatch = 0
    unmatched_ctx = 0

    for bag in args.bag_dirs:
        _TAP.clear()
        try:
            rows, frames, scans = dps._load(Path(bag))
        except (RuntimeError, OSError, ValueError, KeyError) as exc:
            skipped.append(f"{Path(bag).name}:{type(exc).__name__}")
            continue
        run = Path(bag).name.replace("run_", "")
        try:
            passes, peak = dps._passes(run, rows, frames, scans, tuning)
        except (RuntimeError, ValueError, KeyError, IndexError) as exc:
            skipped.append(f"{Path(bag).name}:replay-{type(exc).__name__}")
            continue
        peaks.append(peak)
        if passes:
            bags_with_passes += 1

        # pose -> the snapshot that produced it, for the commit-tick context
        by_pose: dict[tuple[float, float, float], object] = {}
        for _rel, d in rows:
            if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
                continue
            by_pose.setdefault((d.pose_x, d.pose_y, d.pose_yaw), d)
        first_tap: dict[tuple[float, float], tuple] = {}
        for px, py, yaw, key in _TAP:
            if key is not None and key not in first_tap:
                first_tap[key] = (px, py, yaw)

        for p in passes:
            key = (round(p.sign_x, 1), round(p.sign_y, 1))
            tap = first_tap.get(key)
            d = by_pose.get(tap) if tap is not None else None
            if d is None:
                unmatched_ctx += 1
                continue
            if p.commit_speed_mps is None or d.commanded_speed_mps is None:
                unmatched_ctx += 1
                continue
            if abs(d.commanded_speed_mps - p.commit_speed_mps) > 1e-9:
                tap_mismatch += 1
            v = p.commit_speed_mps
            radius = TURN_R_INTERCEPT + TURN_R_SLOPE * v
            records.append(
                {
                    "run": run,
                    "verdict": _verdict(p),
                    "speed": v,
                    "range": p.commit_range_m,
                    "commit_lateral": p.commit_lateral_m,
                    "crossing": p.commit_lateral_m < 0,
                    "needed": max(0.0, -p.commit_lateral_m),
                    "available": p.commit_range_m**2 / (2 * radius),
                    "margin": p.commit_range_m**2 / (2 * radius) - max(0.0, -p.commit_lateral_m),
                    "fwd_clear": d.forward_clearance_m,
                    "risk": str(d.risk) if d.risk is not None else "none",
                    "turn_ahead": abs(d.path_turn_ahead_rad) if d.path_turn_ahead_rad is not None else None,
                    "heading_speed": d.heading_speed_mps,
                    "clearance_speed": d.clearance_speed_mps,
                    "maneuver": p.maneuver_during_pass,
                    "colour": str(p.colour),
                    "corridor": p.corridor,
                }
            )

    print(f"== CORPUS: {len(args.bag_dirs)} bag(s) given, {len(skipped)} unreadable, "
          f"{bags_with_passes} with sign passes")
    if skipped:
        print("   skipped: " + ", ".join(skipped[:10]) + (" ..." if len(skipped) > 10 else ""))
    print(f"   peak believed signs > 8 (physical max) in {sum(1 for p in peaks if p > 8)}/{len(peaks)} runs;"
          f" worst={max(peaks, default=0)} -- NO pass is dropped for this, the shipped rule keeps them")
    print(f"   passes dropped for missing commit context/speed: {unmatched_ctx}")
    print()

    tally = Counter(r["verdict"] for r in records)
    total = sum(tally.values())
    print("== CONTROL 1: pooled three-way tally (diff this against diag_bag_pass_side)")
    print(f"   routing   {tally['routing']:5d}")
    print(f"   execution {tally['execution']:5d}")
    print(f"   ok        {tally['ok']:5d}")
    print(f"   n={total};  execution share of correctly-commanded passes: "
          f"{100 * tally['execution'] / max(1, tally['execution'] + tally['ok']):.1f}%")
    print(f"   tap/_passes speed disagreements: {tap_mismatch} (MUST be 0)")
    print()

    cc = [r for r in records if r["verdict"] in ("execution", "ok")]

    print("== CONTROL 2 (known-present positive): crossing vs holding at commit")
    _rate_table(cc, lambda r: 1 if r["crossing"] else 0,
                lambda i: "already legal" if i == 0 else "must CROSS",
                "legal-at-commit -> outcome")

    print("== HYPOTHESIS: commit speed")
    _rate_table(cc, lambda r: _band(r["speed"], SPEED_BANDS),
                lambda i: f"{SPEED_BANDS[i][0]:.2f}-{min(SPEED_BANDS[i][1], 1.0):.2f}",
                "commanded speed at commit (m/s)")
    speeds = sorted(r["speed"] for r in cc)
    if speeds:
        print(f"   commit speed p10/p50/p90 = {speeds[len(speeds)//10]:.3f} /"
              f" {speeds[len(speeds)//2]:.3f} / {speeds[9*len(speeds)//10]:.3f}")
        print(f"   share of commitments at >= 0.24 m/s: "
              f"{100 * sum(1 for s in speeds if s >= 0.24) / len(speeds):.1f}%")
    for v in ("execution", "ok"):
        s = sorted(r["speed"] for r in cc if r["verdict"] == v)
        if s:
            print(f"   {v:>9}: mean {sum(s)/len(s):.3f}  p50 {s[len(s)//2]:.3f}  n={len(s)}")
    print()

    print("== CONFOUND: is a slow commit just a CORNER commit?")
    have_turn = [r for r in cc if r["turn_ahead"] is not None]
    if have_turn:
        med_turn = sorted(r["turn_ahead"] for r in have_turn)[len(have_turn) // 2]
        print(f"   path_turn_ahead median = {med_turn:.3f} rad; stratifying on it")
        for lab, sel in (("STRAIGHT-ish", lambda r: r["turn_ahead"] < med_turn),
                         ("CORNER-ish", lambda r: r["turn_ahead"] >= med_turn)):
            sub = [r for r in have_turn if sel(r)]
            _rate_table(sub, lambda r: _band(r["speed"], SPEED_BANDS),
                        lambda i: f"{SPEED_BANDS[i][0]:.2f}-{min(SPEED_BANDS[i][1], 1.0):.2f}",
                        f"speed within {lab} (n={len(sub)})")
        slow = [r for r in have_turn if r["speed"] < 0.24]
        fast = [r for r in have_turn if r["speed"] >= 0.24]
        for lab, sub in (("slow commits", slow), ("fast commits", fast)):
            if sub:
                t = sorted(r["turn_ahead"] for r in sub)
                print(f"   {lab:>14}: turn_ahead p50 {t[len(t)//2]:.3f} rad, n={len(t)}")
    _rate_table([r for r in cc if r["risk"] != "none"], lambda r: 0 if r["risk"] == "RiskLevel.SAFE" or r["risk"] == "safe" else 1,
                lambda i: "risk SAFE" if i == 0 else "risk not-safe", "risk level at commit")
    print("   which limiter was binding at commit (heading crawl vs clearance):")
    binder = Counter()
    for r in cc:
        h, c = r["heading_speed"], r["clearance_speed"]
        if h is None or c is None:
            binder["unknown"] += 1
        elif abs(h - r["speed"]) < 1e-6 and abs(c - r["speed"]) >= 1e-6:
            binder["heading"] += 1
        elif abs(c - r["speed"]) < 1e-6 and abs(h - r["speed"]) >= 1e-6:
            binder["clearance"] += 1
        elif abs(h - r["speed"]) < 1e-6:
            binder["both"] += 1
        else:
            binder["neither"] += 1
    print("   " + "  ".join(f"{k}={v}" for k, v in binder.most_common()))
    print()

    print("== CROSSING-ONLY: within passes that must cross, does speed still matter?")
    _rate_table([r for r in cc if r["crossing"]], lambda r: _band(r["speed"], SPEED_BANDS),
                lambda i: f"{SPEED_BANDS[i][0]:.2f}-{min(SPEED_BANDS[i][1], 1.0):.2f}",
                "commit speed | must cross")

    print("== ADJACENT: commit RANGE (the same arc formula is quadratic in it)")
    _rate_table(cc, lambda r: _band(r["range"], RANGE_BANDS),
                lambda i: f"{RANGE_BANDS[i][0]:.2f}-{min(RANGE_BANDS[i][1], 9.0):.2f} m",
                "commit range (m)")
    rngs = sorted(r["range"] for r in cc)
    if rngs:
        print(f"   commit range p10/p50/p90 = {rngs[len(rngs)//10]:.3f} /"
              f" {rngs[len(rngs)//2]:.3f} / {rngs[9*len(rngs)//10]:.3f}")
    print()

    print("== MECHANISM: geometric margin  s^2/(2R) - cross needed,  R = 0.053 + 1.86 v")
    cross = [r for r in cc if r["crossing"]]
    _rate_table(cross, lambda r: 0 if r["margin"] < 0 else (1 if r["margin"] < 0.10 else 2),
                lambda i: ("margin < 0", "0 - 0.10 m", ">= 0.10 m")[i],
                "predicted lateral margin | must cross")
    for v in ("execution", "ok"):
        m = sorted(r["margin"] for r in cross if r["verdict"] == v)
        if m:
            print(f"   {v:>9}: margin mean {sum(m)/len(m):+.3f}  p50 {m[len(m)//2]:+.3f}  n={len(m)}")
    print()

    print("== SPEED DISTRIBUTION at commit (the bands are only as real as this)")
    hist = Counter(round(r["speed"], 3) for r in cc)
    for v, n in sorted(hist.items()):
        print(f"   {v:.3f} m/s  n={n:5d}  {100*n/len(cc):5.1f}%")
    print()

    print("== IS SLOWNESS JUST THE ESCAPE MANOEUVRE / A HARDER GEOMETRY?")
    print("   crossing rate by speed band (if slow commits are more often crossings,")
    print("   speed is a proxy for the already-known predictor):")
    for i, (lo, hi) in enumerate(SPEED_BANDS):
        sub = [r for r in cc if _band(r["speed"], SPEED_BANDS) == i]
        if sub:
            print(f"   {lo:.2f}-{min(hi,1.0):.2f}: crossing {100*sum(1 for r in sub if r['crossing'])/len(sub):5.1f}%"
                  f"  manoeuvre {100*sum(1 for r in sub if r['maneuver'])/len(sub):5.1f}%"
                  f"  risk-not-safe {100*sum(1 for r in sub if r['risk'] not in ('safe','RiskLevel.SAFE'))/len(sub):5.1f}%"
                  f"  range p50 {sorted(r['range'] for r in sub)[len(sub)//2]:.3f}  n={len(sub)}")
    print()
    for lab, sel in (("NO manoeuvre", lambda r: not r["maneuver"]), ("manoeuvre latched", lambda r: r["maneuver"])):
        sub = [r for r in cc if sel(r)]
        _rate_table(sub, lambda r: _band(r["speed"], SPEED_BANDS),
                    lambda i: f"{SPEED_BANDS[i][0]:.2f}-{min(SPEED_BANDS[i][1],1.0):.2f}",
                    f"speed | {lab} (n={len(sub)})")
    print("   FULL stratification: crossing x speed")
    for cr in (False, True):
        sub = [r for r in cc if r["crossing"] == cr]
        _rate_table(sub, lambda r: _band(r["speed"], SPEED_BANDS),
                    lambda i: f"{SPEED_BANDS[i][0]:.2f}-{min(SPEED_BANDS[i][1],1.0):.2f}",
                    f"speed | {'must CROSS' if cr else 'already legal'} (n={len(sub)})")

    print("== LOGISTIC REGRESSION, exec=1 vs ok=0 (standardised coefficients)")
    feats = [
        ("crossing", lambda r: 1.0 if r["crossing"] else 0.0),
        ("speed", lambda r: r["speed"]),
        ("range", lambda r: r["range"]),
        ("turn_ahead", lambda r: r["turn_ahead"] if r["turn_ahead"] is not None else 0.0),
        ("risk_not_safe", lambda r: 0.0 if r["risk"] in ("safe", "RiskLevel.SAFE") else 1.0),
        ("manoeuvre", lambda r: 1.0 if r["maneuver"] else 0.0),
        ("cross_needed_m", lambda r: r["needed"]),
    ]
    X = [[f(r) for _n, f in feats] for r in cc]
    y = [1.0 if r["verdict"] == "execution" else 0.0 for r in cc]
    k = len(feats)
    mu = [sum(x[j] for x in X) / len(X) for j in range(k)]
    sd = [max(1e-9, (sum((x[j] - mu[j]) ** 2 for x in X) / len(X)) ** 0.5) for j in range(k)]
    Z = [[1.0] + [(x[j] - mu[j]) / sd[j] for j in range(k)] for x in X]
    w = [0.0] * (k + 1)
    for _it in range(400):
        g = [0.0] * (k + 1)
        for z, yi in zip(Z, y):
            pz = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, sum(wi * zi for wi, zi in zip(w, z))))))
            for j in range(k + 1):
                g[j] += (pz - yi) * z[j]
        w = [wi - 0.5 * gj / len(Z) for wi, gj in zip(w, g)]
    print(f"   n={len(Z)}   intercept {w[0]:+.3f}")
    for (name, _f), coef in sorted(zip(feats, w[1:]), key=lambda t: -abs(t[1])):
        print(f"   {name:>16} {coef:+.3f}   (odds x{math.exp(coef):.2f} per 1 SD)")


    print()
    print("== ADJACENT, CONDITIONAL ON THE SPEED NULL: commit RANGE within the crossing population")
    _rate_table([r for r in cc if r["crossing"]], lambda r: _band(r["range"], RANGE_BANDS),
                lambda i: f"{RANGE_BANDS[i][0]:.2f}-{min(RANGE_BANDS[i][1],9.0):.2f} m",
                "commit range | must cross")
    _rate_table([r for r in cc if not r["crossing"]], lambda r: _band(r["range"], RANGE_BANDS),
                lambda i: f"{RANGE_BANDS[i][0]:.2f}-{min(RANGE_BANDS[i][1],9.0):.2f} m",
                "commit range | already legal (control: should be flat and low)")
    _rate_table([r for r in cc if r["crossing"]],
                lambda r: 0 if r["needed"] < 0.10 else (1 if r["needed"] < 0.20 else 2),
                lambda i: ("cross <0.10 m", "0.10-0.20 m", ">=0.20 m")[i],
                "how far it had to cross")


if __name__ == "__main__":
    main()

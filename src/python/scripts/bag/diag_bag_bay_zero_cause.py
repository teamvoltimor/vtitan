r"""THROWAWAY: why does bay_exit command zero on ~92% of ticks?

Separates the three zero-speed return paths in ``BayExit._guarded_command``:
  A) settle countdown   (bay_exit.py:720-723)  -- servo slew at a standstill
  B) leg time cap       (bay_exit.py:748-757)  -- BAY_EXIT_LEG_MAX_S
  C) clearance refusal  (bay_exit.py:816-835)  -- gap <= margin

Discriminator: (C) writes ``_last_gap_m`` on the tick it fires, so
``bay_guard_gap_m`` CHANGES and lands <= margin. (B) returns before the gap is
computed, so the published gap is STALE (identical to the previous tick) and
typically well above the margin. (A) also returns before the gap is computed,
so the gap is stale too -- but (A) is identifiable by the run of steering
values ramping monotonically lock-to-lock.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader

MARGIN = 0.001
MOVING = 1e-6


def analyse(bag: Path) -> None:
    reader = open_reader(bag)
    t0 = None
    rows = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic != Topics.NAV_DEBUG:
            continue
        try:
            s = decode_nav_debug(data)
        except Exception:  # noqa: BLE001
            continue
        if str(s.phase) != "bay_exit":
            continue
        rows.append(
            (
                elapsed_seconds(t, t0),
                s.commanded_speed_mps or 0.0,
                s.commanded_steering_norm,
                s.bay_guard_gap_m,
                s.bay_dr_out_m,
                s.bay_dr_along_m,
                s.bay_leg_is_reverse,
            )
        )

    print(f"\n== {bag.name}: {len(rows)} bay_exit ticks")
    if not rows:
        return

    # Segment into runs of zero / moving.
    segs = []
    cur_moving = abs(rows[0][1]) > MOVING
    start = 0
    for i, r in enumerate(rows):
        m = abs(r[1]) > MOVING
        if m != cur_moving:
            segs.append((cur_moving, start, i - 1))
            cur_moving, start = m, i
    segs.append((cur_moving, start, len(rows) - 1))

    zero_segs = [s for s in segs if not s[0]]
    move_segs = [s for s in segs if s[0]]
    print(f"   {len(move_segs)} motion segments, lengths (ticks): {[b - a + 1 for _, a, b in move_segs]}")
    print(f"   {len(zero_segs)} zero segments,   lengths (ticks): {[b - a + 1 for _, a, b in zero_segs]}")
    total_zero = sum(b - a + 1 for _, a, b in zero_segs)
    print(f"   zero ticks {total_zero}/{len(rows)} = {100.0 * total_zero / len(rows):.1f}%")

    # Steering ramp inside each long zero segment: is the servo slewing?
    for _, a, b in zero_segs:
        n = b - a + 1
        if n < 5:
            continue
        st = [r[2] for r in rows[a : b + 1] if r[2] is not None]
        if not st:
            continue
        steps = [abs(st[i + 1] - st[i]) for i in range(len(st) - 1)]
        moved = sum(1 for x in steps if x > 1e-9)
        print(
            f"   zero seg t={rows[a][0]:.2f}-{rows[b][0]:.2f} n={n} "
            f"steer {st[0]:+.3f} -> {st[-1]:+.3f} (changing on {moved}/{len(steps)} steps, "
            f"max step {max(steps) if steps else 0:.4f})"
        )

    # The tick that ENDS each motion segment: what did the guard publish?
    print("   leg-end ticks (first zero tick after a pulse):")
    for _, a, b in move_segs:
        end = b + 1
        if end >= len(rows):
            continue
        prev_gap = rows[b][3]
        gap = rows[end][3]
        stale = (gap is not None and prev_gap is not None and abs(gap - prev_gap) < 1e-12)
        cause = "LEG_MAX/settle (gap STALE)" if stale else (
            "GUARD refusal (gap <= margin)" if gap is not None and gap <= MARGIN else "gap CHANGED but > margin"
        )
        print(
            f"     t={rows[end][0]:.2f} legticks={b - a + 1} rev={rows[b][6]} "
            f"gap_prev={prev_gap} gap_now={gap} dr_out={rows[end][4]} dr_along={rows[end][5]} -> {cause}"
        )

    gaps = [r[3] for r in rows if r[3] is not None]
    if gaps:
        uniq = sorted(set(round(g, 6) for g in gaps))
        print(f"   distinct published gap values: {len(uniq)} -> {uniq[:12]}")
        print(f"   ticks with gap <= {MARGIN}: {sum(1 for g in gaps if g <= MARGIN)}/{len(gaps)}")
    else:
        print("   NO bay_guard_gap_m published at all (field absent/None on every tick)")

    outs = [r[4] for r in rows if r[4] is not None]
    if outs:
        print(f"   dr_out range {min(outs):.4f}..{max(outs):.4f}  final {outs[-1]:.4f}")
    alongs = [r[5] for r in rows if r[5] is not None]
    if alongs:
        print(f"   dr_along range {min(alongs):.4f}..{max(alongs):.4f}  final {alongs[-1]:.4f}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyse(Path(p))

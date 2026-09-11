r"""Does the robot momentarily STOP at a corner, and is the creep floor why?

Reported from the track: the robot pauses briefly at crossings. Two candidate
causes, and they want opposite fixes:

* it is COMMANDED to stop -- something in the speed path writes a zero, or
* it is commanded a speed the drivetrain cannot deliver, and the wheel simply
  does not turn.

The second is the live suspicion. ``diag_bag_drive_response.py`` measured 73.8%
stall in the 0.09-0.11 m/s bin and 2.1% by 0.15-0.20, but binned nothing
between 0.11 and 0.15 -- and the heading term's creep floor sits at **0.152
m/s**, right against that unmeasured edge. The heading cut is what fires at a
corner (``diag_bag_corner_speed.py``: heading binds 57.1% of ticks), so if the
deadband reaches 0.152 then "slows for the corner" and "stops at the corner"
are the same event.

Three measurements:

1. FINE BINS across 0.10-0.22 in 0.01 steps, so the deadband edge is located
   rather than bracketed. Reported with the delivered speed, because a bin that
   stalls half the time and delivers full speed the rest is a different
   failure from one that delivers nothing.

2. STOP EPISODES during ``normal_drive`` only: runs of consecutive ticks with
   the encoder at zero while a non-zero speed is commanded. Counts, durations,
   and what was commanded during them. Restricted to normal_drive because
   bay_exit is a known separate failure and would dominate any pooled figure.

3. ATTRIBUTION. Each stop episode is labelled by whether the commanded speed
   matched the heading term (``heading_speed_mps``), which says whether the
   corner slowdown is what put the chassis under the floor.

4. IS THE FLOOR A FUNCTION OF STEERING LOAD? The operator's reading is that
   the minimum viable speed RISES at a crossing because turning costs torque.
   If so the deadband is not one number but a curve, and a single higher
   ``CREEP_MPS`` either overpays on a straight or still stalls at lock. A speed
   x steering grid answers it: read DOWN a speed column, and a stall rate that
   climbs with steering is load dependence. The two axes are correlated (the
   heading cut fires when the robot is turning), so cells are reported with
   their n and thin ones are not read as evidence.

``/motor/drive_speed`` is a smoothed velocity estimate in DEG/S, not raw
counts, so a reading of exactly 0 is the estimator saying "not turning" rather
than a quantisation artefact.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_creep_stall.py \
        data/live/runs/run_20260908_003520 data/live/runs/run_20260908_004023
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, open_reader
from scripts.common.binning import band_label
from scripts.common.stats import percentile

WHEEL_CIRCUM_M = 0.07 * 3.141592653589793
"""7 cm wheel, the measured diameter in robot.toml."""

STALL_DEG_S = 1.0

_COAST_MIN_EPISODES = 30
"""Coast episodes below which the archive is reporting its own scarcity.

Measured 2026-09-10: the whole 200-bag archive holds 15. The robot commands
a new speed long before the wheel has stopped, so a stopping distance is
simply not an event this data contains."""
"""Below this the wheel is not turning. The estimator reads exact zeros."""

MIN_EPISODE_TICKS = 2
"""Ticks of continuous stall before it counts as a stop rather than a sample."""

CONTROL_HZ = 20.0


def deg_s_to_mps(deg_s: float) -> float:
    return (deg_s / 360.0) * WHEEL_CIRCUM_M


def _fine_bin(v: float) -> str | None:
    if v < 0.10 or v >= 0.22:
        return None
    lo = int(v * 100) / 100.0
    return f"{lo:.2f}-{lo + 0.01:.2f}"


STEER_EDGES = (0.0, 0.15, 0.35, 0.60, 0.90, 1.01)
"""|commanded_steering_norm| bands, from straight to full lock."""

SPEED_EDGES = (0.10, 0.14, 0.16, 0.20, 0.25, 0.35, 1.00)
"""Commanded-speed bands, placed so the 0.152 creep floor has its own column."""


def _band(v: float, edges: tuple[float, ...]) -> str | None:
    return band_label(v, edges, lambda lo, hi: f"{lo:.2f}-{hi:.2f}")


def collect(
    bag_dir: Path,
    fine: dict,
    episodes: list,
    phase_ticks: dict,
    grid: dict,
    by_phase: dict,
    coasts: list,
) -> None:
    reader = open_reader(bag_dir)
    last_drive: float | None = None
    last_cmd: float | None = None
    coast: list = [None, 0.0, 0.0]
    run_len = 0
    run_cmds: list[float] = []
    run_heading = 0

    for _ in iter(int, 1):
        if not reader.has_next():
            break
        topic, data, _t = reader.read_next()
        if topic == Topics.MOTOR_DRIVE_SPEED:
            last_drive = float(deserialize_message(data, Float32).data)
            # COAST: how far the wheel keeps turning after the command
            # goes to zero. `_guarded_command` budgets `v * tau` -- 35 mm
            # at the shipped bay speed, 52 mm at 0.15 -- and refuses any
            # leg it cannot stop inside that. But `tau` is a first-order
            # lag with NO stiction, and a drivetrain that delivers zero
            # below 0.11 m/s does not coast like one. Measured here so
            # the guard's budget can be argued with rather than assumed.
            now = _t * 1e-9
            enc_mps = deg_s_to_mps(abs(last_drive))
            if coast[0] is not None:
                dt = now - coast[1]
                if 0.0 < dt < 0.5:
                    coast[2] += enc_mps * dt
                if enc_mps <= deg_s_to_mps(STALL_DEG_S) or coast[2] > 0.5:
                    coasts.append((coast[0], coast[2]))
                    coast[0] = None
            coast[1] = now
            continue
        if topic != Topics.NAV_DEBUG or last_drive is None:
            continue

        try:
            snap = decode_nav_debug(data)
        except Exception:  # noqa: BLE001
            # A run killed mid-write leaves a truncated final JSON payload.
            # Counted rather than silently swallowed: a bag losing many rows
            # would make every percentage below a fraction of the wrong total.
            phase_ticks["undecodable"] += 1
            continue
        cmd = snap.commanded_speed_mps
        phase = str(getattr(snap.phase, "value", snap.phase))
        if cmd is None or abs(cmd) < 1e-6:
            # The command just fell to zero from a moving one: open a
            # coast window, tagged with the speed it was released from.
            if last_cmd is not None and last_cmd > 0.02 and coast[0] is None:
                coast[0] = last_cmd
                coast[2] = 0.0
            last_cmd = 0.0
            continue
        # A coast window is only evidence if the command STAYED at zero. A
        # command that returns before the wheel stops is the robot being
        # told to drive again, not a stopping distance -- unfiltered, every
        # bin at 0.15 and above saturated at the 0.5 m cap because of it.
        if coast[0] is not None:
            coast[0] = None
        last_cmd = abs(cmd)
        enc = abs(last_drive)
        stalled = enc < STALL_DEG_S

        key = _fine_bin(abs(cmd))
        if key is not None:
            fine[key].append((stalled, deg_s_to_mps(enc)))

        st = snap.commanded_steering_norm
        if st is not None:
            sb = _band(abs(st), STEER_EDGES)
            vb = _band(abs(cmd), SPEED_EDGES)
            if sb is not None and vb is not None:
                cell = grid[(sb, vb)]
                cell[0] += 1
                cell[1] += int(stalled)

        # Per-phase, because the deadband is a DRIVETRAIN fact while the
        # phases ask for very different speeds at very different lock.
        # Aggregated, the bay exit is a rounding error against
        # `normal_drive` -- seconds of a three-minute round -- while being
        # the phase that spends nearly all of its time stalled.
        by_phase[phase][0] += 1
        by_phase[phase][1] += int(stalled)
        by_phase[phase][2] += abs(cmd)
        by_phase[phase][3] += deg_s_to_mps(enc)
        if st is not None:
            by_phase[phase][4] += abs(st)

        if phase != "normal_drive":
            if run_len >= MIN_EPISODE_TICKS:
                episodes.append((run_len, run_cmds, run_heading))
            run_len, run_cmds, run_heading = 0, [], 0
            continue

        phase_ticks["normal_drive"] += 1
        if stalled:
            run_len += 1
            run_cmds.append(abs(cmd))
            hs = snap.heading_speed_mps
            if hs is not None and abs(abs(cmd) - abs(hs)) < 1e-6:
                run_heading += 1
        else:
            if run_len >= MIN_EPISODE_TICKS:
                episodes.append((run_len, run_cmds, run_heading))
            run_len, run_cmds, run_heading = 0, [], 0

    if run_len >= MIN_EPISODE_TICKS:
        episodes.append((run_len, run_cmds, run_heading))


def main() -> int:
    parser = create_bags_parser(description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter)
    args = parser.parse_args()

    fine: dict[str, list] = defaultdict(list)
    episodes: list = []
    phase_ticks: dict[str, int] = defaultdict(int)
    by_phase: dict[str, list[float]] = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0, 0.0])
    coasts: list[tuple[float, float]] = []
    grid: dict = defaultdict(lambda: [0, 0])
    for bag in args.bag_dirs:
        try:
            collect(Path(bag), fine, episodes, phase_ticks, grid, by_phase, coasts)
        except (OSError, RuntimeError, ValueError) as exc:
            # The archive holds a handful of bags with a truncated or locked
            # metadata file. Every other diag here skips them and says so;
            # this one aborted the whole sweep on the first, which is why a
            # 200-bag question could not be asked of it at all.
            print(f"!! {Path(bag).name}: {exc}", flush=True)

    print()
    print()
    print("== -1. HOW FAR DOES IT COAST after the command goes to zero")
    print(
        f"  clean coast episodes in the whole archive: {len(coasts)}"
        " -- a window only counts if the command STAYED at zero until the wheel stopped"
    )
    if len(coasts) < _COAST_MIN_EPISODES:
        print(
            "  NOT MEASURABLE FROM BAGS. The robot practically never commands a"
            " sustained zero, so the stopping distance `_guarded_command` budgets"
            " (`v * SPEED_RESPONSE_TAU_S`) cannot be checked against recorded data"
            " at all -- it wants a bench test, not more bags."
        )
    coast_bins: dict[str, list[float]] = defaultdict(list)
    for released_at, dist in coasts:
        coast_bins[f"{round(released_at, 2):.2f}"].append(dist)
    print(f"  {'released from':>14}{'n':>7}{'coast p50 mm':>15}{'p90 mm':>10}{'v*tau mm':>11}")
    for key in sorted(coast_bins, key=float):
        vals = sorted(coast_bins[key])
        if len(vals) < 15:
            continue
        v = float(key)
        print(
            f"  {v:>14.2f}{len(vals):>7}{percentile(vals, 0.5) * 1000:>15.1f}"
            f"{vals[len(vals) * 9 // 10] * 1000:>10.1f}"
            f"{v * RobotSpecs.SPEED_RESPONSE_TAU_S * 1000:>11.1f}"
        )

    print("== 0. STALL BY PHASE -- the deadband is a drivetrain fact, the phases differ")
    header = f"  {'phase':<18}{'ticks':>8}{'stall%':>9}{'mean cmd':>11}{'mean enc':>11}{'mean |steer|':>14}"
    print(header)
    for name, (n, stalled, cmd_sum, enc_sum, steer_sum) in sorted(by_phase.items(), key=lambda kv: -kv[1][0]):
        if n < 20:
            continue
        print(
            f"  {name:<18}{int(n):>8}{stalled / n:>8.1%}{cmd_sum / n:>11.3f}{enc_sum / n:>11.3f}{steer_sum / n:>14.2f}"
        )

    print("\n== 1. WHERE IS THE DEADBAND EDGE (all phases)")
    print(f"{'cmd bin':>12} {'n':>7} {'stall%':>8} {'delivered p50':>15}")
    for key in sorted(fine):
        rows = fine[key]
        stalls = sum(1 for s, _ in rows if s)
        delivered = [v for _, v in rows]
        print(f"{key:>12} {len(rows):>7} {100 * stalls / len(rows):>7.1f}% {percentile(delivered, 0.5):>14.3f}")

    total = phase_ticks["normal_drive"]
    stalled_ticks = sum(n for n, _, _ in episodes)
    print(f"\n== 2. STOP EPISODES IN normal_drive  (>= {MIN_EPISODE_TICKS} ticks)")
    print(f"  normal_drive ticks: {total}")
    if phase_ticks["undecodable"]:
        print(f"  undecodable rows skipped: {phase_ticks['undecodable']}")
    if not episodes:
        print("  NO stop episodes found.")
        return 0
    durs = [n / CONTROL_HZ for n, _, _ in episodes]
    print(
        f"  episodes: {len(episodes)}   ticks stalled: {stalled_ticks} ({100 * stalled_ticks / total:.1f}% of normal_drive)"
    )
    print(f"  duration s: p50={percentile(durs, 0.5):.2f}  p90={percentile(durs, 0.9):.2f}  max={max(durs):.2f}")
    cmds = [c for _, cs, _ in episodes for c in cs]
    print(
        f"  commanded during a stop: p25={percentile(cmds, 0.25):.3f}  p50={percentile(cmds, 0.5):.3f}  p95={percentile(cmds, 0.95):.3f} m/s"
    )

    print("\n== 3. WAS THE HEADING CUT WHAT PUT IT THERE")
    heading_ticks = sum(h for _, _, h in episodes)
    heading_eps = sum(1 for _, _, h in episodes if h > 0)
    print(
        f"  stalled ticks whose command matched heading_speed_mps: {heading_ticks}/{stalled_ticks} ({100 * heading_ticks / stalled_ticks:.1f}%)"
    )
    print(
        f"  episodes containing at least one such tick: {heading_eps}/{len(episodes)} ({100 * heading_eps / len(episodes):.1f}%)"
    )

    print()
    print("== 4. STALL% BY STEERING LOAD x COMMANDED SPEED  (n in brackets)")
    speed_bands = [f"{lo:.2f}-{hi:.2f}" for lo, hi in zip(SPEED_EDGES, SPEED_EDGES[1:])]
    steer_bands = [f"{lo:.2f}-{hi:.2f}" for lo, hi in zip(STEER_EDGES, STEER_EDGES[1:])]
    header = f"{'|steer|':>12}" + "".join(f"{b:>16}" for b in speed_bands)
    print(header)
    for sb in steer_bands:
        row = f"{sb:>12}"
        for vb in speed_bands:
            n, stalls = grid.get((sb, vb), [0, 0])
            row += f"{'-':>16}" if n == 0 else f"{f'{100 * stalls / n:.1f}% ({n})':>16}"
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

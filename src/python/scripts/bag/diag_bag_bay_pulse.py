r"""The bay exit's duty cycle: how much of it is a command, and how much is nothing.

The operator reports the exit taking ~30 s where it once took 10, and the
clockwise round "going backwards" near the start. Both are the same thing, and
it is not a slow manoeuvre -- it is an INTERMITTENT one.

``BAY_EXIT_GUARD_BLOCK_TICKS``'s own docstring states the test the code relies
on to tell a guard doing its job from one that has trapped itself: *"A guard
that is BOUNDING legs alternates block and motion; only one that has trapped
itself blocks without interruption."* This measures both halves of that test at
once -- the share of ticks commanding zero, AND the cadence of the motion pulses
that interrupt them -- because a failure that alternates perfectly while going
nowhere satisfies the healthy signature and is invisible to it.

Reported per run: bay-exit wall time, the split of ticks into zero / forward /
reverse, the net displacement and net rotation the phase achieved, and the gap
between consecutive motion pulses with their signed speeds. A pendulum shows up
as alternating signs at a fixed period.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_bay_pulse.py \
        data/live/runs/run_20260911_1523* data/live/runs/run_20260911_1528*
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, elapsed_seconds, open_reader

BAY_EXIT_PHASE = "bay_exit"
"""``NavigatorPhase.BAY_EXIT``, compared as a string so a bag recorded before a
rename still reads rather than silently reporting an empty phase."""

MOVING_MPS = 1e-6
"""Anything below this is a commanded stop, not a slow leg. The commanded value
is exact -- this is the navigator's own output, not an odometry reading -- so the
threshold only has to exclude float noise."""

PULSES_SHOWN = 12
"""Enough to see a period and its alternation without printing a whole run."""


def analyse(bag_dir: Path) -> None:
    reader = open_reader(bag_dir)
    t0: int | None = None
    ticks = zero = fwd = rev = 0
    first = last = None
    start_pose = end_pose = None
    pulses: list[tuple[float, float]] = []
    was_moving = False

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic != Topics.NAV_DEBUG:
            continue
        try:
            snap = decode_nav_debug(data)
        except Exception:  # noqa: BLE001
            continue
        if str(snap.phase) != BAY_EXIT_PHASE:
            continue
        now = elapsed_seconds(t, t0)
        speed = snap.commanded_speed_mps or 0.0
        ticks += 1
        if first is None:
            first = now
            start_pose = (snap.pose_x, snap.pose_y, snap.pose_yaw)
        last = now
        end_pose = (snap.pose_x, snap.pose_y, snap.pose_yaw)
        moving = abs(speed) > MOVING_MPS
        if not moving:
            zero += 1
        elif speed > 0:
            fwd += 1
        else:
            rev += 1
        if moving and not was_moving:
            pulses.append((now, speed))
        was_moving = moving

    if not ticks or start_pose is None or end_pose is None:
        print(f"{bag_dir.name:<26} no bay_exit phase")
        return
    if None in start_pose or None in end_pose:
        print(f"{bag_dir.name:<26} bay_exit has no pose -- cannot price the progress")
        return

    moved = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
    turned = math.degrees(abs((end_pose[2] - start_pose[2] + math.pi) % (2 * math.pi) - math.pi))
    print(
        f"\n== {bag_dir.name}: bay_exit {last - first:.1f} s over {ticks} ticks\n"
        f"   commanded zero {100.0 * zero / ticks:.1f}%  forward {100.0 * fwd / ticks:.1f}%  "
        f"reverse {100.0 * rev / ticks:.1f}%\n"
        f"   net displacement {moved:.3f} m, net rotation {turned:.0f} deg"
    )
    if len(pulses) < 2:
        print(f"   {len(pulses)} motion pulse(s) -- no cadence to read")
        return
    gaps = [pulses[i + 1][0] - pulses[i][0] for i in range(len(pulses) - 1)]
    print(f"   {len(pulses)} motion pulses; gaps {[round(g, 2) for g in gaps[:PULSES_SHOWN]]}")
    print(f"   signed speeds  {[round(v, 2) for _, v in pulses[:PULSES_SHOWN]]}")
    alternating = sum(
        1 for i in range(len(pulses) - 1) if pulses[i][1] * pulses[i + 1][1] < 0
    )
    print(
        f"   consecutive pulses of OPPOSITE sign: {alternating}/{len(pulses) - 1} "
        f"({100.0 * alternating / (len(pulses) - 1):.0f}%)"
    )


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()
    for bag_dir in args.bag_dirs:
        try:
            analyse(Path(bag_dir))
        except Exception as exc:  # noqa: BLE001
            print(f"{Path(bag_dir).name:<26} unreadable: {ascii(exc)[:90]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

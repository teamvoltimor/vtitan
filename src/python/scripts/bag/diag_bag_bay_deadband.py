r"""Does the drivetrain actually turn at the bay-exit commanded speed?

``bay_exit_speed_mps`` was raised 0.067 -> 0.10 on 2026-09-06 because the
motor was not moving at the inherited creep: commanded on 876 of 882 ticks
while ``/motor/drive_speed`` read 0 deg/s on 92-97% of them. The TOML shipped
that value with an explicit instruction -- "WATCH ON THE NEXT RUN:
/motor/drive_speed leaving zero (the point), and fin clearance holding (the
risk)" -- and the 2026-09-08 session is the first bay corpus recorded since.

This reads that watch. Per run, restricted to ticks in the ``bay_exit`` phase:

* what speed was COMMANDED (``commanded_speed_mps``),
* what the wheel actually DID (``/motor/drive_speed``, in deg/s -- see
  ``bag_drive_speed_is_deg_per_sec``; NOT rpm), expressed as the fraction of
  commanded ticks the encoder read zero,
* the implied m/s from the encoder against the commanded m/s, so a partial
  deadband (turning, but slower than asked) is distinguishable from a total
  one,
* leg structure: how legs alternate and how long they last, since a ratchet
  that alternates with no travel is the stall, and
* pose travel, accumulated as |ds| -- signed travel CANCELS under a ratchet
  (``signed_wheel_odometry_cancels_under_a_ratchet``), which would report a
  working exit as motionless.

The distinction that matters: if the encoder still reads zero, 0.10 m/s is
still under the floor and the number goes up. If the encoder turns and the
chassis still does not rotate, the speed is NOT the constraint and the fault
is in the leg bounds or the guard.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_bay_deadband.py \
        data/live/runs/run_20260908_*
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader

WHEEL_CIRCUM_M = math.pi * 0.07
"""7 cm wheel (robot.toml). deg/s -> m/s is (deg/s / 360) * circumference."""

ZERO_DEG_S = 1.0
"""Below this the encoder is reporting a stopped wheel, not a slow one."""


def deg_s_to_mps(deg_s: float) -> float:
    return (deg_s / 360.0) * WHEEL_CIRCUM_M


def analyse(bag_dir: Path) -> dict[str, object] | None:
    reader = open_reader(bag_dir)
    t0 = None
    in_bay = False
    bay_ticks = 0
    commanded: list[float] = []
    # /motor/drive_speed arrives on its own topic; hold the most recent value
    # and sample it on each bay-phase nav tick so the two series line up.
    last_drive_deg_s: float | None = None
    sampled_drive: list[float] = []
    legs: list[tuple[bool, float]] = []  # (is_reverse, duration_s)
    leg_start: float | None = None
    leg_kind: bool | None = None
    abs_travel = 0.0
    prev_xy: tuple[float, float] | None = None
    yaws: list[float] = []
    net_yaw = 0.0
    abs_yaw = 0.0
    prev_yaw: float | None = None
    enc_dist = 0.0
    prev_enc_ts: float | None = None
    guard_gaps: list[float] = []
    dr_along: list[float] = []
    dr_out: list[float] = []
    last_ts = 0.0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        ts = elapsed_seconds(t, t0)

        if topic == Topics.MOTOR_DRIVE_SPEED:
            last_drive_deg_s = float(deserialize_message(data, Float32).data)
            continue
        if topic != Topics.NAV_DEBUG:
            continue

        snap = decode_nav_debug(data)
        ph = str(getattr(snap.phase, "value", snap.phase))
        if ph != "bay_exit":
            if in_bay and leg_start is not None:
                legs.append((bool(leg_kind), ts - leg_start))
                leg_start = None
            in_bay = False
            continue

        in_bay = True
        bay_ticks += 1
        last_ts = ts
        if snap.commanded_speed_mps is not None:
            commanded.append(snap.commanded_speed_mps)
        if last_drive_deg_s is not None:
            sampled_drive.append(abs(last_drive_deg_s))
        if snap.bay_guard_gap_m is not None:
            guard_gaps.append(snap.bay_guard_gap_m)
        if snap.bay_dr_along_m is not None:
            dr_along.append(snap.bay_dr_along_m)
        if snap.bay_dr_out_m is not None:
            dr_out.append(snap.bay_dr_out_m)
        if snap.pose_yaw is not None:
            # pose_yaw WRAPS at +-pi, so max-min over the run reads ~360 deg
            # whenever the run crosses the wrap -- it reported a full circle
            # inside a 0.065 m pocket. Accumulate wrapped DIFFERENCES instead.
            if prev_yaw is not None:
                d = math.atan2(math.sin(snap.pose_yaw - prev_yaw), math.cos(snap.pose_yaw - prev_yaw))
                net_yaw += d
                abs_yaw += abs(d)
            prev_yaw = snap.pose_yaw
            yaws.append(snap.pose_yaw)
        # Distance the WHEEL turned, integrated from the encoder. Pose travel
        # is localizer output and jumps on a bad scan match (it reported 2.4 m
        # of travel inside the bay), so it cannot measure whether the chassis
        # actually moved.
        if last_drive_deg_s is not None:
            if prev_enc_ts is not None:
                enc_dist += abs(deg_s_to_mps(last_drive_deg_s)) * (ts - prev_enc_ts)
            prev_enc_ts = ts
        if snap.pose_x is not None and snap.pose_y is not None:
            if prev_xy is not None:
                abs_travel += math.hypot(snap.pose_x - prev_xy[0], snap.pose_y - prev_xy[1])
            prev_xy = (snap.pose_x, snap.pose_y)

        if snap.bay_leg_is_reverse is not None:
            if leg_kind is None or snap.bay_leg_is_reverse != leg_kind:
                if leg_start is not None:
                    legs.append((bool(leg_kind), ts - leg_start))
                leg_kind = snap.bay_leg_is_reverse
                leg_start = ts

    if bay_ticks == 0:
        return None
    if leg_start is not None:
        legs.append((bool(leg_kind), last_ts - leg_start))

    moving = [d for d in sampled_drive if d > ZERO_DEG_S]
    cmd_nonzero = [c for c in commanded if abs(c) > 1e-6]
    return {
        "run": bag_dir.name[-6:],
        "bay_ticks": bay_ticks,
        "cmd_med": statistics.median([abs(c) for c in cmd_nonzero]) if cmd_nonzero else 0.0,
        "cmd_nonzero_pct": 100.0 * len(cmd_nonzero) / bay_ticks,
        "enc_n": len(sampled_drive),
        "enc_zero_pct": 100.0 * (len(sampled_drive) - len(moving)) / max(len(sampled_drive), 1),
        "enc_med_moving_deg_s": statistics.median(moving) if moving else 0.0,
        "enc_med_moving_mps": deg_s_to_mps(statistics.median(moving)) if moving else 0.0,
        "legs": len(legs),
        "leg_med_s": statistics.median([d for _, d in legs]) if legs else 0.0,
        "abs_travel_m": abs_travel,
        "net_yaw_deg": math.degrees(net_yaw),
        "abs_yaw_deg": math.degrees(abs_yaw),
        "enc_dist_m": enc_dist,
        "guard_med": statistics.median(guard_gaps) if guard_gaps else float("nan"),
        "dr_along_absmax": max((abs(v) for v in dr_along), default=float("nan")),
        "dr_out_absmax": max((abs(v) for v in dr_out), default=float("nan")),
    }


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    rows = [r for r in (analyse(b) for b in bags if b.is_dir()) if r]
    if not rows:
        print("no bag reached the bay_exit phase")
        return

    hdr = (f"{'run':8}{'ticks':>6}{'cmd m/s':>9}{'cmd%':>6}{'encN':>6}{'enc0%':>7}"
           f"{'encDeg/s':>9}{'enc m/s':>9}{'legs':>5}{'legS':>6}{'encM':>7}{'poseM':>7}{'netYaw':>8}{'|yaw|':>7}{'guard':>7}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['run']:8}{r['bay_ticks']:6}{r['cmd_med']:9.3f}{r['cmd_nonzero_pct']:6.0f}"
              f"{r['enc_n']:6}{r['enc_zero_pct']:7.1f}{r['enc_med_moving_deg_s']:9.1f}"
              f"{r['enc_med_moving_mps']:9.3f}{r['legs']:5}{r['leg_med_s']:6.2f}"
              f"{r['enc_dist_m']:7.3f}{r['abs_travel_m']:7.3f}"
              f"{r['net_yaw_deg']:8.1f}{r['abs_yaw_deg']:7.1f}{r['guard_med']:7.3f}")

    print()
    zp = [r["enc_zero_pct"] for r in rows if r["enc_n"]]
    if zp:
        print(f"encoder-zero % during bay_exit: min {min(zp):.1f} / median {statistics.median(zp):.1f} / max {max(zp):.1f}")
    cm = [r["cmd_med"] for r in rows]
    print(f"commanded m/s: min {min(cm):.3f} / median {statistics.median(cm):.3f} / max {max(cm):.3f}")
    mv = [r["enc_med_moving_mps"] for r in rows if r["enc_med_moving_mps"] > 0]
    if mv:
        print(f"encoder m/s WHEN MOVING: median {statistics.median(mv):.3f}")
    print(f"|travel| m: {[round(r['abs_travel_m'], 3) for r in rows]}")
    print(f"encoder distance m: {[round(r['enc_dist_m'], 3) for r in rows]}")
    print(f"net rotation deg: {[round(r['net_yaw_deg'], 1) for r in rows]}")
    print(f"abs rotation deg: {[round(r['abs_yaw_deg'], 1) for r in rows]}")


if __name__ == "__main__":
    main()

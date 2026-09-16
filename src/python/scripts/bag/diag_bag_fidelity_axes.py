r"""The HARDWARE half of the sim-divergence audit: measure a real round on the shared axes.

Pair this with ``scripts/sim/diag_fidelity_axes.py``, which measures the SAME
axes on a simulated round and prints them in the same line shapes. Neither
script is interesting alone; the finding is always the difference, and both draw
their definitions from ``scripts/common/fidelity_axes.py`` so the difference is
about the robot rather than about two authors' formatting.

WHY a whole instrument for this. Every tuning verdict this project has shipped
was scored in the simulator, and the simulator's idealisations are not uniformly
optimistic -- which is the trap. The 2026-09-15 audit found the vision frame-miss
rate 2-4x PESSIMISTIC and the terminal-push rule pessimistic too, while the
occlusion band, the escape rotation, the creep gain, the start pose and the
detection range were all optimistic. An idealisation you assume flatters you can
be costing you laps, and the only way to know the sign is to measure it.

AXES, in the order they were worth modelling:

1. **LIDAR sector shares** -- finite and sub-floor returns per robot-frame band.
   This is the axis that was measured in the wrong frame once; see
   ``diag_bag_lidar_frame_census.py``, which prints both frames precisely so
   this one can print a single honest table.
2. **Vision freshness** -- the share of NAV ticks that had a red/green detection
   arrive since the previous tick. Not camera fps and not detections per second:
   what the navigator sees is what it can act on, and the two differ by the
   nav/camera rate ratio.
3. **Escape episodes** -- count, duration, and the yaw the chassis actually
   turned through. The yaw is taken from the IMU, not from the pose, because the
   pose is the localizer's opinion and escapes are exactly when it is worst.
4. **Cruise** -- commanded versus ACHIEVED speed while a sign is committed. The
   ratio is the point: a planner tuned where the two are equal plans distances
   the chassis never covers.
5. **Steering** -- magnitude, per-tick change and sign flips on non-manoeuvre
   ticks only.
6. **Start pose** -- where the round actually began, against what
   ``measure_start_pose`` reported.

TRAP: escape yaw is unwrapped over the WHOLE run before differencing, because a
k_turn burst can exceed 180 degrees in one episode (measured: 275) and wrapping
per-episode silently reports the short way round.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_fidelity_axes.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from scripts.common.bag_io import (
    LIDAR_YAW_OFFSET_RAD,
    create_bags_parser,
    read_motion_streams,
    read_vision_rows_and_scans,
)
from scripts.common.fidelity_axes import (
    SectorCensus,
    contiguous_episodes,
    format_committed,
    format_escapes,
    format_steering,
)
from scripts.common.stats import percentile

_SCAN_STRIDE = 5
_ESCAPE_STEER_FLOOR = 0.3
"""Below this the escape is not really steering, and its yaw says nothing about turn authority."""


def _has_colour(payload: list[dict]) -> bool:
    """Does this vision frame carry a red or green detection?

    String-matched against the whole payload rather than a parsed class enum,
    because the recorded payload schema has changed twice and a diagnostic that
    silently reads zero detections after a schema change is worse than useless.
    """
    return any("red" in str(d).lower() or "green" in str(d).lower() for d in payload)


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    args = parser.parse_args()

    for bag_dir in args.bag_dirs:
        rows, frames, scans = read_vision_rows_and_scans(bag_dir, with_scans=True)
        if not rows:
            print(f"\n=== {bag_dir.name}: no /nav_debug rows")
            continue
        streams = read_motion_streams(bag_dir)
        times = np.array([t for t, _ in rows])
        duration = times[-1] - times[0]
        print(f"\n=== {bag_dir.name}  nav_ticks={len(rows)} vision_frames={len(frames)} scans={len(scans)}")

        imu_t = np.array([t for t, _ in streams.imu_yaw_rad])
        imu_yaw = np.unwrap([y for _t, y in streams.imu_yaw_rad]) if len(imu_t) else np.array([])

        def yaw_at(t: float, imu_t: np.ndarray = imu_t, imu_yaw: np.ndarray = imu_yaw) -> float:
            return float(np.interp(t, imu_t, imu_yaw)) if imu_yaw.size else math.nan

        episodes = contiguous_episodes([snap.active_maneuver_type is not None for _t, snap in rows])
        kinds: dict[str, int] = {}
        durations: list[float] = []
        deltas: list[float] = []
        for a, b in episodes:
            durations.append(rows[b][0] - rows[a][0])
            deltas.append(abs(math.degrees(yaw_at(rows[b][0]) - yaw_at(rows[a][0]))))
            key = str(rows[a][1].active_maneuver_type)
            kinds[key] = kinds.get(key, 0) + 1
        for line in format_escapes(
            kinds=kinds,
            durations_s=durations,
            yaw_deltas_deg=deltas,
            short_episodes=sum(1 for a, b in episodes if b - a + 1 <= 2),
        ):
            print(line)

        turn_rates: list[float] = []
        for a, b in episodes:
            for i in range(a, b):
                steer = rows[i][1].maneuver_steering
                if steer is None or abs(steer) < _ESCAPE_STEER_FLOOR:
                    continue
                dt = rows[i + 1][0] - rows[i][0]
                if dt > 0:
                    turn_rates.append(abs(math.degrees(yaw_at(rows[i + 1][0]) - yaw_at(rows[i][0]))) / dt)
        print(
            f"  escape ticks |steer|>={_ESCAPE_STEER_FLOOR}: n={len(turn_rates)} "
            f"yaw rate deg/s p50={percentile(turn_rates, 0.5):.1f} p90={percentile(turn_rates, 0.9):.1f}"
        )

        achieved = streams.drive_speed_mps()
        ach_t = np.array([t for t, _ in achieved])
        ach_v = np.array([v for _t, v in achieved])
        committed = [(t, s) for t, s in rows if s.committed_sign_x_m is not None and s.active_maneuver_type is None]
        print(
            format_committed(
                committed_ticks=len(committed),
                total_ticks=len(rows),
                commanded_mps=[s.commanded_speed_mps for _t, s in committed if s.commanded_speed_mps is not None],
                achieved_mps=[abs(float(np.interp(t, ach_t, ach_v))) for t, _s in committed] if ach_v.size else [],
            )
        )

        steer = [s.commanded_steering_norm for _t, s in rows if s.commanded_steering_norm is not None and s.active_maneuver_type is None]
        for line in format_steering(steer, duration):
            print(line)

        if frames:
            frame_t = np.array([t for t, _ in frames])
            hits = np.array([_has_colour(payload) for _t, payload in frames])
            cumulative = np.concatenate([[0], np.cumsum(hits)])
            previous = np.concatenate([[times[0] - 0.05], times[:-1]])
            fresh = (cumulative[np.searchsorted(frame_t, times)] - cumulative[np.searchsorted(frame_t, previous)]) > 0
            print(
                f"vision: camera {len(frames) / duration:.1f} fps, frames with red/green={100 * hits.mean():.1f}%  "
                f"NAV ticks with a fresh detection={100 * fresh.mean():.1f}%  nav {len(rows) / duration:.1f} Hz"
            )

        census = SectorCensus()
        for _t, data in scans[::_SCAN_STRIDE]:
            msg = deserialize_message(data, LaserScan)
            raw = np.asarray(msg.ranges, dtype=float)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(raw)) + LIDAR_YAW_OFFSET_RAD
            census.add(np.degrees((angles + math.pi) % (2 * math.pi) - math.pi), raw, finite=np.isfinite(raw))
        print(f"{census.format()}   (robot frame, {census.scans} scans; 'finite' = the driver returned a number)")

        start = [(s.pose_x, s.pose_y, s.pose_yaw) for _t, s in rows[:40] if s.pose_y is not None]
        if start:
            print(
                f"start pose (first 40 ticks): x={percentile([p[0] for p in start], 0.5):.3f} "
                f"y={percentile([p[1] for p in start], 0.5):.3f} "
                f"yaw={math.degrees(percentile([p[2] for p in start], 0.5)):.1f} deg"
            )
        measured = next((s for _t, s in rows if s.start_measured_x is not None), None)
        if measured is not None:
            print(f"  measure_start_pose said ({measured.start_measured_x:.3f},{measured.start_measured_y:.3f})")

        last = rows[-1][1]
        print(f"final: laps={last.laps_completed} wrong_side={last.wrong_side_pass_count} escape_count={last.escape_count}")

    print(
        "\nRun scripts/sim/diag_fidelity_axes.py and diff the blocks. Do NOT assume the sign of a\n"
        "divergence: measured 2026-09-15, vision freshness and the push rule were PESSIMISTIC in sim\n"
        "while occlusion, escape rotation, creep and detection range were optimistic."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

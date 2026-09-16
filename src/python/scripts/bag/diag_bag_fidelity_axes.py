r"""The HARDWARE half of the sim-divergence audit: measure a real round on the shared axes.

Pair this with ``scripts/sim/diag_fidelity_axes.py``, which measures the SAME
axes on a simulated round and prints them in the same line shapes. Neither
script is interesting alone; the finding is always the difference, and both draw
their definitions from ``scripts/common/fidelity_axes.py`` so the difference is
about the robot rather than about two authors' formatting.

WHY a whole instrument for this. Every tuning verdict this project has shipped
was scored in the simulator, and the simulator's idealisations are not uniformly
optimistic -- which is the trap. The audit found some axes PESSIMISTIC and others
optimistic, so an idealisation you assume flatters you can be costing you laps,
and the only way to know the sign is to measure it. See
``adr:0086-simulator-realism`` for the measured verdict.

AXES, in the order they were worth modelling:

1. **LIDAR sector shares** -- finite and sub-floor returns per robot-frame band.
   This is the axis that was measured in the wrong frame once; see
   ``diag_bag_lidar_frame_census.py``, which prints both frames precisely so
   this one can print a single honest table.
2. **Vision freshness** -- the share of NAV ticks that had a fresh colour
   OBSERVATION arrive since the previous tick, where "observation" means what
   ``detection_to_observation`` keeps, not what the detector emitted. Not camera
   fps and not detections per second: what the navigator can act on is the
   subject. The definition lives in ``scripts/common/fidelity_axes.py`` because
   this script and the simulator one computed it differently until 2026-09-16 --
   see that module for the two measured biases, both of which flattered the
   hardware side.
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
k_turn burst can exceed 180 degrees in one episode and wrapping per-episode
silently reports the short way round.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_fidelity_axes.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
from typing import TYPE_CHECKING

import numpy as np
import shared.domain.enums  # noqa: F401  (imported first: the models <-> enums cycle needs a seed)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.domain.models import Pose, SignColor

from scripts.common.bag_io import (
    LIDAR_YAW_OFFSET_RAD,
    create_bags_parser,
    decode_detections,
    read_motion_streams,
    read_vision_rows_and_scans,
)
from scripts.common.fidelity_axes import (
    SectorCensus,
    VisionFreshness,
    contiguous_episodes,
    format_committed,
    format_escapes,
    format_steering,
    format_vision_freshness,
    frame_carries_observation,
)
from scripts.common.stats import percentile

if TYPE_CHECKING:
    from collections.abc import Callable

_SCAN_STRIDE = 5
_ESCAPE_STEER_FLOOR = 0.3
"""Below this the escape is not really steering, and its yaw says nothing about turn authority."""


def _raw_colour_mention(payload: list[dict]) -> bool:
    """Is the string ``red`` or ``green`` anywhere in this frame's payload?

    KEPT ONLY AS A CONTROL, and deliberately no longer the axis. This loose match
    is what this script used to report as vision freshness, and it counts records
    production discards -- a dict with no usable bbox never survives
    ``decode_detections``, and a box whose aspect ratio is not a pillar's never
    survives ``detection_to_observation``. Printing all three shares side by side
    is how the funnel stays visible: 50.2% raw, 39.7% typed, 32.2% kept on
    ``run_20260915_140358``. The schema-drift argument for the loose match still
    holds, which is exactly why it stays on as the control that would go to zero.
    """
    return any("red" in str(d).lower() or "green" in str(d).lower() for d in payload)


def _pose_interpolator(rows: list[tuple[float, object]]) -> Callable[[float], Pose]:
    """Believed pose at an arbitrary time, interpolated from the /nav_debug rows.

    Yaw is unwrapped before interpolation: halfway between +179 and -179 degrees
    is 180, not 0, and the naive average points the camera backwards for exactly
    the frames taken while cornering.
    """
    pose_t = np.array([t for t, s in rows if s.pose_y is not None])
    pose_xyz = np.array([(s.pose_x, s.pose_y, s.pose_yaw) for _t, s in rows if s.pose_y is not None])
    yaw = np.unwrap(pose_xyz[:, 2]) if pose_t.size else np.array([])

    def at(t: float) -> Pose:
        if not pose_t.size:
            return Pose(x=0.0, y=0.0, yaw=0.0)
        return Pose(
            x=float(np.interp(t, pose_t, pose_xyz[:, 0])),
            y=float(np.interp(t, pose_t, pose_xyz[:, 1])),
            yaw=float(np.interp(t, pose_t, yaw)),
        )

    return at


def _vision_lines(
    rows: list[tuple[float, object]],
    frames: list[tuple[float, list[dict]]],
    times: np.ndarray,
    duration: float,
) -> list[str]:
    """The shared vision-freshness block, plus this source's own funnel.

    The FUNNEL is printed because the axis moved: raw is any mention of a colour
    in the payload, typed is what ``decode_detections`` rebuilds, and kept is
    what ``detection_to_observation`` lets through. Only the last is the axis.
    Seeing all three means the next person can tell a detector that went quiet
    from a payload schema that changed under the parser -- which is the failure
    the old loose match was defending against, kept without letting it be the
    headline number again.
    """
    if not frames:
        return []
    frame_t = np.array([t for t, _ in frames])
    pose_at = _pose_interpolator(rows)
    decoded = [(t, decode_detections(payload)) for t, payload in frames]
    previous = np.concatenate([[times[0] - 0.05], times[:-1]])

    def fresh_ticks(frame_hits: list[bool]) -> np.ndarray:
        """Nav ticks that had at least one hit land since the previous tick."""
        cumulative = np.concatenate([[0], np.cumsum(np.array(frame_hits, dtype=int))])
        return (cumulative[np.searchsorted(frame_t, times)] - cumulative[np.searchsorted(frame_t, previous)]) > 0

    raw = fresh_ticks([_raw_colour_mention(payload) for _t, payload in frames])
    typed = fresh_ticks([any(d.color in (SignColor.RED, SignColor.GREEN) for d in dets) for _t, dets in decoded])
    kept = fresh_ticks([frame_carries_observation(dets, pose_at(t)) for t, dets in decoded])
    return format_vision_freshness(
        VisionFreshness(hits=int(kept.sum()), opportunities=len(times), denominator="nav ticks"),
        extra=(
            f"camera {len(frames) / duration:.1f} fps, nav {len(rows) / duration:.1f} Hz; "
            f"funnel over the SAME nav ticks: raw={100 * raw.mean():.1f}% -> "
            f"typed={100 * typed.mean():.1f}% -> kept={100 * kept.mean():.1f}%"
        ),
    )


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

        for line in _vision_lines(rows, frames, times, duration):
            print(line)

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

        # rows[-1] is the navigator's post-run RESET snapshot (phase
        # NOT_YET_STEPPED, every counter back to zero), so reading the outcome
        # off it reports laps=0 for a round that finished three. Take the last
        # snapshot that still describes the round.
        last = next(
            (s for _t, s in reversed(rows) if getattr(s.phase, "name", None) != "NOT_YET_STEPPED"),
            rows[-1][1],
        )
        print(f"final: laps={last.laps_completed} wrong_side={last.wrong_side_pass_count} escape_count={last.escape_count}")

    print(
        "\nRun scripts/sim/diag_fidelity_axes.py and diff the blocks. Do NOT assume the sign of a\n"
        "divergence: measured 2026-09-15, vision freshness and the push rule were PESSIMISTIC in sim\n"
        "while occlusion, escape rotation, creep and detection range were optimistic.\n"
        "The vision share above is the PRODUCTION-OBSERVATION one over NAV TICKS. The simulator's\n"
        "is over GATEWAY POLLS, which is not the same denominator -- both halves now name theirs."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

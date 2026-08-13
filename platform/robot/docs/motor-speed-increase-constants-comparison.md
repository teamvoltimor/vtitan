# Motor speed increase (+60%) — constants comparison

Working note for evaluating a motor swap/upgrade expected to raise top speed
by ~60%. Current ceiling is the bench-measured `[drivetrain] max_speed_mps =
0.156` in `platform/shared/config/robot.toml`. Projected column assumes the
new ceiling is confirmed at **0.156 × 1.6 ≈ 0.25 m/s** — replace with the
actual bench-measured number once available, then recompute the derived rows.

## Drivetrain ceiling

| Constant | File | Current | Projected (×1.6) | Notes |
|---|---|---|---|---|
| `max_speed_mps` | `platform/shared/config/robot.toml` [drivetrain] | 0.156 | ~0.25 (re-measure) | Hard ceiling — `AckermannKinematics` clamps to it. Single source; everything below reads through `RobotSpecs.MAX_SPEED_MPS`. |
| `max_accel_mps2` | same, [drivetrain] | 2.0 | ? | Physical accel limit — depends on the new motor, not derivable from top-speed alone. Bench-measure separately. |

## Speed ladder (auto-scales — fractions of the ceiling)

`SpeedControlParams` in `platform/shared/src/shared/config/navigation_tuning/motion.py`. These recompute automatically once `max_speed_mps` is updated — no code change needed — but the *tiers were chosen* (2026-08-09) based on hardware behavior at the 0.156 ceiling, so the fractions themselves may no longer be the right breakpoints.

| Tier | Fraction | Current m/s | Projected m/s (×1.6) |
|---|---|---|---|
| MIN_FRAC (friction floor) | 0.32 | 0.050 | 0.080 |
| CREEP_FRAC | 0.65 | 0.101 | 0.162 |
| SLOW_FRAC | 0.75 | 0.117 | 0.187 |
| MEDIUM_FRAC | 0.85 | 0.133 | 0.212 |
| FAST_FRAC / MAX_FRAC | 1.00 | 0.156 | 0.250 |

Watch for: MIN_FRAC is the *measured* friction floor at the old motor's torque curve — a new motor may overcome friction at a different fraction, which would need a fresh bench pass, not just a scale-up.

## Steering / geometry (physical — do NOT scale with speed)

| Constant | File | Value | Notes |
|---|---|---|---|
| `max_wheel_angle_deg` | robot.toml [steering] | 55.0 | Bench-measured servo linkage limit, unrelated to motor speed. |
| `rear_steer_ratio` | robot.toml [drivetrain] | 1.0 | Counter-phase 4-wheel steering, unrelated to motor speed. |
| CRAWL heading threshold | motion.py `HeadingErrorZones` | 1.0 rad (~57°) | Servo slew-rate limited, not drive-speed limited — but the robot now *reaches* that heading error faster, so it may need to trigger earlier in time even though the angle threshold itself is unchanged. |

## Control-loop / lookahead (fixed distances — effective reaction time shrinks)

`PurePursuitParams` and `ControlLoopParams`, same file. None of these scale automatically with `max_speed_mps` — they're distances/gains, not fractions of the ceiling. At a higher top speed the same distance corresponds to less *time*, so these are the prime re-tune candidates:

| Constant | Current | Effective reaction time at 0.156 m/s | Effective reaction time at 0.25 m/s |
|---|---|---|---|
| `LOOKAHEAD_SHORT` | 0.20 m | ~1.28 s | ~0.80 s |
| `LOOKAHEAD_LONG` | 0.40 m | ~2.56 s | ~1.60 s |
| `CORNER_PREVIEW_DISTANCE_M` | 0.40 m | ~2.56 s | ~1.60 s |
| Per-tick travel (`CONTROL_HZ` = 20 Hz, dt = 0.05 s) | — | 0.0078 m/tick | 0.0125 m/tick |
| `STEER_KP` | 1.2 | — | gain tuned at old dynamics; re-tune |
| `WALL_MARGIN_SAFETY_M` | 0.03 m | — | crosstrack margin measured 0.103 m vs 0.16 m threshold at old speed; re-measure |
| `ARC_RADIUS` (corridor→corridor tightened tier) | 0.45 / 0.25 m | — | lateral accel at a fixed turn radius rises with v²; recheck the tightened 0.25 m corner radius doesn't need to widen |

## Bottom line

- **Auto-scales, don't touch the code**: the speed ladder (once `max_speed_mps` is updated in `robot.toml` and `task gen:robot-constants` is re-run).
- **Needs a fresh bench/bag pass, not a formula**: `STEER_KP`, understeer/linkage numbers, `WALL_MARGIN_SAFETY_M` / crosstrack threshold, lookahead distances, and whether `ARC_RADIUS` tiers still clear the tighter corners at the new speed.
- **Needs independent bench measurement, not derivable from the +60% figure**: `max_accel_mps2`, and the actual new `max_speed_mps` itself (don't assume the vendor/motor-swap +60% claim holds under load — it's exactly the kind of number that was wrong before, see the 0.156 vs earlier-assumed-0.5 history).

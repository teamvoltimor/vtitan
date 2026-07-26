# These profiles are NOT loaded by anything

Every `.json` file in this directory is dead config. Editing one has no effect
on the robot or the simulator. This was verified 2026-07-25 by attempting every
loader on every file:

```
real_robot   via load_from_json: TypeError: ClearanceZones.__init__() got an unexpected keyword argument 'contact_dist'
real_robot   via load_from_yaml: TypeError  (same — JSON is a YAML subset, so this path fails identically)
conservative via both:           TypeError
simulation   via both:           TypeError
```

Two independent reasons they cannot work:

1. **No caller.** `NavigationTuning` is only ever loaded from a file by
   `src/ros2/navigation/node.py`, from its `--tuning` argument, which defaults
   to unset — in which case the hardcoded `NavigationTuning()` defaults are
   used. `race.launch.py` only forwards `--tuning` when the launch argument is
   non-empty, and it defaults to empty. Nothing anywhere references these three
   filenames.
2. **Incompatible schema.** This is not just key casing. These files use group
   names (`heading_error`, `lookahead`, `collision`, `steering`, `stuck`,
   `vision`) that have no counterpart in `NavigationTuning._GROUPS`
   (`clearance`, `heading`, `pursuit`, `speed`, `escape`, `sensor`,
   `waypoints`), and field names that have no counterpart in the dataclasses.
   Making them load is a rewrite, not a rename.

## The one thing here worth rescuing

`real_robot.json` carries hardware-measured speeds that the live defaults do
**not** reflect:

| | `real_robot.json` | `SpeedControlParams` default |
|---|---|---|
| contact / creep | 0.0195 | 0.05 |
| slow | 0.0455 | 0.15 |
| medium | 0.065 | 0.30 |
| fast | 0.091 | 0.50 |
| full | 0.13 | 0.50 (`MAX_SPEED`) |

The measured drivetrain ceiling is 0.156 m/s
(`kinematics._DEFAULT_MAX_SPEED_MPS`, from 0.796 m / 5.11 s on 2026-07-25), and
`AckermannKinematics.step` clamps to it. So in the live defaults **`MEDIUM_SPEED`
(0.30) and `FAST_SPEED` (0.50) both saturate to the same 0.156 m/s** — the speed
ladder has four rungs but only three distinct values, and the top two are
indistinguishable. `real_robot.json` is the version of this ladder that has
distinct, achievable rungs, and it is the one being ignored.

Before deleting this directory, decide whether that ladder should replace the
defaults. That is a live navigation question, not a cleanup one — it changes
every simulation result.

See `docs/sign-avoidance-investigation.md`.

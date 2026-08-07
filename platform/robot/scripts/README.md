# Robot scripts

Grouped by **what you need in order to run it**, which is the axis that actually
decides where a script can execute and what it can tell you.

| folder | needs | what lives here |
|---|---|---|
| `bag/` | a recorded `.mcap` run | replay diagnostics — drive the real navigation classes over a bag and report what they did |
| `sim/` | nothing | closed-loop simulation sweeps and scenario harnesses |
| `hardware/` | the real robot | live probes and motor/vision utilities that talk to ROS2 topics or peripherals |
| `lib/` | — | shared helpers imported by the above, not run directly |
| `*.sh` | SSH to a Pi | deploy, bootstrap, shutdown and link-verification ops |

The shell ops scripts stay at the top level because `pixi.toml` tasks, the
platform `Taskfile.yml` and the docs all pin their paths.

## Running one

Every script is run from `platform/robot/` with the repo root on `PYTHONPATH`,
which the `dev` pixi env provides:

```
pixi run -e dev python scripts/bag/diag_bag_lap_replay.py vtitan_runs_pulled/run_20260806_180154
pixi run -e dev python scripts/sim/diag_open_laps.py
pixi run -e vision python scripts/hardware/diag_hailo_detector.py IMAGE...
```

`scripts/hardware/diag_hailo_detector.py` needs the `vision` env — `hailo_platform`
is only installed there and on the Pi 5.

## Adding one

Put it in the folder matching what it needs to run. Import shared helpers as
`from scripts.common.bag_io import ...`, and put the repo's robot root on the path
with `parents[2]`, not `parents[1]`:

```python
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows, print_table
```

## Checking the tree still imports

A folder move or a renamed helper breaks imports silently — nothing runs these
in CI. `scripts/common/check_imports.py` loads every script's top level (without
running `main()`) and reports what failed:

```
pixi run -e dev python scripts/common/check_imports.py
```

One expected failure off the Pi 5: `hardware/diag_hailo_detector.py`, for the
missing `hailo_platform` runtime.

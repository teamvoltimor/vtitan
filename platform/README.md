# Voldemorbot v2 — Platform

WRO 2026 Future Engineers simulation and robot platform.

## Structure

```
platform/
├── backend/        # Telemetry API server (FastAPI)
├── frontend/       # 3D visualiser (React + Three.js)
├── simulation/     # Scenario and track generation
├── robot/          # ROS2 navigation (hardware + simulation)
└── training_data/  # Generated scenarios (shared across projects)
```

## Prerequisites

| Tool | Used by | Install |
|------|---------|---------|
| [UV](https://docs.astral.sh/uv/) | backend, simulation | `pip install uv` |
| [Node.js](https://nodejs.org/) | frontend | nodejs.org |
| [Pixi](https://pixi.sh/) | robot (ROS2) | `pip install pixi` |

---

## Backend

Telemetry API server — exposes robot state snapshots over HTTP.

```bash
cd backend
uv sync               # first time only
uv run python main.py

# Run tests
uv run --extra dev pytest
```

Runs on `http://localhost:8010`. Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `TELEMETRY_PORT` | `8010` | Server port |
| `TELEMETRY_SESSIONS_DIR` | `./telemetry_sessions` | Session storage directory |
| `TELEMETRY_RELOAD` | `0` | Set to `1` to enable hot-reload (dev only) |

Sessions are kept up to a maximum of 20 (oldest evicted automatically).

**Endpoints**

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/telemetry/latest` | Latest robot snapshot |
| `GET` | `/telemetry/history?limit=60` | Recent snapshot history |
| `POST` | `/telemetry/record` | Persist a snapshot |
| `GET` | `/docs` | Interactive API docs |

---

## Frontend

3D track visualiser built with React, Three.js and Vite.

```bash
cd frontend
npm install           # first time only
npm run dev
```

Runs on `http://localhost:5173`.

The frontend reads configuration from `frontend/.env.development` (already committed):

| Variable | Default | Description |
|----------|---------|-------------|
| `VITE_TELEMETRY_BASE` | `http://localhost:8010` | Backend URL |
| `VITE_POLL_INTERVAL_MS` | `2500` | Live telemetry poll interval (ms) |

---

## Simulation

Generates randomized WRO 2026 Gazebo scenario SDF files and the base track.
Output always goes to `platform/training_data/`.

```bash
cd simulation
uv sync               # first time only
```

### Generate the base track

Generates `simulation/worlds/wro_track_2026.sdf` from WRO spec dimensions.
Run this once before generating scenarios.

```bash
uv run python main.py generate-track
```

| Flag | Default | Description |
|------|---------|-------------|
| `--output` | `./worlds/wro_track_2026.sdf` | Output SDF path |

### Generate scenarios

```bash
uv run python main.py generate
uv run python main.py generate --challenge obstacles --num-scenarios 50 --randomize-all
```

| Flag | Default | Description |
|------|---------|-------------|
| `--challenge` | `open` | `open` or `obstacles` |
| `--num-scenarios` | `10` | Number of scenarios to generate |
| `--output-dir` | `../training_data` | Root output directory |
| `--base-world` | `./worlds/wro_track_2026.sdf` | Base track SDF template |
| `--randomize-all` | off | Randomize lighting, widths, and starting position |

Scenarios are written to `training_data/<challenge>/scenarios/`.

---

## Robot

ROS2 waypoint-following navigator. Requires Pixi for the ROS2 environment.

```bash
cd robot
pixi install          # first time only — installs ROS2 Kilted via RoboStack
```

### Run the navigator

```bash
pixi run navigate
# or explicitly:
pixi run python main.py navigate --metadata ../training_data/open/scenarios/scenario_0000_metadata.json
```

| Flag | Default | Description |
|------|---------|-------------|
| `--metadata` | required | Path to scenario metadata JSON |
| `--laps` | `3` | Number of laps to complete |
| `--params` | none | Optional `navigator_params.json` for runtime overrides |

### Run the simple driver

```bash
pixi run drive
# or explicitly:
pixi run python main.py drive --direction clockwise --duration 30
```

### Run tests (no ROS2 needed)

```bash
pixi run test
```

---

## Typical workflow

```bash
# 1. Generate the base track (once)
cd simulation && uv run python main.py generate-track

# 2. Generate training scenarios
uv run python main.py generate --num-scenarios 20 --randomize-all

# 3. Start the backend
cd ../backend && uv run python main.py

# 4. Start the frontend (separate terminal)
cd ../frontend && npm run dev

# 5. Run the navigator on a scenario (separate terminal)
cd ../robot && pixi run navigate
```

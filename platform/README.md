# vTitan v2 — Platform

WRO 2026 Future Engineers simulation and robot platform.

## Structure

```
platform/
├── backend/           # Go telemetry API server (HTTP + gRPC)
├── frontend/          # 3D telemetry dashboard (React + Three.js + Vite)
├── gazebo/
│   ├── generator/     # simgen — scenario & track SDF generator (Go)
│   └── runtime/       # Gazebo/ROS2 simulation runtime (Python)
├── robot/             # ROS2 on-robot platform (Pixi + RoboStack)
├── proto/             # buf-managed protobuf definitions
├── shared/            # Shared Python utilities
├── docs/internal/     # Development notes, ADRs, reviews
├── conftest.py        # Pytest config (adds platform root to sys.path)
└── Taskfile.yml       # All available commands
```

## Prerequisites

| Tool | Used by | Install |
|------|---------|---------|
| [Go](https://go.dev/) | backend, simgen | go.dev |
| [UV](https://docs.astral.sh/uv/) | runtime, robot | `pip install uv` |
| [Pixi](https://pixi.sh/) | runtime, robot | `pip install pixi` |
| [Node.js](https://nodejs.org/) | frontend | nodejs.org |
| [Task](https://taskfile.dev/) | all | `go install github.com/go-task/task/v3/cmd/task@latest` |
| [buf](https://buf.build/) | proto | `go install github.com/bufbuild/buf/cmd/buf@latest` |
| [Docker](https://docker.com/) | frontend, compose | docker.com |

## Commands

All platform operations use `task` (see Taskfile.yml). Convention: `module:verb`.

### Umbrella

```bash
task install        # Install all deps (UV + Pixi + Go bootstrap)
task test           # Run all tests (Go + Python)
task lint           # Run all linters (ruff + golangci-lint + ESLint + buf)
task lint:fix       # Auto-fix lint issues
task clean          # Remove generated training data
task clean:all      # Deep clean (training data + cache + Go artifacts)
task init:dev       # One-time dev setup (install + lint)
```

### Backend (Go telemetry server)

HTTP API at `:8010`, gRPC ingest at `:9010`.

```bash
task backend:install    # Bootstrap (buf generate + sqlc + go mod tidy)
task backend:dev        # Dev mode with synthetic data (--sim)
task backend:run        # Production mode (expects real robot gRPC feed)
task backend:build      # Compile binary to backend/bin/server
task backend:test       # Go tests
task backend:lint       # golangci-lint
task backend:fmt        # goimports + gofmt
task backend:sqlc       # Regenerate DB layer from SQL
```

**HTTP endpoints:**

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/v1/telemetry/latest` | Latest robot snapshot |
| `GET` | `/v1/telemetry/history` | Snapshot history |
| `GET` | `/v1/telemetry/topics` | Latest topics snapshot |
| `GET` | `/v1/telemetry/sessions` | List recorded sessions |
| `GET` | `/v1/telemetry/sessions/:id` | Load a session |
| `GET` | `/v1/telemetry/config` | Get robot config |
| `POST` | `/v1/telemetry/robot/config/speed` | Update speed config |
| `GET` | `/v1/telemetry/health` | Health check |
| `GET` | `/v1/telemetry/ws` | WebSocket real-time stream |

**Environment variables (prefix `TELEMETRY_`):**

| Variable | Default | Description |
|----------|---------|-------------|
| `HTTP_ADDR` | `:8010` | HTTP server address |
| `GRPC_ADDR` | `:9010` | gRPC server address |
| `HISTORY_SIZE` | `360` | In-memory ring buffer size |
| `DEV` | `false` | Set to `true` for dev mode |
| `SIM_INTERVAL_MS` | `2500` | Synthetic data interval |
| `MAX_SESSIONS` | `20` | Max recorded sessions |
| `SESSIONS_DIR` | `data/sessions` | Session storage |
| `DB_PATH` | `data/sessions.db` | SQLite database path |

### Frontend (Vite + React telemetry dashboard)

```bash
task frontend:install       # npm ci
task frontend:dev           # Dev server (:5173), ?demo for mock data
task frontend:dev DEMO=true # Dev server with VITE_DEMO=true
task frontend:build         # tsc -b + vite build → dist/
task frontend:preview       # Preview production build
task frontend:typecheck     # tsc -b only
task frontend:lint          # ESLint
task frontend:docker:build  # Build nginx Docker image
task frontend:docker:run    # Run container on :8080
```

**Environment (`.env.development`):**

| Variable | Default | Description |
|----------|---------|-------------|
| `VITE_TELEMETRY_BASE` | `http://localhost:8010` | Backend URL |
| `VITE_POLL_INTERVAL_MS` | `2500` | Poll interval (ms) |

### Simgen (Go scenario generator)

Generates randomized WRO 2026 Gazebo SDF world files and metadata. ~50ms/scenario.

```bash
task simgen:install      # Build binary
task simgen:build        # Compile to robot-go/bin/simgen
task simgen:test         # Go tests
task simgen:lint         # golangci-lint
```

```bash
task gen:track           # Generate/regenerate base track SDF (once)
task gen:scenarios CHALLENGE=open       # Open-challenge scenarios (SCENARIOS=10)
task gen:scenarios CHALLENGE=obstacles  # Obstacles-challenge scenarios
task gen:all             # Both challenges
task gen:preview         # SVG top-down preview (METADATA=path)
```

### Gazebo Runtime (Python/ROS2 simulation)

Gazebo simulation, navigation, recording, and analysis.

```bash
task sim:install           # Install deps (UV + Pixi)
task sim:init              # Init ROS2 env via pixi (one-time)
task sim:gazebo            # Launch Gazebo with base track
task sim:gazebo SDF=path   # Launch Gazebo with a scenario SDF
task sim:rviz              # Launch RViz
task sim:navigate          # Run navigator (METADATA, LAPS overrides)
task sim:analyze           # Analyze generated scenarios
task sim:test              # Run tests
task sim:test QUICK=true   # Fail-fast tests
task sim:lint              # ruff
```

### Robot (on-robot platform)

ROS2 waypoint-following navigator (Pixi + RoboStack).

```bash
task robot:install   # pixi install
task robot:test      # Run tests (pixi -e dev), SCOPE=all|unit|hardware
task robot:lint      # ruff
```

### Proto (buf toolchain)

```bash
task proto ACTION=update      # Update buf deps
task proto ACTION=generate    # Generate Go + Python + TS stubs
task proto ACTION=lint        # Lint proto files
task proto ACTION=breaking    # Breaking change check vs master
```

### Recording pipeline

```bash
task record:run       # Record scenario videos from Gazebo
task record:convert   # Convert ROS2 bags to MP4
task record:frames    # Extract annotated frames from bags
```

### Docker

```bash
task docker:build    # docker compose build
task docker:up       # Start services
task docker:down     # Stop services
task docker:logs     # Follow logs
```

### Workflows

```bash
task workflow:generate   # Track → all scenarios
task workflow:dev        # Install → lint → test
task workflow:record     # Record → convert → frames
```

### Typical workflow

```bash
# 1. Generate base track (once)
task gen:track

# 2. Generate training scenarios
task gen:all SCENARIOS=20

# 3. Start backend with synthetic data
task backend:dev

# 4. Start frontend (separate terminal)
task frontend:dev

# 5. Run the navigator on a scenario
task sim:navigate METADATA=robot-go/training_data/open/scenarios/scenario_0000_metadata.json
```

## Challenge Types

- **Open:** Randomized corridor widths (600mm or 1000mm), no obstacles, no signs
- **Obstacles:** Fixed 1000mm corridors, traffic signs (WRO 36-scenario), parking lot

## Coordinate System

Bottom-left origin (0,0) at south-west, Z up. Track: 3.0×3.0 m. Full mat: 3.2×3.2 m.

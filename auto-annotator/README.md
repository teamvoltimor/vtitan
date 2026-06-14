# Auto-Annotator

SAM2-based image annotation service with a Go orchestration API, gRPC ML service,
and Vite frontend.

## Architecture

```
┌──────────┐   gRPC    ┌──────────────┐
│ Frontend │◄──HTTP──►│  Go API      │◄──gRPC──►│ ML Service  │
│ (Vite)   │           │  (Gin/SQLite)│           │ (SAM/gRPC)  │
└──────────┘           └──────────────┘           └─────────────┘
```

- **`api/`** — Go orchestration service (Gin HTTP API + SQLite + gRPC client)
- **`ml-service/`** — Python ML service (gRPC SAM model server + compute workers)
- **`frontend/`** — Vite + React annotation UI
- **`proto/`** — Shared gRPC contract (`autoannotator.v1.compute`)

## Quick Start

```bash
# 1. Install dependencies (ML service + frontend)
task ml-service:sync
task frontend:install

# 2. Start the full stack (all three services concurrently)
task dev:serve

# 3. Or start individual services in separate terminals
task ml-service:dev     # SAM + gRPC (port 50051)
task api:dev            # Go HTTP API (port 8000)
task frontend:dev       # Vite dev server (port 5173)
```

## Tasks

All commands are available via `task <name>` (see `Taskfile.yml`):

### ML Service

| Task | Description |
|---|---|
| `ml-service:dev` | Start SAM + gRPC compute service |
| `ml-service:dev:model` | Start with a specific SAM model (`MODEL_ID=sam2_hiera_large`) |
| `ml-service:legacy` | Legacy FastAPI server (pre-cutover) |
| `ml-service:sync` | Install dependencies with `uv sync` |

### Go API

| Task | Description |
|---|---|
| `api:dev` | Run the Go API (Gin + SQLite + gRPC) |
| `api:build` | Build binary to `api/bin/` |
| `api:test` | Run Go tests |

### Proto (gRPC contract)

| Task | Description |
|---|---|
| `proto:gen` | Regenerate Go + Python gRPC stubs |
| `proto:gen:go` | Go stubs via `buf` into `api/internal/compute/pb` |
| `proto:gen:py` | Python stubs via `grpc_tools` into `ml-service/src/grpc_server/pb` |

### Frontend

| Task | Description |
|---|---|
| `frontend:dev` | Vite dev server (hot reload) |
| `frontend:build` | Production build |
| `frontend:preview` | Preview production build |
| `frontend:test` | Vitest parity tests |
| `frontend:install` | `npm install` |

### Code Quality

| Task | Description |
|---|---|
| `lint:all` (or `lint`) | Lint all modules |
| `format:all` (or `fmt`) | Format all modules |
| `lint:ml-service` | ruff |
| `lint:api` | golangci-lint |
| `lint:frontend` | Biome |

### Docker

| Task | Description |
|---|---|
| `docker:build:api` | Build Go API image |
| `docker:build:ml-service` | Build ML service image |
| `docker:up` | Start all services via docker-compose |
| `docker:up:detached` | Start in background |
| `docker:down` | Stop services |
| `docker:logs` | Follow logs |

### Full Stack

| Task | Description |
|---|---|
| `dev:serve` | Run all three services concurrently |
| `dev:local` (or `dev`) | Install deps + start full stack |
| `dev:docker` | Start via docker-compose |

### Utility

| Task | Description |
|---|---|
| `clean:build` | Build artifacts and cache |
| `clean:deep` | Full clean including `.venv` |

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `API_PORT` | `8000` | Go API HTTP port |
| `SERVER_PORT` | `8765` | ML service model-server port |
| `GRPC_PORT` | `50051` | gRPC compute port |
| `FRONTEND_PORT` | `80` | Frontend (Docker) port |

## Project Structure

```
auto-annotator/
├── api/                 # Go orchestration API
│   ├── cmd/api/         # Entrypoint
│   ├── internal/        # Handlers, DB, gRPC client
│   └── sqlc.yaml        # SQL code-gen config
├── ml-service/          # Python ML service
│   ├── src/             # Application code
│   ├── grpc_main.py     # gRPC entrypoint
│   ├── models/          # SAM checkpoints
│   └── data/            # Images and labels
├── frontend/            # Vite + React UI
├── proto/               # Shared gRPC contract
│   └── autoannotator/v1/compute.proto
├── Taskfile.yml         # All tasks (`task <name>`)
├── Dockerfile.api
├── Dockerfile.ml-service
├── Dockerfile.frontend
└── docker-compose.yml
```

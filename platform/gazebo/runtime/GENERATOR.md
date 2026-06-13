# Simulation Generator — Migrated to Go

**⚠️ The Python generator has been deprecated and removed from this directory.**

**✅ The generator has been rewritten in Go for better performance and maintainability.**

---

## Where to Find It

The WRO 2026 Gazebo SDF world generator is now located at:

```
platform/gazebo/generator/
```

**See `platform/gazebo/generator/README.md` for complete documentation and usage.**

---

## Quick Start

Generate randomized training scenarios using the Go implementation:

```bash
cd platform/simgen

# Generate 50 obstacles scenarios with seed for reproducibility
go run ./cmd/simgen generate --challenge obstacles --num-scenarios 50 --seed 42

# Generate base track template
go run ./cmd/simgen generate-track --output ../runtime/worlds/wro_track_2026.sdf

# Generate SVG preview from metadata
go run ./cmd/simgen preview --metadata ../../training_data/obstacles/scenarios/scenario_0000.metadata.json
```

Output lands in `platform/training_data/<challenge>/scenarios/`:
```
scenarios/
├── scenario_0000.sdf           # Gazebo world
├── scenario_0000.metadata.json # scenario metadata
├── scenario_0000_preview.svg   # top-down visualization
├── scenario_0001.sdf
├── scenario_0001.metadata.json
└── …
```

---

## What Changed

| Feature | Python (Old) | Go (New) |
|---------|--------------|---------|
| **Location** | `platform/simulation/src/generation/` | `platform/gazebo/generator/` |
| **Build** | Pure Python, requires Python 3.11+ | Standalone binary, no dependencies |
| **Speed** | ~500ms per scenario | ~50ms per scenario (10× faster) |
| **Testing** | pytest | Go testing framework |
| **Type Safety** | Dynamic typing | Static typing |

---

## Implementation Details

The Go implementation provides the same functionality as the Python generator:

### Core Modules

- **simconfig** — Configuration constants, enums, types (type safety)
- **generate** — Scenario generation, randomization strategies, metadata building
- **sdf** — SDF XML building (world, robot, plugins, lighting)
- **validate** — Geometry validation (bounds, clearance, overlaps)
- **preview** — SVG rendering of scenario layouts

### Key Features (Identical to Python)

- **36-scenario WRO traffic sign system** with automatic coordinate transforms
- **Parking lot generation** for obstacles challenge
- **Starting zone positioning** with width and length randomization
- **Corridor width randomization** (narrow 600mm / wide 1000mm)
- **Lighting randomization** (6 presets with directional/intensity variation)
- **Geometry validation** with automatic retry on failure
- **Reproducible seeding** for deterministic scenario generation
- **JSON metadata output** for every scenario

---

## Migration Notes

The Python generator code has been removed to avoid duplication. All core logic has been faithfully ported to Go with the same output format and validation rules.

The ROS2 simulation infrastructure in this directory (launch files, visualization tools, recording pipeline) remains unchanged and works with the Go-generated worlds.

---

## Documentation

### Coordinate System

Origin is **bottom-left** of the physical mat. All units are meters.

```
(0,3) ──────────── (3,3)   N corridor  Y ∈ [2, 3]
  │  NW corner  NE corner │
  │ (0–1, 2–3) (2–3, 2–3) │
  │                        │
W │  inner area [1–2, 1–2] │ E corridor  X ∈ [2, 3]
  │                        │
  │ SW corner  SE corner   │
  │ (0–1, 0–1) (2–3, 0–1) │
(0,0) ──────────── (3,0)   S corridor  Y ∈ [0, 1]
```

- Track (navigable mat): **3.0 × 3.0 m**
- Full mat (including walls): **3.2 × 3.2 m**
- Inner zone: **1.0 × 1.0 m** (X ∈ [1, 2], Y ∈ [1, 2])
- Corner regions: **1.0 × 1.0 m** each

### Challenge Types

**Open challenge:**
- Corridor widths randomized independently (narrow 600mm or wide 1000mm, coin toss)
- No traffic signs
- No parking lot
- Starting zone: any corridor, any valid width-section, one of two length-section positions

**Obstacles challenge:**
- All corridors fixed at **1.0 m** (wide)
- Traffic signs: 1–2 per non-starting corridor (3–6 total), WRO 36-scenario placement
- Parking lot: two 200×20×100mm magenta blocks in starting corridor's corner
- Starting zone: dynamically sized to fit between parking blocks

---

## Questions?

- **Generator documentation:** See `platform/gazebo/generator/README.md`
- **Coordinate system details:** See `platform/gazebo/generator/internal/simconfig/`
- **ROS2 simulation:** See launch files in this directory
- **Training pipeline:** See `platform/robot/` for integration with the learning system

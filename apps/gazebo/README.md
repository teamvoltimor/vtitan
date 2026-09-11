# Gazebo Simulation Platform

This directory contains the Gazebo simulation runtime for WRO 2026.

**The scenario/world generator (`simgen`) has moved.** It lived here as
`gazebo/generator/` until the headless sim path moved to
`src/go`'s own Go-native simulator, at which point it was folded
into that module (it never depended on Gazebo at runtime -- it only emitted
`.sdf` world files as one of several outputs alongside JSON scenario
metadata and an SVG preview). It now lives at `src/go/cmd/simgen`
+ `src/go/internal/simgen/*`, built via `task simgen:build`
(same as before, see `tasks/platform.yml`).

## Directory Structure

```
gazebo/
└── runtime/       ROS2 simulation runtime & data collection (Python)
```

## Components

### simgen (moved -- see src/go)

**Purpose:** Generate randomized Gazebo SDF world files (plus JSON scenario
metadata and an SVG preview) for training data collection.

- **Technology:** Go
- **Speed:** ~50ms per scenario
- **Output:** SDF world files + JSON metadata + SVG previews

**Quick start:**
```bash
cd ../../src/go
go run ./cmd/simgen generate --challenge obstacles --num-scenarios 50 --seed 42
```

---

### [runtime](./runtime/)

**Purpose:** Run Gazebo simulations and collect training data.

- **Technology:** Python, ROS2
- **Includes:** Simulation launcher, data recorder, visualization tools
- **Output:** Sensor data in ROS2 bags + extracted frames

**Quick start:**
```bash
cd runtime
ros2 launch wro_simulation wro_simulation.launch.py
```

**Documentation:** See [runtime/GENERATOR.md](./runtime/GENERATOR.md)

---

## Workflow

1. **Generate scenarios** using the generator:
   ```bash
   cd ../../src/go
   go run ./cmd/simgen generate --challenge obstacles --num-scenarios 50 --seed 42
   ```

2. **Run simulation** with generated worlds using runtime:
   ```bash
   cd gazebo/runtime
   ros2 launch wro_simulation wro_simulation.launch.py
   ```

3. **Collect training data** automatically via the recording pipeline

4. **Analyze results** using visualization tools

---

## Key Concepts

### Coordinate System

All simulations use a **bottom-left origin** (0,0) at the south-west corner, with Z pointing up.

```
(0,3) ──────────── (3,3)   North
  │                        │
  │    1.0×1.0 m          │
  │    inner zone         │
  │   [1.0-2.0, 1.0-2.0]  │
West                       East
  │                        │
  │                        │
(0,0) ──────────── (3,0)   South
```

- Track: **3.0 × 3.0 m**
- Full mat: **3.2 × 3.2 m**

### Challenge Types

**Open:** Randomized corridor widths (600mm or 1000mm), no obstacles, no signs

**Obstacles:** Fixed 1000mm corridors, traffic signs (WRO 36-scenario), parking lot

---

## Integration with Training Pipeline

1. **Generator** produces world files and metadata
2. **Runtime** loads worlds and runs simulations
3. **Recording pipeline** captures sensor data
4. **Training system** uses data for model training

See `src/` for the complete training pipeline integration.

---

## Troubleshooting

**Generator issues:**
- Check `src/go/cmd/simgen`
- Verify Go 1.26+

**Simulation issues:**
- Check `gazebo/runtime/GENERATOR.md`
- Verify ROS2 Humble installation
- Check Gazebo 11+ compatibility

---

## Contributing

- **Generator changes:** See `src/go/cmd/simgen` and `src/go/internal/simgen/`
- **Runtime changes:** See `gazebo/runtime/`
- Keep modules decoupled
- Document coordinate system assumptions

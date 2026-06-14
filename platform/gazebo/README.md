# Gazebo Simulation Platform

This directory contains the complete Gazebo simulation infrastructure for WRO 2026.

## Directory Structure

```
gazebo/
├── generator/     World and scenario generation (Go)
└── runtime/       ROS2 simulation runtime & data collection (Python)
```

## Components

### [generator](./generator/)

**Purpose:** Generate randomized Gazebo SDF world files for training data collection.

- **Technology:** Go
- **Speed:** ~50ms per scenario
- **Output:** SDF world files + JSON metadata + SVG previews

**Quick start:**
```bash
cd generator
go run ./cmd/simgen generate --challenge obstacles --num-scenarios 50 --seed 42
```

**Documentation:** See [generator/README.md](./generator/README.md)

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
   cd gazebo/generator
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

See `platform/robot/` for the complete training pipeline integration.

---

## Troubleshooting

**Generator issues:**
- Check `gazebo/generator/README.md`
- Verify Go 1.26+

**Simulation issues:**
- Check `gazebo/runtime/GENERATOR.md`
- Verify ROS2 Humble installation
- Check Gazebo 11+ compatibility

---

## Contributing

- **Generator changes:** See `gazebo/generator/`
- **Runtime changes:** See `gazebo/runtime/`
- Keep modules decoupled
- Document coordinate system assumptions

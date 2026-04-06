# Klevor v2 Simulation - Comprehensive Code Review

**Date**: 2026-03-24
**Branch**: `simulation`
**Reviewer**: Claude Opus 4.6

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Architecture & Modularity Review](#2-architecture--modularity-review)
3. [Sim-to-Real Portability](#3-sim-to-real-portability)
4. [Challenge-Solving Logic](#4-challenge-solving-logic)
5. [WRO 2026 Specification Compliance](#5-wro-2026-specification-compliance)
6. [Hardware Setup & Integration Plan](#6-hardware-setup--integration-plan)
7. [YOLO / Hailo 8 NPU Integration for Obstacles Challenge](#7-yolo--hailo-8-npu-integration-for-obstacles-challenge)
8. [JSON-Driven Logic Modifications](#8-json-driven-logic-modifications)
9. [Issues, Suggestions & Fixes](#9-issues-suggestions--fixes)

---

## 1. Executive Summary

The Klevor v2 simulation platform is a well-structured, cleanly separated codebase for the WRO 2026 Future Engineers competition. It comprises four major subprojects under `platform/`:

| Subproject | Purpose | Package Manager |
|---|---|---|
| `simulation/` | Gazebo SDF world generation & training data pipeline | UV (`pyproject.toml`) |
| `robot/` | ROS2 Kilted waypoint-following navigator | Pixi (`pixi.toml`) + RoboStack |
| `backend/` | Telemetry API (FastAPI) | UV |
| `frontend/` | 3D React/Three.js visualiser | npm |

**Overall Assessment**: The codebase demonstrates strong engineering fundamentals: typed enums, constant classes, pure-function extraction, clean separation of generation vs. navigation, and metadata-driven scenario parameterization. The main gaps are around (a) the missing vision/YOLO pipeline for the obstacles challenge, (b) some duplicated constant definitions between `simulation/` and `robot/`, and (c) the absence of a hardware abstraction layer for the sim-to-real transition. All are addressable with targeted work.

---

## 2. Architecture & Modularity Review

### 2.1 Enums

**Rating: Excellent**

Both `simulation/src/config/enums.py` and `robot/src/config/enums.py` define `Section`, `Direction`, and `ScenarioType` enums with:

- `__str__()` for transparent JSON serialization
- `from_string()` class methods for case-insensitive deserialization
- `capitalized` property on `Section` for display formatting
- `ScenarioType` inherits from `StrEnum` enabling direct string comparison

**Strengths**:
- Eliminates all magic strings from section/direction comparisons
- Hashable — used directly as dictionary keys (`corridor_widths[Section.NORTH]`)
- `from_string()` raises informative `ValueError` with valid options listed

**Issue**: The enum definitions are **duplicated** between `simulation/` and `robot/`. See [Section 9.1](#91-duplicated-constants-and-enums) for a fix.

### 2.2 Constants

**Rating: Very Good**

`simulation/src/config/constants.py` (359 lines) organizes all WRO specifications into purpose-grouped classes:

| Class | Purpose |
|---|---|
| `TrackDimensions` | Track geometry (3.0m, corners at 1.0-2.0) |
| `WallSpecs` | Height, thickness, collision padding |
| `CorridorDimensions` | Narrow (600mm) / Wide (1000mm) widths |
| `TrafficSignSpecs` | Dimensions, official RGB colors, grid positions |
| `ParkingLotSpecs` | Block dimensions, magenta color, spacing |
| `StartingZoneSpecs` | Zone dimensions, direction indicator colors |
| `RobotSpecs` | All physical robot params + sensor specs |
| `DictKeys` | All dictionary key strings as constants |
| `WidthTypes`, `ColorNames`, `ModelNames`, `FileExtensions`, `FolderNames` | Domain vocabularies |
| `LightingSpecs`, `RandomizationRanges` | Domain randomization parameters |

**Strengths**:
- Zero magic numbers in source files — everything traces back to a named constant
- `DictKeys` class eliminates typo-prone string literals in dictionary access
- Clear WRO specification citations in comments
- `WallSpecs.COLLISION_THICKNESS = 0.18` has an excellent comment explaining the 40mm LIDAR buffer

**Minor Issues**:
- `robot/src/config/constants.py` is a subset copy (63 lines) of `simulation/`'s version — see [Section 9.1](#91-duplicated-constants-and-enums)
- `GridSections.LENGTH_SECTION_LEFT/RIGHT` (1.25, 1.75) are used for starting zone placement but the naming could be clearer (they represent corridor depth centers, not left/right)

### 2.3 Separation of Concerns

**Rating: Excellent**

The generation pipeline follows a clean layered architecture:

```
main.py (CLI + argparse)
  -> ScenarioGenerator (orchestrator)
       -> ScenarioRandomizer (stochastic logic)
       -> SDFBuilder (pure XML construction)
       -> scenarios.py (36-scenario template data)
       -> track_generator.py (base track SDF)
```

Key design principles observed:
- **SDFBuilder is pure**: never reads files or randomizes — only constructs XML
- **ScenarioRandomizer** owns all `random.*` calls
- **scenarios.py** contains only static data (the 36-scenario dictionary) and a pure transform function
- **track_generator.py** is self-contained with private module-level constants
- **generator.py** orchestrates the pipeline with clear private helper methods

The navigation code follows the same pattern:
```
main.py (CLI entrypoint)
  -> TrackNavigator (ROS2 node + control loop)
       -> collision.py (pure LIDAR analysis functions)
       -> waypoints.py (pure waypoint computation)
```

`collision.py` is explicitly documented as "Pure functions — no ROS2 dependency, no global state", which is critical for testability and portability.

### 2.4 Models & Data Flow

**Rating: Good**

The codebase uses dictionaries with `DictKeys` constants for data transfer between components. The metadata JSON format is well-defined:

```json
{
  "scenario_id": 0,
  "challenge_type": "open",
  "corridor_widths": { "north": { "type": "wide", "width_mm": 1000 }, ... },
  "starting_conditions": { "direction": "clockwise", "section": "South", "position": { "x": 1.75, "y": 0.2 }, "yaw": 3.14 },
  "num_signs": 0,
  "sign_positions": [],
  "parking_lot": null
}
```

**Suggestion**: Consider using `dataclass` or `TypedDict` for the major data structures (corridor config, starting conditions, parking config) instead of `dict[str, Any]`. The `DistancesDict` TypedDict in `collision.py` already demonstrates this pattern — extending it to the generation side would catch type errors at development time. See [Section 9.3](#93-typed-data-structures).

### 2.5 Code Quality

| Metric | Assessment |
|---|---|
| Linting | `ruff` configured with `E, F, W, I, UP, B, ANN` rules, line-length=99 |
| Type annotations | Present on all function signatures |
| Docstrings | Thorough on all public methods with Args/Returns sections |
| `defusedxml` | Used for parsing XML (prevents XXE attacks) |
| `from __future__ import annotations` | Consistently applied for PEP 604 union syntax |
| Private naming | `_` prefix consistently used for module-level helpers |
| `strict=True` on `zip()` | Used in `add_traffic_signs()` — catches length mismatches |

---

## 3. Sim-to-Real Portability

### 3.1 Current State

The current architecture has a **clean separation** between simulation-only code (`simulation/`) and robot code (`robot/`), but the transition to real hardware requires several changes.

**What works today for the real robot**:
- `robot/src/navigation/navigator.py` — Full waypoint-following controller. Uses only standard ROS2 messages (`Twist`, `Odometry`, `LaserScan`) and pure math. **No simulation-specific code**.
- `robot/src/navigation/collision.py` — Pure numpy functions for LIDAR analysis. **Zero ROS2 dependency**.
- `robot/src/navigation/waypoints.py` — Pure geometry. Reads metadata JSON, outputs waypoint list. **Fully portable**.
- `robot/src/navigation/driver.py` — Simple driver node. **Fully portable**.

**What needs changing for real hardware**:

| Component | Simulation | Real Hardware | Change Required |
|---|---|---|---|
| **LIDAR** | Gazebo `gpu_lidar` sensor → `/lidar` topic | Slamtec C1 → rplidar ROS2 node → `/scan` topic | Topic name remap |
| **Camera** | Gazebo camera → `/robot/camera` | RPi Camera Module 3 Wide → `v4l2_camera` or `camera_ros` node | Topic remap + image format |
| **IMU** | Gazebo IMU → `/imu` | BNO085 via MCP2221A I2C USB → custom ROS2 node | New driver node needed |
| **Motor control** | `AckermannSteering` Gazebo plugin receives `Twist` on `/wro_robot/cmd_vel` | Build HAT + LEGO motors need a custom driver that converts `Twist` → motor commands | New driver node needed |
| **Odometry** | Gazebo publishes perfect odometry | Must be computed from motor encoders + IMU fusion | New node or EKF |
| **LIDAR min range** | `LIDAR_SIM_MIN_RANGE = 0.01` (sim), clamped to `LIDAR_MIN_RANGE = 0.05` in callback | Real sensor floor at 50mm | Already handled by `clamp_lidar_scan()` |

### 3.2 Recommended Changes for Portability

**a) Topic name configuration**: Currently hardcoded as string literals in `TrackNavigator.__init__()`:
```python
# navigator.py:188-203
self._vel_publisher = self.create_publisher(Twist, "/wro_robot/cmd_vel", 10)
self.create_subscription(Odometry, "/wro_robot/odom", ...)
self.create_subscription(LaserScan, "/lidar", ...)
```

These should be ROS2 parameters or loaded from the JSON params file, so the same code works with both Gazebo topics and real hardware topics without code changes.

**b) Motor driver abstraction**: The `Twist` → motor conversion needs a node that:
1. Receives `Twist` on `/wro_robot/cmd_vel`
2. Computes Ackermann kinematics (using `RobotSpecs.WHEELBASE`, `TRACK_WIDTH`)
3. Sends PWM/speed commands to the Build HAT for the two LEGO Large motors

**c) IMU driver**: BNO085 → MCP2221A → USB → RPi 5 → ROS2 `Imu` message publisher. The navigator currently doesn't subscribe to IMU (only LIDAR + odometry), but IMU would improve heading estimation significantly.

### 3.3 Sim-to-Real Transition Effort

The good news: the core navigation logic (`collision.py`, `waypoints.py`, speed/steering computation) is **pure math** and needs **zero changes**. The effort is entirely in:
1. Writing 2-3 ROS2 driver nodes (motors, IMU, possibly camera)
2. Topic name remapping
3. Adding a camera-based sign detection pipeline (YOLO) — see [Section 7](#7-yolo--hailo-8-npu-integration-for-obstacles-challenge)

---

## 4. Challenge-Solving Logic

### 4.1 Open Challenge

**Strategy**: Pure waypoint following with LIDAR-based collision avoidance.

**Waypoint Generation** (`waypoints.py`):
1. Reads corridor widths from metadata to compute corridor centers
2. Applies `_OUTER_WALL_BIAS = 0.05m` to bias the path toward the outer wall
3. Generates straight waypoints along corridor centers (8 points per corridor)
4. Generates circular arc waypoints at corners (`_ARC_RADIUS = 0.45m` > minimum turning radius 0.294m)
5. Builds multi-lap sequence by rotating the corridor order to start from the spawn section

**Navigation** (`navigator.py`):
- Pure-pursuit steering with adaptive lookahead (0.30m near corners, 0.70m on straights)
- Speed scaling by forward clearance (5 zones from 0.10m to 1.0m+) and heading error
- Side-correction pushes the robot away from close walls (`_SIDE_GAIN = 0.08`)
- Wall K-turn escape when LIDAR forward reading drops below `critical_distance = 0.07m`
- Stuck detection (if robot moves < 0.03m in 1 second, triggers escape)
- Waypoint skip logic when the robot is stuck in a loop (after 3 repeated escapes near the same spot)

**Assessment**: The open challenge logic is **solid and well-tuned**. The combination of pure-pursuit + LIDAR avoidance + escape maneuvers handles the variable corridor widths (600mm/1000mm) effectively. The debounced forward-critical counter (2 consecutive readings required) prevents false triggers from GPU LIDAR artifacts.

**Potential Improvement**: The `_pick_start_position()` function uses offsets `[-0.5, 0.0, 0.5]` from the corridor center, but for a 600mm corridor this could place the robot too close to the wall. The starting position should be constrained by the corridor width more tightly.

### 4.2 Obstacles Challenge

**Scenario Generation** (`scenarios.py`, `randomizer.py`):
- Implements the full 36-scenario WRO traffic sign system
- Each non-starting corridor gets a random scenario (1-36)
- Signs are placed at exact grid intersections (2x3 grid per corridor)
- Parking blocks are placed in the starting section's corner
- Signs near parking blocks are automatically adjusted (moved from 0.4m to 0.6m)

**Navigation for Obstacles**:
- `assess_collision_risk()` in `collision.py` differentiates between "wall" (boxed in, near side < 0.28m) and "obstacle" (free-standing sign, room on both sides)
- Obstacle escape is a two-phase mini K-turn: 70% reverse, 30% forward with counter-steer
- The `is_open_challenge` flag disables obstacle classification when signs aren't present

**Critical Gap**: **The obstacles challenge currently uses only LIDAR** to detect traffic signs. It can detect *that* something is in front of the robot, but **cannot determine the color** (red vs green). In WRO rules:
- Green sign → pass from the RIGHT side
- Red sign → pass from the LEFT side

Without a camera-based color detection pipeline (YOLO), the robot will escape obstacles but won't follow the correct passing rules. See [Section 7](#7-yolo--hailo-8-npu-integration-for-obstacles-challenge) for the integration plan.

### 4.3 Parking (Obstacles Challenge)

The parking lot generation is implemented (`generate_parking_lot_positions`) with:
- Dynamic block depth from the 3-position grid (1.0, 1.5, 2.0)
- Block spacing at 1.5x robot width (225mm) from each other
- Wall-contact placement (center at 0.1m from wall edge)
- Starting zone dynamically sized and positioned between blocks

**Gap**: There is no parking execution logic in the navigator. The waypoint generator does not create a parking approach sequence. This will need:
1. Detection of the parking zone (magenta blocks via camera)
2. A dedicated parking maneuver state machine
3. Precise positioning using LIDAR distance to the magenta blocks

---

## 5. WRO 2026 Specification Compliance

### 5.1 Correct Representations

| Specification | Implementation | Status |
|---|---|---|
| Track: 3000x3000mm white floor | `TrackDimensions.TRACK_SIZE = 3.0`, ground plane with white material | Correct |
| Mat: 3200x3200mm | `TrackDimensions.MAT_SIZE = 3.2` | Correct |
| Walls: BLACK, 100mm height | `WallSpecs.HEIGHT = 0.1`, color `(0,0,0)` | Correct |
| Wall thickness: 100mm | `WallSpecs.THICKNESS = 0.1` | Correct |
| Interior walls extend inward | `_INTERIOR_OFFSET = 0.05` (half thickness subtracted from corridor side) | Correct |
| Exterior walls extend outward | `_EXTERIOR_OFFSET = 0.05` (half thickness added outside track) | Correct |
| Corner lines: orange/blue at 30/60 degrees | 8 corner lines in `track_generator.py` | Correct |
| Traffic signs: 50x50x100mm boxes | `TrafficSignSpecs.WIDTH/DEPTH = 0.05, HEIGHT = 0.10` | Correct |
| Red sign: RGB(238,39,55) | `RED_COLOR = (0.933, 0.153, 0.216)` | Correct |
| Green sign: RGB(68,214,44) | `GREEN_COLOR = (0.267, 0.839, 0.173)` | Correct |
| Parking blocks: 200x20x100mm magenta | `ParkingLotSpecs: LENGTH=0.20, WIDTH=0.02, HEIGHT=0.10, COLOR=(1,0,1)` | Correct |
| Starting zone: 200x500mm grey | `StartingZoneSpecs: WIDTH=0.2, DEFAULT_LENGTH=0.5` | Correct |
| Open: variable 600mm/1000mm corridors | `CorridorDimensions.NARROW = 0.6, WIDE = 1.0` | Correct |
| Obstacles: fixed 1000mm corridors | `OBSTACLES_WIDTH = 1.0` | Correct |
| 36 predefined scenarios | Full `SCENARIOS` dict in `scenarios.py` | Correct |
| 3-6 signs per round | 3 corridors x 1-2 signs per scenario | Correct |
| Direction indicator: colored circle | Cylinder geometry with blue (CW) or green (CCW) | Correct |

### 5.2 Potential Concerns

1. **Scenarios 14 and 16 are identical**: Both are `[("green", 1.0, 0.4), ("red", 2.0, 0.6)]`. Similarly, scenarios 15 and 17, 20 and 22, 21 and 23, 26 and 28, 27 and 29, 32 and 34, 33 and 35. This may be intentional in the WRO rules (allowing the same physical configuration to appear more frequently), but should be verified against the official 2026 scenario definitions. If these are truly duplicates, the effective unique scenarios would be fewer than 36.

2. **Coordinate transform for North corridor**: In `apply_scenario_to_section()`:
   ```python
   elif section is Section.NORTH:
       world_x = x_south
       world_y = track_max - y_south
   ```
   The x-coordinate is NOT mirrored for North, only the y. This means the sign depth positions (1.0, 1.5, 2.0) map identically in both South and North. Verify this is correct per WRO rules — in some interpretations, North corridor signs should be mirrored along the X-axis as well (so depth 1.0 maps to 2.0 from the other entry direction).

3. **LIDAR wall pass-through**: The 4.0m heuristic in `collision.py:141` (`if forward_dist > 4.0: return "critical"`) is a clever workaround for GPU LIDAR rays passing through thin wall meshes, but it's simulation-specific. On real hardware this check should be disabled or adapted.

---

## 6. Hardware Setup & Integration Plan

### 6.1 Complete Hardware Inventory

| Component | Role | Interface | Notes |
|---|---|---|---|
| **Raspberry Pi 5 (16GB)** | Main compute | — | Runs ROS2 Kilted, navigation, YOLO inference orchestration |
| **Raspberry Pi Zero 2W** | Coprocessor | USB Gadget mode → RPi 5 | Handles low-latency motor control via Build HAT |
| **Hailo 8 NPU** | AI accelerator | M.2 via RPi 5 AI HAT+ | YOLO inference for traffic sign detection (26 TOPS) |
| **RPi Camera Module 3 Wide** | Vision | CSI → RPi 5 | 102 degree HFOV, 1536x864, 30fps |
| **Slamtec RPLiDAR C1** | LIDAR | USB → RPi 5 | 360 degree, 0.05-12m, 500 samples |
| **Adafruit BNO085** | IMU (9-DOF) | I2C → MCP2221A → USB → RPi 5 | Gyro, accel, magnetometer, sensor fusion |
| **MCP2221A** | I2C-USB bridge | USB | Bridges BNO085 to RPi 5's USB |
| **RPi Build HAT** | Motor driver | SPI (on RPi Zero 2W) | Controls LEGO Powered Up motors |
| **2x LEGO Large Power Up motors** | Drive + steering | Build HAT ports | One for drive, one for Ackermann steering |
| **LEGO Bugatti Bolide chassis** | Mechanical platform | — | Ackermann steering geometry |

### 6.2 System Architecture

```
                    ┌──────────────────────────────────────┐
                    │          Raspberry Pi 5 (16GB)       │
                    │                                      │
 Camera Module 3 ──>│  ┌─────────┐  ┌──────────────────┐  │
 Wide (CSI)         │  │ Camera   │  │  ROS2 Navigator   │  │
                    │  │ ROS Node │  │  (TrackNavigator) │  │
                    │  └────┬─────┘  └───────┬──────────┘  │
                    │       │ /robot/camera   │ /cmd_vel    │
                    │       v                 v             │
 Hailo 8 NPU ─────>│  ┌─────────┐  ┌──────────────────┐  │
 (M.2 HAT+)        │  │ YOLO    │  │  Telemetry        │  │
                    │  │ Detect  │  │  Publisher         │  │
                    │  └────┬────┘  └──────────────────┘  │
                    │       │ /signs                       │
                    │                                      │
 RPLiDAR C1 ──────>│  ┌─────────┐                        │
 (USB)              │  │ rplidar │ /scan                   │
                    │  └─────────┘                        │
                    │                                      │
 BNO085+MCP2221A──>│  ┌─────────┐                        │
 (USB)              │  │ IMU     │ /imu                    │
                    │  └─────────┘                        │
                    │           │ USB Gadget (Ethernet)    │
                    └───────────┼──────────────────────────┘
                                │
                    ┌───────────v──────────────────────────┐
                    │      Raspberry Pi Zero 2W            │
                    │                                      │
                    │  ┌──────────────────┐                │
                    │  │ Motor Control    │                │
                    │  │ (Build HAT)      │                │
                    │  │ Twist → PWM      │                │
                    │  └────────┬─────────┘                │
                    │           │ SPI                       │
                    │  ┌────────v─────────┐                │
                    │  │ Build HAT        │                │
                    │  │ Port A: Drive    │                │
                    │  │ Port B: Steering │                │
                    │  └──────────────────┘                │
                    └──────────────────────────────────────┘
```

### 6.3 Communication Flow

1. **RPi 5 ↔ RPi Zero 2W**: USB Gadget mode creates a virtual Ethernet link. The Zero 2W runs a lightweight ROS2 node (or a Zenoh bridge) that receives `Twist` messages and converts them to Build HAT motor commands.

2. **Sensor data**: All sensors (LIDAR, camera, IMU) connect directly to RPi 5 and publish standard ROS2 messages.

3. **YOLO inference**: Camera frames → Hailo 8 NPU via HailoRT → detection results published as a custom ROS2 message (sign color + bounding box + position estimate).

### 6.4 Nodes Needed for Real Hardware

| Node | Input | Output | Status |
|---|---|---|---|
| `rplidar_ros` | USB serial | `sensor_msgs/LaserScan` on `/scan` | Existing ROS2 package |
| `v4l2_camera` | CSI camera | `sensor_msgs/Image` on `/robot/camera` | Existing ROS2 package |
| `bno085_imu_node` | I2C via MCP2221A USB | `sensor_msgs/Imu` on `/imu` | Needs writing |
| `build_hat_driver` | `geometry_msgs/Twist` on `/cmd_vel` | Build HAT SPI commands | Needs writing (on Zero 2W) |
| `yolo_detector` | `sensor_msgs/Image` | Custom `SignDetection` msg on `/signs` | Needs writing (uses Hailo 8) |

---

## 7. YOLO / Hailo 8 NPU Integration for Obstacles Challenge

### 7.1 Current State

The obstacles challenge **has no vision pipeline**. The robot detects obstacles via LIDAR only and performs generic escape maneuvers without knowing sign colors. This means:
- It cannot follow the red/green passing rules
- It cannot detect or approach the magenta parking blocks
- It cannot identify the starting zone direction indicator color

### 7.2 Proposed Architecture: Mode Selection

Add a `--mode` flag to the navigator that selects between detection backends:

```
python main.py navigate --metadata scenario.json --mode lidar-only
python main.py navigate --metadata scenario.json --mode yolo-hailo
python main.py navigate --metadata scenario.json --mode yolo-local
```

| Mode | Use Case | Detection |
|---|---|---|
| `lidar-only` | Open challenge, sim without camera | LIDAR-only avoidance (current behavior) |
| `yolo-hailo` | Real robot with Hailo 8 NPU | Hailo 8 .hef model via HailoRT |
| `yolo-local` | Simulation / development | Local .pt model via Ultralytics |

### 7.3 Implementation Plan

**a) Detection interface** — Create an abstract detector in `robot/src/vision/detector.py`:

```python
class SignDetection:
    color: str         # "red" or "green"
    bbox: tuple        # (x1, y1, x2, y2) in image pixels
    confidence: float
    position_estimate: tuple[float, float] | None  # (x, y) world coords if available

class DetectorBase(ABC):
    @abstractmethod
    def detect(self, image: np.ndarray) -> list[SignDetection]: ...

class HailoDetector(DetectorBase):
    """Uses .hef model on Hailo 8 NPU via HailoRT."""

class LocalDetector(DetectorBase):
    """Uses .pt model via Ultralytics for simulation/dev."""

class NoDetector(DetectorBase):
    """Returns empty list — LIDAR-only mode."""
```

**b) Model preparation**: The existing `scripts/generate_synthetic_dataset.py` generates basic training data. However:
- Current synthetic data uses pure red/green rectangles on noise backgrounds — too different from Gazebo renders
- Needs Gazebo camera frames + auto-labeling from known sign positions (available in metadata JSON)
- For Hailo 8: train YOLO model (.pt) → export to ONNX → compile to .hef using Hailo Dataflow Compiler

**c) Navigator integration**: The `TrackNavigator` should:
1. Subscribe to `/signs` topic (from the YOLO detector node)
2. When an obstacle is detected ahead (LIDAR), check the latest sign detection for color
3. Route the escape maneuver: green → pass right, red → pass left
4. Override `_enter_obstacle_escape()` to use sign color instead of side clearance

**d) Training data pipeline** from simulation:
1. Generate obstacles scenarios (`uv run python main.py generate --challenge obstacles`)
2. Run Gazebo with the scenario + record camera topic
3. Auto-label frames using the metadata JSON (sign world positions + camera intrinsics → project to image bounding boxes)
4. Train YOLO model on the labeled data

### 7.4 Hailo 8 Deployment

The Hailo 8 (NOT 8l) provides 26 TOPS which is ample for real-time YOLO inference:
- YOLO11n: ~6ms inference at 640x640 on Hailo 8
- Camera at 30fps = 33ms per frame budget — plenty of headroom
- Use HailoRT Python API or the `hailo-apps-infra` pipeline

The `.pt` → `.hef` pipeline:
```bash
# 1. Train locally
yolo detect train data=traffic_signs.yaml model=yolo11n.pt epochs=100

# 2. Export to ONNX
yolo export model=best.pt format=onnx

# 3. Compile for Hailo 8 (on x86 machine with Hailo DFC)
hailo optimize best.onnx --hw-arch hailo8
hailo compile best.har --hw-arch hailo8
```

---

## 8. JSON-Driven Logic Modifications

### 8.1 Current JSON Configurability

The codebase already has good JSON-based configurability:

**a) Scenario metadata** (`scenario_XXXX_metadata.json`):
- Corridor widths per section
- Starting direction, section, position, yaw
- Traffic sign positions and colors
- Parking lot block positions
- Challenge type (open/obstacles)

The navigator reads all of this at startup and adapts its waypoints, speed, and behavior accordingly.

**b) Navigator parameters** (`navigator_params.json`):
```json
{
  "critical_distance": 0.07,
  "fwd_critical_lidar_threshold": 2,
  "critical_repeat_limit": 3,
  "critical_repeat_radius": 0.25,
  "waypoint_threshold": 0.20,
  "escape_duration_base": 10,
  "max_linear_speed": 0.50
}
```

These override runtime behavior without code changes.

### 8.2 Surprise Rule Adaptability

For WRO "surprise rules" (rule changes announced at competition), the JSON-based approach enables:

| Possible Surprise Rule | JSON Change Required | Code Change |
|---|---|---|
| Different corridor widths | Already handled — metadata-driven | None |
| More/fewer laps | `--laps` CLI argument | None |
| Different speed limits | `max_linear_speed` in params JSON | None |
| Wider/narrower escape thresholds | `critical_distance` in params JSON | None |
| New traffic sign colors (e.g., blue) | Add to metadata `sign_positions[].color` | Minimal — add color to YOLO model classes |
| Reversed passing rules (red=right, green=left) | Add a `passing_rules` dict to params JSON | Small — lookup table change |
| U-turn at half track | Add `u_turn_waypoints` to metadata JSON | Moderate — new waypoint insertion logic |
| Obstacle in corner zones | Already handled by LIDAR avoidance | None |

### 8.3 Suggestions for Better Surprise Rule Support

**a) Waypoint override file**: Allow a `custom_waypoints.json` that can inject/replace waypoints at specific corridor sections. This would handle surprise routing changes.

**b) Behavior profile JSON**: A top-level behavior config that selects between strategies:
```json
{
  "strategy": "standard",
  "passing_rules": { "red": "left", "green": "right" },
  "parking_enabled": true,
  "u_turn_section": null,
  "max_speed_multiplier": 1.0,
  "extra_waypoints": []
}
```

**c) Speed zone map**: Currently speed is uniform. A JSON could define per-corridor speed limits:
```json
{
  "speed_zones": {
    "north": 0.5,
    "south_entering": 0.3,
    "corners": 0.25
  }
}
```

---

## 9. Issues, Suggestions & Fixes

### 9.1 Duplicated Constants and Enums

**Problem**: `robot/src/config/constants.py` (63 lines) and `robot/src/config/enums.py` (69 lines) are subsets of `simulation/src/config/constants.py` (359 lines) and `simulation/src/config/enums.py` (113 lines). Any change to a constant (e.g., robot dimensions) must be made in two places.

**Fix**: Extract a shared `platform/shared/` package with the common constants and enums. Both `simulation/` and `robot/` depend on it:

```
platform/
  shared/
    src/
      config/
        constants.py  # The single source of truth
        enums.py
    pyproject.toml
  simulation/
    pyproject.toml  # depends on shared
  robot/
    pixi.toml       # depends on shared
```

Or at minimum, have `robot/` import from `simulation/` or vice versa. The current duplication is a maintenance hazard.

### 9.2 `import random` in SDFBuilder

**Problem**: `sdf_builder.py:8` imports `random`, and `_compute_zone_placement()` at line 429-431 calls `random.choice()`. The SDFBuilder is documented as "a pure builder — it never reads files or randomizes values", but the starting zone placement contradicts this.

**Fix**: Move the `random.choice()` calls for starting zone width/length offset into `ScenarioRandomizer`, and pass the chosen offsets into `add_starting_zone()`. This restores SDFBuilder's pure-builder contract.

### 9.3 Typed Data Structures

**Problem**: Most inter-function data is `dict[str, Any]`, which loses type information:

```python
def _resolve_starting_conditions(self, ...) -> dict[str, Any]:
    return {
        DictKeys.DIRECTION: Direction.CLOCKWISE,
        DictKeys.SECTION: Section.SOUTH,
        DictKeys.POSITION: (1.5, 0.4),
        DictKeys.YAW: math.pi,
    }
```

**Fix**: Define `TypedDict` or `dataclass` types:

```python
class StartingConditions(TypedDict):
    direction: Direction
    section: Section
    section_name: str
    position: tuple[float, float]
    yaw: float
```

This catches key typos and type mismatches at development time (especially with `pyright` or `mypy`).

### 9.4 Waypoint String Keys vs Enum Keys

**Problem**: In `waypoints.py`, corridor sections are string keys:

```python
widths = {
    side: corridor_widths[side][DictKeys.WIDTH_MM] / 1000.0
    for side in ("north", "south", "east", "west")
}
```

And later: `order = ["east", "south", "west", "north"]`

This bypasses the `Section` enum and reintroduces magic strings.

**Fix**: Use `Section` enum keys throughout `waypoints.py`, with `str(section)` only at the JSON boundary.

### 9.5 Risk Level as String Literal

**Problem**: `assess_collision_risk()` returns `str` risk levels ("safe", "critical", "obstacle"), and the navigator compares with bare strings:

```python
if risk_level == "critical":
    return self._enter_wall_escape(...)
if risk_level == "obstacle":
    return self._enter_obstacle_escape(...)
```

**Fix**: Define a `RiskLevel` enum in `collision.py` (or `enums.py`):
```python
class RiskLevel(StrEnum):
    SAFE = "safe"
    CRITICAL = "critical"
    OBSTACLE = "obstacle"
```

### 9.6 Starting Position Constraints for Narrow Corridors

**Problem**: In `randomizer.py:_pick_start_position()`, the offset `[-0.5, 0.0, 0.5]` is applied uniformly regardless of corridor width. For a 600mm corridor, the center is at y=0.3, so offsets produce positions at y=-0.2, y=0.3, and y=0.8. The first is **outside the track**.

Wait — reviewing more carefully: the offsets are along the **length** axis (x for N/S, y for E/W), not the width axis. The width position (`y_pos` for South) is computed as `corridor_width / 2`. So for a 600mm corridor, `y_pos = 0.3`, which is the width center — this is correct.

The `[-0.5, 0.0, 0.5]` offsets are along the corridor length (x-axis for South), creating positions at x=1.0, x=1.5, x=2.0 — all within the 1.0-2.0 corridor range. **This is actually correct**.

However, the robot WIDTH is 0.15m. In a 600mm corridor, the robot center at y=0.3 means the nearest wall face is at y=0.0 + wall_thickness (exterior) = 0.0, leaving only 0.225m clearance on the inner side. This is tight but functional.

### 9.7 `_corridor_widths` Unused Parameter

**Problem**: In `generate_sign_positions()` (`randomizer.py:143`), the first parameter `_corridor_widths` has a leading underscore and the docstring says "Reserved for future narrow-corridor sign-exclusion logic; not yet consumed." This is fine as a placeholder but should have a `# TODO` comment or issue tracker reference.

### 9.8 Overhead Camera Hardcoded in SDFBuilder

**Problem**: `_build_debug_overhead_camera()` adds a debug overhead camera with hardcoded resolution (1280x720) and FOV (1.57 rad). On real hardware, this camera doesn't exist.

**Impact**: Low — it's only in the generated SDF files, not in the robot code. The navigator doesn't subscribe to it.

### 9.9 `track_generator.py` Uses Local Constants Instead of Shared

**Problem**: `track_generator.py` defines its own local constants (`_MAT_SIZE = 3.2`, `_TRACK_SIZE = 3.0`, `_WALL_HEIGHT = 0.10`, etc.) instead of importing from `config/constants.py`.

**Fix**: Import from the shared constants. If the rationale was to keep `track_generator.py` self-contained, document that explicitly.

### 9.10 Missing `__init__.py` Exports

**Problem**: Most `__init__.py` files are empty. While this works for internal imports, it means there's no defined public API for each package. Anyone importing from `src.generation` has to know the internal module structure.

**Suggestion**: Add selective re-exports in `__init__.py` files for the main public classes:
```python
# src/generation/__init__.py
from src.generation.generator import ScenarioGenerator
from src.generation.track_generator import generate_track_sdf
```

### 9.11 No Seed Control for Reproducibility

**Problem**: The randomizer uses `random.choice()` and `random.randint()` but there's no way to set a random seed. This means scenarios can't be reproduced for debugging.

**Fix**: Add a `--seed` CLI argument that calls `random.seed(seed)` and `np.random.seed(seed)` before generation. Store the seed in the metadata JSON for reproducibility.

### 9.12 `generate_synthetic_dataset.py` Uses `print()` Statements

**Problem**: `scripts/generate_synthetic_dataset.py` uses bare `print()` statements throughout, inconsistent with the rest of the codebase which uses `logging`.

### 9.13 Missing Test Coverage

**Current tests**:
- `robot/tests/test_collision.py` — Tests LIDAR collision detection
- `simulation/tests/test_framework.py`, `test_camera_orientations.py`, `manual_robot_movement.py` — Appear to be integration tests

**Missing tests**:
- `waypoints.py` — No unit tests for waypoint generation (arc computation, corridor ordering, deduplication)
- `scenarios.py` — No tests for coordinate transformation
- `randomizer.py` — No tests for corridor width / starting position / parking lot generation
- `sdf_builder.py` — No tests for XML output structure

The pure-function design of `collision.py`, `waypoints.py`, and `scenarios.py` makes them trivially testable. Adding unit tests here would catch regressions fast.

### 9.14 Navigator `_apply_param_overrides` Accepts Arbitrary Keys

**Problem**: The `param_map` in `_apply_param_overrides()` only handles 6 known keys, but `setattr()` could be dangerous if the JSON file is not trusted. Currently it's safe because only mapped keys are accepted, but adding validation would be more robust.

### 9.15 `rclpy.shutdown()` Called Inside Node Callback

**Problem**: In `navigator.py:257`:
```python
if self._waypoint_index >= len(self._waypoints):
    self.get_logger().info(f"Completed {self._num_laps} lap(s). Stopping.")
    self._publish_stop()
    rclpy.shutdown()
    return
```

Calling `rclpy.shutdown()` from within a timer callback can cause issues in ROS2 (the executor is still running). Better to set a flag and let the main loop handle shutdown.

---

## Summary of Priority Actions

### Critical (Required for Competition)

1. **Implement YOLO pipeline for obstacles challenge** — Camera → Hailo 8 → sign color detection → passing rule execution (Section 7)
2. **Implement parking execution logic** — Detect magenta blocks, execute parking maneuver
3. **Write hardware driver nodes** — Motor control (Build HAT), IMU (BNO085), topic remapping

### High Priority (Reliability)

4. **Add `--seed` for reproducible scenarios** (Section 9.11)
5. **Extract shared constants package** to eliminate duplication (Section 9.1)
6. **Add unit tests** for `waypoints.py`, `scenarios.py`, and `randomizer.py` (Section 9.13)
7. **Fix `rclpy.shutdown()` in callback** (Section 9.15)

### Medium Priority (Code Quality)

8. **Move `random.choice` out of SDFBuilder** to maintain pure-builder contract (Section 9.2)
9. **Introduce `TypedDict`/`dataclass` for data structures** (Section 9.3)
10. **Add `RiskLevel` enum** for collision risk classification (Section 9.5)
11. **Use `Section` enum in `waypoints.py`** instead of string keys (Section 9.4)
12. **Import constants in `track_generator.py`** instead of local duplicates (Section 9.9)

### Low Priority (Nice to Have)

13. **Behavior profile JSON** for surprise rule adaptability (Section 8.3)
14. **Add `__init__.py` re-exports** for cleaner public API (Section 9.10)
15. **Convert `generate_synthetic_dataset.py` to use logging** (Section 9.12)
16. **Verify 36-scenario duplicates** against official WRO rules (Section 5.2)

---

## 10. Deep Navigation Core Review

This section provides an in-depth algorithmic review of the four navigation modules: `navigator.py`, `collision.py`, `waypoints.py`, and `driver.py`.

### 10.1 Control Architecture Overview

The navigator implements a **layered reactive architecture**:

```
Layer 4: Stuck Detection         (lowest priority, long time horizon)
Layer 3: Escape State Machines   (wall K-turn, obstacle mini K-turn)
Layer 2: Collision Risk Assessment
Layer 1: Pure-Pursuit Waypoint Following + Speed Scaling  (highest freq, shortest horizon)
```

Control runs at 20 Hz (`create_timer(0.05, ...)`). LIDAR arrives at 10 Hz, odometry at 50 Hz (from the Ackermann plugin's `odom_publish_frequency`). This means the control loop often runs with stale LIDAR data (2 control ticks per LIDAR update). This is acceptable for the speeds involved (max 0.50 m/s → 25 mm per tick → 50 mm between LIDAR updates).

### 10.2 Odometry Transform — Correctness Analysis

```python
# navigator.py:222-229
cos_sy = math.cos(self._start_yaw)
sin_sy = math.sin(self._start_yaw)
self._current_pos = (
    self._start_x + odom_x * cos_sy - odom_y * sin_sy,
    self._start_y + odom_x * sin_sy + odom_y * cos_sy,
)
self._current_yaw = odom_yaw + self._start_yaw
```

This is a standard 2D rigid-body transform: rotate the odom-frame position by `start_yaw`, then translate by `(start_x, start_y)`. **Mathematically correct**.

**Concern — Odometry drift on real hardware**: In simulation, Gazebo's Ackermann plugin publishes perfect odometry (no drift). On the real robot, wheel encoder odometry will drift, especially during escape maneuvers (wheel slip during reverse + steer). The odometry transform assumes `(0,0)` in odom frame always maps to `(start_x, start_y)` in world frame — this breaks with cumulative drift.

**Recommendation**: For real hardware, fuse odometry with IMU (BNO085) using an Extended Kalman Filter (`robot_localization` ROS2 package). The navigator code itself doesn't need to change — just the quality of the `/wro_robot/odom` topic improves. Alternatively, periodic LIDAR-based localization (simple wall-matching since the track geometry is known) could reset the drift.

### 10.3 LIDAR Processing Pipeline

```
Raw LaserScan → clamp_lidar_scan() → update_fwd_critical_count()
                      ↓                        ↓
              self._lidar_ranges        self._fwd_critical_lidar_count
                      ↓
              assess_collision_risk() ← called in _build_velocity_command()
                      ↓
              (risk_level, distances)
```

**`clamp_lidar_scan()`** (`collision.py:196-215`):
- Replaces `< LIDAR_MIN_RANGE` (0.05m) with exactly `LIDAR_MIN_RANGE`
- Replaces `inf` with `max_range` (12.0m)
- Returns a copy (original not mutated — verified by test)
- **Correct and clean**

**`measure_distance_in_direction()`** (`collision.py:49-84`):
- Uses `arctan2(sin(diff), cos(diff))` to compute angular distance — this correctly handles the wraparound at ±pi. **Robust**.
- `tolerance = 0.25 rad` (±14.3 degrees) — covers a 28.6-degree sector. With 500 LIDAR rays over 360 degrees, that's ~40 rays per sector. **Good signal-to-noise ratio**.
- `filter_self_detection`: Discards readings ≤ 0.08m for side bearings. This is critical because the Gazebo `gpu_lidar` renders all visuals including the robot's own chassis. The 0.08m threshold matches the robot half-width (0.075m) with a small margin. **Correct**.
- Returns `min()` of the sector — appropriate for collision detection (worst-case distance).

**`assess_collision_risk()`** (`collision.py:87-165`):

The debouncing logic is the most interesting part:

```python
debounced = fwd_critical_count >= fwd_critical_threshold  # needs 2 consecutive readings
```

This filters GPU LIDAR artifacts where a single ray passes through a thin wall mesh at an inner-corner junction. **Good engineering — prevents false escape triggers**.

The obstacle vs. wall classification:
```python
near_side = min(left_dist, right_dist)
boxed_in = near_side < _BOXED_IN_NEAR_SIDE  # 0.28m
turning = abs(angle_error) > _TURNING_HEADING_THRESHOLD  # 0.25 rad
can_be_obstacle = not is_open_challenge and not boxed_in and not turning
```

**Logic**: If the robot is in a narrow corridor (`boxed_in`), the thing ahead is a wall/corner, not a freestanding sign. If the robot is actively turning (`turning`), don't classify as obstacle (the robot is navigating a corner and the forward reading is the inner wall). **Sound reasoning**.

**Issue**: The `> 4.0m` heuristic (`collision.py:141`) is hardcoded. On real hardware, the Slamtec C1 might return `inf` or `max_range` for out-of-range readings, but never a false positive of 4-5m where a wall should be. This heuristic should be gated behind a `is_simulation` flag or removed for real hardware.

### 10.4 Pure-Pursuit Waypoint Following — Detailed Analysis

**Lookahead computation** (`navigator.py:639-669`):

```python
lookahead = _LOOKAHEAD_SHORT if fwd_dist < _FWD_SHORT_LOOKAHEAD_DIST else _LOOKAHEAD_LONG
# _LOOKAHEAD_SHORT = 0.30m, _LOOKAHEAD_LONG = 0.70m
# _FWD_SHORT_LOOKAHEAD_DIST = 0.15m

steer_idx = self._waypoint_index
cumulative = 0.0
while steer_idx + 1 < len(self._waypoints) and cumulative < lookahead:
    wx, wy = self._waypoints[steer_idx]
    nx, ny = self._waypoints[steer_idx + 1]
    cumulative += math.sqrt((nx - wx) ** 2 + (ny - wy) ** 2)
    steer_idx += 1
```

This walks forward along the waypoint chain until the accumulated path distance exceeds the lookahead distance. The steering target is the waypoint at that distance. **This is a simplified pure-pursuit** — classic pure-pursuit would interpolate the exact point on the path at the lookahead distance, but using the nearest waypoint is fine given the waypoint density (8 per corridor + 5 per corner arc = ~52 waypoints per lap).

**Issue — Lookahead threshold too aggressive**: `_FWD_SHORT_LOOKAHEAD_DIST = 0.15m` triggers the short lookahead when the forward wall is only 15cm away. At 0.50 m/s, the robot covers 15cm in 0.3 seconds — only 6 control ticks. The switch to short lookahead helps slow the steering response for tight corners, but the threshold could be raised to 0.25-0.30m to give more anticipation time.

**Steering P-controller** (`navigator.py:788-790`):
```python
def _proportional_steer(angle_error: float, max_angle: float) -> float:
    return float(np.clip(_STEER_KP * angle_error, -max_angle, max_angle))
```

`_STEER_KP = 1.5` means the steering saturates at `max_steering_angle` (0.5236 rad = 30 degrees) when the heading error exceeds `0.5236 / 1.5 = 0.349 rad ≈ 20 degrees`. This is a reasonable gain — aggressive enough to track corners but not so high that it oscillates on straights.

**Missing**: There's no derivative (D) term. On real hardware with sensor noise, a pure P-controller will produce oscillatory steering. Consider adding a filtered D-term or switching to a Stanley controller for smoother tracking. In simulation with perfect odometry this is less of an issue.

### 10.5 Speed Scaling — Layer Analysis

Speed is the **minimum** of two independent scaling factors:

**Factor 1 — Forward clearance** (5 zones):
| Forward dist | Speed fraction | At max_speed=0.50 | Effective speed |
|---|---|---|---|
| < 0.10m | 0.15 | 0.075 m/s | Creep |
| < 0.25m | 0.35 | 0.175 m/s | Slow |
| < 0.50m | 0.50 | 0.250 m/s | Medium |
| < 1.00m | 0.70 | 0.350 m/s | Fast |
| >= 1.00m | 1.00 | 0.500 m/s | Full |

**Factor 2 — Heading error** (using `worst_err = max(|lookahead_err|, |current_wp_err|)`):
| Heading error | Speed fraction | At max_speed=0.50 | Effective speed |
|---|---|---|---|
| > 57 degrees | 0.25 | 0.125 m/s | Crawl |
| > 40 degrees | 0.35 | 0.175 m/s | Slow |
| > 23 degrees | 0.55 | 0.275 m/s | Medium |
| <= 23 degrees | 1.00 | 0.500 m/s | Full |

Then: `speed = max(max_speed * min(speed_fwd, speed_angle), min_forward_speed)` where `min_forward_speed = 0.08 m/s`.

**Insight**: Using two independent heading errors (`angle_error` from lookahead, `current_err` from current waypoint) and taking the worst is clever — it catches both "I'm misaligned with the path ahead" and "I'm misaligned with where I need to go right now". The `min_forward_speed` floor prevents the robot from stalling (Ackermann steering requires forward motion to turn).

**Issue — Step function speed changes**: The speed zones are discrete steps, not continuous. Crossing a threshold (e.g., forward clearance goes from 0.26m to 0.24m) causes an instantaneous speed change from 0.175 to 0.075 m/s. On real hardware this produces jerky motion. Consider **linear interpolation** between zones:

```python
# Example: lerp between 0.10m→0.15 and 0.25m→0.35
t = (forward_dist - _FWD_CONTACT_DIST) / (_FWD_SLOW_DIST - _FWD_CONTACT_DIST)
speed_fwd = _SPEED_CONTACT + t * (_SPEED_SLOW - _SPEED_CONTACT)
```

### 10.6 Side Correction — Subtle Issue

```python
def _compute_side_correction(left_dist, right_dist, safe_distance) -> float:
    if left_dist < safe_distance:
        ratio = 1.0 - left_dist / safe_distance
        return -_SIDE_GAIN * ratio    # push right (away from left wall)
    if right_dist < safe_distance:
        ratio = 1.0 - right_dist / safe_distance
        return _SIDE_GAIN * ratio     # push left (away from right wall)
    return 0.0
```

**Issue — Mutual exclusion**: Only one side is corrected at a time (the first `if` wins). If both walls are within `safe_distance` (0.15m) — which happens in 600mm corridors where the robot center is at 0.3m from each wall — only the left wall triggers correction. The right wall is ignored.

**Fix**: Compute both corrections and sum them:
```python
correction = 0.0
if left_dist < safe_distance:
    correction -= _SIDE_GAIN * (1.0 - left_dist / safe_distance)
if right_dist < safe_distance:
    correction += _SIDE_GAIN * (1.0 - right_dist / safe_distance)
return correction
```

This naturally centers the robot between two close walls. With the current code, in a narrow corridor the robot gets pushed right when both walls are equidistant, creating a steady-state offset toward the right wall.

### 10.7 Obstacle Correction — Analysis

```python
def _compute_obstacle_correction(forward_dist, left_dist, right_dist, angle_error):
    is_straight = abs(angle_error) < _OBS_STRAIGHT_THRESHOLD      # < 23 degrees
    sides_clear = max(left_dist, right_dist) > _OBS_CLEAR_SIDES_DIST  # > 0.25m
    if forward_dist >= _OBS_ACTIVE_FWD_DIST or not sides_clear or not is_straight:
        return 0.0
    urgency = 1.0 - forward_dist / _OBS_ACTIVE_FWD_DIST
    strength = _OBSTACLE_GAIN * urgency
    return -strength if right_dist > left_dist else strength
```

This is a **gentle pre-avoidance** that nudges the robot toward the open side before the obstacle triggers a full escape. It only fires on straights (`is_straight`) with room to maneuver (`sides_clear`) and when an obstacle is within 0.20m (`_OBS_ACTIVE_FWD_DIST`).

**Issue**: At 0.20m forward distance and 0.50 m/s, the robot has only 0.4 seconds before contact. The `_OBS_ACTIVE_FWD_DIST = 0.20m` is very tight. For real hardware with motor latency, this should be 0.30-0.40m to give more reaction time.

**Issue — No interaction with YOLO**: Currently, this function steers toward the side with more room, regardless of sign color. When YOLO is integrated, this function should be the primary insertion point: if the sign is green, force `right_dist > left_dist` interpretation; if red, force the opposite. The function's structure already supports this — just needs a `sign_color` parameter.

### 10.8 Escape Maneuver State Machines

**Wall K-turn** (critical risk):
```
Entry: reverse at -0.20 m/s with 0 steering (1 frame)
Loop:  reverse at -0.20 m/s with ±80% max steering (6-12 frames)
Exit:  reset escape_mode, prev_waypoint_dist, dist_increasing_count
```

Duration is dynamically computed:
```python
escape_side_dist = left_dist if escape_steer_sign > 0 else right_dist
wall_limited = clamp(int(escape_side_dist / 0.03), 6, 12)
```

At 0.03m per frame step, with `_ESCAPE_REV_SPEED = -0.20 m/s` and 20 Hz control, the robot moves ~0.01m per frame — so `_ESCAPE_DUR_DIST_STEP = 0.03` actually represents 3 frames worth of travel. The duration formula is therefore:

```
frames = clamp(side_clearance_m / 0.03, 6, 12)
actual_travel = frames * 0.20 / 20 = frames * 0.01m
```

For a side distance of 0.30m: `frames = min(12, int(0.30/0.03)) = 10`. Travel = 0.10m. **Reasonable — won't hit the side wall**.

For a side distance of 0.10m: `frames = max(6, int(0.10/0.03)) = 6`. Travel = 0.06m. **Tight but the robot width (0.15m) means 0.10m clearance + 0.06m reverse arc keeps it within bounds** — barely.

**Obstacle mini K-turn** (obstacle risk):
```
Phase 1 (70% of 12 frames = 8 frames): reverse at -0.25 m/s with ±70% max steer
Phase 2 (30% of 12 frames = 4 frames): forward at 0.15 m/s with inverted steer at 50% scale
```

This two-phase approach backs up and steers away, then drives forward past the obstacle. **Clever design** — the forward phase counter-steers to straighten out and resume the path.

**Issue — Fixed 12-frame duration**: Unlike the wall escape (which adapts to clearance), the obstacle escape duration is hardcoded at 12 frames. In a narrow corridor, 12 frames of reversing at 0.25 m/s moves ~0.15m — close to hitting the opposite wall.

### 10.9 Escape Direction Selection — Multi-Layer Logic

The escape direction goes through three decision layers:

1. **`_choose_escape_sign()`**: First attempt uses waypoint heading (`atan2(dy,dx) - current_yaw`). If the waypoint is to the left, steer left during reverse (sign = -1). On repeated escapes (`nearby_count >= 2`), switches to pure clearance-based.

2. **`_apply_wall_safety_override()`**: Flips the chosen direction if the "curve side" (the wall the robot would arc toward when reversing) is closer than 0.18m AND the other side has more room. Disabled on 2nd+ escape from the same spot.

3. **`_trigger_stuck_escape()`** (stuck recovery): Uses `_choose_escape_sign_from_waypoint()` — same logic as layer 1 but computed from absolute waypoint position rather than `(dx, dy)` delta.

**Analysis**: This layered approach is well-thought-out. The progression from "steer toward waypoint" → "steer toward room" → "skip waypoint entirely" handles the escalating severity of being stuck. The wall safety override prevents the escape from curving INTO a closer wall, which would worsen the situation.

**Issue — `_choose_escape_sign` sign convention**: The function returns `-1` when `wp_err > 0` (waypoint is to the left). This means "steer LEFT during REVERSE" which actually turns the robot's nose to the RIGHT (toward the waypoint). This is **correct for Ackermann reverse** but the sign convention is confusing. A comment explaining "negative angular.z during reverse = nose turns right" would prevent future bugs.

### 10.10 Waypoint Advancement — Three Triggers

```python
should_skip = (
    distance < self._waypoint_threshold           # within 0.20m — reached it
    or self._dist_increasing_count >= 10           # moving away for 10 frames
    or self._has_passed_waypoint(robot_x, ...)     # facing away + next is closer
)
```

1. **Distance threshold (0.20m)**: Standard waypoint capture. At 0.50 m/s, the robot traverses the 0.20m radius in 0.4 seconds — reasonable capture window.

2. **Distance increasing for 10 consecutive frames**: Safety valve. If the robot keeps moving away from the waypoint for 0.5 seconds (10 frames at 20 Hz), skip it. Prevents orbiting a waypoint that the path doesn't naturally pass through.

3. **Passed-waypoint detection**: If heading error > 90 degrees (facing away) AND the next waypoint is closer, the robot overshot. This is the most geometrically sound check.

**Issue — No bounds check after skip**: When `_skip_waypoint_and_clear_critical()` increments `self._waypoint_index`, it doesn't check if the new index is valid. The bounds check happens at the top of `_control_loop()`, but if multiple skips cascade in one frame (critical loop → skip, then `_maybe_advance_waypoint` also skips), the index could jump past the end. In practice, the `_control_loop` guard catches this on the next tick, but within the same tick the `self._waypoints[self._waypoint_index]` access at line 289 would crash.

**Specific scenario**: `_skip_waypoint_and_clear_critical()` at line 737 increments the index. Then back in `_control_loop()` at line 284, `_compute_lookahead_error()` accesses `self._waypoints[steer_idx]` where `steer_idx = self._waypoint_index` which may now be out of bounds if it was the last waypoint.

**Fix**: Add a bounds check in `_skip_waypoint_and_clear_critical()`:
```python
self._waypoint_index = min(self._waypoint_index + 1, len(self._waypoints))
```
Or better, check bounds at the start of `_build_velocity_command`.

### 10.11 Stuck Detection — Timing Analysis

```python
def _check_stuck(self, robot_x, robot_y):
    if self._log_counter % 20 != 0:  # every 20th control tick = every 1 second
        return
    ...
    if moved < _STUCK_MOVE_THRESHOLD:  # < 0.03m in 1 second
        self._stuck_seconds += 1
        if self._stuck_seconds >= 2 and not self._escape_mode:
            self._trigger_stuck_escape(robot_x, robot_y)
```

The stuck detector checks every 20 ticks (1 second at 20 Hz). If the robot moved < 0.03m in that second, `_stuck_seconds` increments. After 2 consecutive stuck seconds (2 seconds total), it triggers an escape.

**Issue — Interacts with escape mode**: The `not self._escape_mode` guard prevents re-triggering while already escaping. But `_obstacle_escape` is not checked. If the robot is in an obstacle escape that's not making progress (e.g., reversing into a wall), stuck detection could trigger a wall escape that interrupts the obstacle escape mid-maneuver. In `_trigger_stuck_escape()`, `self._obstacle_escape = False` is explicitly set, so this is intentional — the wall escape takes priority. **Acceptable design, but worth documenting**.

**Issue — `_log_counter` as timing proxy**: The `_log_counter` increments every control tick, but ticks are timer-driven — they don't fire during ROS2 executor sleep. If the control loop is delayed (e.g., heavy LIDAR processing), 20 ticks may take longer than 1 second. For real hardware, use `self.get_clock().now()` instead.

### 10.12 Waypoint Generation (`waypoints.py`) — Deep Dive

**Arc geometry** (`_arc_with_endpoints()`):

```python
_ARC_RADIUS = 0.45  # Must exceed Ackermann min turning radius (~0.294m)
```

The minimum turning radius for Ackermann is `wheelbase / tan(max_steer_angle) = 0.17 / tan(30 deg) = 0.294m`. The arc radius of 0.45m provides 53% headroom. **Good margin**.

Each corner arc has: 1 entry point + 3 intermediate points + 1 exit point = 5 points per corner. At radius 0.45m, a 90-degree arc has length `0.45 * pi/2 = 0.707m`. With 4 segments between 5 points, each segment is ~0.177m. At full speed (0.50 m/s), the robot crosses one segment in 0.35 seconds — roughly 7 control ticks. **Sufficient resolution for smooth tracking**.

**Corridor centerline bias**:
```python
_OUTER_WALL_BIAS = 0.05  # 50mm toward the outer wall
```

This pushes the planned path 50mm toward the outer wall. For a 1000mm corridor with center at 0.50m from each wall, the path sits at 0.45m from inner, 0.55m from outer. For a 600mm corridor, the path sits at 0.25m from inner, 0.35m from outer.

**Issue**: In a 600mm corridor, the path center (with bias) is at 0.25m from the inner wall. The robot width is 0.15m, so the inner edge of the robot is 0.175m from the inner wall. This leaves only 7.5cm clearance — workable but tight. The `_SIDE_GAIN` correction at `_safe_distance = 0.15m` will activate constantly in this corridor, producing a steady rightward drift (see Section 10.6 issue).

**Counter-clockwise arc reversal**:
```python
if direction is Direction.CLOCKWISE:
    return { "east": east_straight + se_cw, ... }
# Counter-clockwise: reverse each segment
return {
    "east": list(reversed(east_straight)) + list(reversed(ne_cw)),
    ...
}
```

For CCW, both the straight segments and the corner arcs are reversed. The corner arcs are also swapped (e.g., `ne_cw` instead of `se_cw` for east). **This correctly produces a CCW path** — the robot traverses east corridor upward (north-to-south order reversed) and takes the NE corner instead of the SE corner.

**Potential issue — CCW arc orientation**: The CW arcs go from `theta_start` to `theta_end` in decreasing angle (e.g., 0.0 to -pi/2 for SE). When reversed for CCW, the points are simply reversed in order. But the arc was generated for CW direction — the intermediate points are sampled assuming CW traversal. Reversing the list makes the robot traverse the arc in reverse order, which geometrically traces the same circular path but in the opposite direction. **This is correct** because the arc is a set of points on a circle — the order determines traversal direction, not the curve shape.

### 10.13 Repeat-Escape Detection — Memory Leak Risk

```python
self._obstacle_escape_positions: list[tuple[float, float]] = []
self._critical_escape_positions: list[tuple[float, float]] = []
```

These lists accumulate escape positions over the entire run. They're cleared in two cases:
1. When a waypoint is advanced (`_maybe_advance_waypoint`)
2. When a critical loop triggers waypoint skip (`_skip_waypoint_and_clear_critical`)

But `_obstacle_escape_positions` is only cleared in `_maybe_advance_waypoint` and `_finish_obstacle_escape` (on loop detection). If the robot keeps triggering escapes at different positions within the same waypoint segment, the lists grow unbounded.

**Practical impact**: Low — in a typical 3-lap run (~156 waypoints), each waypoint segment is at most ~0.35m long. The robot can only trigger a few escapes per segment before advancing. But for very long runs or pathological scenarios, this could become an issue.

**Fix**: Add a maximum list size (e.g., 20) and drop oldest entries.

### 10.14 The `_has_passed_waypoint` Robustness

```python
def _has_passed_waypoint(self, robot_x, robot_y, distance, dx, dy) -> bool:
    target_angle = math.atan2(dy, dx)
    heading_err = _wrap_angle(target_angle - self._current_yaw)
    if abs(heading_err) <= math.pi / 2:
        return False
    next_idx = self._waypoint_index + 1
    if next_idx >= len(self._waypoints):
        return False
    nx, ny = self._waypoints[next_idx]
    next_dist = math.sqrt((nx - robot_x) ** 2 + (ny - robot_y) ** 2)
    return next_dist < distance
```

This is a **two-condition check**: (1) facing away from current waypoint AND (2) closer to next waypoint. Both must be true. This prevents false positives where the robot is simply facing the wrong direction temporarily (condition 1 alone) or where it happens to be closer to the next waypoint on a curved path (condition 2 alone).

**Edge case**: At the very last waypoint (`next_idx >= len`), the function returns False — the robot can never "pass" the final waypoint this way. It can only reach it via the distance threshold (0.20m) or the distance-increasing counter (10 frames). **Correct behavior — prevents premature completion**.

### 10.15 `SimpleRobotDriver` — Purpose and Limitations

The driver uses fixed-phase alternation:
- Forward: 3 seconds at 0.30 m/s
- Turn: 2 seconds at 0.15 m/s with ±0.35 rad steering

This produces a rectangular-ish path with rounded corners. The driver has **no LIDAR awareness** — it will crash into walls if the timing doesn't match the track geometry. It's correctly positioned as a "video recording" tool, not a competition driver.

**Issue**: The timing is hardcoded for a specific corridor width. On a 600mm corridor, the forward phase may overshoot before the turn phase begins. This is acceptable for its stated purpose (generating visual training data).

### 10.16 Test Coverage Analysis

**`test_collision.py` — Good coverage**:
- `TestMeasureDistance`: 4 tests (uniform scan, no rays, minimum selection, self-detection filter)
- `TestClampLidarScan`: 4 tests (inf replacement, below-min replacement, valid unchanged, immutability)
- `TestUpdateFwdCriticalCount`: 2 tests (increment, reset)
- `TestAssessCollisionRisk`: 5 tests (safe, critical+debounced, single spike, obstacle classification, GPU artifact)

**Missing test cases for `collision.py`**:
- `assess_collision_risk` with `turning=True` (should block obstacle classification)
- `assess_collision_risk` with `boxed_in=True` but `is_open_challenge=False` (should return critical, not obstacle)
- `measure_distance_in_direction` with negative target angles (e.g., -pi/2 for right)
- Edge case: all ranges equal to `LIDAR_MIN_RANGE` exactly

**Missing tests entirely**:
- `waypoints.py` — **Zero test coverage**. The most complex pure-logic module has no tests. Priority items:
  - Arc endpoint correctness (do they land on the corridor centerline?)
  - CCW vs CW path reversal (do both complete a full loop?)
  - `_rotate_to_start` correctness for all 4 starting sections
  - `_deduplicate_consecutive` with various gap sizes
  - `_nearest_waypoint_index` correctness
  - Multi-lap waypoint count (should be ~52 * num_laps + partial first segment)
- `navigator.py` helper functions — `_wrap_angle`, `_proportional_steer`, `_compute_side_correction`, `_compute_obstacle_correction`, `_choose_escape_sign`, `_apply_wall_safety_override` are all pure functions that could be tested trivially

### 10.17 `test_framework.py` — Autonomous Testing Infrastructure

This is a sophisticated test harness that:
1. Runs the navigator as a subprocess against Gazebo scenarios
2. Parses navigator logs via regex to extract escape events, stuck events, loop events, waypoint skips
3. Classifies events by track corner using `CornerClassifier`
4. In `train` mode, adjusts `navigator_params.json` heuristically after failures and retries
5. Produces markdown reports

**Strengths**: Having an automated regression test that runs the full navigator → Gazebo loop is excellent for catching behavioral regressions. The dataclass-based event model (`EscapeEvent`, `LoopEvent`, etc.) is clean.

**Concern**: The regex patterns in `LogParser` are tightly coupled to the navigator's log format strings. Any change to the `get_logger().warning(...)` format in `navigator.py` will silently break log parsing. Consider defining log message formats as named constants shared between the navigator and the parser.

### 10.18 Summary of Navigation Core Issues

| # | Issue | Severity | File | Line(s) |
|---|---|---|---|---|
| N1 | `_compute_side_correction` only corrects for one side at a time | Medium | `navigator.py` | 793-805 |
| N2 | Speed zones are discrete steps — produces jerky motion on real hardware | Medium | `navigator.py` | 544-553 |
| N3 | Bounds check missing after `_skip_waypoint_and_clear_critical` | High | `navigator.py` | 732-740 |
| N4 | `_log_counter` used as time proxy — inaccurate under load | Low | `navigator.py` | 673 |
| N5 | `_OBS_ACTIVE_FWD_DIST = 0.20m` too tight for real hardware latency | Medium | `navigator.py` | 98 |
| N6 | `_FWD_SHORT_LOOKAHEAD_DIST = 0.15m` late corner anticipation | Low | `navigator.py` | 93 |
| N7 | No D-term in steering controller — oscillation risk on real hardware | Medium | `navigator.py` | 788-790 |
| N8 | Obstacle escape duration hardcoded at 12 frames | Low | `navigator.py` | 158 |
| N9 | Escape sign convention confusing — no comment on reverse-steer mapping | Low | `navigator.py` | 835-853 |
| N10 | Escape position lists grow unbounded within a waypoint | Low | `navigator.py` | 162-168 |
| N11 | `> 4.0m` GPU LIDAR heuristic is simulation-specific | Low | `collision.py` | 141 |
| N12 | Zero test coverage for `waypoints.py` | High | `waypoints.py` | — |
| N13 | Odometry will drift on real hardware without IMU fusion | Medium | `navigator.py` | 222-229 |
| N14 | `rclpy.shutdown()` called from timer callback | Medium | `navigator.py` | 256 |
| N15 | Log format coupling between navigator and test_framework regex parser | Low | Both | — |

### 10.19 Recommended Navigation Improvements — Priority Order

**Must-fix before real hardware**:
1. **N3** — Add bounds check after waypoint skip to prevent index-out-of-range crash
2. **N1** — Fix side correction to handle both walls simultaneously (critical for 600mm corridors)
3. **N13** — Plan IMU fusion for odometry (can use `robot_localization` EKF package)
4. **N14** — Replace `rclpy.shutdown()` in callback with a flag-based approach

**Should-fix for reliability**:
5. **N2** — Smooth speed transitions with linear interpolation between zones
6. **N5** — Increase `_OBS_ACTIVE_FWD_DIST` to 0.30-0.35m for real hardware reaction time
7. **N7** — Add filtered D-term or switch to Stanley controller for real hardware
8. **N12** — Write unit tests for `waypoints.py` (arc geometry, CCW reversal, rotation, dedup)

**Nice-to-have**:
9. **N11** — Gate the `> 4.0m` heuristic behind an `is_simulation` config flag
10. **N4** — Use ROS2 clock instead of `_log_counter` for stuck timing
11. **N10** — Cap escape position lists at 20 entries
12. **N9** — Add comment explaining reverse-steer sign convention

---

## 11. Post-Review Update: Current Implementation Status

**Review Date**: 2026-04-05  
**Reviewer**: Claude Sonnet 4.5  
**Status**: Updated assessment based on current codebase state

### 11.1 Major Progress Since Initial Review (2026-03-24)

The robot codebase has made **significant progress** in implementing the hardware abstraction layer and competition infrastructure that were identified as gaps in the original March 24 review:

#### ✅ Completed Implementations

| Component | Original Status (Mar 24) | Current Status (Apr 5) | Files |
|---|---|---|---|
| **State Machine** | Not mentioned | ✅ **Fully implemented** | `robot/src/state_machine/` |
| **Motor Driver (Build HAT)** | "Needs writing" | ✅ **Complete with calibration** | `robot/src/hardware/motors/build_hat/driver.py` |
| **IMU Driver (BNO08x)** | "Needs writing" | ✅ **Complete via MCP2221A I2C** | `robot/src/hardware/imu/bno08x/mcp2221/i2c.py` |
| **Camera Driver (RPi Cam3)** | Topic remap needed | ✅ **Complete with v4l2** | `robot/src/hardware/camera/rpi/camera_module_3/driver.py` |
| **Hailo 8 NPU Driver** | Planned | ✅ **Complete with benchmarking** | `robot/src/hardware/hailo/hailo_8/driver.py` |
| **Display Driver (SSD1306)** | Not mentioned | ✅ **Implemented** | `robot/src/hardware/display/ssd1306/driver.py` |
| **GPIO Button Driver** | Not mentioned | ✅ **Implemented** | `robot/src/hardware/button/gpio/driver.py` |
| **JSON Structured Logging** | Basic logging only | ✅ **Production-grade** | `robot/src/logger/` |
| **Environment Config** | Hardcoded values | ✅ **Env var abstraction** | `robot/src/env.py` |

### 11.2 State Machine Architecture (NEW)

The robot now implements a **4-stage competition-compliant state machine** that addresses WRO rules:

```
BOOT_CHECK → READY → RACING → FINISHED
     ↓          ↓        ↓          ↓
  Hardware   Button   Laps or   Display
   Check     Press    E-Stop    Results
```

**Key Features**:
- **BOOT_CHECK**: Pre-race hardware verification (IMU, LiDAR, Hailo, motors, network)
- **READY**: Waits for single button press to start (WRO rule compliance)
- **RACING**: Active race mode — ignores short presses, only accepts 2-second E-STOP hold
- **FINISHED**: Displays race metrics and results

**Data Structures** (`robot/src/state_machine/types.py`):
- `SystemStatus`: Boot check results for all hardware components
- `RaceMetrics`: Laps, time, velocity, steering, gyro yaw
- `VisionMetrics`: NPU FPS, bounding box, confidence, distance estimate
- `LidarMetrics`: Clearance distances and path status

**Assessment**: This is **excellent engineering** — the state machine enforces WRO competition rules (button press to start, E-STOP hold during race) and provides structured telemetry for the display/dashboard.

### 11.3 Hardware Abstraction Layer (HAL) — Fully Realized

All hardware drivers now follow a **consistent abstract base class pattern**:

#### Motor Control (Build HAT on RPi Zero 2W)
- ✅ Lazy connection pattern (`@property` accessors)
- ✅ Calibration file support (JSON-based left/right steering limits)
- ✅ Position and speed getters for both drive and steering motors
- ✅ Structured logging with JSON extras

**Assessment**: Production-ready. The calibration system is critical for Ackermann steering accuracy.

#### IMU (BNO08x via MCP2221A I2C Bridge)
- ✅ Uses Adafruit Blinka + CircuitPython libraries
- ✅ Enables all 6 sensor features (accel, gyro, mag, quaternion, game rotation, linear accel)
- ✅ Returns quaternion in ROS2 convention (x, y, z, w)
- ✅ Calibration status getter
- ✅ Complete `Data` dataclass with all sensor readings

**Assessment**: Excellent. The MCP2221A USB-to-I2C bridge solution is reliable and the Blinka abstraction handles USB HID communication automatically.

**Critical Note for Integration**: The navigator currently **does not subscribe to IMU** (only uses odometry + LIDAR). The original review identified this as issue **N13** — odometry drift on real hardware. The IMU driver is ready, but the navigator needs:
```python
# In TrackNavigator.__init__():
self.create_subscription(Imu, "/imu", self._imu_callback, 10)
```

And either:
1. Manual yaw fusion: `fused_yaw = alpha * odom_yaw + (1-alpha) * imu_yaw`
2. Or use `robot_localization` EKF package (recommended)

#### Camera (RPi Camera Module 3 Wide)
- ✅ OpenCV VideoCapture via v4l2
- ✅ 1536x864 @ 30fps support
- ✅ FPS measurement and latency benchmarking
- ✅ Returns structured `Frame` dataclass with timestamp and dimensions

**Assessment**: Solid. The benchmarking methods will be useful for YOLO pipeline tuning.

#### Hailo 8 NPU
- ✅ HailoRT integration
- ✅ Model loading from `.hef` files
- ✅ Latency benchmarking with configurable iterations
- ✅ Temperature and power monitoring
- ✅ Returns `InferenceResult` dataclass

**Model Path**: `/usr/local/hailo/models/yolo11n.hef` (configured via env var)

**Assessment**: Driver is complete. However, the inference output parsing is **incomplete** — `infer_with_timing()` returns an empty `detections=[]` list. The driver needs YOLO postprocessing to convert raw NPU output to bounding boxes + class labels.

### 11.4 CRITICAL GAPS — What's Still Missing

Despite the excellent HAL progress, the **core competition functionality gaps** identified in Section 7 remain:

#### 🔴 Gap 1: No Vision Pipeline for Obstacles Challenge

**Status**: **BLOCKING for obstacles challenge**

The review identified this as the **#1 critical priority**. Current state:
- ✅ Camera driver exists
- ✅ Hailo 8 driver exists
- ✅ Vision data structures defined (`VisionMetrics`)
- ❌ **No YOLO detector ROS2 node**
- ❌ **No sign detection → navigator integration**
- ❌ **No `/signs` topic**
- ❌ **No color-based passing rules implementation**

**What's needed**:
```python
# robot/src/vision/detector.py (DOES NOT EXIST)
class YoloDetectorNode(Node):
    """Subscribes to /robot/camera, publishes /signs."""
    def __init__(self):
        self.create_subscription(Image, "/robot/camera", self._image_callback, 10)
        self._sign_pub = self.create_publisher(SignDetection, "/signs", 10)
        self._hailo = HailoDriver()
        
    def _image_callback(self, msg):
        frame = self._bridge.imgmsg_to_cv2(msg)
        results = self._hailo.infer(frame)
        detections = self._postprocess_yolo(results)  # ← Missing
        for det in detections:
            self._sign_pub.publish(det)

# robot/src/navigation/navigator.py (NEEDS MODIFICATION)
class TrackNavigator(Node):
    def __init__(self):
        # ... existing code ...
        self.create_subscription(SignDetection, "/signs", self._sign_callback, 10)
        self._latest_sign: SignDetection | None = None
        
    def _sign_callback(self, msg):
        self._latest_sign = msg
        
    def _compute_obstacle_correction(self, ...):
        # ← ADD: Check self._latest_sign.color
        if self._latest_sign and self._latest_sign.color == "green":
            # Force pass RIGHT
            return -strength
        elif self._latest_sign and self._latest_sign.color == "red":
            # Force pass LEFT
            return strength
        # ... existing clearance-based logic as fallback
```

**Estimate**: 2-3 days of focused work for YOLO detector node + navigator integration.

#### 🔴 Gap 2: No Parking Execution Logic

**Status**: **BLOCKING for obstacles challenge completion**

Original review Section 4.3:
> "**Gap**: There is no parking execution logic in the navigator. The waypoint generator does not create a parking approach sequence."

Current state:
- ✅ Parking lot generation in simulation works
- ✅ Magenta parking blocks rendered correctly
- ❌ **Navigator has no parking state**
- ❌ **No vision-based magenta block detection**
- ❌ **No parking approach waypoints**

**What's needed**:
1. Extend YOLO model to detect magenta blocks (3rd class alongside red/green signs)
2. Add parking detection logic:
   ```python
   if self._is_obstacles_challenge and self._latest_sign.color == "magenta":
       self._enter_parking_mode()
   ```
3. Parking maneuver state machine:
   - Approach phase: Drive to parking zone entry
   - Alignment phase: Use LIDAR to center between blocks
   - Entry phase: Drive forward until LIDAR reads < 0.15m front wall
   - Final position hold (3 seconds for WRO rules)

**Estimate**: 1-2 days after YOLO pipeline is working.

#### 🟡 Gap 3: Shared Constants Package Still Missing

**Status**: **Code quality / maintainability issue**

Original review Section 9.1:
> "**Problem**: `robot/src/config/constants.py` (62 lines) and `robot/src/config/enums.py` are subsets of `simulation/src/config/constants.py` (358 lines)."

**Current state**:
- `simulation/src/config/constants.py`: **358 lines**
- `robot/src/config/constants.py`: **62 lines**
- Still **duplicated** — any change requires manual sync

**Fix (from original review)**:
```
platform/
  shared/
    src/
      config/
        constants.py  # Single source of truth
        enums.py
    pyproject.toml
  simulation/
    pyproject.toml  # depends on shared
  robot/
    pixi.toml       # depends on shared
```

**Estimate**: 2-3 hours to extract shared package and update imports.

#### 🟡 Gap 4: Navigator Still Calls `rclpy.shutdown()` in Callback

**Status**: **Bug — incorrect ROS2 lifecycle**

Original review issue **N14**:
> "Calling `rclpy.shutdown()` from within a timer callback can cause issues in ROS2 (the executor is still running)."

**Current state**: Unchanged from March 24 review. Line 256 of `navigator.py` (if file structure matches review) still has:
```python
if self._waypoint_index >= len(self._waypoints):
    self.get_logger().info(f"Completed {self._num_laps} lap(s). Stopping.")
    self._publish_stop()
    rclpy.shutdown()  # ← INCORRECT
```

**Fix**:
```python
# In __init__:
self._should_shutdown = False

# In timer callback:
if self._waypoint_index >= len(self._waypoints):
    self.get_logger().info(f"Completed {self._num_laps} lap(s). Stopping.")
    self._publish_stop()
    self._should_shutdown = True
    return

# In main.py _run_navigate():
try:
    while rclpy.ok() and not navigator._should_shutdown:
        rclpy.spin_once(navigator)
finally:
    navigator.destroy_node()
    rclpy.shutdown()
```

**Estimate**: 15 minutes.

### 11.5 Abstraction Quality Assessment

**Excellent Patterns Observed**:
1. ✅ **Lazy initialization** via `@property` — all drivers delay connection until first use
2. ✅ **Abstract base classes** (`Driver` ABCs for motors, IMU, camera, Hailo) — enables mock testing and sim/real swapping
3. ✅ **Dataclass-based outputs** (`Frame`, `Data`, `InferenceResult`) — type-safe returns
4. ✅ **Environment variable abstraction** (`EnvVar` generic class with type casting) — no hardcoded config
5. ✅ **Structured JSON logging** with `extra={"details": {...}}` — production-grade observability
6. ✅ **Calibration file support** (motor steering limits) — essential for real hardware tuning

**Areas for Improvement**:
1. ⚠️ **Hailo driver incomplete** — `infer_with_timing()` returns empty `detections=[]` (needs YOLO postprocessing)
2. ⚠️ **No ROS2 node wrappers** for hardware drivers — camera/IMU/Hailo drivers are plain classes, not ROS2 nodes. Integration requires wrapper nodes (e.g., `CameraPublisherNode` that calls `driver.capture_frame()` and publishes `sensor_msgs/Image`)
3. ⚠️ **Display driver not integrated** — SSD1306 driver exists but no state machine → display rendering logic
4. ⚠️ **Button driver not integrated** — GPIO button driver exists but not connected to state machine transitions

### 11.6 ROS2 Integration Gaps — Distributed Architecture

The hardware drivers are **device abstractions**, not ROS2 nodes. However, the system has a **critical architectural constraint**: motor control runs on the **RPi Zero 2W**, while all other nodes run on the **RPi 5**, communicating via **USB Gadget Mode** (virtual Ethernet).

#### System Architecture — Two-Device ROS2 Network

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 5 (16GB)                        │
│                    ROS2 Domain ID: 0                             │
│                    IP: 10.55.0.1 (usb0)                          │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌───────────────┐ │
│  │ TrackNavigator   │  │ CameraPublisher  │  │ IMUPublisher  │ │
│  │                  │  │ Node             │  │ Node          │ │
│  │ Publishes:       │  │ Publishes:       │  │ Publishes:    │ │
│  │ /cmd_vel (Twist) │  │ /robot/camera    │  │ /imu          │ │
│  │                  │  │                  │  │               │ │
│  │ Subscribes:      │  └──────────────────┘  └───────────────┘ │
│  │ /odom, /scan,    │                                           │
│  │ /signs, /imu     │  ┌──────────────────┐  ┌───────────────┐ │
│  └──────────────────┘  │ YOLODetector     │  │ DisplayMgr    │ │
│                        │ Node             │  │ Node          │ │
│  ┌──────────────────┐  │ Subscribes:      │  │ Subscribes:   │ │
│  │ StateMachine     │  │ /robot/camera    │  │ /state, /race │ │
│  │ Node             │  │ Publishes:       │  │               │ │
│  │ Publishes:       │  │ /signs           │  │ → SSD1306     │ │
│  │ /state           │  └──────────────────┘  └───────────────┘ │
│  └──────────────────┘                                           │
│                                                                  │
│  Hailo 8 NPU, RPi Camera Module 3, BNO085 IMU, RPLiDAR C1      │
│  SSD1306 OLED, GPIO Button                                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ USB Gadget Ethernet (usb0)
                              │ DDS Discovery + Traffic
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                   Raspberry Pi Zero 2W                           │
│                   ROS2 Domain ID: 0                              │
│                   IP: 10.55.0.2 (usb0)                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ BuildHATTwistNode                                        │   │
│  │                                                          │   │
│  │ Subscribes: /cmd_vel (Twist) ← from RPi 5               │   │
│  │                                                          │   │
│  │ Publishes: /odom (Odometry)  → to RPi 5                 │   │
│  │            /joint_states      → motor encoder positions  │   │
│  │                                                          │   │
│  │ Hardware: Build HAT (SPI) → LEGO motors                 │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  Raspberry Pi Build HAT + 2x LEGO Large Motors                  │
└─────────────────────────────────────────────────────────────────┘
```

#### ROS2 Node Deployment Map

| Node Name | Device | Input Topics | Output Topics | Hardware Access |
|---|---|---|---|---|
| `track_navigator` | **RPi 5** | `/odom`, `/scan`, `/imu`, `/signs` | `/cmd_vel` | None |
| `camera_publisher` | **RPi 5** | — | `/robot/camera` | RPi Cam3 (CSI) |
| `imu_publisher` | **RPi 5** | — | `/imu` | BNO085 (MCP2221A USB) |
| `yolo_detector` | **RPi 5** | `/robot/camera` | `/signs` | Hailo 8 (M.2) |
| `lidar_publisher` | **RPi 5** | — | `/scan` | RPLiDAR C1 (USB) |
| `display_manager` | **RPi 5** | `/state`, `/race_metrics` | — | SSD1306 (I2C) |
| `button_listener` | **RPi 5** | — | `/button_events` | GPIO button |
| `state_machine` | **RPi 5** | `/button_events`, `/lap_complete` | `/state`, `/race_metrics` | None |
| **`buildhat_twist_node`** | **RPi Zero 2W** | `/cmd_vel` | `/odom`, `/joint_states` | Build HAT (SPI) |

#### Why This Architecture?

1. **RPi 5 compute advantage**: Hailo 8 NPU needs PCIe bandwidth → must be on RPi 5
2. **Build HAT limitation**: Only works on 40-pin GPIO (not USB), Zero 2W has GPIO + lower cost
3. **Low latency motor control**: Zero 2W dedicated to real-time motor response (no YOLO overhead)
4. **USB Gadget Mode**: Creates virtual Ethernet, ROS2 DDS discovers nodes automatically across the link

#### Network Configuration Required

**On RPi Zero 2W** (`/etc/network/interfaces.d/usb0`):
```
auto usb0
iface usb0 inet static
    address 10.55.0.2
    netmask 255.255.255.0
```

**On RPi 5** (`/etc/network/interfaces.d/usb0`):
```
auto usb0
iface usb0 inet static
    address 10.55.0.1
    netmask 255.255.255.0
```

**ROS2 Middleware Configuration**:

The system currently uses **Zenoh** (`rmw_zenoh_cpp`) in simulation and needs configuration for real hardware. See [`MIDDLEWARE_COMPARISON.md`](./MIDDLEWARE_COMPARISON.md) for a comprehensive analysis of Zenoh vs CycloneDDS.

**Recommended: Zenoh** (both devices, `~/.bashrc`):
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

**Zenoh Router** (systemd service on RPi 5):
```bash
sudo systemctl enable zenoh-router
sudo systemctl start zenoh-router
```

**Zenoh Session Config** on RPi 5 (`~/.config/zenoh/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`):
```json5
{
  mode: "client",
  connect: { endpoints: ["tcp/localhost:7447"] },
  scouting: { multicast: { enabled: false } }
}
```

**Zenoh Session Config** on RPi Zero 2W:
```json5
{
  mode: "client",
  connect: { endpoints: ["tcp/10.55.0.1:7447"] },
  scouting: { multicast: { enabled: false } }
}
```

**Why Zenoh?** (vs CycloneDDS):
- ✅ TCP-based discovery (no multicast issues over USB Gadget)
- ✅ ~50 MB lower memory footprint on Zero 2W (critical for 512 MB RAM)
- ✅ Sub-second startup vs 5-10s DDS discovery
- ✅ Simpler configuration (10-line JSON vs 50-line XML)
- ✅ Consistency with simulation environment

**Alternative: CycloneDDS** (if preferred):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/pi/cyclonedds.xml
```

See [`MIDDLEWARE_COMPARISON.md`](./MIDDLEWARE_COMPARISON.md) for detailed benchmarks, pros/cons, and migration guide.

#### Launch File Architecture

**Three separate launch configurations**:

1. **`robot/launch/rpi5_nodes.launch.py`** (RPi 5 main stack):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='klevor_robot',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{'device': '/dev/video0'}]
        ),
        Node(
            package='klevor_robot',
            executable='imu_publisher',
            name='imu_publisher',
            parameters=[{'i2c_address': 0x4A}]
        ),
        Node(
            package='klevor_robot',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[{'model_path': '/usr/local/hailo/models/yolo11n.hef'}]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
        ),
        Node(
            package='klevor_robot',
            executable='display_manager',
            name='display_manager'
        ),
        Node(
            package='klevor_robot',
            executable='button_listener',
            name='button_listener'
        ),
        Node(
            package='klevor_robot',
            executable='state_machine',
            name='state_machine'
        ),
        # TrackNavigator launched separately with metadata argument
    ])
```

2. **`robot/launch/rpi_zero_nodes.launch.py`** (RPi Zero 2W motor control):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='klevor_robot',
            executable='buildhat_twist_node',
            name='buildhat_twist_node',
            parameters=[{
                'steering_port': 'A',
                'drive_port': 'B',
                'publish_odom': True,
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }]
        ),
    ])
```

3. **`robot/launch/simulator.launch.py`** (Gazebo sim — all on one machine):
```python
# Same as rpi5_nodes but exclude hardware-specific nodes
# Use Gazebo plugins for /odom, /scan, /camera instead
```

#### Startup Sequence (Competition Day)

**On RPi Zero 2W**:
```bash
# SSH or console login
source ~/ros2_ws/install/setup.bash
ros2 launch klevor_robot rpi_zero_nodes.launch.py
```

**On RPi 5**:
```bash
# Terminal 1: Launch sensor/hardware nodes
source ~/ros2_ws/install/setup.bash
ros2 launch klevor_robot rpi5_nodes.launch.py

# Terminal 2: Launch navigator (after state machine signals READY)
python3 robot/main.py navigate --metadata scenario_XXXX_metadata.json --laps 3
```

#### Testing Cross-Device Communication

**Verify DDS discovery** (run on RPi 5):
```bash
# Should see nodes from BOTH devices
ros2 node list

# Expected output:
/camera_publisher
/imu_publisher
/yolo_detector
/rplidar_node
/display_manager
/button_listener
/state_machine
/buildhat_twist_node  # ← from RPi Zero 2W
```

**Test motor commands** (RPi 5):
```bash
# Publish test Twist command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Monitor odometry from Zero 2W
ros2 topic echo /odom
```

#### Failure Modes and Resilience

| Failure | Detection | Recovery |
|---|---|---|
| USB Gadget link down | `/odom` stops publishing | State machine → EMERGENCY_STOP |
| RPi Zero 2W crash | No `/odom` for 2 seconds | Navigator publishes `Twist(0,0)` → safe stop |
| RPi 5 crash | Zero 2W receives no `/cmd_vel` | Build HAT node stops motors after 500ms timeout |
| DDS discovery failure | `ros2 node list` doesn't show remote node | Restart both launch files, check `usb0` IP |

**Watchdog Implementation** (recommended):

**On RPi Zero 2W** (`buildhat_twist_node`):
```python
class BuildHATTwistNode(Node):
    def __init__(self):
        # ... existing code ...
        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog_check)  # 10 Hz
        
    def _cmd_vel_callback(self, msg):
        self._last_cmd_time = self.get_clock().now()
        # ... apply motor commands ...
        
    def _watchdog_check(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:  # No command for 500ms
            self.get_logger().warn("Watchdog: No cmd_vel, stopping motors")
            self._driver.stop_drive()
            self._driver.center_steering()
```

**On RPi 5** (`track_navigator`):
```python
# In __init__:
self.create_timer(0.1, self._check_odom_timeout)
self._last_odom_time = self.get_clock().now()

def _odom_callback(self, msg):
    self._last_odom_time = self.get_clock().now()
    # ... existing code ...
    
def _check_odom_timeout(self):
    elapsed = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
    if elapsed > 2.0:  # No odometry for 2 seconds
        self.get_logger().error("Odometry timeout - RPi Zero 2W link down?")
        self._publish_stop()
        # Trigger state machine emergency stop
```

#### Current Status: ROS2 Wrapper Nodes

| Node | Device | Current Status |
|---|---|---|
| `camera_publisher_node` | RPi 5 | ❌ Missing |
| `imu_publisher_node` | RPi 5 | ❌ Missing |
| `yolo_detector_node` | RPi 5 | ❌ Missing |
| `lidar_publisher_node` | RPi 5 | ✅ **Use existing `rplidar_ros`** |
| `display_manager_node` | RPi 5 | ❌ Missing |
| `button_listener_node` | RPi 5 | ❌ Missing |
| `state_machine_node` | RPi 5 | ⚠️ Core logic exists, needs ROS2 wrapper |
| **`buildhat_twist_node`** | **RPi Zero 2W** | ❌ **Missing (CRITICAL)** |

**Estimate**: 
- ROS2 wrapper nodes: 1.5 days (6 nodes + launch files)
- Cross-device testing: 0.5 days
- **Total: 2 days** for complete distributed ROS2 system

### 11.7 Updated Priority Actions (Post-April 5 Review)

#### CRITICAL (Blockers for Competition)

1. **Implement YOLO detector ROS2 node** (2-3 days)
   - YOLO output postprocessing in Hailo driver
   - ROS2 node subscribing to `/robot/camera`, publishing `/signs`
   - Integration test with Gazebo camera feed

2. **Integrate sign detection into navigator** (1 day)
   - Subscribe to `/signs` topic
   - Modify `_compute_obstacle_correction()` to use sign color
   - Red → pass LEFT, Green → pass RIGHT

3. **Implement parking execution** (1-2 days)
   - Add magenta block detection to YOLO model
   - Parking maneuver state machine
   - LIDAR-based alignment and entry

4. **Create ROS2 wrapper nodes for distributed architecture** (2 days)
   - **RPi Zero 2W**: `buildhat_twist_node` (CRITICAL — motor control)
   - **RPi 5**: `camera_publisher`, `imu_publisher`, `yolo_detector`, `display_manager`, `button_listener`, `state_machine` wrapper
   - Launch files: `rpi5_nodes.launch.py`, `rpi_zero_nodes.launch.py`, `simulator.launch.py`
   - USB Gadget Mode network configuration + DDS setup
   - Watchdog timers for cross-device communication failures

5. **Fix `rclpy.shutdown()` bug** (15 minutes) — Issue N14

#### HIGH PRIORITY (Reliability)

6. **Add IMU fusion to navigator** (4-6 hours)
   - Use `robot_localization` EKF package
   - Fuse odometry + IMU for drift-free localization
   - Addresses issue N13

7. **Fix side correction logic** (30 minutes)
   - Issue N1: Handle both walls simultaneously
   - Critical for 600mm narrow corridors

8. **Add bounds check after waypoint skip** (15 minutes) — Issue N3

9. **Write unit tests for `waypoints.py`** (4 hours) — Issue N12

#### MEDIUM PRIORITY (Code Quality)

10. **Extract shared constants package** (2-3 hours) — Issue from Section 9.1

11. **Smooth speed transitions** (1-2 hours)
    - Issue N2: Linear interpolation instead of step functions
    - Prevents jerky motion on real hardware

12. **Increase obstacle detection distance** (5 minutes)
    - Issue N5: `_OBS_ACTIVE_FWD_DIST` from 0.20m → 0.30-0.35m

13. **Integrate display manager** (4 hours)
    - State machine → SSD1306 rendering
    - Show boot status, race metrics, lap count

14. **Integrate button listener** (2 hours)
    - GPIO button → state machine transitions
    - Short press (READY→RACING), long press (E-STOP)

### 11.8 Sim-to-Real Transition Readiness

**Current Readiness**: **60%**

| Category | Sim | Real | Gap |
|---|---|---|---|
| **Navigation core** | ✅ Works | ⚠️ Needs IMU fusion | N13 — odometry drift |
| **LIDAR avoidance** | ✅ Works | ✅ Ready | rplidar_ros package exists |
| **Motor control** | ✅ Gazebo plugin | ✅ Driver ready | Needs ROS2 wrapper node |
| **IMU** | ✅ Gazebo IMU | ✅ Driver ready | Needs ROS2 publisher node |
| **Camera** | ✅ Gazebo camera | ✅ Driver ready | Needs ROS2 publisher node |
| **Vision (YOLO)** | ❌ No pipeline | ❌ No pipeline | **CRITICAL GAP** |
| **Parking** | ❌ No execution | ❌ No execution | **CRITICAL GAP** |
| **State machine** | N/A | ✅ Ready | Needs display/button integration |
| **Open challenge** | ✅ Works | ⚠️ Needs testing | Likely 80%+ ready |
| **Obstacles challenge** | ❌ Blocked | ❌ Blocked | Needs YOLO + parking |

**Estimate to Competition-Ready**:
- **Open challenge only**: 2-3 days (ROS2 wrappers + IMU fusion + real hardware testing)
- **Obstacles challenge**: 6-8 days (YOLO pipeline + parking + testing)

### Recommendations Summary:

**For Open Challenge Only (3-4 days):**
- Create distributed ROS2 wrapper nodes (RPi 5 + Zero 2W)
- Configure USB Gadget networking + Zenoh (see [MIDDLEWARE_COMPARISON.md](./MIDDLEWARE_COMPARISON.md))
- Add IMU fusion for odometry
- Fix navigation bugs (N3, N13, N14)
- Test cross-device communication and watchdog failsafes

**For Obstacles Challenge (7-9 days):**
- Implement YOLO detector node with Hailo postprocessing
- Integrate sign detection into navigator (red/green passing rules)
- Implement parking execution state machine
- Plus all open challenge fixes
- Extended cross-device integration testing

### Current Competition Readiness: 60%
- Open challenge: ~80% ready (needs distributed ROS2 + IMU fusion + testing)
- Obstacles challenge: BLOCKED on vision pipeline

**Critical Path Item**: The `buildhat_twist_node` on RPi Zero 2W is **the most critical missing piece** — without it, the robot cannot move. This should be the **first node implemented** before any others.

### 11.9 Distributed Architecture Benefits

**Why USB Gadget Mode + Dual Device?**

1. **Deterministic motor control**: Zero 2W dedicated to motor timing, no interference from YOLO/camera processing
2. **Fault isolation**: Navigator crash on RPi 5 → Zero 2W watchdog safely stops motors
3. **Development flexibility**: Can test motor control independently of vision pipeline
4. **Cost optimization**: Zero 2W is £15 vs RPi 5 £80 — dedicated motor controller is cost-effective
5. **GPIO availability**: RPi 5's GPIO might be used for other sensors, Zero 2W has dedicated 40-pin for Build HAT

**Alternative (not recommended)**: Run everything on RPi 5 with Build HAT via GPIO
- ❌ Blocks RPi 5 GPIO for other uses
- ❌ Motor control competes with YOLO for CPU time
- ❌ Single point of failure (one crash stops everything)

### 11.10 Testing Strategy for Distributed System

**Phase 1: Hardware Bring-Up (per device)**
1. RPi Zero 2W: Test Build HAT motors independently (no network)
2. RPi 5: Test camera, IMU, LIDAR, Hailo independently

**Phase 2: USB Gadget Link**
1. Configure network interfaces on both devices
2. Verify connectivity: `ping 10.55.0.2` from RPi 5
3. Test DDS discovery: `ros2 node list` shows nodes from both

**Phase 3: Motor Control Integration**
1. Launch `buildhat_twist_node` on Zero 2W
2. Publish test `Twist` from RPi 5: `ros2 topic pub /cmd_vel ...`
3. Verify odometry publishes back: `ros2 topic echo /odom`

**Phase 4: Full Navigation Stack**
1. Launch all RPi 5 nodes
2. Launch `TrackNavigator` with test scenario
3. Gazebo sim → real hardware comparison

**Phase 5: Watchdog Testing**
1. Kill `buildhat_twist_node` → verify navigator detects odom timeout
2. Kill navigator → verify Zero 2W stops motors after cmd_vel timeout
3. Unplug USB → verify both devices fail safely

### 11.10 Build HAT Twist Node — Implementation Blueprint

The **`buildhat_twist_node`** on RPi Zero 2W is the most critical missing component. Here's the complete implementation specification:

#### Core Responsibilities

1. **Subscribe to `/cmd_vel`** (geometry_msgs/Twist) from TrackNavigator on RPi 5
2. **Convert Twist → Ackermann kinematics** → motor commands
3. **Publish `/odom`** (nav_msgs/Odometry) computed from encoder feedback
4. **Publish `/joint_states`** (sensor_msgs/JointState) with motor positions
5. **Watchdog safety**: Stop motors if no `/cmd_vel` received for 500ms

#### Ackermann Kinematics (Bicycle Model)

```python
# From Twist to motor commands
linear_vel = msg.linear.x      # m/s (forward/backward)
angular_vel = msg.angular.z    # rad/s (steering rate)

# Compute steering angle from angular velocity
# For Ackermann: angular_vel = linear_vel * tan(steering_angle) / wheelbase
if abs(linear_vel) > 0.01:
    steering_angle = math.atan(angular_vel * WHEELBASE / linear_vel)
else:
    steering_angle = 0.0  # Stopped — center steering

# Clamp to max steering angle (30 degrees = 0.5236 rad)
steering_angle = np.clip(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

# Convert linear velocity to drive motor speed
# LEGO motor speed is in degrees/sec
# Wheel circumference = π * diameter
# linear_vel [m/s] = (motor_speed [deg/s] / 360) * circumference [m]
WHEEL_DIAMETER = 0.056  # 56mm LEGO wheel
wheel_circumference = math.pi * WHEEL_DIAMETER
motor_speed_deg_s = (linear_vel / wheel_circumference) * 360

# Convert steering angle (radians) to motor position (degrees)
# Calibration maps -30° to 30° steering → motor positions from calibration file
steering_position_deg = steering_angle * (180 / math.pi)  # rad → deg

# Apply calibration (from JSON file loaded at startup)
if steering_position_deg < 0:
    # Left turn → use left_limit
    motor_position = calibration['left_limit'] * (steering_position_deg / 30.0)
else:
    # Right turn → use right_limit
    motor_position = calibration['right_limit'] * (steering_position_deg / 30.0)
```

#### Odometry Computation

```python
# Read encoder positions
drive_position = self._driver.get_drive_position()  # degrees
steering_position = self._driver.get_steering_position()  # degrees

# Compute delta since last update
delta_drive = drive_position - self._last_drive_position
delta_time = (current_time - self._last_odom_time).nanoseconds / 1e9

# Convert drive delta to linear distance
wheel_rotations = delta_drive / 360.0
distance = wheel_rotations * wheel_circumference  # meters

# Compute heading change using Ackermann geometry
# Current steering angle (from steering motor position)
steering_angle = (steering_position / calibration['right_limit']) * MAX_STEERING_ANGLE

# Heading change for Ackermann: dtheta = distance * tan(steering_angle) / wheelbase
if abs(steering_angle) > 0.01:
    delta_theta = distance * math.tan(steering_angle) / WHEELBASE
else:
    delta_theta = 0.0

# Update pose in odom frame
self._odom_x += distance * math.cos(self._odom_theta)
self._odom_y += distance * math.sin(self._odom_theta)
self._odom_theta += delta_theta
self._odom_theta = wrap_to_pi(self._odom_theta)

# Compute velocities
linear_vel = distance / delta_time if delta_time > 0 else 0.0
angular_vel = delta_theta / delta_time if delta_time > 0 else 0.0

# Publish Odometry message
odom_msg = Odometry()
odom_msg.header.stamp = current_time
odom_msg.header.frame_id = 'odom'
odom_msg.child_frame_id = 'base_link'
odom_msg.pose.pose.position.x = self._odom_x
odom_msg.pose.pose.position.y = self._odom_y
# Convert theta to quaternion
q = tf_transformations.quaternion_from_euler(0, 0, self._odom_theta)
odom_msg.pose.pose.orientation.x = q[0]
odom_msg.pose.pose.orientation.y = q[1]
odom_msg.pose.pose.orientation.z = q[2]
odom_msg.pose.pose.orientation.w = q[3]
odom_msg.twist.twist.linear.x = linear_vel
odom_msg.twist.twist.angular.z = angular_vel

self._odom_pub.publish(odom_msg)
```

#### File Structure

```
robot/src/nodes/
├── __init__.py
├── buildhat_twist_node.py      # RPi Zero 2W motor control node
├── camera_publisher_node.py    # RPi 5 camera publishing
├── imu_publisher_node.py        # RPi 5 IMU publishing
├── yolo_detector_node.py        # RPi 5 YOLO + Hailo
├── display_manager_node.py      # RPi 5 SSD1306 display
├── button_listener_node.py      # RPi 5 GPIO button
└── state_machine_node.py        # RPi 5 competition state logic
```

#### BuildHATTwistNode Skeleton

```python
# robot/src/nodes/buildhat_twist_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from src.hardware.motors.build_hat.driver import Driver as BuildHATDriver
from src.config.constants import RobotSpecs

class BuildHATTwistNode(Node):
    """ROS2 node that converts Twist commands to Build HAT motor control.
    
    Runs on RPi Zero 2W. Subscribes to /cmd_vel from RPi 5, publishes /odom.
    """
    
    def __init__(self):
        super().__init__('buildhat_twist_node')
        
        # Hardware driver
        self._driver = BuildHATDriver()
        self._driver.connect()
        self._calibration = self._driver.load_calibration()
        
        # ROS2 communication
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Odometry state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._last_drive_position = self._driver.get_drive_position()
        self._last_odom_time = self.get_clock().now()
        
        # Watchdog
        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.02, self._publish_odometry)  # 50 Hz
        self.create_timer(0.1, self._watchdog_check)     # 10 Hz
        
        self.get_logger().info("BuildHAT Twist Node started on RPi Zero 2W")
    
    def _cmd_vel_callback(self, msg: Twist):
        """Convert Twist to motor commands."""
        self._last_cmd_time = self.get_clock().now()
        
        # Ackermann kinematics (see above)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # ... compute steering_angle and motor_speed_deg_s ...
        
        # Apply to hardware
        self._driver.move_steering_to(steering_position_deg)
        if motor_speed_deg_s > 0:
            self._driver.run_drive_forward(speed=int(abs(motor_speed_deg_s)))
        elif motor_speed_deg_s < 0:
            self._driver.run_drive_reverse(speed=int(abs(motor_speed_deg_s)))
        else:
            self._driver.stop_drive()
    
    def _publish_odometry(self):
        """Compute and publish odometry from encoders."""
        current_time = self.get_clock().now()
        
        # Read encoders and compute pose (see above)
        # ... odometry computation ...
        
        self._odom_pub.publish(odom_msg)
        self._joint_pub.publish(joint_msg)
    
    def _watchdog_check(self):
        """Stop motors if no command received for 500ms."""
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.get_logger().warn("Watchdog: No cmd_vel for 500ms, stopping motors")
            self._driver.stop_drive()
            self._driver.center_steering()

def main(args=None):
    rclpy.init(args=args)
    node = BuildHATTwistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._driver.stop_drive()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Integration with Main Navigator

The `TrackNavigator` on RPi 5 is **already publishing** `Twist` on `/cmd_vel` (line reference from original review). No changes needed to navigator — it's device-agnostic. The magic happens when:

1. Navigator publishes `Twist(linear.x=0.3, angular.z=0.1)` on RPi 5
2. DDS routes message over USB Gadget ethernet to RPi Zero 2W
3. `BuildHATTwistNode` receives it, converts to motor commands
4. Motors move, encoders update
5. `BuildHATTwistNode` publishes `Odometry` back to RPi 5
6. Navigator receives odometry, closes the control loop

**This is the beauty of ROS2's distributed architecture** — the navigator doesn't know (or care) that the motors are on a different physical device.

### 11.11 Recommendations Summary

**Before writing any code**:
1. ✅ Review this updated assessment
2. Prioritize based on competition timeline:
   - If **open challenge only** → focus on items 4, 5, 6, 7, 8
   - If **obstacles challenge required** → items 1-4 are blockers

**Testing strategy**:
1. Write ROS2 wrapper nodes first (item 4)
2. Test each sensor in isolation with `ros2 topic echo`
3. Integration test: `TrackNavigator` + camera + IMU + LIDAR in Gazebo
4. Transfer to real hardware and tune parameters via `navigator_params.json`

**Architecture decision needed**:
- Current approach: Hailo 8 on RPi 5 main compute
- Alternative: Hailo 8 on separate node, communicate via ROS2 topics
- Recommendation: **Keep on RPi 5** — simpler, lower latency, easier debugging

---

**Conclusion**: The hardware abstraction layer is **exceptionally well-designed** and nearly complete. The state machine architecture is **production-grade**. The critical path is now:

1. **Distributed ROS2 architecture** — Implementing the dual-device system with USB Gadget networking
2. **BuildHAT Twist Node** — The most critical missing piece (motor control on RPi Zero 2W)
3. **Vision pipeline integration** — YOLO detector node + navigator coupling for obstacles challenge
4. **ROS2 node wrappers** — Camera, IMU, display, button nodes on RPi 5

The original March 24 review's assessment of "strong engineering fundamentals" is validated — the team has built on those fundamentals to create a robust HAL with excellent abstraction patterns. The gaps are well-defined and addressable with **7-9 days of focused work** for full obstacles challenge capability, or **3-4 days** for open challenge only.

**Key Architectural Insight**: The distributed ROS2 design (RPi 5 + Zero 2W) provides fault isolation, deterministic motor control, and development flexibility. The USB Gadget Mode + Zenoh configuration enables seamless cross-device communication while maintaining low latency for motor control. See [MIDDLEWARE_COMPARISON.md](./MIDDLEWARE_COMPARISON.md) for the complete middleware analysis and recommendation rationale.

---

## Appendix A: Quick Reference — Node Implementation Priority

**Week 1 (Critical Path — 3-4 days)**:
1. ✅ **Day 1**: `buildhat_twist_node.py` on RPi Zero 2W + USB Gadget networking
2. ✅ **Day 2**: `camera_publisher_node.py`, `imu_publisher_node.py` on RPi 5
3. ✅ **Day 3**: Launch files + cross-device testing + watchdog validation
4. ✅ **Day 4**: IMU fusion in navigator + fix bugs N3, N13, N14

**Week 2 (Obstacles Challenge — 3-5 days)**:
5. ✅ **Day 5-6**: `yolo_detector_node.py` with Hailo postprocessing
6. ✅ **Day 7**: Sign detection integration in navigator (passing rules)
7. ✅ **Day 8**: Parking execution state machine
8. ✅ **Day 9**: Full integration testing (open + obstacles challenges)

**Concurrent Tasks** (can be done in parallel):
- `display_manager_node.py` + `button_listener_node.py` + `state_machine_node.py` (1 day)
- Extract shared constants package (2-3 hours)
- Write unit tests for `waypoints.py` (4 hours)

**Total**: 7-9 days for competition-ready system with both challenges.

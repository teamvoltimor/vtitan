# WRO 2026 Gazebo Simulation

Complete Gazebo Harmonic simulation for generating robot POV training data for the **WRO Future Engineers 2026** competition.

## Features

✅ **Official WRO 2026 Specifications** - Accurate track dimensions, colors, and rules
✅ **Two Challenge Types** - Open Challenge and Obstacles Challenge
✅ **36 Traffic Sign Scenarios** - Official WRO predefined pillar configurations
✅ **Dynamic Parking Lot** - Grid-based positioning with proper spacing
✅ **Full Randomization** - Lighting, starting conditions, corridor widths
✅ **Automated Generation** - Batch create hundreds of scenarios
✅ **Video Recording Pipeline** - Complete ROS2-based pipeline for generating training videos
✅ **Frame Extraction** - Automatic extraction of frames for YOLO training
✅ **Clean Architecture** - Organized constants, documentation, and code

## Quick Start

### Generate and Record Training Videos (Complete Pipeline)

```bash
# Generate 10 scenarios and record videos automatically
cd simulation/scripts
python3 record_scenario_videos.py --challenge open --num-scenarios 10 --duration 30

# Test the pipeline (1 scenario, 15 seconds)
./test_recording.sh
```

Videos will be saved to `./training_data/open/videos/`

**📹 See [docs/VIDEO_RECORDING_GUIDE.md](docs/VIDEO_RECORDING_GUIDE.md) for complete video recording documentation**

### Generate Scenarios Only

```bash
# Generate 10 open challenge scenarios
cd simulation/scripts
python3 generate_training_data.py --challenge open --num-scenarios 10

# Generate 10 obstacles challenge scenarios
python3 generate_training_data.py --challenge obstacles --num-scenarios 10 --randomize-all

# Launch a scenario
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/../models
gz sim ~/wro_training_data/open/scenarios/scenario_0000.sdf
```

See [docs/QUICKSTART.md](docs/QUICKSTART.md) for detailed instructions.

## Directory Structure

```
simulation/
├── docs/                   # Documentation
│   ├── QUICKSTART.md       # Getting started guide
│   ├── WRO_SPECIFICATIONS.md  # Official WRO 2026 specs
│   └── CHANGELOG.md        # Changes and improvements
├── worlds/                 # Gazebo world files
│   └── wro_track_2026.sdf  # Base WRO 2026 track
├── models/                 # Gazebo model definitions
│   ├── traffic_pillar/     # Traffic sign models
│   └── parking_limitation/ # Parking block models
├── scripts/                # Training data generation
│   ├── generate_training_data.py  # Main generator
│   ├── constants.py        # WRO specifications constants
│   └── extract_frames_and_annotate.py  # Frame extraction
├── launch/                 # ROS2 launch files
│   └── wro_simulation.launch.py
└── README.md              # This file
```

## Key Specifications

All specifications are defined in `scripts/constants.py` for easy reference and modification.

### Track Dimensions
- **Mat size**: 3200mm × 3200mm
- **Track size**: 3000mm × 3000mm
- **Wall height**: 100mm (black)
- **Coordinate system**: Bottom-left origin (0,0) to top-right (3.0, 3.0)

### Open Challenge
- **Corridor widths**: 600mm OR 1000mm per section (randomized)
- **Traffic signs**: None
- **Starting zone**: 500mm × 200mm (randomized position)
- **Laps**: 3 required

### Obstacles Challenge
- **Corridor width**: 1000mm fixed (all sides)
- **Inner area**: 1000mm × 1000mm corner section
- **Traffic signs**: 3-6 pillars (36 predefined scenarios)
  - Grid positions at intersections: (1.0, 1.5, 2.0) × (0.4, 0.6)
  - Red: RGB(238, 39, 55) | Green: RGB(68, 214, 44)
- **Parking lot**: 2 magenta blocks (200×20×100mm)
  - Positioned at grid depths, spaced 1.5× robot width
  - Perpendicular to corridor, touching outer wall
- **Starting zone**: Resized and positioned between parking blocks

## Architecture

### Constants (`scripts/constants.py`)

All WRO specifications are centralized in `constants.py`:

```python
from constants import (
    Section, Direction,   # Enums for type safety
    TrackDimensions,      # Track and mat sizes
    WallSpecs,            # Wall dimensions and colors
    CorridorDimensions,   # Corridor widths and divisions
    TrafficSignSpecs,     # Sign dimensions, colors, grid positions
    ParkingLotSpecs,      # Parking block specifications
    StartingZoneSpecs,    # Starting zone dimensions
    RobotSpecs,           # Robot dimensions
    TrackMarkings,        # Corner lines and markings
    LightingSpecs,        # Lighting parameters
)

# Example usage - Dimensions
sign_height = TrafficSignSpecs.HEIGHT  # 0.10 meters
red_color = TrafficSignSpecs.RED_COLOR  # (0.933, 0.153, 0.216)

# Example usage - Enums
section = Section.NORTH  # Type-safe section reference
direction = Direction.CLOCKWISE  # Type-safe direction
print(section)  # Prints: 'north'
print(section.capitalized)  # Prints: 'North'
```

### Generator (`scripts/generate_training_data.py`)

Main script for generating randomized scenarios:

- **WROTrainingDataGenerator**: Creates scenario world files
- **36 Scenario System**: Official WRO traffic sign configurations
- **Parking Lot Positioning**: Grid-based with intelligent spacing
- **Starting Zone Adjustment**: Dynamic sizing for obstacles challenge
- **Metadata Generation**: JSON files with complete scenario information

## Usage Examples

### Video Recording (Recommended for Model Training)

```bash
# Complete pipeline: Generate scenarios and record videos
python3 record_scenario_videos.py \
    --challenge open \
    --num-scenarios 50 \
    --duration 30

# With frame extraction for YOLO training
python3 record_scenario_videos.py \
    --challenge obstacles \
    --num-scenarios 100 \
    --duration 45 \
    --randomize-all \
    --extract-frames \
    --num-frames 20

# Convert existing ROS2 bags to videos
python3 convert_bags_to_videos.py \
    --bag-dir ~/wro_recordings \
    --output-dir ~/videos
```

See [docs/VIDEO_RECORDING_GUIDE.md](docs/VIDEO_RECORDING_GUIDE.md) for complete documentation.

### Scenario Generation Only

```bash
# Open challenge with default settings
python3 generate_training_data.py --challenge open --num-scenarios 50

# Obstacles challenge with full randomization
python3 generate_training_data.py \
    --challenge obstacles \
    --num-scenarios 100 \
    --randomize-all \
    --output-dir ~/my_dataset
```

### Output Structure

```
~/wro_training_data/
├── open/
│   ├── scenarios/          # Generated world files
│   │   ├── scenario_0000.sdf
│   │   ├── scenario_0000_metadata.json
│   │   └── ...
│   ├── videos/             # Recorded videos (if using video pipeline)
│   │   ├── scenario_0000.mp4
│   │   ├── scenario_0001.mp4
│   │   └── ...
│   └── frames/             # Extracted frames (if --extract-frames used)
│       ├── scenario_0000/
│       │   ├── frame_0000.jpg
│       │   └── ...
│       └── ...
└── obstacles/
    └── (same structure)
```

### Metadata Example

```json
{
  "scenario_id": 0,
  "challenge_type": "obstacles",
  "corridor_widths": {
    "north": {"type": "wide", "width_mm": 1000},
    "south": {"type": "wide", "width_mm": 1000},
    "east": {"type": "wide", "width_mm": 1000},
    "west": {"type": "wide", "width_mm": 1000}
  },
  "starting_conditions": {
    "direction": "clockwise",
    "section": "south",
    "position": {"x": 1.15, "y": 0.1},
    "yaw": 1.5708
  },
  "num_signs": 5,
  "has_parking_lot": true,
  "sign_positions": [
    {"x": 1.0, "y": 2.6, "color": "green"},
    {"x": 1.5, "y": 2.4, "color": "red"}
  ],
  "parking_lot": {
    "block1_position": {"x": 1.0, "y": 0.1},
    "block2_position": {"x": 1.3, "y": 0.1},
    "depth": 1.0
  }
}
```

## Requirements

### System
- Ubuntu 22.04 or 24.04
- Python 3.10+
- 8GB+ RAM

### Software
- Gazebo Harmonic (or Gazebo Classic 11)
- ROS2 Humble/Jazzy (optional, for robot integration)
- Python packages: `numpy`, `pyyaml`

### Installation

```bash
# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic

# Install Python dependencies
pip3 install numpy pyyaml

# Set model path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models
```

## Documentation

- **[VIDEO_RECORDING_GUIDE.md](docs/VIDEO_RECORDING_GUIDE.md)** - Complete guide for recording training videos
- **[QUICKSTART.md](docs/QUICKSTART.md)** - Get started in 5 minutes
- **[WRO_SPECIFICATIONS.md](docs/WRO_SPECIFICATIONS.md)** - Complete WRO 2026 technical reference
- **[CHANGELOG.md](docs/CHANGELOG.md)** - Recent improvements and changes

## Troubleshooting

### Gazebo not finding models
```bash
# Add to ~/.bashrc
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/simulation/models
```

### Invalid coordinates in metadata
- Check coordinate system: Origin at (0,0) bottom-left, max at (3.0, 3.0)
- Verify interior wall calculations for variable corridor widths

### Traffic signs not appearing
- Obstacles challenge only (open challenge has no signs)
- Check metadata: `num_signs` should be 3-6 for obstacles
- Verify grid positions: signs only at (0.4, 0.6) × (1.0, 1.5, 2.0)

### Parking blocks colliding with starting zone
- Obstacles challenge automatically adjusts starting zone size
- Zone resized to 90% of gap between parking blocks
- Zone positioned at center between blocks

## Contributing

1. All specifications in `scripts/constants.py`
2. Follow existing code style
3. Update documentation in `docs/`
4. Test with both challenge types

## License

This simulation follows official WRO Future Engineers 2026 competition specifications.

## Credits

**Official WRO Rules**: https://wro-association.org/
**Simulation**: WRO 2026 Training Data Generator
**Last Updated**: 2026-02-08

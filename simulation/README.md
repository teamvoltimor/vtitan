# WRO Gazebo Simulation for Training Data Generation

Complete Gazebo simulation package for generating robot POV training videos for the WRO Future Engineers competition.

## Features

- **Two Challenge Types**: Open Challenge and Obstacles Challenge
- **Full Randomization**: Lighting, colors, physics, object positions
- **Automated Video Recording**: Robot camera POV for model training
- **Dataset Generation**: YOLO-format annotations, extracted frames
- **ROS2 Integration**: Compatible with ROS2 Humble/Jazzy

## Directory Structure

```
simulation/
├── worlds/              # Gazebo world files
│   └── wro_track_base.sdf
├── models/              # Gazebo model definitions
│   ├── traffic_pillar/
│   └── obstacle_block/
├── urdf/                # Robot URDF description
│   └── wro_robot.urdf.xacro
├── config/              # Challenge configurations
│   ├── open_challenge.yaml
│   └── obstacles_challenge.yaml
├── scripts/             # Training data generation scripts
│   └── generate_training_data.py
├── launch/              # ROS2 launch files
│   └── wro_simulation.launch.py
└── README.md
```

## Requirements

### System Requirements
- Ubuntu 22.04 or 24.04
- ROS2 Humble or Jazzy
- Gazebo Harmonic (or Gazebo Classic 11)
- Python 3.10+
- 8GB+ RAM
- GPU recommended (NVIDIA preferred)

### Install Dependencies

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Install Gazebo
sudo apt install ros-humble-ros-gz

# Install additional ROS2 packages
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-plugins \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-cv-bridge

# Install Python dependencies
pip3 install opencv-python numpy pyyaml
```

## Quick Start

### 1. Generate Training Scenarios

Generate 100 randomized scenarios for the Open Challenge:

```bash
cd simulation/scripts
python3 generate_training_data.py \
    --challenge open \
    --num-scenarios 100 \
    --randomize-all \
    --output-dir ~/wro_training_data
```

Generate scenarios for the Obstacles Challenge:

```bash
python3 generate_training_data.py \
    --challenge obstacles \
    --num-scenarios 150 \
    --randomize-all \
    --output-dir ~/wro_training_data_obstacles
```

### 2. Launch a Scenario in Gazebo

Set environment variable for Gazebo to find models:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models
```

Launch base world:

```bash
gz sim ../worlds/wro_track_base.sdf
```

Or launch a generated scenario:

```bash
gz sim ~/wro_training_data/scenarios/scenario_0000.sdf
```

### 3. Launch with ROS2

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace (if using colcon)
cd ~/wro_ws
colcon build --packages-select wro_simulation
source install/setup.bash

# Launch simulation
ros2 launch wro_simulation wro_simulation.launch.py
```

Launch specific scenario:

```bash
ros2 launch wro_simulation wro_simulation.launch.py \
    world:=scenario_0000.sdf
```

### 4. Control the Robot

Use keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/wro_robot/cmd_vel
```

Or use joystick:

```bash
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node \
    --ros-args --remap cmd_vel:=/wro_robot/cmd_vel
```

### 5. View Camera Feed

View camera in RViz2:

```bash
rviz2
# Add -> By topic -> /wro_robot/camera/image_raw
```

Or use rqt_image_view:

```bash
ros2 run rqt_image_view rqt_image_view \
    /wro_robot/camera/image_raw
```

### 6. Record Training Data

Record ROS2 bag (all topics):

```bash
ros2 bag record -a -o wro_training_bag
```

Record only camera:

```bash
ros2 bag record /wro_robot/camera/image_raw \
    -o wro_camera_only
```

Extract frames from bag:

```bash
ros2 bag play wro_camera_only &
python3 extract_frames_from_bag.py
```

## Configuration

### Challenge Configuration Files

Edit `config/open_challenge.yaml` or `config/obstacles_challenge.yaml` to customize:

- Number of traffic pillars/obstacles
- Randomization parameters (lighting, colors, physics)
- Robot start position
- Recording settings
- Dataset format

Example customization:

```yaml
traffic_signs:
  placement:
    num_pillars_min: 8
    num_pillars_max: 12

randomization:
  lighting:
    sun_intensity:
      min: 0.3  # Darker
      max: 2.0  # Brighter
```

### Randomization Features

The simulation supports comprehensive domain randomization:

1. **Lighting Randomization**
   - Sun intensity: 0.5-1.5×
   - Ambient light variation
   - Direction changes

2. **Color Randomization**
   - Gaussian noise on RGB values
   - Realistic color drift
   - Per-pillar variation

3. **Physics Randomization**
   - Friction coefficients
   - Mass variations
   - Surface properties

4. **Position Randomization**
   - Random pillar placement
   - Random obstacle placement
   - Collision-free generation

5. **Camera Noise**
   - Gaussian sensor noise
   - Motion blur (future)
   - Exposure variation (future)

## Training Data Output

After running `generate_training_data.py`, you'll have:

```
wro_training_data/
├── scenarios/
│   ├── scenario_0000.sdf
│   ├── scenario_0000_metadata.json
│   ├── scenario_0001.sdf
│   └── ...
└── frames/
    ├── scenario_0000/
    │   ├── frame_0000.jpg
    │   ├── frame_0001.jpg
    │   └── ...
    └── scenario_0001/
        └── ...
```

### Metadata Format

Each `scenario_XXXX_metadata.json` contains:

```json
{
  "scenario_id": 0,
  "challenge_type": "open",
  "world_file": "scenario_0000.sdf",
  "num_pillars": 8,
  "num_obstacles": 0,
  "pillar_positions": [
    {"x": 0.5, "y": 0.8, "color": "red"},
    {"x": -0.3, "y": 1.0, "color": "green"}
  ],
  "obstacle_positions": []
}
```

Use this metadata to:
- Generate ground-truth annotations
- Filter scenarios by difficulty
- Analyze model performance by scenario type

## Advanced Usage

### Generate YOLO Annotations

Convert simulation metadata to YOLO format:

```bash
python3 generate_yolo_annotations.py \
    --metadata-dir ~/wro_training_data/scenarios \
    --frames-dir ~/wro_training_data/frames \
    --output-dir ~/wro_yolo_dataset
```

This creates:
```
wro_yolo_dataset/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── data.yaml
```

### Automated Recording Pipeline

Run complete pipeline (generate + launch + record + extract):

```bash
./scripts/automated_recording_pipeline.sh \
    --challenge open \
    --num-scenarios 50 \
    --duration 30
```

### Parallel Scenario Generation

Generate scenarios faster with parallel processing:

```bash
python3 generate_training_data.py \
    --challenge open \
    --num-scenarios 1000 \
    --randomize-all \
    --parallel 8  # Use 8 processes
```

## Training Computer Vision Models

### YOLO Training Example

```bash
cd ~/wro_yolo_dataset

# Train YOLO26 (latest version)
yolo task=detect mode=train \
    model=yolo26n.pt \
    data=data.yaml \
    epochs=100 \
    imgsz=640 \
    batch=16
```

### Classical CV Validation

Test HSV-based detection on generated frames:

```python
import cv2
import numpy as np

# Load frame
img = cv2.imread('frame_0000.jpg')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Detect red pillars
red_mask = cv2.inRange(hsv,
    np.array([0, 100, 100]),
    np.array([10, 255, 255]))

# Find contours
contours, _ = cv2.findContours(red_mask,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)

print(f"Detected {len(contours)} red objects")
```

## Sim-to-Real Transfer

### Progressive Transfer Strategy

1. **Train on Pure Simulation** (80% accuracy)
   - Use 100+ randomized scenarios
   - Full domain randomization enabled

2. **Collect Real Data** (10-20 images)
   - Run physical robot on practice track
   - Capture diverse lighting/angles

3. **Fine-tune Model** (10-20 epochs)
   - Start with sim-trained weights
   - Fine-tune on real data

4. **Test on Real Robot**
   - Measure accuracy on real track
   - Collect failure cases

5. **Augment Simulation** (Iterate)
   - Add failure cases to simulation
   - Re-train with mixed data
   - Repeat until >95% accuracy

### Fine-tuning Command

```bash
yolo task=detect mode=train \
    model=runs/detect/train/weights/best.pt \
    data=real_data.yaml \
    epochs=20 \
    imgsz=640 \
    batch=8 \
    lr0=0.001  # Lower learning rate for fine-tuning
```

## Performance Tips

### Gazebo Performance

- **Use GPU acceleration**: Enable GPU plugins for sensors
- **Reduce visual quality**: Lower shadows/reflections for faster sim
- **Disable unused sensors**: Comment out LiDAR if only training vision
- **Run headless**: Use `gz sim -s` for server-only mode

### Recording Performance

- **Reduce FPS**: 15 FPS sufficient for training (vs 30 FPS)
- **Compress videos**: Use H.264 compression
- **Batch processing**: Generate scenarios offline, record later

### Dataset Size Recommendations

- **Minimum viable**: 500 frames (10 scenarios × 50 frames each)
- **Good performance**: 5,000 frames (100 scenarios)
- **Production quality**: 20,000+ frames (400 scenarios)

## Troubleshooting

### Gazebo won't launch
```bash
# Check Gazebo version
gz --version

# Reset Gazebo cache
rm -rf ~/.gz/sim

# Check model paths
echo $GZ_SIM_RESOURCE_PATH
```

### Robot not spawning
```bash
# Verify URDF syntax
check_urdf wro_robot.urdf.xacro

# Check ROS2 bridge
ros2 topic list | grep wro_robot
```

### Camera not publishing
```bash
# Check camera topic
ros2 topic hz /wro_robot/camera/image_raw

# View camera info
ros2 topic echo /wro_robot/camera/camera_info
```

### Poor model performance
- Increase randomization range
- Add more lighting variation
- Include occlusion scenarios
- Fine-tune on real data

## References

- [Gazebo Documentation](https://gazebosim.org/docs)
- [ROS2 Humble Tutorials](https://docs.ros.org/en/humble/)
- [YOLO26 Training Guide](../docs/development/YOLO26_QUICK_START.md)
- [WRO Simulation Strategy](../docs/proposals/software/01_SIMULATION_STRATEGY.md)

## License

MIT License - See LICENSE file for details

## Support

For issues or questions:
- Open an issue on GitHub
- Check documentation in `/docs`
- Contact: team@teamsteelbot.com

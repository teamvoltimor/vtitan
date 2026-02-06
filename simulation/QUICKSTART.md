# Quick Start Guide: Generate Training Data in 5 Minutes

This guide will get you generating robot POV training videos in under 5 minutes.

## Prerequisites

- Ubuntu 22.04/24.04 (or WSL2 on Windows)
- Python 3.10+
- 4GB+ free disk space

## Installation (5 commands)

```bash
# 1. Install system dependencies
sudo apt update
sudo apt install python3-pip python3-opencv

# 2. Install Python packages
pip3 install numpy opencv-python pyyaml

# 3. Navigate to simulation directory
cd simulation/scripts

# 4. Generate test scenarios (10 scenarios, fast)
python3 generate_training_data.py \
    --challenge open \
    --num-scenarios 10 \
    --output-dir ~/wro_test_data

# 5. View the generated worlds
ls -lh ~/wro_test_data/scenarios/
```

You now have 10 randomized world files ready to use!

## Option A: Without ROS2 (Manual Recording)

If you don't have ROS2 installed, you can still use the generated worlds:

### 1. Install Gazebo Standalone

```bash
# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic
```

### 2. Launch a World

```bash
# Set model path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/../models

# Launch first scenario
gz sim ~/wro_test_data/scenarios/scenario_0000.sdf
```

### 3. View the Camera Feed

In Gazebo GUI:
1. Click the "Plugins" menu (top right)
2. Select "Image Display"
3. Select topic: `/camera/image`

### 4. Record Manually

Use screen recording software (OBS, SimpleScreenRecorder) to capture the camera view while you:
- Spawn a robot model manually in Gazebo
- Drive it around using Gazebo's interface
- Save the recording as MP4

## Option B: With ROS2 (Automated Recording)

For full automation with ROS2:

### 1. Install ROS2

```bash
# Quick install (Ubuntu 22.04)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop-full
```

Or use the automated script:

```bash
# Run setup script (from project root)
./scripts/setup_wsl2_dev.sh
```

### 2. Install Gazebo + ROS2 Integration

```bash
sudo apt install ros-humble-ros-gz \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-cv-bridge
```

### 3. Run Automated Pipeline

```bash
# Navigate to scripts
cd simulation/scripts

# Run complete pipeline (generate + record + extract)
./automated_pipeline.sh \
    --challenge open \
    --num-scenarios 20 \
    --duration 30
```

This will:
- Generate 20 randomized scenarios
- Launch each in Gazebo
- Record camera feed for 30 seconds
- Extract frames
- Create YOLO dataset

Output will be in `~/wro_training_data/`

## Quick Test: Generate 1 Scenario and View

```bash
# Generate single scenario
python3 generate_training_data.py \
    --challenge open \
    --num-scenarios 1 \
    --output-dir ~/wro_single_test

# View metadata
cat ~/wro_single_test/scenarios/scenario_0000_metadata.json

# Launch in Gazebo
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/../models
gz sim ~/wro_single_test/scenarios/scenario_0000.sdf
```

You should see:
- A 3m × 3m track with black walls
- Random colored pillars (red/green)
- Gray ground plane

## Generate Production Dataset

For real training, generate 100-500 scenarios:

```bash
# Open Challenge - 100 scenarios
python3 generate_training_data.py \
    --challenge open \
    --num-scenarios 100 \
    --randomize-all \
    --output-dir ~/wro_training_open

# Obstacles Challenge - 150 scenarios
python3 generate_training_data.py \
    --challenge obstacles \
    --num-scenarios 150 \
    --randomize-all \
    --output-dir ~/wro_training_obstacles
```

This creates:
- 100-150 unique world files
- Randomized lighting (0.5-1.5× intensity)
- Randomized colors (Gaussian noise)
- Randomized object positions
- Metadata JSON for each scenario

## Next Steps

1. **Manual Recording (No ROS2)**:
   - Launch worlds in Gazebo
   - Record camera view with screen capture
   - Extract frames with ffmpeg
   - Manually annotate (use LabelImg or Roboflow)

2. **Automated Recording (With ROS2)**:
   - Run `automated_pipeline.sh`
   - Get complete YOLO dataset automatically
   - Start training immediately

3. **Train YOLO Model**:
   ```bash
   # Install YOLO
   pip install ultralytics

   # Train on generated data
   yolo task=detect mode=train \
       model=yolo26n.pt \
       data=~/wro_training_data/yolo_dataset/data.yaml \
       epochs=100 \
       imgsz=640
   ```

4. **Deploy to Real Robot**:
   - Transfer trained model to Raspberry Pi
   - Test on practice track
   - Fine-tune on real data
   - Win competition! 🏆

## Troubleshooting

### "gz: command not found"
- Install Gazebo: See Option A instructions above

### "ModuleNotFoundError: No module named 'cv2'"
```bash
pip3 install opencv-python
```

### "No models found"
```bash
# Set model path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/../models

# Verify
echo $GZ_SIM_RESOURCE_PATH
```

### World is completely dark
- Check metadata file for lighting parameters
- Regenerate with `--randomize-all` flag

### Pillars not appearing
- Check metadata file for pillar positions
- Verify model path is set correctly
- Models should be in `simulation/models/`

## FAQ

**Q: Do I need ROS2?**
A: No, you can generate worlds and use Gazebo standalone. ROS2 enables automation.

**Q: How much disk space do I need?**
A: ~100MB per 10 scenarios (worlds only), ~10GB for 100 scenarios with videos/frames.

**Q: Can I run this on Windows?**
A: Use WSL2 (Windows Subsystem for Linux). Follow Ubuntu instructions.

**Q: How long does generation take?**
A: 10 scenarios = ~5 seconds, 100 scenarios = ~30 seconds (worlds only).

**Q: How do I customize colors or lighting?**
A: Edit `simulation/config/open_challenge.yaml` or `obstacles_challenge.yaml`

## Help

For more details, see:
- `simulation/README.md` - Complete documentation
- `/docs/proposals/software/01_SIMULATION_STRATEGY.md` - Strategy guide
- `/docs/development/YOLO26_QUICK_START.md` - Training guide

**Support**: Open an issue on GitHub or email team@teamsteelbot.com

---

**Ready to generate data? Start with the 5-command installation above! ⬆️**

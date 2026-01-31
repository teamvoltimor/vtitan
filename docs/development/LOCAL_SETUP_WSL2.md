# Local Development Setup - WSL2 + ROS2 Kilted Kaiju

## Hardware Requirements ✅
- **Your Setup:** 32GB RAM, RTX 4050, WSL2
- **Perfect for:** ROS2 + Gazebo simulation + ML training
- **GPU Support:** CUDA in WSL2 for YOLO training (if needed)

## Architecture Overview

```
┌─────────────────────────────────────────┐
│         Your Laptop (WSL2)              │
├─────────────────────────────────────────┤
│                                         │
│  ROS2 Humble Development Environment    │
│  ├─ Gazebo Simulation                   │
│  ├─ Vision Nodes (Python)               │
│  ├─ Control Nodes (Python)              │
│  ├─ Mock Sensors                        │
│  └─ RViz2 Visualization                 │
│                                         │
│  YOLO Training (optional)               │
│  ├─ PyTorch + CUDA                      │
│  ├─ YOLOv8 training                     │
│  └─ ONNX export                         │
│                                         │
└─────────────────────────────────────────┘
                 │
                 │ (Port when ready)
                 ↓
┌─────────────────────────────────────────┐
│      Raspberry Pi 5 (Competition)       │
├─────────────────────────────────────────┤
│  ROS2 Humble (same packages!)           │
│  ├─ Real sensors (Camera, LiDAR, IMU)   │
│  ├─ Hailo-8L acceleration               │
│  └─ micro-ROS bridge to Pico 2W         │
└─────────────────────────────────────────┘
```

---

## Phase 1: WSL2 + ROS2 Setup

### Step 1.1: Install ROS2 Humble

```bash
# Update WSL2
sudo apt update && sudo apt upgrade -y

# Add ROS2 repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (includes RViz2, Gazebo dependencies)
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
```

### Step 1.2: Configure Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/teamsteelbot_ws/install/setup.bash" >> ~/.bashrc  # Will create this workspace

# Reload
source ~/.bashrc
```

### Step 1.3: Create Workspace

```bash
# Create ROS2 workspace
mkdir -p ~/teamsteelbot_ws/src
cd ~/teamsteelbot_ws

# Build (empty for now)
colcon build
source install/setup.bash
```

---

## Phase 2: Install Simulation Tools

### Step 2.1: Gazebo Garden (for ROS2 Humble)

```bash
# Install Gazebo Garden
sudo apt install ros-humble-ros-gz -y

# Test installation
gz sim -v4
```

### Step 2.2: Install Additional ROS2 Packages

```bash
# Navigation and localization
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
sudo apt install ros-humble-robot-localization -y

# Vision
sudo apt install ros-humble-vision-msgs -y
sudo apt install ros-humble-image-transport -y
sudo apt install ros-humble-cv-bridge -y

# Visualization
sudo apt install ros-humble-rqt -y
sudo apt install ros-humble-plotjuggler-ros -y

# LiDAR (for simulation testing)
sudo apt install ros-humble-laser-geometry -y
```

### Step 2.3: Python Dependencies

```bash
# Computer vision
pip3 install opencv-python opencv-contrib-python
pip3 install numpy scipy

# YOLO (if using deep learning approach)
pip3 install ultralytics  # YOLOv8
pip3 install torch torchvision  # PyTorch (CUDA support if available)

# ROS Python tools
pip3 install transforms3d
```

---

## Phase 3: Project Structure Setup

### Step 3.1: Create Package Structure

```bash
cd ~/teamsteelbot_ws/src

# Create main packages
ros2 pkg create --build-type ament_python teamsteelbot_bringup
ros2 pkg create --build-type ament_python teamsteelbot_vision
ros2 pkg create --build-type ament_python teamsteelbot_control
ros2 pkg create --build-type ament_python teamsteelbot_sensors
ros2 pkg create --build-type ament_cmake teamsteelbot_msgs
ros2 pkg create --build-type ament_python teamsteelbot_simulation
```

### Step 3.2: Directory Structure

```
~/teamsteelbot_ws/
├── src/
│   ├── teamsteelbot_bringup/
│   │   ├── launch/
│   │   │   ├── simulation.launch.py       # Gazebo simulation
│   │   │   └── robot.launch.py            # Real robot (RPi5)
│   │   ├── config/
│   │   │   ├── simulation_params.yaml
│   │   │   └── robot_params.yaml
│   │   └── package.xml
│   │
│   ├── teamsteelbot_simulation/
│   │   ├── teamsteelbot_simulation/
│   │   │   ├── mock_camera_node.py        # Simulated camera
│   │   │   ├── mock_lidar_node.py         # Simulated LiDAR
│   │   │   └── track_generator.py         # Generate test tracks
│   │   ├── worlds/
│   │   │   └── race_track.sdf             # Gazebo world
│   │   ├── models/
│   │   │   └── racer_robot/               # URDF model
│   │   └── package.xml
│   │
│   ├── teamsteelbot_vision/
│   │   ├── teamsteelbot_vision/
│   │   │   ├── sign_detector_yolo.py      # YOLO-based detector
│   │   │   ├── sign_detector_classic.py   # Classical CV detector
│   │   │   └── detector_base.py           # Base class
│   │   └── package.xml
│   │
│   ├── teamsteelbot_control/
│   │   ├── teamsteelbot_control/
│   │   │   ├── decision_node.py           # Main decision logic
│   │   │   ├── state_machine.py           # FSM implementation
│   │   │   └── speed_controller.py        # Adaptive speed
│   │   └── package.xml
│   │
│   ├── teamsteelbot_sensors/
│   │   ├── teamsteelbot_sensors/
│   │   │   ├── sensor_fusion_node.py      # Mock IMU + encoders
│   │   │   └── distance_sensor_node.py    # Mock VL53L0X
│   │   └── package.xml
│   │
│   └── teamsteelbot_msgs/
│       ├── msg/
│       │   ├── SignDetection.msg
│       │   ├── ColorReading.msg
│       │   └── RobotState.msg
│       └── CMakeLists.txt
│
├── models/                                 # AI models
│   ├── yolov8n_signs.pt                   # PyTorch checkpoint
│   └── yolov8n_signs.onnx                 # ONNX export
│
├── datasets/                               # Training data
│   ├── images/
│   ├── labels/
│   └── synthetic/                          # Gazebo-generated
│
└── docs/
    └── development/
```

---

## Phase 4: Create Minimal Simulation

### Step 4.1: Simple Robot URDF

Create `~/teamsteelbot_ws/src/teamsteelbot_simulation/models/racer_robot/robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="racer_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Add Gazebo plugins for camera, sensors -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/racer</namespace>
          <remapping>~/image_raw:=camera/image_raw</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Step 4.2: Minimal Launch File

Create `~/teamsteelbot_ws/src/teamsteelbot_bringup/launch/simulation.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', 'empty.sdf'],
            output='screen'
        ),

        # Camera publisher (mock for now)
        Node(
            package='teamsteelbot_simulation',
            executable='mock_camera_node',
            name='camera',
            output='screen'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
```

---

## Phase 5: Development Workflow

### Daily Development Loop

```bash
# Terminal 1: Build workspace
cd ~/teamsteelbot_ws
colcon build --symlink-install  # Symlink allows live Python edits
source install/setup.bash

# Terminal 2: Launch simulation
ros2 launch teamsteelbot_bringup simulation.launch.py

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /camera/image_raw

# Terminal 4: Visualize with RViz2
rviz2
```

### Testing Individual Nodes

```bash
# Test vision node standalone
ros2 run teamsteelbot_vision sign_detector_classic

# Test decision node
ros2 run teamsteelbot_control decision_node

# Check node graph
rqt_graph
```

---

## Phase 6: GPU Setup (Optional - for YOLO Training)

### Enable CUDA in WSL2

```bash
# Check if NVIDIA driver is visible
nvidia-smi

# Install CUDA toolkit (if needed)
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/ /"
sudo apt update
sudo apt install cuda-toolkit-12-3 -y

# Install cuDNN for PyTorch
# (Follow PyTorch installation guide for WSL2)
```

### Test PyTorch GPU

```python
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"Device: {torch.cuda.get_device_name(0)}")
```

---

## Phase 7: Simulation-to-Real Transfer Checklist

When moving from laptop to RPi5:

### 7.1: Code Transfer
```bash
# On laptop: Export packages
cd ~/teamsteelbot_ws/src
tar -czf teamsteelbot_packages.tar.gz teamsteelbot_*

# Transfer to RPi5 (USB, scp, git, etc.)
scp teamsteelbot_packages.tar.gz pi@raspberrypi.local:~/
```

### 7.2: Hardware-Specific Changes

**Swap these files:**
- `launch/simulation.launch.py` → `launch/robot.launch.py`
- `mock_camera_node.py` → Real camera node (libcamera)
- `mock_lidar_node.py` → RPLiDAR driver
- `mock_sensors.py` → Real I2C sensors (IMU, VL53L0X, TCS34725)

**Keep these (no changes needed!):**
- Vision detection nodes
- Decision/control logic
- State machine
- Custom messages

### 7.3: Configuration Changes

**simulation_params.yaml** (Laptop):
```yaml
camera:
  fps: 60  # High for testing
  resolution: [640, 480]

use_gazebo: true
use_mock_sensors: true
```

**robot_params.yaml** (RPi5):
```yaml
camera:
  fps: 30  # Realistic for Pi Camera
  resolution: [640, 480]

use_gazebo: false
use_mock_sensors: false
hailo_acceleration: true
```

---

## Next Steps

### Immediate (Week 1):
1. ✅ Install ROS2 Humble on WSL2
2. ✅ Create workspace structure
3. ✅ Verify basic ROS2 commands work
4. ✅ Install Gazebo and test launch

### Short-term (Week 2-3):
1. Create minimal simulation world (track with colored signs)
2. Implement vision detection node (start with classical CV)
3. Implement basic state machine (follow line, detect signs)
4. Test in simulation

### Mid-term (Week 4-6):
1. Add YOLO detection (train on synthetic data from Gazebo)
2. Implement sensor fusion
3. Fine-tune control logic
4. Record ROS bags for analysis

### Long-term (Week 7+):
1. Port to RPi5
2. Integrate real sensors
3. Test on physical track
4. Iterate and optimize

---

## Troubleshooting

### Common WSL2 Issues

**GUI applications not showing:**
```bash
# Install WSLg (Windows 11) or VcXsrv (Windows 10)
# Set DISPLAY variable if needed
export DISPLAY=:0
```

**ROS2 daemon issues:**
```bash
ros2 daemon stop
ros2 daemon start
```

**Gazebo performance:**
```bash
# Reduce rendering quality in Gazebo GUI
# Or run headless: gz sim -s (server only)
```

---

## Resources

- **ROS2 Tutorials:** https://docs.ros.org/en/humble/Tutorials.html
- **Gazebo Tutorials:** https://gazebosim.org/docs
- **YOLOv8 Docs:** https://docs.ultralytics.com/
- **WSL2 GPU Guide:** https://learn.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl

---

## Summary

**Your laptop is perfect for:**
- ✅ ROS2 development and testing
- ✅ Vision algorithm development
- ✅ Control logic testing
- ✅ YOLO training (with RTX 4050!)
- ✅ Simulation of full robot

**Transfer to RPi5 later for:**
- Real sensor integration
- Hailo acceleration
- micro-ROS bridge
- Physical testing

**Key Advantage:** Develop 80% of code on fast laptop, only need RPi5 for final integration!

# Simulation Strategy for Testing Before Real Robot

**Last Updated:** 2026-01-17

This document outlines how to use simulators to test and validate your robot programs BEFORE building physical hardware, significantly reducing development risk and accelerating iteration.

---

## 🎯 Why Simulate First?

### Benefits of Simulation-Driven Development

1. **Faster Iteration** ⚡
   - Test algorithm changes in minutes (vs hours on real robot)
   - No need to rebuild/reflash firmware
   - Parallel testing (run multiple simulations simultaneously)

2. **Lower Risk** 🛡️
   - No hardware damage from crashes
   - Test extreme scenarios safely
   - Validate before expensive hardware purchases

3. **Better Testing Coverage** 📊
   - Test 1000+ scenarios overnight
   - Systematic edge case exploration
   - Automated regression testing

4. **Learning & Training** 🎓
   - Learn ROS2/algorithms without hardware
   - Generate training data for ML models
   - Debug in controlled environment

5. **Documentation** 📝
   - Screenshots/videos for journal
   - Reproducible results
   - Demonstrate thoroughness

---

## 🏗️ Recommended Simulator: Gazebo

### Why Gazebo?

**Gazebo** is the industry-standard robotics simulator, with excellent ROS2 integration.

**Advantages:**
- ✅ ROS2 native integration (ROS2 topics/services work directly)
- ✅ Physics engine (realistic dynamics)
- ✅ Sensor simulation (camera, LiDAR, IMU)
- ✅ 3D visualization
- ✅ Large community & resources
- ✅ Free & open-source

**Which Version?**
- **Gazebo Harmonic** (2024+) - Latest, best for ROS2 Humble
- **Gazebo Classic 11** - Older but well-documented

**Recommendation:** Start with **Gazebo Harmonic** if using ROS2.

---

## 📦 Installation

### Option A: Install on Desktop PC (Recommended)

Simulation is compute-intensive. Use a desktop/laptop with:
- Ubuntu 22.04 or 24.04
- Decent GPU (NVIDIA preferred for visualization)
- 8GB+ RAM

```bash
# Install ROS2 Humble
sudo apt install ros-humble-desktop-full

# Install Gazebo Harmonic
sudo apt install ros-humble-ros-gz

# Install additional packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-plugins
```

### Option B: Install on Raspberry Pi 5 (Slower but Possible)

```bash
# Install ROS2 Humble on Pi OS
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install Gazebo (lighter configuration)
sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
```

**Note:** Pi5 will run Gazebo slower. Use desktop for development, Pi5 for final validation.

---

## 🏁 WRO Track Simulation

### Step 1: Create WRO Track Environment

You need to model the WRO Future Engineers track in Gazebo.

#### Create Track SDF (Simulation Description Format)

```xml
<!-- wro_track.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="wro_track">
    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 -0.5 -1.0</direction>
    </light>

    <!-- Ground plane (track surface) -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>3 3</size> <!-- 3m x 3m track -->
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>3 3</size>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Walls (black outer boundary) -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 1.5 0.15 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>3 0.05 0.3</size></box>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>3 0.05 0.3</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Repeat for south, east, west walls -->

    <!-- Green pillar (traffic sign) -->
    <model name="green_pillar_1">
      <static>true</static>
      <pose>0.5 0.5 0.15 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient> <!-- Pure green -->
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Red pillar -->
    <model name="red_pillar_1">
      <static>true</static>
      <pose>-0.5 0.5 0.15 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient> <!-- Pure red -->
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Add more pillars as needed -->

    <!-- Physics settings -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

  </world>
</sdf>
```

**Save as:** `~/wro_ws/src/wro_simulation/worlds/wro_track.sdf`

---

### Step 2: Create Robot Model (URDF)

Model your robot with sensors.

```xml
<!-- robot.urdf.xacro -->
<?xml version="1.0"?>
<robot name="wro_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link (chassis) -->
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
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Camera plugin (Gazebo) -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>2.094</horizontal_fov> <!-- 120 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>RGB8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>~/image_raw:=/camera/image_raw</remapping>
          <remapping>~/camera_info:=/camera/camera_info</remapping>
        </ros>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_ray">
      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.2</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=/scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Wheels and differential drive (simplified) -->
  <!-- Add wheels, joints, and differential_drive_controller plugin -->

</robot>
```

---

### Step 3: Launch Simulation

Create a ROS2 launch file:

```python
# wro_simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Paths
    world_file = os.path.join(
        os.getenv('HOME'), 'wro_ws/src/wro_simulation/worlds/wro_track.sdf')
    robot_urdf = os.path.join(
        os.getenv('HOME'), 'wro_ws/src/wro_description/urdf/robot.urdf.xacro')

    return LaunchDescription([
        # Launch Gazebo with WRO track
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r ', world_file]}.items()
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'wro_robot',
                '-file', robot_urdf,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # Bridge Gazebo ↔ ROS2 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen'
        ),
    ])
```

**Launch:**
```bash
ros2 launch wro_simulation wro_simulation.launch.py
```

---

## 🧪 Testing Workflow

### Phase 1: Sensor Validation (Week 1-2)

**Goal:** Verify sensors work correctly in simulation.

1. **Camera Test:**
   ```bash
   ros2 run rqt_image_view rqt_image_view
   # Select /camera/image_raw
   # Verify you see green/red pillars
   ```

2. **LiDAR Test:**
   ```bash
   ros2 topic echo /scan
   # Verify distance readings
   ```

3. **Drive Test (Teleop):**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   # Drive robot manually, verify movement
   ```

---

### Phase 2: Vision Algorithm Testing (Week 3-5)

**Goal:** Test vision detection WITHOUT physical robot.

1. **Record Camera Frames:**
   ```bash
   ros2 bag record /camera/image_raw
   # Drive around track manually (teleop)
   # Collect diverse lighting, angles
   ```

2. **Test YOLO:**
   ```python
   # Test YOLO detection on recorded bag
   from ultralytics import YOLO
   model = YOLO('traffic_signs.pt')

   # Read bag, run detection, measure accuracy
   for image in rosbag_images:
       results = model(image)
       # Log detections
   ```

3. **Test Classical CV:**
   ```python
   # Test HSV detection on recorded bag
   for image in rosbag_images:
       hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       green_mask = cv2.inRange(hsv, lower_green, upper_green)
       # Find contours, log results
   ```

4. **Compare Accuracy:**
   - YOLO: X% accuracy
   - Classical CV: Y% accuracy
   - Choose best OR use hybrid

---

### Phase 3: Control & Navigation (Week 6-8)

**Goal:** Test autonomous navigation in simulation.

1. **Implement State Machine:**
   ```python
   class RaceController(Node):
       def __init__(self):
           self.state = 'SEARCH_FOR_SIGN'

       def update(self, detections, obstacles):
           if self.state == 'SEARCH_FOR_SIGN':
               if detections:
                   self.state = 'APPROACH_SIGN'
           elif self.state == 'APPROACH_SIGN':
               # Steer towards sign
               pass
           # ... etc
   ```

2. **Test Autonomous Laps:**
   ```bash
   ros2 launch wro_navigation autonomous.launch.py
   # Monitor progress
   ```

3. **Measure Performance:**
   - Lap time (sim time)
   - Sign detection rate
   - Collision count

---

### Phase 4: Scenario Testing (Week 9)

**Goal:** Test edge cases and failure modes.

**Test Scenarios:**

1. **Lighting Variations:**
   - Modify SDF to change ambient/diffuse lighting
   - Test: Bright, dim, shadows

2. **Sign Occlusions:**
   - Place obstacles partially blocking signs
   - Verify detection still works

3. **Surprise Rules:**
   - Add new sign types mid-track
   - Change sign meanings (green=right instead of left)
   - Test adaptability

4. **Sensor Failures:**
   - Programmatically disable camera/LiDAR
   - Verify fallback modes work

---

## 🤖 ML Training in Simulation (Proposal 3)

### Reinforcement Learning with Gazebo

If using Vision Transformer (Proposal 3), train in simulation:

#### Setup Gym Environment

```python
import gymnasium as gym
from stable_baselines3 import SAC

class WROGazeboEnv(gym.Env):
    def __init__(self):
        self.observation_space = gym.spaces.Box(
            low=0, high=255, shape=(224, 224, 3), dtype=np.uint8)
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, 0.0]),  # [steering, throttle]
            high=np.array([1.0, 1.0]),
            dtype=np.float32)

    def step(self, action):
        # Send action to Gazebo (cmd_vel)
        self.publish_cmd_vel(action)

        # Get observation (camera image)
        obs = self.get_camera_image()

        # Calculate reward
        reward = self.calculate_reward()

        # Check if done (collision or lap complete)
        done = self.check_done()

        return obs, reward, done, {}

    def reset(self):
        # Reset robot position
        # Return initial observation
        pass

# Train
env = WROGazeboEnv()
model = SAC('CnnPolicy', env, verbose=1)
model.learn(total_timesteps=1_000_000)  # 3-4 days on GPU
model.save('wro_rl_policy')
```

**Benefits:**
- Generate 1M+ training steps in simulation (impossible on real robot)
- Parallel training (run multiple Gazebo instances)
- Safe exploration (crashes don't damage hardware)

---

## 📊 Sim-to-Real Transfer

### Challenge: Simulation ≠ Reality

**Domain Gap Issues:**
- Lighting differs (sim is perfect, reality has glare/shadows)
- Colors differ (sim RGB values ≠ real camera RGB)
- Physics differs (friction, motor response)

### Solutions:

#### 1. Domain Randomization

Vary simulation parameters:

```python
# Randomize lighting
light_intensity = random.uniform(0.5, 1.5)
gazebo.set_light_intensity(light_intensity)

# Randomize colors (add noise)
pillar_color_green = np.random.normal([0, 255, 0], [10, 10, 10])

# Randomize physics
friction = random.uniform(0.5, 1.5)
gazebo.set_friction(friction)
```

Train on randomized sim → generalizes better to real world.

#### 2. Fine-Tuning on Real Data

1. Collect 1000 real images
2. Fine-tune simulated model on real data (10-20 epochs)
3. Test on real robot

#### 3. Progressive Transfer

1. Train 80% in sim
2. Test on real robot (collect failure cases)
3. Add failures to sim (augment dataset)
4. Re-train
5. Repeat

---

## 🎮 Alternative: Lightweight Simulation (Non-ROS2 Proposals)

### For Proposal 2 (Minimalist, no ROS2):

If not using ROS2, you can still test vision algorithms:

#### Option A: Synthetic Data Generation (Blender)

1. **Create 3D scene in Blender:**
   - Model WRO track
   - Model green/red pillars
   - Add camera

2. **Render 10,000+ images:**
   - Vary: Camera position, lighting, sign positions
   - Export as PNG/JPEG

3. **Test Classical CV:**
   ```python
   for img_path in synthetic_images:
       img = cv2.imread(img_path)
       detections = detect_signs_hsv(img)
       # Validate against ground truth
   ```

#### Option B: Video Replay Testing

1. **Record video** of manual driving on practice track (phone camera)
2. **Extract frames** (ffmpeg)
3. **Test algorithm** on frames
4. **Measure accuracy**

---

## 📈 Simulation Metrics

Track these metrics during simulation testing:

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Lap Completion Rate** | >95% | Successful laps / total attempts |
| **Sign Detection Accuracy** | >90% | Correct detections / total signs |
| **Collision Rate** | <5% | Collisions / total laps |
| **Average Lap Time** | <30s | Mean of successful laps |
| **Decision Latency** | <50ms | Time from camera frame to cmd_vel |

---

## 🚀 Deployment: Sim → Real

### Step-by-Step Transfer

1. **Validate in Sim:**
   - 100+ successful laps
   - >95% sign detection
   - <5% collision rate

2. **Prepare Real Robot:**
   - Build hardware
   - Flash firmware
   - Calibrate sensors

3. **Initial Real Tests (Slow Speed):**
   - Start at 0.5 m/s
   - Verify detection works
   - Check motor response

4. **Gradual Speed Increase:**
   - 0.5 → 1.0 → 1.5 → 2.0 → 2.5 m/s
   - Test at each speed level
   - Tune PID if needed

5. **Real Track Testing:**
   - 50+ test runs
   - Record ROS bags (or CSV logs)
   - Analyze failures
   - Iterate

---

## 🛠️ Recommended Tools

### Simulation
- **Gazebo Harmonic** - Physics simulation
- **RViz2** - Visualization (ROS2)
- **Blender** - Synthetic data generation

### Testing
- **ROS Bags** - Record/replay (ROS2)
- **PlotJuggler** - Plot metrics (ROS2)
- **Pytest** - Automated testing (Python)

### ML/AI
- **PyTorch** - Model training
- **Stable-Baselines3** - RL training
- **Weights & Biases** - Training monitoring

---

## 📅 Simulation Timeline

### Recommended Schedule

**Weeks 1-2: Setup**
- Install Gazebo + ROS2
- Create WRO track model
- Create robot URDF
- Test sensors

**Weeks 3-4: Vision Development**
- Test YOLO / Classical CV
- Collect synthetic training data
- Measure accuracy

**Weeks 5-6: Control Development**
- Implement state machine
- Test PID controllers
- First autonomous laps

**Weeks 7-8: Optimization**
- Speed tuning
- Failure mode testing
- Scenario coverage

**Week 9: Sim-to-Real Prep**
- Final validation
- Document learnings
- Prepare deployment

**Week 10+: Real Robot Testing**

---

## ✅ Simulation Checklist

Before building real robot:

- [ ] Gazebo simulation runs smoothly (30+ FPS)
- [ ] Camera publishes images correctly
- [ ] LiDAR provides accurate distance readings
- [ ] Robot moves in response to cmd_vel
- [ ] Vision algorithm detects green/red signs (>90% accuracy)
- [ ] State machine completes autonomous laps (>95% success)
- [ ] Collision avoidance works
- [ ] Fallback modes tested (sensor failures)
- [ ] Recorded 100+ simulated test runs
- [ ] Performance metrics documented
- [ ] Screenshots/videos for journal prepared

---

## 🎓 Learning Resources

**Gazebo:**
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [ROS2 + Gazebo Guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

**URDF Modeling:**
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

**Reinforcement Learning:**
- [Stable-Baselines3 Docs](https://stable-baselines3.readthedocs.io/)
- [OpenAI Gym Tutorial](https://gymnasium.farama.org/)

**Blender Rendering:**
- [Blender Python API](https://docs.blender.org/api/current/)

---

## 🎯 Final Recommendation

### For Proposal 4 (ROS2 Edge Racer) - RECOMMENDED:

1. **Install Gazebo + ROS2** (Weeks 1-2)
2. **Create WRO track SDF** (Week 2)
3. **Model robot URDF with sensors** (Week 2)
4. **Test vision (YOLO + Classical CV) in sim** (Weeks 3-5)
5. **Develop control logic in sim** (Weeks 6-7)
6. **Validate with 100+ sim runs** (Week 8)
7. **Build real robot** (Weeks 9-10)
8. **Transfer to real robot** (Weeks 10-11)

### For Proposal 3 (Cognitive Racer):

Same as above, PLUS:
1. **Setup Gym environment** (Week 3)
2. **Train RL policy in sim** (Weeks 6-8, 1M steps)
3. **Fine-tune on real data** (Week 10)

### For Proposal 2 (Minimalist):

Since no ROS2:
1. **Use Blender for synthetic data** (Week 2)
2. **Test classical CV on synthetic images** (Week 3)
3. **Validate with video replay** (Week 4)
4. **Build real robot** (Weeks 5-6)

---

## 🏁 Conclusion

**Simulation is NOT optional - it's critical for success!**

By testing in simulation first:
- ✅ Reduce hardware damage risk
- ✅ Iterate 10× faster
- ✅ Test edge cases systematically
- ✅ Generate impressive journal content
- ✅ Increase win probability

**Start simulating NOW - before buying any hardware!**

See `/systems` for complete proposals and timelines.

# Proposal 4: ROS2 Edge Racer - Best of All Worlds 🏆

## Executive Summary

**Philosophy:** Combine ROS2's professional ecosystem with Klevor's proven hardware stack (Pi5 + Hailo + RPi Camera) for maximum reuse, adaptability, and performance.

**Core Innovation:** ROS2 nodes running on Raspberry Pi 5 with Hailo-8L acceleration, bridging professional robotics with accessible hardware.

**Target Performance:**
- Speed: 2.6 m/s (balanced)
- Control Loop: 50 Hz (ROS2 standard for vision-based control)
- Vision Processing: 30-60 FPS (YOLO on Hailo)
- Hardware Reuse: 95% from Klevor!

---

## 1. Technology Stack

### Core Framework: ROS2 Humble
- **Middleware:** DDS for inter-node communication
- **Operating System:** Raspberry Pi OS (64-bit) with RT-PREEMPT patches
- **Build System:** Colcon
- **Languages:**
  - Python 3.11 (80%) - rapid development, node orchestration
  - C++ (20%) - performance-critical paths

**Why ROS2?**
- ✅ Professional ecosystem with mature packages
- ✅ Easy simulation testing (Gazebo)
- ✅ Modular design (swap algorithms easily)
- ✅ Strong visualization tools (RViz2, PlotJuggler)
- ✅ Behavior Trees for adaptability (surprise rules)

### Primary Compute: Raspberry Pi 5 (8GB) ✅ REUSED
- **From Klevor:** Already proven!
- **CPU:** Quad-core ARM Cortex-A76 @ 2.4 GHz
- **Sufficient for:** ROS2 + Python + light computer vision
- **Power:** 5V/5A (lower than Jetson)

### AI Accelerator: Hailo-8L ✅ REUSED
- **From Klevor:** HUGE advantage - already integrated!
- **Performance:** 13 TOPS @ 4W
- **Integration:** ONNX/TensorFlow → Hailo HEF conversion
- **Usage:** YOLO inference OR classical CV acceleration

### Camera: Raspberry Pi Camera Module 3 ✅ REUSED
- **Specs:** 12MP Sony IMX708
- **Frame Rate:** 30-60 FPS @ 640×480
- **Integration:** Native Pi5 support (libcamera, V4L2)
- **Advantage:** Hardware-accelerated ISP

### Motor Controller: Raspberry Pi Pico 2W ✅ REUSED
- **Communication:** micro-ROS bridge to ROS2
- **Protocol:** Same USB-CDC as Klevor
- **Advantage:** Seamless ROS2 integration via micro-ROS

---

## 2. Hardware Configuration (95% Reuse!)

### Sensors (All from Klevor)

#### Vision: RPi Camera Module 3 ✅
- ROS2 node: `camera_publisher` (sensor_msgs/Image)
- Frame rate: 30 FPS for YOLO, 60 FPS for classical CV

#### LiDAR: RPLiDAR C1 ✅
- ROS2 package: `rplidar_ros`
- Usage: Nav2 costmap, wall-following backup

#### IMU: BNO08X ✅
- Custom ROS2 node or existing driver
- Topic: sensor_msgs/Imu
- Integration: robot_localization EKF

#### Distance: 2-3× VL53L0X ✅
- Custom ROS2 node
- Topic: sensor_msgs/Range
- Usage: Collision prevention

#### OPTIONAL: TCS34725 Color Sensor ✅
- From Proposal 2's innovation
- Custom ROS2 node
- Topic: custom_msgs/ColorReading
- Usage: Physical sign validation (unique!)

### Actuators

#### LEGO Motor Options (Choose One)

**Option 1: LEGO SPIKE Prime Large Motor** (Recommended)
- Custom adapter board → Pico 2W
- micro-ROS node on Pico publishes:
  - nav_msgs/Odometry (from encoders)
  - Subscribes to geometry_msgs/Twist (velocity commands)

**Option 2: LEGO EV3 Large Motor** (If SPIKE unavailable)
- Same integration approach
- More community resources available

#### Steering: LEGO SPIKE Medium Angular Motor OR Digital Servo
- micro-ROS node on Pico
- Subscribes to: custom_msgs/SteeringCommand

### Power System

**Battery:** 2S LiPo (7.4V) or 3S (11.1V)
- 2S: Lighter, sufficient for Pi5 + Hailo
- 3S: Better for LEGO motors (9V nominal)

**Distribution:**
- Pi5: 5V/5A buck converter
- Hailo: Powered via M.2 from Pi5
- LEGO Motors: 9V buck (if 3S) or 7.4V direct (if 2S with voltage adapter)
- Pico: 5V/1A from Pi5 or separate buck

---

## 3. Vision & AI Strategy (Flexible!)

### Approach A: YOLO on Hailo (Recommended)

**Why YOLO?**
- Robust to lighting, occlusions, angles
- Hailo optimization: 30-60 FPS on Pi5
- Easy to train/fine-tune

**Pipeline:**
1. Camera → `/camera/image_raw`
2. ROS2 `sign_detector_node` (Python):
   - Subscribes to images
   - Runs YOLO via Hailo SDK
   - Publishes `vision_msgs/Detection2DArray`
3. ROS2 `decision_node`:
   - Subscribes to detections
   - Publishes `geometry_msgs/Twist` (velocity commands)

**Training:**
- Collect 5k+ images (manual driving + synthetic)
- Train YOLOv8-Nano or YOLOv5s
- Export to ONNX → Convert to Hailo HEF
- Expected: 30-60 FPS on Hailo

**Fallback:** Classical HSV detection (GPU-accelerated)

### Approach B: Classical CV + Hardware Validation

**From Proposal 2:**
- HSV color segmentation (120 FPS)
- TCS34725 physical validation
- Triple redundancy voting

**ROS2 Integration:**
- `color_detector_node`: HSV pipeline
- `color_sensor_node`: TCS34725 readings
- `voter_node`: 2-of-3 consensus (camera, color sensor, LiDAR position)

**Advantage:** Fastest, most explainable, unique innovation

### Hybrid Approach (Best of Both) ⭐

**Primary:** YOLO on Hailo (robust)
**Fallback:** Classical CV (fast, simple)
**Validation:** TCS34725 color sensor (unique!)

```python
# ROS2 decision node pseudo-code
if yolo_confidence > 0.7:
    use_yolo_detection()
elif classical_cv_confidence > 0.8:
    use_classical_cv()
else:
    # Uncertainty - slow down and retry
    reduce_speed()

# Physical validation (when passing sign)
if abs(camera_color - tcs34725_color) > threshold:
    log_error()
    trigger_failsafe()
```

---

## 4. ROS2 Architecture

### Node Graph

```
┌─────────────────┐
│  Camera Driver  │ → /camera/image_raw
└─────────────────┘
         ↓
┌─────────────────┐
│ Sign Detector   │ → /detections (YOLO/Classical CV)
│ (Hailo/GPU)     │
└─────────────────┘
         ↓
┌─────────────────┐
│ Decision Node   │ → /cmd_vel
│ (State Machine) │
└─────────────────┘
         ↓
┌─────────────────┐
│  micro-ROS      │ → Pico 2W (USB-CDC)
│  Bridge         │
└─────────────────┘
         ↓
┌─────────────────┐
│ LEGO Motors     │
└─────────────────┘

┌─────────────────┐
│ RPLiDAR Driver  │ → /scan (LaserScan)
└─────────────────┘
         ↓
┌─────────────────┐
│ Obstacle Avoid  │ → /obstacles
└─────────────────┘

┌─────────────────┐
│ IMU + Encoders  │ → /odom, /imu
└─────────────────┘
         ↓
┌─────────────────┐
│ robot_          │ → /odometry/filtered (EKF fusion)
│ localization    │
└─────────────────┘
```

### Key ROS2 Packages

1. **Nav2** (Optional - for complex path planning)
   - May be overkill for fixed track
   - Use if adaptability to surprise rules is priority

2. **robot_localization** (Recommended)
   - EKF sensor fusion (IMU + encoders + LiDAR)
   - Provides accurate odometry

3. **vision_msgs** (Standard)
   - Detection2D for YOLO outputs

4. **rplidar_ros** (Available)
   - Driver for RPLiDAR C1

5. **micro_ros_agent** (Critical)
   - Bridge between ROS2 and Pico 2W

---

## 5. Performance Optimizations

### Speed: 2.6 m/s Target

**Adaptive Speed Controller:**
```python
class AdaptiveSpeedNode(Node):
    def __init__(self):
        self.max_speed = 2.6  # m/s
        self.current_speed = 0.0

    def calculate_target_speed(self, detections, obstacles):
        # Start with max speed
        target = self.max_speed

        # Reduce if sign detected (preparing to turn)
        if detections and detections[0].distance < 2.0:
            target = min(target, 2.0)

        # Reduce if obstacle close
        if obstacles.min_distance < 1.0:
            target = min(target, 1.5)

        # Smooth acceleration
        return self.smooth_accel(target)
```

### Decision Latency: ~20-30ms

**ROS2 Performance Tips:**
- Use intra-process communication (zero-copy)
- Real-time executor priority settings
- Lock-free data structures where possible

```python
# Intra-process optimization
options = NodeOptions()
options.use_intra_process_comms(True)
node = SignDetectorNode(options)
```

### Reliability: Multi-Sensor Fusion

**robot_localization EKF:**
- Fuses IMU, wheel odometry, LiDAR SLAM
- Provides robust state estimate

**Redundancy:**
- YOLO + Classical CV fallback
- TCS34725 validation
- LiDAR wall-following backup

---

## 6. Documentation Approach

### Engineer's Journal: ROS2 Showcase

**Leverage ROS2 Tools:**

1. **ROS Bag Recording:**
   ```bash
   ros2 bag record -a -o test_run_$(date +%s)
   ```
   - Record ALL topics during test runs
   - Replay for analysis and visualization

2. **RViz2 Visualizations:**
   - Real-time 3D view (camera, LiDAR, detections, path)
   - Screenshot for journal
   - Shows professional-grade development

3. **PlotJuggler Graphs:**
   - Plot speed, steering, detection confidence over time
   - Include performance graphs in journal

4. **rqt_graph:**
   - Node graph diagram → include in architecture section
   - Shows professional software engineering

### Journal Structure (50+ pages)

1. **Introduction**
   - Problem statement
   - ROS2 choice justification

2. **Hardware Architecture**
   - Component diagram
   - 95% Klevor reuse explanation
   - LEGO motor integration

3. **Software Architecture**
   - ROS2 node graph (rqt_graph export)
   - Package descriptions
   - State machine diagram

4. **Vision Pipeline**
   - YOLO training process (if used)
   - Classical CV algorithm (if used)
   - Attention maps or detection examples

5. **Testing & Results**
   - 50+ test runs data
   - ROS bag analysis
   - Performance graphs (PlotJuggler)
   - Lap time statistics

6. **Innovations**
   - TCS34725 color sensor validation (unique!)
   - ROS2 on Pi5 + Hailo (rare combination)
   - Hybrid YOLO + Classical CV

---

## 7. Differentiation

### Unique Advantages

1. **ROS2 on Pi5 + Hailo** 🚀
   - Rare combination (most ROS2 robots use Jetson)
   - Shows creative problem-solving
   - Cost-effective professional stack

2. **95% Klevor Reuse** ♻️
   - Fastest development (proven hardware)
   - Reduces risk significantly
   - Shows engineering efficiency

3. **Hardware Color Sensor Validation** 🎨
   - From Proposal 2 innovation
   - No other team will have this
   - Easy to explain, impressive

4. **Professional Development Tools** 🛠️
   - ROS bags for reproducible testing
   - RViz2 visualization
   - Simulation capability (Gazebo)

5. **Flexibility** 🔄
   - Can switch between YOLO and Classical CV
   - Easy to add new behaviors (surprise rules)
   - Modular ROS2 nodes

### vs. Other Proposals

| Feature | ROS2 Edge (This) | Minimalist | Cognitive | Velocity Edge |
|---------|-----------------|------------|-----------|---------------|
| **Hardware Reuse** | 95% | 80% | 90% | 60% |
| **Dev Speed** | Fast | Fastest | Slow | Medium |
| **Adaptability** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Cost** | Low | Low | Low | High |
| **Professional Appeal** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Risk** | Low-Medium | Low | High | Medium |

---

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| ROS2 learning curve | Medium | Medium | Extensive tutorials, start with simple nodes, 2-week learning buffer |
| Hailo YOLO conversion | Medium | Medium | Test ONNX→Hailo early (Week 2), fallback to Classical CV |
| micro-ROS setup | Low | Medium | Well-documented, Pico examples available |
| Performance on Pi5 | Low | Medium | ROS2 runs fine on Pi4, Pi5 is more powerful |
| LEGO motor integration | Low | Medium | Custom adapter PCB, test early (Week 2) |

---

## 9. Implementation Plan (11 weeks)

### Weeks 1-2: ROS2 Setup
- [ ] Install ROS2 Humble on Pi5
- [ ] Create workspace, hello-world nodes
- [ ] micro-ROS on Pico 2W
- [ ] LEGO motor adapter board design + fabrication

### Weeks 3-4: Sensor Integration
- [ ] Camera node (libcamera → ROS2 Image)
- [ ] RPLiDAR driver (rplidar_ros)
- [ ] IMU node (custom or existing driver)
- [ ] VL53L0X node (custom Range publisher)
- [ ] TCS34725 node (optional but recommended)

### Weeks 5-6: Vision Pipeline
- **Path A (YOLO):**
  - [ ] Collect/generate training data (5k+ images)
  - [ ] Train YOLOv8-Nano
  - [ ] Export ONNX → Hailo HEF
  - [ ] ROS2 detector node

- **Path B (Classical CV):**
  - [ ] Implement HSV detection (GPU-accelerated)
  - [ ] Multi-frame averaging
  - [ ] ROS2 detector node

- **Both Paths:**
  - [ ] Fallback mechanism
  - [ ] TCS34725 validation logic

### Weeks 7-8: Control & Navigation
- [ ] State machine (BehaviorTree or simple FSM)
- [ ] Decision node (detections → cmd_vel)
- [ ] Speed controller (adaptive)
- [ ] robot_localization EKF setup
- [ ] Emergency stop / collision avoidance

### Week 9: Integration & First Laps
- [ ] Connect all nodes
- [ ] Launch file for full system
- [ ] First autonomous lap 🎉
- [ ] Debug and iterate

### Week 10: Testing Marathon
- [ ] 50+ test runs
- [ ] Record ROS bags
- [ ] Analyze performance
- [ ] Fix issues, optimize

### Week 11: Documentation
- [ ] Engineer's Journal (50+ pages)
- [ ] Export RViz screenshots
- [ ] Generate PlotJuggler graphs
- [ ] Code cleanup
- [ ] Backup robot prep

---

## 10. Critical Files & Structure

```
teamsteelbot_ros2/
├── src/
│   ├── teamsteelbot_bringup/
│   │   ├── launch/
│   │   │   └── race.launch.py              # Main launch file
│   │   └── config/
│   │       ├── camera.yaml
│   │       ├── ekf.yaml                    # robot_localization config
│   │       └── controllers.yaml
│   │
│   ├── teamsteelbot_vision/
│   │   ├── teamsteelbot_vision/
│   │   │   ├── sign_detector_yolo.py      # YOLO node (Hailo)
│   │   │   ├── sign_detector_classic.py   # Classical CV node
│   │   │   └── color_sensor_node.py       # TCS34725 validation
│   │   └── package.xml
│   │
│   ├── teamsteelbot_control/
│   │   ├── teamsteelbot_control/
│   │   │   ├── decision_node.py           # State machine + logic
│   │   │   ├── speed_controller.py        # Adaptive speed
│   │   │   └── voter_node.py              # Multi-sensor voting
│   │   └── package.xml
│   │
│   ├── teamsteelbot_sensors/
│   │   ├── teamsteelbot_sensors/
│   │   │   ├── bno08x_node.py            # IMU driver
│   │   │   └── vl53l0x_node.py           # Distance sensors
│   │   └── package.xml
│   │
│   └── teamsteelbot_msgs/
│       ├── msg/
│       │   ├── ColorReading.msg           # TCS34725 custom message
│       │   └── SignDetection.msg          # Custom detection format
│       └── package.xml
│
├── firmware/
│   └── pico_2w_microros/                  # micro-ROS + motor control
│       ├── main.cpp
│       ├── motor_driver.cpp
│       └── usbcdc_bridge.cpp
│
├── models/                                 # AI models (if using YOLO)
│   ├── yolov8n_traffic_signs.onnx
│   └── yolov8n_traffic_signs.hef         # Hailo format
│
├── docs/
│   ├── journal/
│   ├── hardware/
│   └── tutorials/
│
└── README.md
```

---

## 11. Verification & Testing

### Unit Testing
- **ROS2 Launch Tests:** Automated node startup checks
- **Vision Testing:** Static image dataset (50+ images)
- **Control Testing:** Simulated inputs → verify outputs

### Integration Testing
- **Sensor Fusion:** Verify EKF combines IMU + encoders + LiDAR
- **End-to-End:** Camera → Decision → Motors (full pipeline)
- **Failover:** Simulate sensor failures, verify fallbacks

### Performance Testing
- **Lap Time:** Target <30s (competitive), <25s (top tier)
- **Detection Accuracy:** >90% (all lighting conditions)
- **Latency:** Camera → Motor command <50ms

### ROS Bag Analysis
```bash
# Record test run
ros2 bag record -a

# Analyze later
ros2 bag info test_run.db3
ros2 bag play test_run.db3
```

---

## 12. Success Criteria

### Minimum (Competition Entry)
- ✅ Completes 1 autonomous lap
- ✅ Sign detection >70% accuracy
- ✅ ROS2 node graph functional
- ✅ Journal documents architecture

### Competitive (Top 40%)
- ✅ Completes 5/5 laps
- ✅ Sign detection >85% accuracy
- ✅ Lap time <30s
- ✅ Journal shows ROS2 proficiency

### Winning (Top 15%)
- ✅ Completes 10/10 laps
- ✅ Sign detection >92% accuracy
- ✅ Lap time <25s
- ✅ Journal: 50+ pages, ROS visualizations, unique TCS34725 innovation
- ✅ Demonstrates ROS2 + Hailo integration (rare!)

---

## 13. Why This Proposal Wins 🏆

### Quantitative Advantages
1. **Maximum Reuse:** 95% Klevor hardware → fastest development
2. **Cost Effective:** ~$400 total (vs $800 for Jetson setup)
3. **Speed:** 2.6 m/s (balanced - not too risky, not too slow)
4. **Professional Tools:** ROS2 ecosystem (bags, visualization, simulation)

### Qualitative Advantages
1. **Unique Combination:** ROS2 + Pi5 + Hailo (rare, creative)
2. **Hardware Innovation:** TCS34725 color sensor (no one else has this)
3. **Flexibility:** Can pivot between YOLO and Classical CV
4. **Documentation:** ROS tools make impressive journal easy (RViz, bags, plots)

### Risk Profile
- **Low Hardware Risk:** 95% proven from Klevor
- **Low Schedule Risk:** Fast development (11 weeks), mature tools
- **Medium Learning Risk:** ROS2 learning curve (mitigated by extensive resources)
- **High Win Probability:** Balanced approach with professional presentation

### Judge Appeal
- **Professional Stack:** ROS2 is industry standard
- **Innovation:** Pi5 + Hailo + ROS2 integration
- **Explainable:** Clear node graph, easy to understand
- **Visual Impact:** RViz demonstrations, PlotJuggler graphs
- **Unique Validation:** TCS34725 physical color checking

---

## Conclusion

**ROS2 Edge Racer** is the optimal hybrid approach:
- **Best Hardware Reuse:** 95% from proven Klevor stack
- **Professional Ecosystem:** ROS2 for adaptability and documentation
- **Cost Effective:** Raspberry Pi 5 instead of expensive Jetson
- **Unique Innovation:** TCS34725 color sensor validation
- **Flexible:** Choose YOLO or Classical CV (or both!)
- **Fast Development:** 11 weeks (2-week testing buffer)

This proposal combines:
- Proposal 1's ROS2 ecosystem and professional tools
- Proposal 2's hardware innovation (TCS34725) and simplicity
- Proposal 3's Hailo integration and proven hardware stack

**Best For:**
- Teams with some Python/ROS experience (or willing to learn)
- Want professional robotics framework without expensive hardware
- Value modularity and adaptability for surprise rules
- Want impressive documentation tools (RViz, bags, graphs)

**Win Probability:** ⭐⭐⭐⭐⭐ (Highest overall - balanced risk/reward!)

**Let's build this! 🚀🏁**

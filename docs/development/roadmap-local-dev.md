# Development Roadmap - Local Laptop Phase

## Overview

This roadmap guides you through developing the ROS2 Edge Racer on your laptop (WSL2) before porting to RPi5. Each phase builds on the previous one and can be tested independently.

**Total Timeline:** 6-8 weeks on laptop → 3-4 weeks on RPi5

---

## Phase 1: Foundation (Week 1)

### Goals
- ROS2 Kilted environment working
- Understand ROS2 basics
- Simple pub/sub working
- Experience 10x faster Python executor!

### Tasks

#### Day 1-2: Installation
- [ ] Ensure WSL2 is Ubuntu 24.04 (required for Kilted)
- [ ] Run `bash scripts/setup_wsl2_dev.sh`
- [ ] Verify ROS2 Kilted: `ros2 --version`
- [ ] Test Gazebo Ionic: `gz sim empty.sdf`
- [ ] Read: `docs/development/ROS2-KILTED-UPDATE.md`
- [ ] Complete ROS2 beginner tutorials (2-3 hours)

#### Day 3-5: First Nodes
- [ ] Create hello world publisher/subscriber
- [ ] Create mock camera node (publishes test images)
- [ ] Visualize in RViz2
- [ ] Understand ROS2 topics, nodes, QoS

#### Day 6-7: Custom Messages
- [ ] Define `SignDetection.msg`
- [ ] Define `RobotState.msg`
- [ ] Build and test custom messages
- [ ] Create simple echo nodes

### Deliverables
- ✅ ROS2 workspace builds successfully
- ✅ Mock camera publishes images at 30 Hz
- ✅ Can visualize topics in RViz2
- ✅ Understand basic ROS2 concepts

### Code to Write
```
teamvoldemor_simulation/
├── mock_camera_node.py       # Publishes test images
└── test_subscriber_node.py   # Echoes received data

teamvoldemor_msgs/
├── msg/SignDetection.msg
└── msg/RobotState.msg
```

---

## Phase 2: Vision Pipeline (Week 2-3)

### Goals
- Classical CV sign detection working
- Process images in real-time
- Publish detection results

### Tasks

#### Classical CV Detector (Week 2)
- [ ] HSV color thresholding for red/green/blue
- [ ] Contour detection and filtering
- [ ] Bounding box extraction
- [ ] Confidence scoring
- [ ] ROS2 node that subscribes to camera, publishes detections

#### Testing (Week 2)
- [ ] Create test image dataset (20+ images with signs)
- [ ] Unit tests for color detection
- [ ] Measure FPS (target: 30+ on laptop, 60+ possible)
- [ ] Test different lighting conditions

#### YOLO26 Detector (Week 3 - Recommended) 🚀
- [ ] Generate synthetic training images (1000 in 2 minutes!)
- [ ] Optional: Collect/label real images with Roboflow (100-500 for fine-tuning)
- [ ] Train **YOLO26-nano** (FASTEST on your RTX 4050! 43% faster than YOLO11!)
- [ ] Export to ONNX
- [ ] ROS2 node with ONNX Runtime (GPU-accelerated)
- [ ] Compare Classical CV vs YOLO26 performance
- [ ] Expected: 250-300 FPS on laptop, 15-20 FPS on RPi5 CPU
- [ ] Monitor Hailo Model Zoo for YOLO26 support (expected March-April 2026)

### Deliverables
- ✅ Vision node detects signs in static images
- ✅ Runs at 30+ FPS on laptop (250+ FPS with YOLO26!)
- ✅ Publishes Detection2DArray messages
- ✅ Trained YOLO26 model with 40.9 mAP (better than YOLO11's 39.5)

### Code to Write
```
teamvoldemor_vision/
├── sign_detector_classic.py  # HSV-based detector
├── sign_detector_yolo.py     # YOLO-based (optional)
├── detector_base.py          # Abstract base class
└── utils/
    ├── color_spaces.py       # HSV ranges, conversions
    └── geometry.py           # Bounding box math
```

### Test Data Structure
```
datasets/
├── test_images/
│   ├── red_sign_01.jpg
│   ├── green_sign_01.jpg
│   └── blue_sign_01.jpg
├── test_images_labeled/      # For YOLO training
│   ├── images/
│   └── labels/
└── test_results/              # Output with bounding boxes
```

---

## Phase 3: Decision & Control (Week 3-4)

### Goals
- State machine for race logic
- Decision node responds to detections
- Speed control implemented

### Tasks

#### State Machine (Week 3)
- [ ] Define states: IDLE, RACING, TURNING_LEFT, TURNING_RIGHT, STOPPING
- [ ] Implement transitions based on detections
- [ ] Add timeout logic (if no detection for X seconds)
- [ ] Unit tests for state transitions

#### Decision Node (Week 4)
- [ ] Subscribe to sign detections
- [ ] Publish velocity commands (Twist messages)
- [ ] Implement decision logic:
  - Red sign → turn right
  - Green sign → turn left
  - Blue sign → increase speed (or other action)
- [ ] Speed controller with acceleration limits

#### Mock Motor Controller (Week 4)
- [ ] Simple node that receives Twist commands
- [ ] Logs commands to console (for testing)
- [ ] Later: Replace with micro-ROS bridge to real Pico

### Deliverables
- ✅ State machine with full test coverage
- ✅ Decision node responds correctly to sign detections
- ✅ Velocity commands generated at 50 Hz
- ✅ Smooth acceleration/deceleration

### Code to Write
```
teamvoldemor_control/
├── state_machine.py          # FSM implementation
├── decision_node.py          # Main decision logic
├── speed_controller.py       # Adaptive speed with limits
└── tests/
    └── test_state_machine.py # Unit tests
```

---

## Phase 4: Sensor Fusion (Week 4-5)

### Goals
- Mock sensor nodes (LiDAR, IMU, distance sensors)
- Sensor fusion with robot_localization
- Odometry estimation

### Tasks

#### Mock Sensors (Week 4)
- [ ] Mock LiDAR (publishes LaserScan with simulated walls)
- [ ] Mock IMU (publishes Imu messages with simulated acceleration)
- [ ] Mock distance sensors (publishes Range messages)

#### robot_localization EKF (Week 5)
- [ ] Configure EKF to fuse IMU + odometry
- [ ] Tune noise parameters
- [ ] Publish filtered odometry
- [ ] Visualize in RViz2 (show robot pose estimate)

#### Optional: Nav2 Integration
- [ ] Set up Nav2 costmap (if doing dynamic obstacle avoidance)
- [ ] May be overkill for fixed track - decide based on rules

### Deliverables
- ✅ Mock sensors publish realistic data
- ✅ EKF provides filtered odometry
- ✅ Can visualize robot state in RViz2
- ✅ Understand coordinate transforms (tf2)

### Code to Write
```
teamvoldemor_sensors/
├── mock_lidar_node.py
├── mock_imu_node.py
└── mock_distance_sensors_node.py

teamvoldemor_bringup/
└── config/
    └── ekf_config.yaml       # robot_localization parameters
```

---

## Phase 5: Integration & Simulation (Week 5-6)

### Goals
- All nodes running together
- Complete simulation in Gazebo (or mock environment)
- End-to-end testing

### Tasks

#### Launch Files (Week 5)
- [ ] Create `simulation.launch.py` (starts all nodes)
- [ ] Parameter files for easy tuning
- [ ] RViz2 configuration saved
- [ ] Test full pipeline: camera → vision → decision → motors

#### Simple Track Simulation (Week 5-6)
**Option A: Full Gazebo World**
- [ ] Create SDF world with track and signs
- [ ] Spawn robot model
- [ ] Camera sees simulated signs

**Option B: Mock Track (Simpler, Recommended)**
- [ ] Script that generates images of signs at different positions
- [ ] Simulates robot moving through track
- [ ] Faster iteration than full Gazebo

#### End-to-End Testing (Week 6)
- [ ] Run 10+ simulated laps
- [ ] Record ROS bags
- [ ] Analyze with PlotJuggler (speed, detections, decisions)
- [ ] Measure lap times (virtual)
- [ ] Debug and optimize

### Deliverables
- ✅ Single launch command starts entire system
- ✅ Simulated robot completes full lap
- ✅ ROS bags recorded for analysis
- ✅ Performance graphs generated
- ✅ Code ready for RPi5 port!

### Code to Write
```
teamvoldemor_bringup/
├── launch/
│   ├── simulation.launch.py       # All mock sensors + logic
│   └── visualization.launch.py    # RViz2 + PlotJuggler
└── config/
    ├── simulation_params.yaml
    └── rviz_config.rviz

teamvoldemor_simulation/
├── track_simulator.py             # Generates mock track images
└── worlds/
    └── race_track.sdf              # (If using Gazebo)
```

---

## Phase 6: Optimization & Features (Week 6-7)

### Goals
- Performance tuning
- Add unique features (TCS34725 color validation logic)
- Prepare for hardware port

### Tasks

#### Performance Optimization
- [ ] Profile with `ros2 topic hz` (measure actual rates)
- [ ] Optimize vision pipeline (reduce latency)
- [ ] Tune PID controllers (if using)
- [ ] Test edge cases (no detections, multiple detections)

#### Unique Features
- [ ] Implement TCS34725 validation logic (mock sensor for now)
- [ ] Voting system (camera + color sensor agreement)
- [ ] Fallback behaviors (wall-following with LiDAR)

#### Documentation (Start Engineer's Journal)
- [ ] Document architecture (node graph, state machine)
- [ ] Record design decisions
- [ ] Take screenshots of RViz, graphs
- [ ] Write algorithm explanations

### Deliverables
- ✅ System runs reliably for 30+ minutes
- ✅ Unique validation feature implemented
- ✅ Journal started (20+ pages drafted)
- ✅ Code is clean and commented

---

## Phase 7: Hardware Preparation (Week 7-8)

### Goals
- Prepare code for RPi5 port
- Test hardware components individually
- Plan integration strategy

### Tasks

#### Code Refactoring
- [ ] Separate simulation vs hardware in launch files
- [ ] Create hardware abstraction layer
- [ ] Document hardware-specific TODOs
- [ ] Test that same code runs on RPi5 (without sensors first)

#### Hardware Testing (if you have RPi5 available)
- [ ] Test libcamera on RPi5
- [ ] Test Hailo SDK (ONNX → HEF conversion)
- [ ] Test RPLiDAR driver
- [ ] Test I2C sensors (IMU, VL53L0X, TCS34725)

#### Transfer Plan
- [ ] Git repository for code sync (GitHub/GitLab)
- [ ] Or: SCP/rsync scripts
- [ ] Document differences between laptop and RPi5 setup

### Deliverables
- ✅ Code is portable (simulation vs hardware)
- ✅ Hardware components tested individually
- ✅ Clear plan for integration
- ✅ Laptop development phase complete!

---

## Phase 8: Port to RPi5 (Week 9-11) - Not on Laptop!

This happens AFTER laptop development is solid.

### Quick Port Checklist
- [ ] Transfer workspace to RPi5
- [ ] Install ROS2 Humble on RPi5
- [ ] Replace mock nodes with hardware nodes:
  - `mock_camera_node.py` → Real camera (libcamera)
  - `mock_lidar_node.py` → `rplidar_ros`
  - `mock_imu_node.py` → BNO08X driver
- [ ] Set up micro-ROS bridge to Pico 2W
- [ ] Test sensors individually
- [ ] Test vision with Hailo acceleration
- [ ] Full integration testing
- [ ] Track testing (50+ laps)
- [ ] Final journal documentation

---

## Development Best Practices

### Daily Workflow
```bash
# Morning: Pull latest code
cd ~/teamvoldemor_ws
git pull

# Build
colcon build --symlink-install
source install/setup.bash

# Run tests
colcon test
colcon test-result --verbose

# Develop (edit Python files, they reload automatically with --symlink-install)

# Test individual node
ros2 run teamvoldemor_vision sign_detector_classic

# Test full system
ros2 launch teamvoldemor_bringup simulation.launch.py

# Evening: Record results
ros2 bag record -a -o test_run_$(date +%s)

# Commit changes
git add .
git commit -m "Implemented X feature"
git push
```

### Testing Strategy
1. **Unit Tests:** Test functions in isolation (pytest)
2. **Node Tests:** Test nodes with mock data (launch_testing)
3. **Integration Tests:** Test multiple nodes together (ROS bags)
4. **System Tests:** Full pipeline in simulation

### Git Workflow
```bash
# Feature branches
git checkout -b feature/vision-detector
# ... work ...
git commit -m "Add HSV color detection"
git push origin feature/vision-detector
git checkout main
git merge feature/vision-detector
```

---

## Key Milestones & Demos

### Milestone 1 (End of Week 2)
**Demo:** Vision node detects signs in test images
- Show RViz with camera feed + bounding boxes
- Measure detection accuracy on 20 test images

### Milestone 2 (End of Week 4)
**Demo:** Decision node responds to detections
- Mock camera publishes sign detections
- Decision node outputs correct velocity commands
- Show state machine transitions

### Milestone 3 (End of Week 6)
**Demo:** Full simulation lap
- Launch entire system
- Robot completes virtual lap
- Show ROS bag playback with PlotJuggler graphs

### Milestone 4 (End of Week 8)
**Demo:** Hardware-ready code
- Same code runs on laptop and RPi5 (with config changes)
- Show side-by-side comparison

---

## Resources for Learning

### ROS2 Tutorials (Start Here!)
1. **Beginner:** https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html
   - Topics, nodes, services (2 hours)
2. **Intermediate:** https://docs.ros.org/en/humble/Tutorials/Intermediate.html
   - Launch files, parameters, actions (3 hours)
3. **Advanced:** Custom messages, QoS, tf2 (ongoing)

### Computer Vision
- OpenCV tutorials: https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
- YOLOv8 docs: https://docs.ultralytics.com/

### ROS2 Tools
- **rqt_graph:** Visualize node connections
- **PlotJuggler:** Plot time-series data from bags
- **RViz2:** 3D visualization
- **ros2 bag:** Record and replay

### Example Code
- ROS2 examples: https://github.com/ros2/examples
- Nav2 getting started: https://navigation.ros.org/

---

## Success Criteria (Laptop Phase)

### Minimum Success
- [ ] ROS2 environment set up
- [ ] Vision detects signs (>70% accuracy on test images)
- [ ] Decision node makes correct decisions
- [ ] All nodes communicate successfully

### Target Success (Ready for RPi5 Port)
- [ ] Vision detects signs (>85% accuracy)
- [ ] Simulated robot completes full lap
- [ ] ROS bags recorded and analyzed
- [ ] Code is modular and well-documented
- [ ] Unique features implemented (TCS34725 logic)

### Stretch Goals
- [ ] YOLO model trained and working
- [ ] Full Gazebo simulation with 3D track
- [ ] Multiple lap strategies tested
- [ ] Simulation matches expected physical performance

---

## When to Move to RPi5?

**Don't rush to hardware!** Stay on laptop until:

1. ✅ Vision detection works reliably (>85% accuracy)
2. ✅ Decision logic is solid (tested with 100+ scenarios)
3. ✅ Simulated laps complete successfully
4. ✅ Code is modular (easy to swap sim for hardware)
5. ✅ You understand ROS2 well (comfortable debugging)

**Hardware integration will be 10x faster if software is already working!**

---

## Estimated Time Breakdown

| Phase | Time | Cumulative | Key Activities |
|-------|------|------------|----------------|
| 1. Foundation | 1 week | 1 week | ROS2 setup, basic nodes |
| 2. Vision | 2 weeks | 3 weeks | Detection algorithm |
| 3. Control | 1.5 weeks | 4.5 weeks | State machine, decisions |
| 4. Sensors | 1 week | 5.5 weeks | Fusion, odometry |
| 5. Integration | 1.5 weeks | 7 weeks | Full simulation |
| 6. Optimization | 1 week | 8 weeks | Tuning, features |
| 7. Hardware Prep | 1 week | 9 weeks | Port preparation |
| **LAPTOP TOTAL** | **8-9 weeks** | | **Ready for RPi5!** |

Then 3-4 weeks on RPi5 for hardware integration and testing.

**Total project: 11-13 weeks (leaves 2+ week buffer for competition)**

---

## Questions? Stuck?

### Debug Checklist
1. Is ROS2 sourced? `echo $ROS_DISTRO` (should say "humble")
2. Are nodes running? `ros2 node list`
3. Are topics publishing? `ros2 topic hz /camera/image_raw`
4. Check logs: `ros2 run <package> <node> --ros-args --log-level debug`

### Common Issues
- **"Package not found":** Did you build? `colcon build`
- **"Topic not updating":** Check QoS settings (reliability, durability)
- **"Node crashes immediately":** Check Python imports, dependencies

### Get Help
- ROS2 Discord: https://discord.gg/ros
- ROS Answers: https://answers.ros.org/
- Stack Overflow: Tag `ros2`

---

**You're ready to start! Begin with Phase 1, Day 1. 🚀**

**Pro tip:** Focus on getting one thing working at a time. ROS2 has a learning curve, but it's worth it for the modularity and professional ecosystem!

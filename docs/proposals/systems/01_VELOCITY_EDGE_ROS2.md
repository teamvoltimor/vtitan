# Proposal 1: Velocity Edge - ROS2 Performance Architecture

## Executive Summary

**Philosophy:** Leverage the battle-tested ROS2 ecosystem with cutting-edge hardware for maximum performance, adaptability, and professional-grade autonomy.

**Core Technology:** NVIDIA Jetson Orin Nano + ROS2 Humble + Stereo Vision + YOLOv8

**Target Performance:**
- Speed: 2.5 m/s (aggressive but safe)
- Control Loop: 100 Hz (ROS2 standard)
- Vision Processing: 60+ FPS (YOLO inference)
- Adaptability: Best-in-class for surprise rules

---

## 1. Architecture & Technology Stack

### Core Framework: ROS2 Humble
- **Middleware:** DDS (Data Distribution Service) for inter-node communication
- **Operating System:** Ubuntu 22.04 with RT-PREEMPT kernel patches
- **Navigation Stack:** Nav2 for path planning and obstacle avoidance
- **Build System:** Colcon with CMake/Python setuptools

**Why ROS2?**
- **Proven:** Industry standard for autonomous robots (self-driving cars, drones, industrial)
- **Ecosystem:** Thousands of packages (SLAM, planning, control, visualization)
- **Simulation:** Gazebo integration for rapid testing without hardware
- **Community:** Massive support, tutorials, and troubleshooting resources
- **Modularity:** Easy to swap algorithms (e.g., try different path planners)

### Primary Compute: NVIDIA Jetson Orin Nano (8GB)
- **CPU:** 6-core ARM Cortex-A78AE @ 2.0 GHz
- **GPU:** 1024-core NVIDIA Ampere with 32 Tensor Cores
- **AI Performance:** 40 TOPS (vs Hailo's 26 TOPS)
- **Memory:** 8GB LPDDR5 (shared CPU/GPU)
- **Power:** 7W - 15W (configurable)
- **Size:** 70×45×31mm (compact!)

**Advantages over RPi5:**
- 3× better AI performance
- Native CUDA support (all ML frameworks)
- Better ROS2 integration and support
- Hardware-accelerated video encoding/decoding

### Motor Controller: Raspberry Pi Pico 2W
- **Reused from Klevor:** Proven reliability
- **Communication:** micro-ROS bridge to ROS2 (USB-CDC)
- **Dual-core:** One core for motor control, one for micro-ROS
- **PIO:** Programmable I/O for precise encoder reading

### Languages
- **C++:** Performance-critical nodes (vision, control)
- **Python:** Rapid prototyping, non-critical nodes
- **Ratio:** 70% C++, 30% Python

---

## 2. Hardware Configuration

### Sensors

#### Vision: OV9281 Stereo Camera Pair (Global Shutter)
- **Specs:** 1MP (1280×800), 120 FPS, global shutter
- **Interface:** USB 3.0 (high bandwidth)
- **Baseline:** 60mm stereo separation
- **Key Feature:** Global shutter eliminates motion blur at high speeds
  - Rolling shutter causes distortion when moving fast
  - Global shutter captures entire frame simultaneously
  - Critical for 2.5 m/s speeds!

**Usage:**
- **Left Camera:** RGB image for YOLO sign detection
- **Stereo Pair:** Depth map for obstacle avoidance
- **Frame Rate:** 30 FPS (sufficient for 2.5 m/s)

#### LiDAR: RPLiDAR C1 (Reused from Klevor)
- **Why keep it?** Proven 2D navigation, wall-following backup
- **Redundancy:** Validates stereo depth, provides 360° awareness
- **Integration:** ROS2 driver available (rplidar_ros)

#### IMU: BNO08X (Reused from Klevor)
- **Why keep it?** Excellent sensor fusion, reliable orientation
- **Integration:** Custom ROS2 node (or use existing I2C library)
- **Usage:** robot_localization package for EKF sensor fusion

#### Distance: 3× VL53L0X Time-of-Flight (Reused)
- **Mounting:** Front-left, front-center, front-right (45° angles)
- **Usage:** Short-range collision prevention (<2m)
- **Integration:** Custom ROS2 node publishing PointCloud2

#### Encoders: High-Resolution Rotary Encoders
- **Specs:** 2048 CPR (counts per revolution)
- **Mounting:** Drive wheels (odometry)
- **Integration:** micro-ROS on Pico 2W publishes Odometry messages
- **Usage:** robot_localization EKF fusion with IMU + depth + lidar

### Actuators

#### **LEGO Motor Options (Recommended)** 🧱

##### Option 1: LEGO SPIKE Prime Large Motor (Recommended for Drive)
- **Power:** 8.5V nominal, stall torque 25 N·cm
- **Speed:** 175 RPM no-load
- **Encoder:** Integrated absolute encoder (360° per rotation)
- **Interface:** UART or I2C (via custom adapter to Pico 2W)
- **Advantages:**
  - Bulletproof reliability (designed for student abuse)
  - Precise position/speed feedback built-in
  - Metal gearbox (no stripping)
  - Easy mounting with LEGO Technic holes

**Integration:**
- Adapter board: LEGO motor connector → Pico 2W UART/I2C
- ROS2 messages: cmd_vel → Pico 2W → LEGO motor
- Odometry feedback: Encoder → Pico 2W → ROS2 Odometry

##### Option 2: LEGO Technic XL Motor (Alternative)
- **Power:** 9V nominal, higher torque than SPIKE
- **Speed:** ~110 RPM no-load
- **Encoder:** External (or use optical sensor on wheel)
- **Interface:** Custom PWM driver
- **Advantages:** More torque for heavier robots, widely available

##### Option 3: LEGO EV3 Large Servo Motor
- **Power:** 9V nominal, 20 N·cm running torque
- **Speed:** 160-170 RPM
- **Encoder:** Integrated rotary encoder (360 CPR)
- **Interface:** Custom adapter (6-pin RJ12 connector)
- **Advantages:**
  - Massive EV3 community support
  - Known performance characteristics
  - Cheap and abundant on secondary market

**Recommendation:** **LEGO SPIKE Prime Large Motor** for best balance of power, precision, and modern support.

#### Steering: LEGO Servo Motor or Digital Servo

##### Option A: LEGO SPIKE Medium Angular Motor (Servo Mode)
- **Precision:** Absolute positioning via encoder
- **Speed:** 250 RPM (fast response)
- **Torque:** 10 N·cm (sufficient for steering)
- **Integration:** Same UART/I2C adapter as drive motor

##### Option B: Traditional Digital Servo (25 kg-cm)
- **Control:** Standard PWM from Pico 2W
- **Feedback:** Position potentiometer
- **Mounting:** Requires custom bracket (LEGO Technic compatible)

**Recommendation:** **LEGO SPIKE Medium Angular Motor** for ecosystem consistency and encoder feedback.

### Power System

#### Battery: 3S LiPo (11.1V nominal)
- **Capacity:** 2200-3000 mAh
- **Discharge Rate:** 30C
- **Reasoning:** Jetson needs stable high power (15W), LEGO motors 8-9V nominal

#### Power Distribution:
- **Jetson Orin Nano:** Barrel jack 9-20V input (use 11.1V direct or buck to 12V)
- **LEGO Motors:** Buck converter to 9V/3A
- **Pico 2W:** 5V buck converter (1A)
- **Sensors:** 3.3V/5V from Jetson or Pico

#### Power Monitoring: INA260
- **Integration:** ROS2 node publishing battery_state messages
- **Safety:** Nav2 can reduce speed at low battery

---

## 3. Vision & AI Strategy

### Sign Detection: YOLOv8-Nano

**Why YOLO?**
- **Speed:** 60+ FPS on Jetson Orin Nano (TensorRT optimization)
- **Accuracy:** State-of-the-art object detection
- **Flexibility:** Easy to fine-tune on custom traffic sign dataset
- **Ecosystem:** Massive community, Ultralytics framework

**Architecture:**
- **Input:** 640×640 RGB image from left OV9281 camera
- **Output:** Bounding boxes + class (green_pillar, red_pillar) + confidence
- **Model Size:** YOLOv8n (3.2M parameters, ~6MB)

**Training Pipeline:**

1. **Dataset Creation:**
   - **Synthetic Data:** Blender/Gazebo scenes with traffic signs
     - Render 10,000+ images with varying:
       - Lighting (bright, dim, shadows, glare)
       - Angles (0-45° off-axis)
       - Distances (0.5m - 4m)
       - Occlusions (partial sign visibility)
   - **Real-World Data:** Practice runs with manual labeling
     - Collect 1,000+ real images
     - Label with LabelImg or Roboflow

2. **Data Augmentation:**
   - Random brightness/contrast adjustments
   - Horizontal flips (not vertical - pillars are always upright)
   - Random crops and zooms
   - Color jitter (simulate different lighting)

3. **Training:**
   ```bash
   yolo train model=yolov8n.pt data=traffic_signs.yaml epochs=100 imgsz=640
   ```
   - **Transfer Learning:** Start from COCO-pretrained weights
   - **Hardware:** Train on desktop GPU (RTX 3060+) or cloud (Lambda Labs)
   - **Time:** 2-3 days for 100 epochs

4. **TensorRT Optimization:**
   ```bash
   yolo export model=best.pt format=engine device=0  # FP16 precision
   ```
   - **Result:** 60+ FPS inference on Jetson Orin Nano

**Integration:**
```python
# ROS2 node: sign_detector_node
class SignDetectorNode(Node):
    def __init__(self):
        self.model = YOLO('best.engine')  # TensorRT model
        self.subscription = self.create_subscription(
            Image, '/camera/left/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(
            Detection2DArray, '/detections', 10)

    def image_callback(self, msg):
        # Convert ROS Image to numpy
        img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLO inference
        results = self.model(img, conf=0.6)  # 60% confidence threshold

        # Publish detections
        detections = self.convert_to_ros(results)
        self.publisher.publish(detections)
```

### Backup: Classical CV (HSV Filtering)

**Why backup?** If YOLO fails (low confidence, occlusion), fall back to simple color detection.

**Pipeline:**
- HSV conversion → green/red thresholding → contour detection
- Runs in parallel at 120 FPS (GPU-accelerated)
- If YOLO confidence < 0.7, use classical CV result

### Stereo Depth Integration

**Purpose:** 3D understanding of environment

**Pipeline:**
1. **Stereo Rectification:** Align left/right images (calibration matrix)
2. **Disparity Map:** Block matching algorithm (OpenCV CUDA)
3. **Depth Calculation:** depth = (baseline × focal_length) / disparity
4. **Point Cloud:** Convert depth map to 3D points

**ROS2 Integration:**
```python
# stereo_depth_node
disparity = stereo.compute(left_img, right_img)  # GPU-accelerated
depth = (baseline * focal) / disparity
point_cloud = depth_to_pointcloud(depth, camera_matrix)
self.pub.publish(point_cloud)  # PointCloud2 message
```

**Usage:**
- **Obstacle Avoidance:** Nav2 costmap uses point cloud
- **Sign Distance:** Validate YOLO detection distance (is sign actually 2m away?)
- **Collision Prevention:** Complements VL53L0X sensors

---

## 4. Performance Optimizations

### Speed: 2.5 m/s Target

**Nav2 Configuration:**
```yaml
# controller_server.yaml
controller_frequency: 20.0  # 20 Hz control loop

FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  max_linear_vel: 2.5
  max_angular_vel: 2.0
  lookahead_dist: 0.8  # Look 0.8m ahead
  regulated_linear_scaling_min_radius: 0.5  # Slow down for tight turns
```

**Adaptive Speed:**
- **Curvature-Based:** Slow down when steering angle > 15°
- **Obstacle-Based:** Reduce speed if obstacle within 1m
- **Confidence-Based:** Slow down if sign detection confidence < 0.8

### Decision-Making: 10ms Latency

**ROS2 Performance Tips:**
1. **Real-Time Executor:** Use rclcpp executor with RT threads
2. **Zero-Copy:** Use intra-process communication for image topics
3. **Priority:** Set vision/control nodes to high priority (chrt -f 90)
4. **CPU Pinning:** Pin critical nodes to specific cores

**Example:**
```cpp
// High-priority real-time node
auto options = rclcpp::NodeOptions();
options.use_intra_process_comms(true);  // Zero-copy
auto node = std::make_shared<ControlNode>(options);

// Set thread priority
sched_param param;
param.sched_priority = 90;
pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
```

### Reliability: Sensor Fusion

**robot_localization EKF:**
- **Inputs:**
  - IMU (orientation, angular velocity)
  - Wheel odometry (linear velocity)
  - LiDAR (position via SLAM)
  - Stereo depth (obstacle positions)

- **Output:** Fused Odometry message (position, velocity, covariance)

**Watchdog:**
- **Nav2 built-in:** Automatically stops if no updates from sensors/vision
- **Custom:** Python node monitors all topic health, triggers recovery

**Fail-Safe:**
- If YOLO and classical CV both fail → use LiDAR wall-following
- If stereo depth fails → rely on LiDAR + VL53L0X
- If Jetson crashes → Pico 2W emergency stop (watchdog)

---

## 5. Documentation Approach

### Engineer's Journal: ROS2 Advantages

**Automated Data Collection:**
```bash
# Record all topics during test run
ros2 bag record -a -o test_run_$(date +%s)

# Later, analyze bag file
ros2 bag info test_run_123456.db3
ros2 topic echo /detections < test_run_123456.db3
```

**Visualization:**
- **RViz2:** Real-time 3D visualization (point clouds, detections, planned path)
  - Screenshot RViz during runs → include in journal
- **PlotJuggler:** Graph topics (speed, steering, detection confidence)
  - Generate performance graphs automatically

**Simulation Results:**
- **Gazebo:** Create WRO track environment
  - Test algorithms in simulation before hardware
  - Include simulation screenshots in journal (shows thoroughness)

**Code Repository:**
- **GitHub:** Open-source ROS2 packages
  - QR code in journal → link to repo
  - Demonstrates software engineering best practices

**Journal Structure:**
1. **System Architecture:** ROS2 node graph diagram (rqt_graph)
2. **Algorithm Explanations:** YOLO training process, stereo vision math
3. **Simulation Results:** Gazebo testing before hardware
4. **Hardware Integration:** Photos of Jetson, cameras, LEGO motors
5. **Performance Data:** Bag file analysis (lap times, detection rates)
6. **Lessons Learned:** What worked, what didn't, iterations

---

## 6. Differentiation

### Unique Advantages

1. **Professional-Grade Stack:**
   - ROS2 is used in industry (self-driving cars, warehouse robots)
   - Shows understanding of real-world autonomous systems

2. **Stereo Depth:**
   - 3D understanding vs 2D LiDAR alone
   - Better obstacle classification (is it tall enough to hit?)

3. **YOLO Robustness:**
   - Handles occlusions, angles, lighting better than classical CV
   - Can detect signs even when partially hidden

4. **Simulation-Driven:**
   - Test in Gazebo before building hardware
   - Faster iteration, less hardware risk

5. **Adaptability:**
   - Easy to add new behaviors (behavior trees in Nav2)
   - Surprise rules? Add a new node!

### vs. Other Proposals

| Feature | Velocity Edge (This) | Minimalist Racer | Cognitive Racer |
|---------|---------------------|------------------|-----------------|
| **Ecosystem** | ROS2 (massive) | FreeRTOS (minimal) | Custom Python |
| **Simulation** | Gazebo (excellent) | None | Gazebo (manual setup) |
| **Vision** | YOLO (robust) | Classical CV (fast) | ViT (cutting-edge) |
| **Adaptability** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Dev Speed** | Medium (ROS2 learning) | Fast (simple) | Slow (ML training) |

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| ROS2 learning curve | High | Medium | Use Nav2 out-of-the-box, extensive tutorials |
| Jetson power consumption | Medium | Medium | Power profiling, dynamic frequency scaling |
| Stereo calibration | Medium | Medium | Automated calibration routine, validate with LiDAR |
| YOLO training dataset quality | High | High | Synthetic data (10k images), real data augmentation |
| LEGO motor integration | Low | Medium | Custom adapter PCB, test early (Week 2) |

---

## 8. Reuse from Klevor

**Keep (60%):**
- ✅ RPLiDAR C1
- ✅ BNO08X IMU
- ✅ VL53L0X sensors
- ✅ Raspberry Pi Pico 2W (motor controller)
- ✅ Monitoring concepts (adapt to ROS2 topics)

**Replace (40%):**
- ❌ RPi5 → Jetson Orin Nano (better AI, ROS2 support)
- ❌ Go → C++/Python ROS2 nodes
- ❌ Hailo AI → Jetson integrated GPU (simpler)
- ❌ CLIP → YOLO (task-specific)

---

## 9. Implementation Plan (12 weeks)

### Weeks 1-2: ROS2 Setup
- [ ] Install ROS2 Humble on Jetson
- [ ] Create workspace, hello-world node
- [ ] LEGO motor adapter board design + fab
- [ ] micro-ROS on Pico 2W

### Weeks 3-4: Sensor Integration
- [ ] Stereo camera calibration
- [ ] LiDAR ROS2 driver
- [ ] IMU + VL53L0X nodes
- [ ] robot_localization EKF setup

### Weeks 5-7: Vision Pipeline
- [ ] Collect/generate training dataset (10k+ images)
- [ ] Train YOLOv8-Nano (100 epochs)
- [ ] TensorRT optimization
- [ ] ROS2 sign detector node
- [ ] Classical CV backup

### Weeks 8-9: Nav2 Integration
- [ ] Configure Nav2 params
- [ ] Behavior tree for race logic
- [ ] Costmap with stereo depth + LiDAR
- [ ] Test in Gazebo simulation

### Weeks 10-11: Hardware Testing
- [ ] First autonomous lap
- [ ] Performance tuning
- [ ] 50+ test runs
- [ ] Bag file analysis

### Week 12: Documentation
- [ ] Engineer's Journal
- [ ] Code cleanup
- [ ] Video demonstrations

---

## 10. Critical Files

```
teamsteelbot-v2/
├── src/
│   ├── velocity_edge_bringup/
│   │   ├── launch/
│   │   │   └── race.launch.py              # Main launch file
│   │   └── config/
│   │       ├── nav2_params.yaml
│   │       └── ekf.yaml
│   ├── sign_detector/
│   │   ├── sign_detector/
│   │   │   ├── sign_detector_node.py       # YOLO inference
│   │   │   └── classical_cv_backup.py
│   │   └── models/
│   │       └── best.engine                 # TensorRT model
│   ├── stereo_depth/
│   │   └── stereo_depth/
│   │       └── stereo_depth_node.cpp       # CUDA-accelerated
│   └── lego_motor_driver/
│       └── lego_motor_driver/
│           └── motor_driver_node.cpp       # micro-ROS bridge
├── docs/                                     # As in main plan
└── README.md
```

---

## 11. LEGO Motor Integration Details

### Hardware: Custom Adapter Board

**Schematic:**
```
LEGO SPIKE Motor (6-pin) → Custom PCB → Raspberry Pi Pico 2W

Pin 1: Motor+ (PWM)
Pin 2: Motor- (PWM)
Pin 3: GND
Pin 4: Encoder A
Pin 5: Encoder B
Pin 6: VCC (3.3V)
```

**Components:**
- H-bridge motor driver (DRV8833 or L298N)
- Level shifters (3.3V ↔ 5V for encoder signals)
- Connectors: 6-pin JST for LEGO, headers for Pico

**Software: micro-ROS Node**
```cpp
// Pico 2W firmware
void motor_callback(const geometry_msgs::msg::Twist& msg) {
    // Extract linear velocity
    float target_speed = msg.linear.x;  // m/s

    // Convert to motor RPM (wheel radius, gear ratio)
    float target_rpm = (target_speed / WHEEL_CIRCUMFERENCE) * 60;

    // PID control
    float pwm = pid_controller.update(target_rpm, current_rpm);

    // Set motor PWM
    set_motor_pwm(pwm);
}

void encoder_interrupt() {
    // Read encoder, update current_rpm
    encoder_count++;
}

void publish_odometry() {
    // Calculate distance from encoder
    float distance = (encoder_count / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE;

    // Publish to ROS2
    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = distance / dt;
    odom_publisher->publish(odom);
}
```

### Alternative: LEGO EV3 Gyro Motor

If SPIKE motors unavailable, use EV3 Large Servo Motor:
- **Adapter:** 6-pin RJ12 connector
- **Protocol:** UART communication (documented by LEGO community)
- **Driver:** Existing Python library (python-ev3dev) → adapt to C++

---

## Conclusion

**Velocity Edge** leverages the professional ROS2 ecosystem for maximum adaptability and performance. Stereo vision + YOLO provides robust detection, while Nav2 handles complex navigation. LEGO motors offer reliability with built-in encoders.

**Best For:**
- Teams comfortable with ROS2 and ML
- Need maximum adaptability for surprise rules
- Want simulation-driven development

**Win Probability:** ⭐⭐⭐⭐ (High, if ROS2 learning curve managed)

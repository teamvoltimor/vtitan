# Software Stack Options

All software frameworks, vision algorithms, and control strategies for the WRO Future Engineers 2026 robot.

---

## 🏗️ Framework Options

### Option A: ROS2 Humble ⭐⭐ RECOMMENDED

**Type:** Professional robotics middleware

| Aspect | Details |
|--------|---------|
| **Middleware** | DDS (Data Distribution Service) |
| **OS** | Ubuntu 22.04 or Pi OS 64-bit with RT-PREEMPT |
| **Languages** | C++ and Python |
| **Build System** | Colcon |
| **Learning Curve** | Medium (2 weeks) |
| **Community** | Huge (thousands of packages) |

**Key Packages:**
- `nav2` - Navigation stack (path planning, behavior trees)
- `robot_localization` - EKF sensor fusion
- `rplidar_ros` - LiDAR driver
- `micro_ros_agent` - Pico 2W communication bridge
- `vision_msgs` - Standard detection messages

**Advantages:**
- ✅ Industry-standard framework
- ✅ Modular (easy to swap algorithms)
- ✅ Excellent simulation (Gazebo)
- ✅ Built-in visualization (RViz2)
- ✅ ROS bags for testing/documentation
- ✅ Best for adaptability (behavior trees)
- ✅ Strong community support

**Disadvantages:**
- ❌ Learning curve (2 weeks)
- ❌ Overhead vs bare-metal
- ❌ More complex setup

**Performance:**
- Control loop: 50-100 Hz (adequate for vision-based control)
- Decision latency: ~20-30ms

**Used in:**
- ✅ Proposal 1: Velocity Edge (with Jetson)
- ✅ Proposal 4: ROS2 Edge Racer (with Pi5 + Hailo) ⭐ RECOMMENDED

**Best for:**
- Teams wanting professional experience
- Need for adaptability (surprise rules)
- Want simulation capability
- Value documentation tools

**Code Example:**
```python
# ROS2 sign detector node
class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector')

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10)

        # Publish detections
        self.publisher = self.create_publisher(
            Detection2DArray, '/detections', 10)

    def image_callback(self, msg):
        # Run YOLO on Hailo
        detections = self.model.predict(msg)

        # Publish results
        self.publisher.publish(detections)
```

---

### Option B: FreeRTOS / Linux RT-PREEMPT ⭐ FASTEST

**Type:** Real-time operating system / Real-time kernel

| Aspect | Details |
|--------|---------|
| **OS** | FreeRTOS or Linux with RT patches |
| **Languages** | C++17 (100%) |
| **Build System** | CMake |
| **Learning Curve** | Low-Medium |
| **Community** | FreeRTOS: Medium, Linux RT: Large |

**Advantages:**
- ✅ Deterministic scheduling (<1ms jitter)
- ✅ Lowest latency (<5ms)
- ✅ Highest control loop frequency (200 Hz)
- ✅ Minimal overhead
- ✅ Direct hardware access

**Disadvantages:**
- ❌ FreeRTOS on Pi5 is experimental
- ❌ No simulation capability
- ❌ Manual testing required
- ❌ Less modular (C++ rebuild for changes)

**Performance:**
- Control loop: 200 Hz (deterministic)
- Decision latency: <5ms (fastest!)

**Used in:**
- ✅ Proposal 2: Minimalist Racer (bare-metal approach)

**Best for:**
- Teams prioritizing speed/reliability
- Prefer classical engineering
- Want simplest, fastest execution
- Don't need simulation

**Code Example:**
```cpp
// FreeRTOS control loop task
void control_loop_task(void *params) {
    constexpr uint32_t PERIOD_MS = 5;  // 200 Hz
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // Read sensors (<1ms)
        SensorData data = read_all_sensors();

        // Detect signs (<3ms)
        SignDetection sign = detect_sign_hsv(data.camera_frame);

        // Decide action (<0.5ms)
        ControlCommand cmd = state_machine.update(sign, data);

        // Send to motors (<0.1ms)
        motor_queue.push(cmd);

        // Total: ~4.6ms (well under 5ms budget)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));
    }
}
```

---

### Option C: Custom Python/PyTorch Stack

**Type:** Research-oriented ML framework

| Aspect | Details |
|--------|---------|
| **OS** | Pi OS 64-bit with RT patches |
| **Languages** | Python 3.11 (80%), C++ (20%) |
| **AI Framework** | PyTorch 2.0+, ONNX Runtime |
| **Communication** | ZeroMQ (low-latency IPC) |
| **Learning Curve** | High (ML expertise required) |

**Advantages:**
- ✅ Cutting-edge ML capabilities
- ✅ Fast prototyping (Python)
- ✅ Flexible experimentation
- ✅ Simulation support (Gazebo + PyTorch)

**Disadvantages:**
- ❌ Requires ML expertise
- ❌ Training uncertainty
- ❌ Longer development (14 weeks)
- ❌ Higher risk

**Performance:**
- Control loop: 50 Hz
- Decision latency: ~30ms
- Inference: 30 FPS (ViT on Hailo)

**Used in:**
- ✅ Proposal 3: Cognitive Racer (Vision Transformer)

**Best for:**
- Teams with ML/AI experience
- Want cutting-edge innovation
- Willing to accept higher risk
- Have GPU for training

**Code Example:**
```python
# PyTorch Vision Transformer inference
class VisionTransformerPolicy:
    def __init__(self):
        # Load ONNX model on Hailo
        self.model = HailoEngine('vit_racer.hef')

    async def predict(self, image):
        # Preprocess
        tensor = self.preprocess(image)

        # Inference on Hailo (30 FPS)
        outputs = self.model.run(tensor)

        # Decode outputs
        steering = outputs['steering'][0]  # -1 to 1
        throttle = outputs['throttle'][0]  # 0 to 1

        return {'steering': steering, 'throttle': throttle}
```

---

## 👁️ Vision Algorithm Options

### Option A: YOLO on Hailo ⭐ RECOMMENDED

**Type:** Deep learning object detection

**Model:** YOLOv8-Nano or YOLOv5s

**Hardware:** Hailo AI HAT+ 26 TOPS

**Performance:**
- FPS: 30-60 on Hailo
- Accuracy: ~94% (after fine-tuning)
- Robustness: Excellent (lighting, occlusions, angles)

**Training:**
1. Collect 5k+ images (manual driving + synthetic Gazebo)
2. Train with Ultralytics framework
3. Export to ONNX
4. Convert ONNX → Hailo HEF (Hailo Dataflow Compiler)

**Advantages:**
- ✅ Robust to lighting variations
- ✅ Handles occlusions
- ✅ Works at various angles
- ✅ Hailo optimized (fast inference)
- ✅ Mature tooling

**Disadvantages:**
- ❌ Requires training data
- ❌ Slower than classical CV (30-60 FPS vs 120 FPS)
- ❌ Hailo ONNX conversion can be tricky

**Used in:**
- ✅ Proposal 1: Velocity Edge (YOLO on Jetson)
- ✅ Proposal 4: ROS2 Edge Racer (YOLO on Hailo) ⭐ RECOMMENDED

**Example:**
```python
from ultralytics import YOLO

# Load Hailo-optimized model
model = YOLO('traffic_signs.hef')

# Inference
results = model(image, conf=0.7)
for detection in results:
    bbox = detection.boxes.xyxy
    confidence = detection.boxes.conf
    class_id = detection.boxes.cls  # 0=green, 1=red
```

---

### Option B: Classical Computer Vision (HSV) ⭐ FASTEST

**Type:** Hand-crafted algorithm

**Approach:** HSV color segmentation + contour detection

**Hardware:** Pi5 GPU (VideoCore VII) acceleration

**Performance:**
- FPS: 120 (GPU-accelerated)
- Accuracy: ~82-90% (lighting dependent)
- Latency: <3ms

**Pipeline:**
1. RGB → HSV conversion (GPU)
2. Color thresholding (green: H=60-90°, red: H=0-10° or 170-180°)
3. Morphological operations (noise reduction)
4. Contour detection
5. Shape filtering (aspect ratio, area)
6. Multi-frame averaging (temporal stability)

**Advantages:**
- ✅ Fastest (120 FPS)
- ✅ Explainable (no "black box")
- ✅ No training required
- ✅ Low latency (<3ms)
- ✅ Easy to debug
- ✅ Robust to motion blur

**Disadvantages:**
- ❌ Sensitive to extreme lighting
- ❌ Manual parameter tuning required
- ❌ Less robust than YOLO
- ❌ **Doesn't use AI HAT+ 26 TOPS!**

**Used in:**
- ✅ Proposal 2: Minimalist Racer (primary)
- ✅ Proposal 1, 3, 4 (as fallback)

**Example:**
```cpp
// HSV color detection (C++)
cv::Mat hsv;
cv::cvtColor(bgr_frame, hsv, cv::COLOR_BGR2HSV);

// Green threshold
cv::Mat green_mask;
cv::inRange(hsv,
    cv::Scalar(60, 100, 100),   // Lower
    cv::Scalar(90, 255, 255),   // Upper
    green_mask);

// Find contours
std::vector<std::vector<cv::Point>> contours;
cv::findContours(green_mask, contours,
    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// Filter by aspect ratio (vertical pillars)
for (const auto& contour : contours) {
    cv::Rect bbox = cv::boundingRect(contour);
    double aspect = (double)bbox.height / bbox.width;

    if (aspect > 1.5 && aspect < 4.0) {
        // Valid sign detected!
    }
}
```

---

### Option C: Vision Transformer (ViT) ⭐ MOST INNOVATIVE

**Type:** Deep learning - transformer architecture

**Model:** DINOv2 ViT-Small (21.7M parameters)

**Hardware:** Hailo AI HAT+ 26 TOPS

**Performance:**
- FPS: 30 on Hailo
- Accuracy: ~94% (after training)
- End-to-end: Image → Steering + Throttle + Classification

**Training:**
1. **Imitation Learning:** 50+ hours manual driving
2. **Reinforcement Learning:** Simulation in Gazebo (1M steps)
3. **Domain Adaptation:** Sim-to-real transfer
4. **Active Learning:** Continuous improvement

**Advantages:**
- ✅ Most innovative (first ViT in WRO!)
- ✅ End-to-end learning
- ✅ Learns optimal strategies
- ✅ Best for surprise rules (learns context)
- ✅ Explainable via attention maps
- ✅ Fully uses AI HAT+ 26 TOPS

**Disadvantages:**
- ❌ Highest risk (training may fail)
- ❌ Requires ML expertise
- ❌ Needs GPU for training
- ❌ Unpredictable until trained
- ❌ Longest development

**Used in:**
- ✅ Proposal 3: Cognitive Racer (primary) ⭐ MOST INNOVATIVE

**Example:**
```python
# Vision Transformer inference
class VisionTransformer(nn.Module):
    def __init__(self):
        super().__init__()
        # DINOv2 ViT-Small backbone
        self.backbone = torch.hub.load('facebookresearch/dinov2',
                                       'dinov2_vits14')

        # Task-specific heads
        self.steering_head = nn.Sequential(
            nn.Linear(384, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Tanh()  # -1 to 1
        )

        self.throttle_head = nn.Sequential(
            nn.Linear(384, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid()  # 0 to 1
        )

    def forward(self, image):
        # Extract features
        features = self.backbone(image)  # [batch, 384]

        # Predict actions
        steering = self.steering_head(features)
        throttle = self.throttle_head(features)

        return steering, throttle
```

---

## 🎮 Control Strategy Options

### Option A: PID Control ⭐ PROVEN

**Type:** Classical control theory

**Components:**
- **Steering PID:** Target angle → PWM to servo
- **Speed PID:** Target velocity → PWM to motor

**Tuning Method:** Ziegler-Nichols or manual

**Advantages:**
- ✅ Well-understood
- ✅ Predictable behavior
- ✅ Easy to tune
- ✅ Explainable

**Used in:** Proposals 1, 2, 4

---

### Option B: End-to-End Learning (No PID)

**Type:** Neural network output → direct motor commands

**Used in:** Proposal 3 (ViT outputs steering/throttle directly)

**Advantages:**
- ✅ Learns optimal control
- ✅ Adapts to robot dynamics

**Disadvantages:**
- ❌ Less predictable
- ❌ Harder to debug

---

### Option C: Behavior Trees (Nav2)

**Type:** Hierarchical state machine

**Framework:** ROS2 Nav2

**Advantages:**
- ✅ Best for adaptability
- ✅ Easy to add new behaviors
- ✅ Visual editing tools

**Used in:** Proposals 1, 4 (optional)

---

## 🔄 Sensor Fusion Options

### Option A: EKF (Extended Kalman Filter) ⭐ RECOMMENDED

**Package:** `robot_localization` (ROS2)

**Inputs:**
- IMU (orientation, angular velocity)
- Wheel encoders (linear velocity)
- LiDAR (position via scan matching)

**Output:** Fused odometry (position, velocity, covariance)

**Used in:** Proposals 1, 4 (ROS2-based)

---

### Option B: Manual Fusion (Weighted Average)

**Type:** Simple sensor combination

**Used in:** Proposals 2, 3

**Example:**
```cpp
// Combine IMU + encoder velocity
double fused_velocity =
    0.7 * encoder_velocity +
    0.3 * imu_estimated_velocity;
```

---

## 📊 Software Stack Comparison

| Aspect | ROS2 (Prop 1, 4) | Bare-Metal (Prop 2) | Custom Python/ML (Prop 3) |
|--------|------------------|---------------------|---------------------------|
| **Framework** | ROS2 Humble | FreeRTOS / Linux RT | PyTorch + asyncio |
| **Language** | C++/Python | C++17 only | Python/C++ |
| **Vision** | YOLO | Classical HSV | Vision Transformer |
| **Control Loop** | 50-100 Hz | **200 Hz** ⭐ | 50 Hz |
| **Latency** | 20-30ms | **<5ms** ⭐ | 30ms |
| **Simulation** | ✅ Gazebo | ❌ None | ✅ Gazebo |
| **Modularity** | ✅ Excellent | ❌ Low | Medium |
| **Learning Curve** | Medium | Low | **High** |
| **Dev Time** | 11-12 weeks | **10 weeks** ⭐ | 14 weeks |
| **Uses AI HAT+?** | ✅ YES (Prop 4) | ❌ NO | ✅ YES |
| **Innovation** | ⭐⭐⭐⭐ | ⭐⭐⭐ | **⭐⭐⭐⭐⭐** |
| **Risk** | Low-Med | **Low** ⭐ | **High** |

---

## 🎯 Software Recommendation

### Recommended: ROS2 + YOLO (Hailo) + Classical CV Fallback (Proposal 4)

**Configuration:**
```yaml
Framework: ROS2 Humble
OS: Pi OS 64-bit with RT patches

Vision:
  Primary: YOLOv8-Nano on Hailo AI HAT+ (30-60 FPS)
  Fallback: Classical HSV CV (120 FPS)
  Validation: TCS34725 color sensor (physical)

Control:
  Loop Rate: 50 Hz
  Steering: PID controller
  Speed: Adaptive (curvature-based)

Fusion:
  Package: robot_localization (EKF)
  Inputs: IMU + Encoders + LiDAR

Navigation:
  Primary: Custom state machine
  Optional: Nav2 (for complex scenarios)
```

**Why this is optimal:**
- ✅ Uses AI HAT+ 26 TOPS (YOLO inference)
- ✅ Professional framework (ROS2)
- ✅ Flexibility (YOLO + Classical CV)
- ✅ Simulation capability (Gazebo)
- ✅ Modular (easy to adapt)
- ✅ Balanced risk/reward
- ✅ Excellent documentation tools

**Development Timeline:** 11 weeks

---

## Alternative: Vision Transformer (Proposal 3)

**If you have ML expertise and want cutting-edge innovation:**

```yaml
Framework: Custom Python stack
AI: PyTorch 2.0+

Vision:
  Model: DINOv2 ViT-Small on Hailo AI HAT+
  Training: Imitation + RL + Domain Adaptation
  Output: End-to-end (image → steering/throttle)
  Fallback: Classical HSV CV

Innovation:
  - PMW3901 optical flow sensor
  - Attention map explainability
  - Active learning loop
```

**Development Timeline:** 14 weeks

**Risk:** High (training may not converge)

**Reward:** Most impressive if successful!

---

## 🚫 Not Recommended: Jetson-based (Proposal 1)

**Reason:** You already have AI HAT+ 26 TOPS! Don't waste $500 on Jetson.

If you want ROS2 + YOLO, use Proposal 4 instead (same benefits, $0 cost).

---

## 📚 Learning Resources

### ROS2:
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [The Construct - ROS2 courses](https://www.theconstructsim.com/)
- [Articulated Robotics YouTube](https://www.youtube.com/c/ArticulatedRobotics)

### Classical CV:
- [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html)
- [PyImageSearch](https://www.pyimagesearch.com/)

### Vision Transformers:
- [DINOv2 Paper](https://arxiv.org/abs/2304.07193)
- [Hugging Face Transformers](https://huggingface.co/docs/transformers/)

### Hailo:
- [Hailo Documentation](https://hailo.ai/developer-zone/)
- Your existing VoldemorBot CLIP implementation!

---

See `/systems` for complete proposals combining these software stacks with hardware.

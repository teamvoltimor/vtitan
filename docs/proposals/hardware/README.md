# Hardware Configuration Options

All hardware platforms, sensors, and actuators for the WRO Future Engineers 2026 robot.

---

## 🖥️ Main Compute Platforms

### Option A: Raspberry Pi 5 16GB + AI HAT+ 26 TOPS ⭐⭐ RECOMMENDED

**Status:** ✅ Already have from Klevor!

| Component | Spec | Source |
|-----------|------|--------|
| **CPU** | Quad-core ARM Cortex-A76 @ 2.4 GHz | Klevor |
| **RAM** | 16GB LPDDR4X | Klevor |
| **GPU** | VideoCore VII | Klevor |
| **AI Accelerator** | Hailo AI HAT+ 26 TOPS | Klevor |
| **Storage** | microSD or NVMe | Klevor |
| **Power** | 5V/5A (27W max) | Klevor |
| **Cost** | $0 (reused) | |

**Advantages:**
- ✅ 100% reuse from Klevor
- ✅ AI HAT+ 26 TOPS (better than Hailo-8L 13 TOPS assumed!)
- ✅ 16GB RAM (better than 8GB assumed!)
- ✅ Low power consumption
- ✅ Already proven in Klevor with CLIP
- ✅ You know the Hailo SDK!

**Disadvantages:**
- ❌ Less powerful than Jetson (but AI HAT+ 26 TOPS is excellent)
- ❌ Single camera interface (no native stereo)

**Used in:**
- ✅ Proposal 2: Minimalist Racer
- ✅ Proposal 3: Cognitive Racer
- ✅ Proposal 4: ROS2 Edge Racer

**Models supported on AI HAT+:**
- YOLOv5, YOLOv8 (object detection)
- Vision Transformers (DINOv2, ViT)
- MobileNet, EfficientNet
- CLIP (already tested!)

---

### Option B: NVIDIA Jetson Orin Nano 8GB ❌ NOT RECOMMENDED

**Status:** ❌ Would need to purchase (~$500)

| Component | Spec | Source |
|-----------|------|--------|
| **CPU** | 6-core ARM Cortex-A78AE @ 2.0 GHz | New purchase |
| **GPU** | 1024-core NVIDIA Ampere + 32 Tensor Cores | New purchase |
| **RAM** | 8GB LPDDR5 (shared) | New purchase |
| **AI Performance** | 40 TOPS | New purchase |
| **Power** | 7-15W | New purchase |
| **Cost** | ~$500 | |

**Advantages:**
- ✅ More powerful AI (40 TOPS vs 26 TOPS)
- ✅ Native CUDA support
- ✅ Better ROS2 integration

**Disadvantages:**
- ❌ Expensive ($500)
- ❌ You already have AI HAT+ 26 TOPS!
- ❌ Wastes existing Klevor investment
- ❌ Higher power consumption
- ❌ Makes no sense given your hardware!

**Used in:**
- ❌ Proposal 1: Velocity Edge (NOT RECOMMENDED)

**Verdict:** Don't buy this! You already have excellent AI hardware.

---

## 📷 Camera Options

### Option A: Raspberry Pi Camera Module 3 Wide ⭐ RECOMMENDED

**Status:** ✅ Already have from Klevor!

| Spec | Value |
|------|-------|
| **Sensor** | Sony IMX708 (12MP, 1/2.43") |
| **FOV** | 120° (wide) |
| **Resolution** | 4608×2592 (still), 2304×1296 (video) |
| **Frame Rate** | 30-120 FPS (resolution dependent) |
| **Interface** | CSI-2 (native Pi5) |
| **Features** | PDAF autofocus, HDR, low-light optimized |
| **Cost** | $0 (reused) |

**Advantages:**
- ✅ Native Pi5 integration
- ✅ Hardware ISP acceleration
- ✅ Wide 120° FOV (sees signs earlier)
- ✅ Excellent low-light performance
- ✅ Already proven in Klevor

**Used in:**
- ✅ Proposal 2, 3, 4 (all Pi5-based)

---

### Option B: OV9281 Stereo Camera Pair ❌ NOT RECOMMENDED

**Status:** ❌ Would need to purchase (~$100)

| Spec | Value |
|------|-------|
| **Sensor** | OV9281 (1MP, global shutter) |
| **Resolution** | 1280×800 |
| **Frame Rate** | 120 FPS |
| **Interface** | USB 3.0 |
| **Baseline** | 60mm stereo separation |
| **Cost** | ~$100 |

**Advantages:**
- ✅ Stereo depth perception
- ✅ Global shutter (no motion blur)
- ✅ High frame rate

**Disadvantages:**
- ❌ Expensive ($100)
- ❌ You already have RPi Camera 3!
- ❌ USB bandwidth overhead
- ❌ Requires Jetson for processing

**Used in:**
- ❌ Proposal 1: Velocity Edge (with Jetson)

**Verdict:** Don't buy this! Your RPi Camera 3 Wide is excellent.

---

## 🎯 LiDAR

### RPLiDAR C1 ⭐ FROM KLEVOR

**Status:** ✅ Already have!

| Spec | Value |
|------|-------|
| **Range** | 0.2-12m |
| **Sample Rate** | 8000 samples/sec |
| **Scan Rate** | 10 Hz |
| **Angular Resolution** | 0.5° |
| **Interface** | UART (115200 baud) |
| **Cost** | $0 (reused) |

**Uses:**
- 2D obstacle detection
- Wall-following backup mode
- Distance validation
- SLAM (if needed)

**Used in:** ALL proposals

---

## 📡 IMU

### BNO08X 9-DOF IMU ⭐ FROM KLEVOR

**Status:** ✅ Already have!

| Spec | Value |
|------|-------|
| **Sensors** | 3-axis accelerometer, gyro, magnetometer |
| **Output** | Quaternion, Euler angles, raw data |
| **Update Rate** | 100 Hz |
| **Interface** | I2C or UART |
| **On-Chip Fusion** | Yes (BNO080 sensor fusion algorithm) |
| **Cost** | $0 (reused) |

**Uses:**
- Orientation tracking
- Velocity estimation
- Sensor fusion (EKF with odometry)

**Used in:** ALL proposals

---

## 📏 Distance Sensors

### VL53L0X Time-of-Flight ⭐ FROM KLEVOR

**Status:** ✅ Already have 2-4 units!

| Spec | Value |
|------|-------|
| **Range** | 0.03-2m (accurate to 2cm) |
| **Technology** | 940nm laser ToF |
| **Update Rate** | Up to 50 Hz |
| **Interface** | I2C |
| **FoV** | 25° |
| **Cost** | $0 (reused) |

**Typical Mounting:**
- 2× front corners (±45°) - collision prevention
- 1× front center - straight-ahead distance
- 1× rear (optional) - backup sensors

**Used in:** ALL proposals

---

## 🎨 Optional: Color Sensor

### TCS34725 RGB Color Sensor ⭐ UNIQUE INNOVATION

**Status:** ➕ Need to purchase (~$8)

| Spec | Value |
|------|-------|
| **Channels** | RGBC (Red, Green, Blue, Clear) |
| **Resolution** | 16-bit per channel |
| **Interface** | I2C |
| **Light Source** | Requires illumination (white LED) |
| **Cost** | ~$8 |

**Innovation:** Physical color validation!
- Camera detects sign color
- Robot drives past sign
- TCS34725 validates color physically
- 2-of-3 voting: Camera + Color Sensor + LiDAR

**Used in:**
- ✅ Proposal 2: Minimalist Racer (PRIMARY innovation)
- ✅ Proposal 4: ROS2 Edge Racer (OPTIONAL)

**Recommendation:** BUY THIS! $8 for unique validation is worth it.

---

## 🌊 Optional: Optical Flow Sensor

### PMW3901 Optical Flow Sensor ⭐ UNIQUE INNOVATION

**Status:** ➕ Need to purchase (~$30)

| Spec | Value |
|------|-------|
| **Resolution** | 80×80 pixels |
| **Update Rate** | 60-120 Hz |
| **Velocity Range** | ±2 m/s |
| **Interface** | SPI |
| **Mounting** | Bottom-facing, 10cm above ground |
| **Cost** | ~$30 |

**Innovation:** Ground-truth velocity measurement!
- Detects wheel slip
- Independent of encoder errors
- Accurate velocity for control

**Used in:**
- ✅ Proposal 3: Cognitive Racer (PRIMARY innovation)

**Recommendation:** Optional. Only if doing ML/RL training.

---

## ⚙️ Motor Controller

### Raspberry Pi Pico 2W ⭐ FROM KLEVOR

**Status:** ✅ Already have!

| Spec | Value |
|------|-------|
| **CPU** | Dual Cortex-M33 @ 150 MHz |
| **RAM** | 520KB SRAM |
| **Flash** | 2MB |
| **PIO** | 2× Programmable I/O (for encoders) |
| **Connectivity** | WiFi (2.4 GHz) + Bluetooth |
| **Interface to Pi5** | USB-CDC (proven from Klevor) |
| **Cost** | $0 (reused) |

**Uses:**
- Motor PWM control
- Encoder reading (PIO for precision)
- Servo control
- Communicate with Pi5 via USB-CDC

**Used in:** ALL proposals

---

## 🚗 Motor Options

See separate documents:
- `01-motor-comparison.md` - Comparison of motor types
- `02-build-hat-motor-option.md` - Using LEGO Build HAT
- `03-lego-dual-motor-build-hat.md` - Dual motor setup

### Quick Summary:

**Recommended:** LEGO SPIKE Prime Large Motor
- Integrated encoder
- Proven reliability
- 25 N·cm torque

**Alternative:** LEGO EV3 Large Motor
- Cheaper (secondary market)
- Well-documented
- 20 N·cm torque

**Interface:** Custom adapter board OR Raspberry Pi Build HAT

---

## 🔋 Power System

### Battery Options

**Option A: 2S LiPo (7.4V nominal)** - Lighter
- Pi5: 5V/5A buck
- Motors: 7.4V direct or buck to 6-9V
- Sufficient for most configurations

**Option B: 3S LiPo (11.1V nominal)** - More headroom
- Pi5: 5V/5A buck
- Motors: 9V buck (ideal for LEGO motors)
- Better for higher current draw

**Recommended:** 2200-3000 mAh, 30C discharge

**From Klevor:** ✅ Likely already have compatible battery

---

## 📊 Hardware Configuration Comparison

| Component | Minimal (Prop 2) | ML/AI (Prop 3) | ROS2 (Prop 4) | Jetson (Prop 1) |
|-----------|-----------------|----------------|---------------|-----------------|
| **Main Compute** | Pi5 16GB | Pi5 16GB | Pi5 16GB | Jetson Orin |
| **AI Accelerator** | AI HAT+ (unused) | AI HAT+ 26 TOPS | AI HAT+ 26 TOPS | Jetson GPU 40 TOPS |
| **Camera** | RPi Cam 3 | RPi Cam 3 | RPi Cam 3 | Stereo OV9281 |
| **LiDAR** | RPLiDAR C1 | RPLiDAR C1 | RPLiDAR C1 | RPLiDAR C1 |
| **IMU** | BNO08X | BNO08X | BNO08X | BNO08X |
| **Distance** | 2× VL53L0X | 4× VL53L0X | 2-3× VL53L0X | 3× VL53L0X |
| **Color Sensor** | ✅ TCS34725 | ❌ | ✅ TCS34725 | ❌ |
| **Optical Flow** | ❌ | ✅ PMW3901 | ❌ | ❌ |
| **Motor Ctrl** | Pico 2W | Pico 2W | Pico 2W | Pico 2W |
| | | | | |
| **New Hardware Cost** | $8 | $30 | $8 | $630 |
| **Reuse %** | 90% | 100% | 100% | 60% |
| **Uses AI HAT+?** | ❌ NO | ✅ YES | ✅ YES | ❌ (replaced) |

---

## 🎯 Hardware Recommendation

**Choose:** Pi5 16GB + AI HAT+ 26 TOPS (100% reuse)

**Add:**
- TCS34725 color sensor ($8) - Unique validation
- PMW3901 optical flow ($30) - Only if doing ML/RL

**Motors:**
- LEGO SPIKE Prime OR EV3 motors
- Custom adapter OR Raspberry Pi Build HAT

**Total new cost:** $8-38 (vs $630 for Jetson!)

**Why this is optimal:**
- ✅ Uses all existing Klevor hardware
- ✅ AI HAT+ 26 TOPS fully utilized
- ✅ Minimal cost
- ✅ Proven reliability

See `/comparisons` for full analysis.

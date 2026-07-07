# Decision Summary

**Based on actual VoldemorBot hardware analysis**

---

## What You Have

- **Raspberry Pi 5 (16GB RAM)** - Better than proposals assumed
- **AI HAT+ 26 TOPS** - 2x better than Hailo-8L
- **RPi Camera Module 3 Wide** - Suitable for vision
- **Raspberry Pi Pico 2 WH** - Motor controller
- **RPLiDAR C1** - Spatial awareness
- **BNO08X IMU** - 9-axis orientation
- **Motors and Servo** - Already integrated
- **Go codebase + Hailo CLIP** - Existing Hailo SDK knowledge

**Total value: approximately $500 of hardware ready to deploy**

---

## Top Recommendation

### Proposal 4: ROS2 Edge Racer

**Why:**
- Uses 100% of your hardware (including AI HAT+ 26 TOPS)
- **Uses your Bugatti Bolide LEGO parts** (905 Technic pieces)
- Professional ROS2 framework
- Cost: **$38-103** (depends on motor choice - see Motor Options below)
- Timeline: 11 weeks
- You already know Hailo SDK from CLIP work

**What you get:**
- YOLO on AI HAT+ for robust sign detection (ML where it helps)
- Classical CV backup
- TCS34725 color sensor ($8) for unique innovation
- **LEGO differential steering option** ($73-103) - simplest, uses your parts
- **Alternative: INJORA + optical flow** ($38) - fastest, lowest cost
- ROS2 tools (bags, RViz, PlotJuggler) for impressive documentation
- **JSON/YAML configuration** - meets your requirement
- **Handles surprise rules** - edit config file, no programming on-site
- **Hybrid approach:** YOLO detects objects, config decides actions

**Path forward:**
1. Week 1: Order sensors + choose motor option, install ROS2, test YOLO on Hailo
2. Weeks 2-9: Build system (sensor integration, vision, control, odometry)
3. Weeks 10-11: Testing (50+ runs) + Documentation
4. Weeks 12-13: Buffer for polish

**See:** [04-ros2-edge-racer-hybrid.md](docs/proposals/systems/04-ros2-edge-racer-hybrid.md) | [Motor Analysis](docs/proposals/hardware/02-build-hat-motor-option.md)

---

## Alternative Not Recommended (Due to Configuration Requirements)

### Proposal 3: Cognitive Racer

**Why it looked promising:**
- Vision Transformer (first in WRO)
- Uses 100% of your hardware
- Cost: **$0-30** (optional optical flow)
- Most impressive technically

**Critical Flaw: Cannot Handle Surprise Rules**

**The Problem:**
- ML models have **fixed learned behaviors** after training
- Cannot adapt to surprise rules announced on competition day
- Would need retraining on-site (impossible)
- Example: If surprise rule is "turn RIGHT at green blocks instead of LEFT"
  - Model learned to turn left (baked into weights)
  - Cannot override without retraining
  - **Deal-breaker for WRO**

**Your Requirement:**
> "I would like to be able to setup the robot by just a simple JSON or file and don't have to do program on-site"

**Proposal 3 Fails This:**
- Cannot change learned behaviors via config
- Only hyperparameters adjustable (not actions)
- Requires retraining for behavior changes
- **Not suitable for competitions with surprise rules**

**Flexibility Score:** Poor (2/5)

**See:** [03-cognitive-racer-vit.md](docs/proposals/systems/03-cognitive-racer-vit.md) | [Configuration Analysis](docs/proposals/comparisons/04-configuration-flexibility.md)

---

## Not Recommended

### Proposal 1: Velocity Edge (Jetson)
**Why not:** You already have AI HAT+ 26 TOPS. No need to buy Jetson ($500).

### Proposal 2: Minimalist Racer
**Why not:** Wastes your AI HAT+ 26 TOPS on classical CV. Your $70 AI hardware sits idle.

---

## Motor Options for ROS2 (Proposal 4)

**Current Motor:** INJORA 180 Motor (20,500 RPM, no encoder)

**You have LEGO Technic Bugatti Bolide #42151 (905 parts)**

### Option 1A: Full LEGO Differential Steering (Top Choice)

**Cost:** $73-103 (2x LEGO Motors + Build HAT + Sensor)

**Setup:**
- 2x LEGO Large Motor 88013 ($40-60) - Differential drive (both for driving)
- 1x Raspberry Pi Build HAT ($25-35) - Controls motors
- 1x TCS34725 color sensor ($8)
- **Chassis from your Bugatti Bolide parts** (905 pieces - free)

**How Differential Steering Works:**
- Left motor drives left wheel, right motor drives right wheel
- Turn left: Left slow, right fast
- Turn right: Left fast, right slow
- Spin in place: Left forward, right backward (zero turning radius)

**Advantages:**
- Simplest setup (only 2 motors, no steering servo or mechanism)
- Uses your LEGO parts (905 Technic pieces from Bugatti)
- Zero turning radius (spin in place - ideal for WRO)
- Dual encoder odometry (360 CPR x 2 motors)
- ROS2 built-in support (diff_drive_controller)
- No Pico 2W needed (Build HAT does everything)
- LEGO Education standard (proven, reliable)
- Easy mechanical integration (LEGO mounting)
- Professional appearance (LEGO chassis)
- Most WRO robots use differential steering

**Disadvantages:**
- Slower (0.4 m/s vs 1.25 m/s with INJORA)
- Wheel slip when turning (skids sideways)

**Is 0.4 m/s sufficient?**
- Yes. Zero turning radius compensates for lower speed
- WRO values precision over raw speed
- Better reliability means fewer retries, better overall time
- Most educational robotics competitions use this speed

**Choose if:**
- You want to use your Bugatti Bolide LEGO parts
- You want simplest setup (2 motors, no steering mechanism)
- Precision matters more than raw speed for your track
- You want proven LEGO Education hardware

**See:** [03-lego-dual-motor-build-hat.md](docs/proposals/hardware/03-lego-dual-motor-build-hat.md)

---

### Option 1B: Keep INJORA + Add PMW3901 Optical Flow (Speed Choice)

**Cost:** $30 (PMW3901 sensor)

**Advantages:**
- 3x faster than LEGO (1.25 m/s vs 0.4 m/s) - matters for racing
- Slip-immune odometry (measures actual ground movement)
- Unique approach for innovation points in documentation
- Reuse working motor (zero mechanical changes)
- Cheaper than Build HAT option
- Works well with ROS2 odometry (no wheel slip issues)

**Why it suits WRO racing:**
- Speed matters in racing - INJORA's 1.25 m/s gives competitive lap times
- Optical flow is more accurate than wheel encoders (no slip on turns)
- Unique approach = innovation points in documentation

**See:** [01-motor-comparison.md](docs/proposals/hardware/01-motor-comparison.md)

---

### Option 2: Build HAT + LEGO (Ackermann Steering) - Car-Like

**Cost:** $60-90 (Build HAT $25-35 + 2 Motors $35-55)

**Setup:**
- 1x LEGO Large Motor 88013 for rear drive
- 1x LEGO Medium Motor 45603 for front steering
- 1x Raspberry Pi Build HAT
- Build Ackermann steering linkage from LEGO parts

**Advantages:**
- Car-like steering (more realistic)
- Less wheel slip (front wheels pivot)
- Official Python library (buildhat)
- Proven hardware (LEGO Education)
- Eliminates Pico 2W

**Disadvantages:**
- More complex (need Ackermann linkage)
- Harder to build (steering geometry critical)
- Larger turning radius (cannot spin in place)
- More tuning needed (steering calibration)
- Slower (0.4 m/s)
- Overkill for WRO (differential is simpler and better)

**Verdict:** Not recommended. Differential steering (Option 1A) is simpler, more reliable, and better for WRO.

**See:** [02-build-hat-motor-option.md](docs/proposals/hardware/02-build-hat-motor-option.md) | [03-lego-dual-motor-build-hat.md](docs/proposals/hardware/03-lego-dual-motor-build-hat.md)

---

### Option 3: Keep INJORA Only (No Encoder)

**Cost:** $0

**Trade-off:**
- Zero cost, already working, fast (1.25 m/s)
- No encoder means relying on IMU + LiDAR for odometry (less accurate)
- Workable but not ideal for ROS2 Nav2

**When to choose:** Absolute budget constraint, can start immediately

---

### Motor Recommendation for Proposal 4

**Three viable options - choose based on your priorities:**

#### Option 1A: Full LEGO Differential (Top Choice)

**Cost:** $73-103

**Choose if:**
- You want to use your Bugatti Bolide LEGO parts (905 pieces)
- You want simplest setup (2 motors, no steering mechanism)
- You want zero turning radius (spin in place)
- You want proven LEGO Education hardware
- You want easiest ROS2 integration (diff_drive_controller)
- Precision matters more than raw speed
- You want to eliminate Pico 2W (cleaner architecture)

**Cost breakdown:**
- 2x LEGO Large Motor 88013: $40-60
- Build HAT: $25-35
- TCS34725 color sensor: $8
- **Total: $73-103**

**Speed:** 0.4 m/s (acceptable for WRO with zero turning radius)

---

#### Option 1B: INJORA + PMW3901 (Speed Choice)

**Cost:** $38

**Choose if:**
- Speed is critical (need 1.25 m/s for competitive lap times)
- You want innovation points (optical flow odometry)
- You want lowest budget
- You are comfortable with moderate integration work
- You want to keep existing mechanical setup

**Cost breakdown:**
- TCS34725 color sensor: $8
- PMW3901 optical flow: $30
- **Total: $38**

**Speed:** 1.25 m/s (3x faster than LEGO)

---

#### Option 3: Keep INJORA Only (Budget Choice)

**Cost:** $8

**Choose if:**
- Absolute budget constraint
- Need to start immediately
- Accept IMU/LiDAR-only odometry

---

**Recommendation:**

**Option 1A (Full LEGO Differential)** if you want to use your Bugatti Bolide parts and value simplicity and reliability.

**Option 1B (INJORA + PMW3901)** if speed is critical for your competition track.

Both are viable - the choice depends on whether you prioritize:
- **LEGO:** Simplicity, zero turning radius, use your parts
- **INJORA:** Speed, innovation, lower cost

---

## Quick Decision Tree

```
Need on-site configuration without programming?
-> YES (you do) -> Only Proposal 4 works

Do you want to waste your AI HAT+ 26 TOPS?
-> NO -> Do not choose Proposal 2

Do you want to spend $600 on redundant hardware?
-> NO -> Do not choose Proposal 1

Can you handle surprise rules with ML model?
-> NO -> Do not choose Proposal 3
```

**Proposal 4 (ROS2 Edge Racer) is the only choice that meets your requirements.**

---

## Side-by-Side Comparison (Updated with Configuration Flexibility)

| Factor | Proposal 4 (ROS2) | Proposal 3 (ViT) |
|--------|------------------|------------------|
| **Cost** | $38-103 | $30 |
| **Hardware Reuse** | 100% | 100% |
| **Uses Bugatti LEGO** | Yes (Option 1A) | No |
| **Uses AI HAT+** | YOLO | ViT |
| **Timeline** | 11 weeks | 14 weeks |
| **Risk** | Low-Medium | Medium-High |
| **Innovation** | High (Diff/Optical Flow) | Very High (ViT) |
| **Speed** | 0.4-1.25 m/s (3 options) | 1.25 m/s |
| **Motor Options** | 3 choices | 1 choice |
| **JSON Config** | Yes | No |
| **On-Site Changes** | 30 seconds | Impossible |
| **Surprise Rules** | Easy | Cannot adapt |
| **Flexibility** | Excellent | Poor |
| **Win Probability** | High | Low (due to inflexibility) |
| **Best for** | **WRO Competitions** | Research/Fixed tracks |

---

## Recommendation

### Proposal 4 (ROS2 Edge Racer) is the only viable choice.

**Critical Reasons:**

1. **Meets your configuration requirement.**
   - JSON/YAML configuration (exactly what you asked for)
   - Edit config file, restart robot (30 seconds)
   - No programming on-site
   - Handles WRO surprise rules

2. **Handles Surprise Rules completely.**
   - YOLO detects objects (detection does not change)
   - Config decides actions (change anytime)
   - Behavior trees for complex logic (XML configs)
   - Example: "Turn RIGHT at green" -> Edit one line in YAML

3. **Hybrid Intelligence Approach**
   - ML where it helps (YOLO object detection)
   - Rules where flexibility needed (action decisions)
   - Best of both worlds

4. **Uses All Your Hardware**
   - AI HAT+ 26 TOPS for YOLO (not wasted)
   - 16GB Pi5 runs ROS2 smoothly
   - All sensors integrated

5. **Professional Framework**
   - ROS2 designed for robotics competitions
   - Industry standard tools
   - Excellent documentation

6. **Motor Flexibility**
   - INJORA + PMW3901 ($38) - Fast + Innovation
   - OR Build HAT + LEGO ($53-73) - Easy + Proven
   - Both work with ROS2

7. **Modest Cost and Timeline**
   - $38-73 total investment
   - 11-week timeline (2-week buffer)

**Why Proposal 3 (ViT) fails your requirement:**
- ML behaviors are **fixed after training**
- Cannot change actions via config
- Would need retraining for surprise rules (impossible on-site)
- **Deal-breaker for WRO**

**Example On-Site Scenario:**
```
Competition announces: "Green cubes now worth 2x points, must stop 5 seconds"

With Proposal 4:
1. Edit mission_config.yaml (30 seconds)
2. Change: stop_duration: 3.0 -> 5.0
3. Restart robot (10 seconds)
4. Ready to compete

With Proposal 3:
1. Model learned to stop for 3 seconds
2. Cannot override without retraining
3. Retraining impossible on-site
4. You are stuck
```

---

## Next Steps

### Week 1 Actions:

1. **Order hardware** - Choose your motor option:

   **Option 1A (Top Choice):** Full LEGO Differential
   - [ ] 2x LEGO Large Motor 88013 ($40-60)
   - [ ] Raspberry Pi Build HAT ($25-35)
   - [ ] TCS34725 color sensor ($8)
   - **Total: $73-103**
   - Best for: **Using your Bugatti Bolide parts + Simplest setup + Zero turning radius**

   **Option 1B (Speed Choice):** INJORA + Optical Flow
   - [ ] TCS34725 color sensor ($8)
   - [ ] PMW3901 optical flow sensor ($30)
   - **Total: $38**
   - Best for: **Racing speed + Innovation + Lowest cost**

   **Option 3 (Budget):** INJORA Only
   - [ ] TCS34725 color sensor ($8)
   - **Total: $8**
   - Best for: **Absolute budget constraint**

2. **Install ROS2 Humble:**
   ```bash
   # On Raspberry Pi 5
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

3. **Test Hailo:**
   - [ ] Verify AI HAT+ working
   - [ ] Test YOLO inference (adapt from CLIP)

4. **Review VoldemorBot code:**
   - [ ] USB-CDC protocol
   - [ ] Challenge handlers
   - [ ] Hailo CLIP integration

5. **Create project structure:**
   ```bash
   mkdir -p ~/teamvoldemor_ros2_ws/src
   cd ~/teamvoldemor_ros2_ws
   ```

6. **Motor options (all work with ROS2):**

   **Top Choice:** Full LEGO Differential
   - 2x LEGO motors (differential steering)
   - Uses your Bugatti Bolide parts
   - Simplest setup (no steering mechanism)
   - Zero turning radius (spin in place)
   - Dual encoder odometry
   - Total: $73-103

   **Speed Choice:** INJORA + PMW3901
   - Keep INJORA 180 motor (1.25 m/s)
   - Add PMW3901 for odometry (slip-immune)
   - Lowest cost option with speed
   - Total: $38

   **Budget Choice:** INJORA Only
   - Zero new motor cost
   - Relies on IMU/LiDAR odometry
   - Total: $8

---

## You Are Ready

**Key advantages:**
- You have all hardware needed (except $38-103 in sensors/motors)
- You have 16GB Pi5 (better than assumed)
- You have AI HAT+ 26 TOPS (better than assumed)
- You have **LEGO Technic Bugatti Bolide** (905 parts - ideal for chassis)
- You have **working INJORA motor** (1.25 m/s - competitive for racing)
- You already know Hailo SDK
- You have working Go codebase to learn from
- You only need **$38-103 in new hardware** depending on motor choice

**Motor options for ROS2 Edge Racer:**
- **Full LEGO Differential** ($73-103) - Uses your Bugatti parts + Simplest + Zero turning radius
- **INJORA + PMW3901** ($38) - Speed + Innovation + Lowest cost
- **INJORA Only** ($8) - Budget option

Choose Proposal 4 (ROS2 Edge Racer) and make it happen.

---

## Full Documentation

- **Configuration Flexibility:** [04-configuration-flexibility.md](docs/proposals/comparisons/04-configuration-flexibility.md) - **Read this first**
- **LEGO Differential Setup:** [03-lego-dual-motor-build-hat.md](docs/proposals/hardware/03-lego-dual-motor-build-hat.md) - **Uses your Bugatti parts**
- **Comparison:** [01-proposal-comparison.md](docs/proposals/comparisons/01-proposal-comparison.md)
- **Hardware Analysis:** [03-actual-hardware-analysis.md](docs/proposals/comparisons/03-actual-hardware-analysis.md)
- **Proposal 4 Details:** [04-ros2-edge-racer-hybrid.md](docs/proposals/systems/04-ros2-edge-racer-hybrid.md)
- **Proposal 3 Details:** [03-cognitive-racer-vit.md](docs/proposals/systems/03-cognitive-racer-vit.md)
- **Motor Comparison:** [01-motor-comparison.md](docs/proposals/hardware/01-motor-comparison.md)
- **Build HAT Analysis:** [02-build-hat-motor-option.md](docs/proposals/hardware/02-build-hat-motor-option.md)

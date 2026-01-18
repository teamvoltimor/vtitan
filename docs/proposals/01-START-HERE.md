# 🚀 START HERE - WRO Future Engineers 2026 Proposals

**Last Updated:** 2026-01-17

Welcome! This guide will help you navigate all the proposal documents and make the best decision for your robot.

---

## 📋 What We've Organized

All proposals have been reorganized into clear categories:

```
proposals/
├── 01-start-here.md              ← YOU ARE HERE
├── README.md                      ← Folder structure overview
│
├── hardware/                      ← Hardware platform options
│   ├── README.md                  ← Complete hardware guide
│   ├── 01-motor-comparison.md
│   ├── 02-build-hat-motor-option.md
│   ├── 03-lego-dual-motor-build-hat.md
│   └── 04-hailo-calibration-guide.md   ← GPU calibration for AI HAT+
│
├── software/                      ← Software stack options
│   ├── README.md                  ← Complete software guide
│   └── 01-simulation-strategy.md       ← Testing with Gazebo
│
├── systems/                       ← Complete robot proposals
│   ├── 01-velocity-edge-ros2.md        (Jetson - NOT recommended)
│   ├── 02-minimalist-racer-classical.md (Fast but wastes AI HAT+)
│   ├── 03-cognitive-racer-vit.md        (Innovative ML approach)
│   └── 04-ros2-edge-racer-hybrid.md     (RECOMMENDED!)
│
└── comparisons/                   ← Decision tools & analysis
    ├── 01-proposal-comparison.md        (Full comparison matrix)
    ├── 02-technology-choices.md
    ├── 03-actual-hardware-analysis.md   (Your Klevor hardware!)
    └── 04-configuration-flexibility.md
```

---

## 🎯 Quick Start: 3-Step Decision Process

### Step 1: Understand Your Hardware

**Read:** `/hardware/README.md`

**Key Insight:** You have **Raspberry Pi 5 16GB + AI HAT+ 26 TOPS!**
- This is BETTER than what the original proposals assumed
- Don't waste money on Jetson Orin Nano!
- Use your existing Klevor hardware (100% reuse possible)

**New Hardware Cost:** $0-38 (depending on proposal)

---

### Step 2: Choose Your Software Approach

**Read:** `/software/README.md`

**Three main options:**

1. **ROS2 + YOLO (Proposal 4)** ⭐⭐ RECOMMENDED
   - Professional framework
   - YOLO on your AI HAT+ 26 TOPS
   - Flexible (Classical CV fallback)
   - Best balance of innovation + reliability

2. **Classical CV (Proposal 2)** ⭐ Fast but wastes AI HAT+
   - Fastest (200 Hz, 3.0 m/s)
   - Simplest development
   - ⚠️ Doesn't use your $70 AI HAT+ hardware!

3. **Vision Transformer (Proposal 3)** ⭐⭐ Most Innovative
   - Cutting-edge AI
   - Uses AI HAT+ 26 TOPS fully
   - Higher risk (ML training)
   - You already have Hailo CLIP experience!

---

### Step 3: Review Complete System Proposals

**Read:** `/systems/` folder

**Rankings (based on YOUR actual hardware):**

#### 🥇 #1: Proposal 4 - ROS2 Edge Racer
**File:** `systems/04-ros2-edge-racer-hybrid.md`

**Why #1:**
- ✅ Uses AI HAT+ 26 TOPS (YOLO inference)
- ✅ 100% hardware reuse
- ✅ Cost: $8 (optional color sensor)
- ✅ Professional ROS2 framework
- ✅ Balanced risk/reward
- ✅ 11-week timeline

**Best for:** Most teams - balanced approach

---

#### 🥈 #2: Proposal 3 - Cognitive Racer
**File:** `systems/03-cognitive-racer-vit.md`

**Why #2:**
- ✅ Uses AI HAT+ 26 TOPS fully (ViT)
- ✅ You already have Hailo CLIP experience!
- ✅ 100% hardware reuse
- ✅ Most innovative
- ⚠️ Higher risk (ML training)

**Best for:** Teams with ML expertise wanting cutting-edge innovation

---

#### 🥉 #3: Proposal 2 - Minimalist Racer
**File:** `systems/02-minimalist-racer-classical.md`

**Why dropped to #3:**
- ✅ Fastest (3.0 m/s, 200 Hz)
- ✅ Lowest risk
- ✅ Unique color sensor validation
- ❌ **WASTES your AI HAT+ 26 TOPS!**

**Modified recommendation:** Add YOLO backup to use AI HAT+

**Best for:** Speed-focused teams willing to add AI fallback

---

#### ❌ #4: Proposal 1 - Velocity Edge
**File:** `systems/01-velocity-edge-ros2.md`

**Why DON'T choose:**
- ❌ Requires Jetson ($500) when you have AI HAT+ 26 TOPS
- ❌ Only 60% reuse
- ❌ Makes no sense for your hardware

**Verdict:** Ignore this proposal!

---

## 🛠️ Supporting Documents

### Simulation Strategy
**File:** `software/01-simulation-strategy.md`

**What it covers:**
- How to use Gazebo simulator
- Test your code BEFORE building hardware
- Create WRO track in simulation
- Generate training data for ML
- Sim-to-real transfer strategies

**Why important:**
- Test 1000+ scenarios safely
- Faster iteration (10× vs real robot)
- Required for ML training (Proposal 3)
- Impressive journal content

**Timeline:** Start simulation in Week 1-2!

---

### Hailo GPU Calibration
**File:** `hardware/04-hailo-calibration-guide.md`

**What it covers:**
- How to calibrate AI models for Hailo AI HAT+
- GPU requirements (RTX 3060+, 32GB RAM)
- Calibration dataset (1000+ images)
- Step-by-step YOLO → Hailo workflow
- Expected performance (40-60 FPS YOLO!)

**Why important:**
- Required for Proposals 3 & 4 (AI-based)
- 10-50× faster with GPU vs CPU
- Proper calibration = <1% accuracy loss

**Timeline:** Needed in Weeks 5-6 (after training)

---

## 📊 Comparison Tools

### Main Comparison Matrix
**File:** `comparisons/01-proposal-comparison.md`

**66-page comprehensive comparison** including:
- Side-by-side feature comparison
- Updated rankings based on YOUR hardware
- Decision matrices
- Scenario-based recommendations
- Cost breakdowns
- Risk analysis

**Read this if:** You want ALL the details

---

### Your Actual Hardware Analysis
**File:** `comparisons/05-actual-hardware-analysis.md`

**What changed:**
- Original: Hailo-8L (13 TOPS) + Pi5 8GB
- **Actual: AI HAT+ 26 TOPS + Pi5 16GB!** ⭐⭐

**Impact:**
- Proposals 3 & 4 jumped to #1 and #2
- Proposal 1 (Jetson) is now wasteful
- You can run larger models faster!

---

## 🎓 Learning Path

### For Beginners
1. Read `/hardware/README.md` (understand your hardware)
2. Read `/software/README.md` (understand software options)
3. Read `systems/04-ros2-edge-racer-hybrid.md` (recommended proposal)
4. Read `11-simulation-strategy.md` (how to test)
5. Start building!

### For ML Enthusiasts
1. Read `comparisons/05-actual-hardware-analysis.md` (your AI HAT+ power!)
2. Read `systems/03-cognitive-racer-vit.md` (ViT proposal)
3. Read `12-hailo-calibration-guide.md` (how to use AI HAT+)
4. Read `11-simulation-strategy.md` (RL training in Gazebo)
5. Start collecting data!

### For Speed-Focused Teams
1. Read `systems/02-minimalist-racer-classical.md`
2. **IMPORTANT:** Add YOLO backup to use AI HAT+ (don't waste it!)
3. Read `11-simulation-strategy.md` (test HSV CV)
4. Start building for speed!

---

## 🚦 Decision Flowchart

```
START
  |
  ├─ Do you have ML/PyTorch experience?
  │   ├─ YES → Proposal 3 (Cognitive Racer - ViT)
  │   └─ NO → Continue
  |
  ├─ Do you want to learn professional ROS2?
  │   ├─ YES → Proposal 4 (ROS2 Edge Racer) ⭐ RECOMMENDED
  │   └─ NO → Continue
  |
  ├─ Is maximum speed your #1 priority?
  │   ├─ YES → Proposal 2 (Minimalist) + ADD YOLO backup
  │   └─ NO → Proposal 4 (ROS2 Edge Racer) ⭐ DEFAULT
```

---

## ⚡ TL;DR - Just Tell Me What to Do!

### RECOMMENDED: Proposal 4 - ROS2 Edge Racer

**Configuration:**
```yaml
Hardware:
  - Raspberry Pi 5 16GB (from Klevor)
  - Hailo AI HAT+ 26 TOPS (from Klevor)
  - RPi Camera Module 3 Wide (from Klevor)
  - RPLiDAR C1 (from Klevor)
  - BNO08X IMU (from Klevor)
  - 2× VL53L0X (from Klevor)
  - Pico 2W (from Klevor)
  - TCS34725 color sensor (NEW - $8)

Software:
  - ROS2 Humble
  - YOLOv8-Nano on Hailo AI HAT+ (30-60 FPS)
  - Classical HSV CV (fallback, 120 FPS)
  - TCS34725 physical validation
  - 50 Hz control loop

Timeline: 11 weeks
Cost: $8
Reuse: 100% base hardware
Win Probability: ⭐⭐⭐⭐⭐
```

**What to do RIGHT NOW:**

1. **Week 1:** Read full proposal (`systems/04-ros2-edge-racer-hybrid.md`)
2. **Week 2:** Set up simulation (`11-simulation-strategy.md`)
3. **Week 3:** Install ROS2 on Pi5
4. **Week 4:** Test sensors in simulation
5. **Week 5:** Collect training data (1000+ images)
6. **Week 6:** Train YOLO & calibrate with Hailo (`12-hailo-calibration-guide.md`)
7. **Weeks 7-11:** Build, test, iterate!

---

## 📞 Need Help?

### If you're confused about...

**Hardware choices:**
→ Read `/hardware/README.md`

**Software frameworks:**
→ Read `/software/README.md`

**Which proposal to choose:**
→ Read `comparisons/01-proposal-comparison.md`

**How to use your AI HAT+:**
→ Read `12-hailo-calibration-guide.md`

**How to test without hardware:**
→ Read `11-simulation-strategy.md`

**Everything at once:**
→ Start with THIS file (01-start-here.md) and follow the Learning Path above!

---

## ✅ Next Steps

- [ ] Read recommended proposal (Proposal 4)
- [ ] Review simulation strategy
- [ ] Review Hailo calibration guide
- [ ] Order TCS34725 color sensor ($8)
- [ ] Set up desktop GPU machine (for Hailo calibration)
- [ ] Install Gazebo + ROS2 (for simulation)
- [ ] Start building! 🚀

---

**Good luck with WRO Future Engineers 2026!**

**Your hardware is excellent - use it wisely!** 💪🤖

---

## 📄 Document Map

Quick reference to all files:

| File | Purpose |
|------|---------|
| `01-start-here.md` | **YOU ARE HERE** - Navigation guide |
| `README.md` | Folder structure overview |
| `/hardware/README.md` | Complete hardware options guide |
| `/software/README.md` | Complete software options guide |
| `/systems/04-ros2-edge-racer-hybrid.md` | **RECOMMENDED PROPOSAL** |
| `/systems/03-cognitive-racer-vit.md` | Alternative: ML/ViT approach |
| `/systems/02-minimalist-racer-classical.md` | Alternative: Speed-focused |
| `/systems/01-velocity-edge-ros2.md` | ❌ Don't choose (Jetson) |
| `/comparisons/01-proposal-comparison.md` | Full comparison matrix |
| `/comparisons/03-actual-hardware-analysis.md` | Your Klevor hardware analysis |
| `software/01-simulation-strategy.md` | Gazebo simulation guide |
| `hardware/04-hailo-calibration-guide.md` | GPU calibration for AI HAT+ |

**Total pages:** ~250+ pages of comprehensive analysis!

**Time to read everything:** 4-6 hours
**Time to read essentials:** 1-2 hours (this file + Proposal 4 + Simulation + Hailo)

---

**Happy building! 🎉**

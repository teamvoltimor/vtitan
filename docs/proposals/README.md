# Proposals Organization

This folder contains all proposals for the WRO Future Engineers 2026 robot, organized by category for easy navigation.

## 📁 Folder Structure

### `/hardware` - Hardware Configuration Options
Hardware platforms, sensors, actuators, and power systems.

- **Platform Options:**
  - Pi5 + AI HAT+ 26 TOPS (Recommended - 100% reuse from VoldemorBot)
  - Jetson Orin Nano (Not recommended - wastes existing hardware)

- **Sensor Configurations:**
  - Base sensors (from VoldemorBot): Camera, LiDAR, IMU, Distance
  - Optional additions: TCS34725 color sensor, PMW3901 optical flow

- **Motor Options:**
  - LEGO SPIKE Prime motors
  - LEGO EV3 motors
  - LEGO Build HAT integration

- **Guides:**
  - `04-hailo-calibration-guide.md` - GPU calibration for AI HAT+

### `/software` - Software Stack Options
Programming frameworks, vision algorithms, and control strategies.

- **Framework Options:**
  - ROS2 Humble (Professional, modular)
  - FreeRTOS (Bare-metal, fastest)
  - Custom Python/PyTorch (ML research)

- **Vision Approaches:**
  - YOLO on Hailo (AI-based, robust)
  - Classical CV (HSV, fast, explainable)
  - Vision Transformer (Cutting-edge ML)

- **Guides:**
  - `01-simulation-strategy.md` - Testing with Gazebo before building

### `/systems` - Complete System Proposals
Full robot proposals combining hardware + software.

- Proposal 1: Velocity Edge (Jetson + ROS2) - ❌ Not recommended
- Proposal 2: Minimalist Racer (Pi5 + Classical CV) - ⚠️ Wastes AI HAT+
- Proposal 3: Cognitive Racer (Pi5 + ViT) - ⭐ Innovative
- Proposal 4: ROS2 Edge Racer (Pi5 + ROS2) - ⭐⭐ Recommended

### `/comparisons` - Analysis & Decision Tools
Comparison matrices, decision guides, and recommendations.

## 🎯 Quick Navigation

### If you want to...

**Understand hardware options:**
→ See `/hardware`

**Understand software approaches:**
→ See `/software`

**See complete robot proposals:**
→ See `/systems`

**Compare and decide:**
→ See `/comparisons`

## 📊 Current Recommendation

**Hardware:** Pi5 16GB + AI HAT+ 26 TOPS (100% reuse from VoldemorBot)
**Software:** ROS2 + YOLO (Hailo) + Classical CV fallback
**Proposal:** #4 (ROS2 Edge Racer)
**Cost:** $8 (TCS34725 color sensor)
**Win Probability:** ⭐⭐⭐⭐⭐

See `/comparisons/01-proposal-comparison.md` for full analysis.

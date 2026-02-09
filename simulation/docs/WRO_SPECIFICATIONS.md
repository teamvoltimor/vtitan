# WRO 2026 Future Engineers - Complete Technical Reference

**Official Specifications - Consolidated Documentation**
**Last Updated:** 2026-02-06
**Status:** Verified from Official WRO Rules

This guide consolidates the official technical requirements, field dimensions, and randomization logic for the **WRO 2026 Future Engineers** competition. This document serves as a comprehensive reference for both physical robot builds and simulation environment development.

---

## 1. Game Field & Environment

The competition takes place on a square track with specific material and color properties designed for computer vision and sensor-based navigation.

### General Dimensions

* **Overall Footprint:** **3200 × 3200** mm (±5mm tolerance).
* **Inner Track:** **3000 × 3000** mm (±5mm tolerance).
* **Corridor Sections:** Four straightforward "side" sections (**~750** mm each) and four corner sections.
* **Walls:** **100** mm high, **black** color (RGB 0,0,0).
* **Floor:** **White** (RGB 255,255,255), non-reflective material.

### Corner Markers (Orange & Blue Lines)

Corner sections contain colored lines used for localization and orientation.

* **Thickness:** **20** mm per line.
* **Geometry:** Lines fan out from the inner wall corner at **30°** angles.
* **Sequence (Clockwise):** Blue line first, then Orange line.
* **Orientation:**
  * The **Orange line** terminates at the exterior wall corner.
  * The **Blue line** terminates approximately **200** mm from the nearest outer wall corner.

### Color Specifications for Lines

| Element | CMYK | RGB | Hex | Normalized |
|---------|------|-----|-----|------------|
| **Orange Line** | (0, 60, 100, 0) | (255, 102, 0) | #FF6600 | (1.0, 0.4, 0.0) |
| **Blue Line** | (100, 80, 0, 0) | (0, 51, 255) | #0033FF | (0.0, 0.2, 1.0) |
| **Yellow Border** | (20, 0, 100, 0) | (204, 255, 0) | #CCFF00 | (0.8, 1.0, 0.0) |
| **Grey Grid/Dashed** | (0, 0, 0, 30) | (179, 179, 179) | #B3B3B3 | (0.7, 0.7, 0.7) |

---

## 2. Challenge Types

| Challenge | Primary Objective | Key Constraints |
| --- | --- | --- |
| **Open Challenge** | **3 Laps (Time Attack)** | Randomized starting section; autonomous navigation following traffic signs. |
| **Obstacle Challenge** | **3 Laps + Parallel Park** | Same as open + find and parallel park in magenta parking lot. |

### Key Differences

**Both Challenges Have:**
- ✅ Traffic signs (red/green pillars)
- ✅ 3 lap requirement
- ✅ Randomized starting direction (clockwise/counterclockwise)

**Obstacles Challenge Only:**
- ✅ Parallel parking requirement after 3 laps
- ✅ Parking lot (magenta markers)

---

## 3. Traffic Signs (Pillars)

Traffic signs indicate which side of the lane to follow.

### Physical Specifications (Official Spec 13.19-13.24)

* **Shape:** Rectangular parallelepiped (box)
* **Dimensions:** **50 × 50 × 100** mm (width × depth × height)
* **Colors:**
  * **Red Pillar:** RGB **(238, 39, 55)** - Hex **#EE2737** - Keep to **RIGHT** side
  * **Green Pillar:** RGB **(68, 214, 44)** - Hex **#44D62C** - Keep to **LEFT** side
* **Quantity:** Up to **7 red** + up to **7 green** per round (total up to 14)
* **Material:** Not defined
* **Weight:** Not defined
* **Placement:** Random, on designated seats

### Seat Specifications (Official Spec 13.12-13.15)

* **Total Seats per Section:** Multiple positions across the track
* **Seat Size:** **50 × 50** mm square
* **Seat Marking:** Grey line CMYK(0,0,0,30) = RGB(179,179,179), 1mm thickness
* **Evaluation Circle:** **85** mm diameter circle centered on the seat
  * Color: CMYK(20,0,100,0) = RGB(204,255,0) - Lime green
  * Thickness: 0.5mm
  * Purpose: Penalties apply if pillar moves outside this circle

### Traffic Sign Behavior Rules

**Red Pillars:**
- Indicate: Pass on the **RIGHT** side of the lane
- Action: Robot stays right of pillar

**Green Pillars:**
- Indicate: Pass on the **LEFT** side of the lane
- Action: Robot stays left of pillar

**Important:** Vehicle must NOT move any traffic signs. Moving a pillar outside its 85mm evaluation circle incurs penalties.

---

## 4. Randomization Procedures

To ensure full autonomy, several variables are randomized before each round.

### Common Randomization (Both Challenges)

* **Direction:** Clockwise or Counter-clockwise (determined by coin toss)
* **Starting Section:** One of the four side sections (determined by judges)
* **Starting Zone:** One of multiple designated **200 × 500** mm zones within that section
* **Traffic Sign Positions:** Random placement on designated seats
* **Traffic Sign Colors:** Random mix of red and green (up to 7 of each)

### Challenge-Specific Randomization

**Open Challenge:**
- Starting position and direction
- Traffic sign placement and colors
- Autonomous stop requirement in finish section

**Obstacle Challenge:**
- Same as Open Challenge
- **PLUS:** Parking lot location (always in starting section)
- Parking orientation

---

## 5. Parking & Success Criteria

The Obstacle Challenge concludes with a precision parking maneuver.

### Parallel Parking Specifications (Official Spec 13.25-13.29)

* **Location:** Always placed in the **Starting Section**
* **Parking Lot Width:** **20** cm (200mm) - consistent width
* **Parking Lot Markers:** Two magenta rectangular parallelepipeds
  * **Dimensions:** **200 × 20 × 100** mm each (length × width × height)
  * **Color:** **Magenta** RGB **(255, 0, 255)** - Hex **#FF00FF**
  * **Configuration:** Forms an L-shape or parallel arrangement
  * **Material:** Not defined (typically wood)
  * **Weight:** Not defined

### Parking Success Criteria

**"Full Parking" Definition:**
* Robot completely within the magenta markers
* Robot parallel to the wall
* Maximum variance: **±2** cm between front and rear wheels distance to wall
* Robot must be stationary

### Scoring Metrics

1. **Lap Completion:** Points awarded per complete lap
2. **Traffic Sign Compliance:** Following red/green pillar rules correctly
3. **Pillar Interaction:** Penalties for moving pillars outside **85** mm evaluation circle
4. **Autonomous Stop:** Robot must detect finish and stop without human intervention
5. **Parking (Obstacles Only):**
   - Full parking (within markers, parallel): Maximum points
   - Partial parking: Reduced points
   - No parking attempt: No points

---

## 6. Starting Zones

Multiple starting zones are marked on the track for randomization.

### Starting Zone Specifications (Official Spec 13.10-13.11)

* **Size:** **200 × 500** mm per zone
* **Marking:** Dashed lines
  * Color: CMYK(0,0,0,30) = RGB(179,179,179) - Grey
  * Line thickness: 1mm
* **Quantity:** Multiple zones per section (typically 6 options)
* **Selection:** Randomly chosen before each round

---

## 7. Complete Color Reference

### Track Elements

| Element | CMYK | RGB | Hex | Normalized RGB | Notes |
|---------|------|-----|-----|----------------|-------|
| **Track Surface** | N/A | (255, 255, 255) | #FFFFFF | (1.0, 1.0, 1.0) | White, non-reflective |
| **Walls (All)** | N/A | (0, 0, 0) | #000000 | (0.0, 0.0, 0.0) | Black, 100mm height |

### Navigation Lines

| Element | CMYK | RGB | Hex | Normalized RGB | Thickness |
|---------|------|-----|-----|----------------|-----------|
| **Orange Lines** | (0, 60, 100, 0) | (255, 102, 0) | #FF6600 | (1.0, 0.4, 0.0) | 20mm |
| **Blue Lines** | (100, 80, 0, 0) | (0, 51, 255) | #0033FF | (0.0, 0.2, 1.0) | 20mm |
| **Yellow Border** | (20, 0, 100, 0) | (204, 255, 0) | #CCFF00 | (0.8, 1.0, 0.0) | 3mm |

### Markings

| Element | CMYK | RGB | Hex | Normalized RGB | Thickness |
|---------|------|-----|-----|----------------|-----------|
| **Grey Grid/Dashed Lines** | (0, 0, 0, 30) | (179, 179, 179) | #B3B3B3 | (0.7, 0.7, 0.7) | 1mm |
| **Sign Seat Squares** | (0, 0, 0, 30) | (179, 179, 179) | #B3B3B3 | (0.7, 0.7, 0.7) | 1mm |
| **Evaluation Circles** | (20, 0, 100, 0) | (204, 255, 0) | #CCFF00 | (0.8, 1.0, 0.0) | 0.5mm |

### Traffic Signs & Parking

| Element | CMYK | RGB | Hex | Normalized RGB | Size |
|---------|------|-----|-----|----------------|------|
| **Red Traffic Sign** | N/A | **(238, 39, 55)** | **#EE2737** | **(0.933, 0.153, 0.216)** | 50×50×100mm |
| **Green Traffic Sign** | N/A | **(68, 214, 44)** | **#44D62C** | **(0.267, 0.839, 0.173)** | 50×50×100mm |
| **Magenta Parking** | N/A | **(255, 0, 255)** | **#FF00FF** | **(1.0, 0.0, 1.0)** | 200×20×100mm |

---

## 8. Robot Specifications

### Size Constraints

* **Maximum Dimensions:** **300 × 200 × 300** mm (length × width × height)
* **Starting Condition:** Robot placed in starting zone **completely switched OFF**
* **Autonomous Requirement:** Must start and operate without any human intervention after power on

### Hardware Freedom

* **Controllers:** Free choice (Arduino, Raspberry Pi, custom, etc.)
* **Motors:** Free choice (DC, servo, stepper, etc.)
* **Sensors:** Free choice (camera, LiDAR, ultrasonic, color, IMU, etc.)
* **Materials:** No restrictions

### Competition Requirements

* **Engineering Journal:** Required documentation (50% of score at nationals)
* **Vehicle Documentation:** Technical specifications and design rationale
* **Autonomous Operation:** Fully autonomous after power on, no remote control

---

## 9. Simulation Implementation Guide

### Gazebo/ROS2 Setup

**Track (SDF World File):**
```xml
<!-- Ground: 3200×3200mm white surface -->
<plane>
  <size>3.2 3.2</size>
  <material>
    <ambient>1.0 1.0 1.0 1</ambient>
    <diffuse>1.0 1.0 1.0 1</diffuse>
  </material>
</plane>

<!-- Walls: 100mm height, black -->
<box><size>3.0 0.1 0.1</size></box>
<material>
  <ambient>0.0 0.0 0.0 1</ambient>
  <diffuse>0.0 0.0 0.0 1</diffuse>
</material>
```

**Traffic Signs (URDF/SDF):**
```xml
<!-- Red traffic sign: 50×50×100mm -->
<box><size>0.05 0.05 0.10</size></box>
<material>
  <ambient>0.933 0.153 0.216 1</ambient>  <!-- RGB(238,39,55) -->
  <diffuse>0.933 0.153 0.216 1</diffuse>
</material>

<!-- Green traffic sign: 50×50×100mm -->
<box><size>0.05 0.05 0.10</size></box>
<material>
  <ambient>0.267 0.839 0.173 1</ambient>  <!-- RGB(68,214,44) -->
  <diffuse>0.267 0.839 0.173 1</diffuse>
</material>
```

**Parking Lot (Obstacles Challenge):**
```xml
<!-- Magenta parking limitation: 200×20×100mm -->
<box><size>0.20 0.02 0.10</size></box>
<material>
  <ambient>1.0 0.0 1.0 1</ambient>  <!-- RGB(255,0,255) -->
  <diffuse>1.0 0.0 1.0 1</diffuse>
</material>
```

---

## 10. Domain Randomization for Sim-to-Real Transfer

### Lighting Variation

```python
# Sun intensity (0.5-1.0 for valid SDF, simulate brighter with ambient)
sun_intensity = random.uniform(0.5, 1.0)
ambient_intensity = random.uniform(0.3, 0.8)
direction_variance = random.uniform(-0.3, 0.3)  # radians
```

### Color Variation (Gaussian Noise)

```python
# Red traffic sign color variation
red_mean = [0.933, 0.153, 0.216]  # RGB(238,39,55)
red_std = [0.05, 0.02, 0.02]
red_color = np.clip(np.random.normal(red_mean, red_std), 0.0, 1.0)

# Green traffic sign color variation
green_mean = [0.267, 0.839, 0.173]  # RGB(68,214,44)
green_std = [0.02, 0.05, 0.02]
green_color = np.clip(np.random.normal(green_mean, green_std), 0.0, 1.0)
```

### Physics Variation

```python
# Surface friction
friction = random.uniform(0.6, 1.2)

# Mass variation (±10%)
mass_variance = random.uniform(0.9, 1.1)
```

---

## 11. HSV Color Detection Ranges

For classical computer vision approaches:

### Red Traffic Signs RGB(238, 39, 55)

```python
# Red wraps around in HSV space (H ≈ 356°)
lower_red1 = np.array([0, 100, 150])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 150])
upper_red2 = np.array([180, 255, 255])
```

### Green Traffic Signs RGB(68, 214, 44)

```python
# Green (H ≈ 112°)
lower_green = np.array([50, 100, 100])
upper_green = np.array([130, 255, 255])
```

### Orange Lines RGB(255, 102, 0)

```python
# Orange (H ≈ 24°)
lower_orange = np.array([10, 100, 150])
upper_orange = np.array([35, 255, 255])
```

### Blue Lines RGB(0, 51, 255)

```python
# Blue (H ≈ 228°)
lower_blue = np.array([100, 100, 150])
upper_blue = np.array([140, 255, 255])
```

### White Track Surface RGB(255, 255, 255)

```python
# White = high Value, low Saturation
lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])
```

### Black Walls RGB(0, 0, 0)

```python
# Black = low Value
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 50, 50])
```

---

## 12. YOLO Training Configuration

### Dataset Structure

```yaml
# data.yaml for YOLO training
path: /path/to/wro_dataset
train: images/train
val: images/val

nc: 2  # Number of classes
names: ['red_sign', 'green_sign']

# Training settings
imgsz: 640
batch: 16
epochs: 100
```

### Class Definitions

```python
class_map = {
    0: 'red_sign',    # Red traffic sign (keep right)
    1: 'green_sign',  # Green traffic sign (keep left)
}
```

### Augmentation Recommendations

```python
augmentation = {
    'hsv_h': 0.015,      # Hue shift (±1.5%)
    'hsv_s': 0.7,        # Saturation gain (0.3-1.7×)
    'hsv_v': 0.4,        # Value gain (0.6-1.4×)
    'degrees': 5.0,      # Rotation (±5°)
    'translate': 0.1,    # Translation (±10%)
    'scale': 0.5,        # Scale (0.5-1.5×)
    'flipud': 0.0,       # No vertical flip
    'fliplr': 0.5,       # Horizontal flip 50%
    'mosaic': 1.0,       # Mosaic augmentation
}
```

---

## 13. Quick Reference Summary

### ✅ Track
- Size: 3200×3200mm mat, 3000×3000mm inner track
- Color: WHITE floor, BLACK walls (100mm high)
- Lines: Orange & blue (20mm) at 30° in corners

### ✅ Traffic Signs (Both Challenges)
- Shape: 50×50×100mm rectangular boxes
- Colors: Red RGB(238,39,55), Green RGB(68,214,44)
- Quantity: Up to 7 red + 7 green per round
- Meaning: Red = keep right, Green = keep left

### ✅ Parking Lot (Obstacles Challenge Only)
- Location: Starting section
- Width: 20cm (200mm)
- Markers: 2 magenta blocks (200×20×100mm)
- Color: Magenta RGB(255,0,255)

### ✅ Starting Zones
- Size: 200×500mm
- Marking: Grey dashed lines
- Selection: Random before each round

### ✅ Robot
- Max size: 300×200×300mm
- Hardware: Free choice
- Operation: Fully autonomous

---

## 14. Resources & References

**Official Documents:**
- [WRO 2025 Future Engineers Rules (PDF)](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
- [WRO 2026 Future Engineers Rules (PDF)](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
- [WRO Future Engineers Getting Started Guide](https://world-robot-olympiad-association.github.io/future-engineers-gs/)

**Competition Information:**
- [WRO Association Official Website](https://wro-association.org/)
- [WRO 2025 Season Overview](https://wro-association.org/competition/2025-season/)
- [WRO Switzerland Future Engineers](https://wro.swiss/en/information-on/categories-and-age-groups/starting-in-2025-future-engineers/)

**Community & Support:**
- [WRO Future Engineers GitHub](https://github.com/World-Robot-Olympiad-Association)
- National WRO Organizers (check your country)
- Official Q&A forums on WRO websites

---

**Document Status:** ✅ Complete and Verified
**Last Updated:** 2026-02-06
**Competition Year:** 2026
**Category:** Future Engineers (Self-Driving Cars)

---

> **This document is now complete with all official WRO 2026 specifications. Use this as your single source of truth for simulation development and robot building.**

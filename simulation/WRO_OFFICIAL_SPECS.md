# WRO Future Engineers Official Specifications

This document contains the official colors and specifications from the WRO Future Engineers competition rules.

## Official Track Colors (RGB Values)

### Track Surface
- **Floor/Mat**: Light grey
  - RGB: `(191, 191, 191)` or normalized `(0.75, 0.75, 0.75)`
  - Hex: `#BFBFBF`
  - Material: Non-reflective mat surface

### Boundary Walls
- **Outer Walls**: Pure black
  - RGB: `(0, 0, 0)`
  - Hex: `#000000`
  - Height: 30cm (300mm)
  - Thickness: ~5cm

### Parking Zone
- **Parking Walls**: Magenta
  - RGB: `(255, 0, 255)`
  - Hex: `#FF00FF`
  - Height: 15cm (150mm)
  - Depth/thickness: Varies by implementation

## Traffic Signs (Pillars)

### Pillar Specifications
- **Diameter**: 6cm (60mm)
  - Radius: 3cm in simulation
- **Height**: 30cm (300mm)
- **Material**: Cylindrical shape
- **Placement**: Random positions on track

### Pillar Colors

#### Red Pillar
- **RGB**: `(255, 0, 0)` - Pure red
- **Hex**: `#FF0000`
- **Meaning**: Indicates right turn direction
- **Detection**: Red color detection (HSV or YOLO)

#### Green Pillar
- **RGB**: `(0, 255, 0)` - Pure green
- **Hex**: `#00FF00`
- **Meaning**: Indicates left turn direction
- **Detection**: Green color detection (HSV or YOLO)

**IMPORTANT**: WRO uses ONLY red and green pillars. No blue pillars.

## Obstacles (Obstacles Challenge Only)

### Obstacle Block Specifications
- **Dimensions**: 10cm × 10cm × 10cm cube (100mm)
- **Material**: Solid block, movable
- **Weight**: Lightweight, can be pushed by robot

### Obstacle Colors
Obstacles can be:
- **Red blocks** - Same color as red pillars `(255, 0, 0)`
- **Green blocks** - Same color as green pillars `(0, 255, 0)`

**Rules**:
- Red obstacles: Pass on the right side
- Green obstacles: Pass on the left side
- Obstacles are placed randomly on track
- 3-6 obstacles per run (varies)

## Track Dimensions

### Overall Layout
- **Inner space**: 3m × 3m (3000mm × 3000mm)
- **Wall height**: 30cm (300mm)
- **Track material**: Non-reflective grey mat
- **Wall material**: Black painted walls or foam board

### Starting Zone
- **Location**: One corner of track (marked)
- **Dimensions**: Approximately 30cm × 30cm
- **Marking**: White line or tape
- **Robot orientation**: Facing into track (perpendicular to wall)

### Parking Zone (Some Challenges)
- **Location**: Corner opposite to start
- **Marked by**: Magenta colored walls
- **Dimensions**: Varies (typically 50cm × 50cm)
- **Purpose**: Final parking objective

## Lighting Conditions

### Competition Environment
- **Standard**: Indoor lighting, fluorescent or LED
- **Intensity**: Moderate, consistent across track
- **Shadows**: Minimal, avoid direct sunlight
- **Variability**: Can vary between venues

### Simulation Recommendations
To simulate realistic conditions:
- Sun intensity: 0.8-1.2 (80-120% of standard)
- Ambient light: 0.4-0.6 (40-60%)
- Direction variance: ±0.3 radians
- Add Gaussian noise to camera sensor

## Competition-Specific Rules

### Open Challenge
- **Traffic signs**: Red and green pillars only
- **Obstacles**: None
- **Objective**: Complete 3 laps following traffic signs
- **Direction**: Clockwise or counterclockwise (indicated by first sign)

### Obstacles Challenge
- **Traffic signs**: Red and green pillars
- **Obstacles**: 3-6 red/green blocks
- **Objective**: Complete 3 laps while avoiding obstacles
- **Rules**:
  - Pass red obstacles on the right
  - Pass green obstacles on the left
  - Must not touch obstacles

## Color Detection Best Practices

### HSV Color Ranges (Calibration Required)

**Red (HSV)**:
- Hue: 0-10 or 170-180 (red wraps around)
- Saturation: 100-255
- Value: 100-255

**Green (HSV)**:
- Hue: 40-80
- Saturation: 100-255
- Value: 100-255

**Black Walls (HSV)**:
- Hue: Any
- Saturation: 0-50
- Value: 0-50

**Grey Floor (HSV)**:
- Hue: Any
- Saturation: 0-30
- Value: 100-200

### YOLO Training Recommendations
- **Classes**: 3 (red, green, obstacle)
- **Image size**: 640×480 or 640×640
- **Epochs**: 100-200
- **Augmentation**:
  - Brightness: ±30%
  - Contrast: ±20%
  - Hue shift: ±5° (subtle)
  - Gaussian noise: σ=5-10

## Simulation Implementation

### Gazebo Color Settings (SDF)

**Light Grey Floor**:
```xml
<ambient>0.75 0.75 0.75 1</ambient>
<diffuse>0.75 0.75 0.75 1</diffuse>
<specular>0.05 0.05 0.05 1</specular>
```

**Black Walls**:
```xml
<ambient>0.0 0.0 0.0 1</ambient>
<diffuse>0.0 0.0 0.0 1</diffuse>
<specular>0.0 0.0 0.0 1</specular>
```

**Magenta Parking**:
```xml
<ambient>1.0 0.0 1.0 1</ambient>
<diffuse>1.0 0.0 1.0 1</diffuse>
<specular>0.3 0.3 0.3 1</specular>
```

**Red Pillar**:
```xml
<ambient>1.0 0.0 0.0 1</ambient>
<diffuse>1.0 0.0 0.0 1</diffuse>
<specular>0.2 0.2 0.2 1</specular>
```

**Green Pillar**:
```xml
<ambient>0.0 1.0 0.0 1</ambient>
<diffuse>0.0 1.0 0.0 1</diffuse>
<specular>0.2 0.2 0.2 1</specular>
```

## Domain Randomization for Sim-to-Real

To handle real-world color variations:

### Color Noise (Gaussian)
- Red channel: mean=1.0, std=0.05
- Green channel: mean=0.0, std=0.02 (for red pillars)
- Green channel: mean=1.0, std=0.05 (for green pillars)

### Lighting Variation
- Intensity multiplier: 0.5-1.5
- Direction variation: ±30°
- Ambient light: 30-80%

### Camera Noise
- Gaussian sensor noise: σ=0.007
- Motion blur (if moving fast)
- Exposure variation: ±20%

## Physical Measurements

### Confirmed Specifications
These are the official WRO measurements:

- ✅ Track inner space: 3m × 3m
- ✅ Wall height: 30cm
- ✅ Pillar diameter: 6cm
- ✅ Pillar height: 30cm
- ✅ Obstacle size: 10cm cube
- ✅ Colors: Black walls, light grey floor, magenta parking

### Robot Constraints
- Maximum size: 30cm × 20cm × 20cm (length × width × height)
- Maximum weight: No official limit (practical: <2kg)
- Starting position: Within starting zone

## References

- **WRO Official Website**: https://wro-association.org/
- **Future Engineers Rules**: Check latest competition year rules PDF
- **Technical Forum**: WRO Community Forums

## Updates and Variations

**Note**: Competition rules may vary slightly by year and region. Always verify with:
1. Current year's official rule book
2. Local/regional competition organizers
3. Practice track at your venue (if available)

**Calibration Recommendation**:
- Bring color calibration card to venue
- Test detection in actual lighting
- Adjust HSV thresholds on-site if needed

---

**Last Updated**: 2026-02-06
**Specification Version**: Based on WRO Future Engineers 2025/2026 season
**Document Maintainer**: TeamSteelBot

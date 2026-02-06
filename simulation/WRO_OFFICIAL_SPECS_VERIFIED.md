# WRO Future Engineers Official Specifications (VERIFIED)

**Source**: Official WRO Future Engineers Competition Rules
**Last Verified**: 2026-02-06

This document contains **verified** specifications from the official WRO documentation.

---

## ✅ VERIFIED: Track Specifications

### Overall Dimensions (Spec 13.1)
- **Game mat size**: 3200mm × 3200mm (±5mm)
  - 3.2m × 3.2m in simulation
- **Internal racetrack**: 3000mm × 3000mm (±5mm)
  - 3.0m × 3.0m in simulation

### Track Color (Spec 13.2)
- **Main colour**: WHITE
  - RGB: `(255, 255, 255)`
  - Normalized: `(1.0, 1.0, 1.0)`
  - Hex: `#FFFFFF`

**Implementation**:
```xml
<ambient>1.0 1.0 1.0 1</ambient>
<diffuse>1.0 1.0 1.0 1</diffuse>
```

---

## ✅ VERIFIED: Walls

### Exterior Walls (Spec 13.3-13.4)
- **Inner height**: 100mm (0.1m)
- **Inner colour**: BLACK
  - RGB: `(0, 0, 0)`
  - Hex: `#000000`
- **Outer colour**: Not defined (can be any)
- **Thickness**: Not defined (use ~100mm for simulation)

**Implementation**:
```xml
<!-- Wall positioned at track boundary -->
<pose>0 1.5 0.05 0 0 0</pose>  <!-- Z at 50mm (half of 100mm height) -->
<box><size>3.0 0.1 0.1</size></box>  <!-- 100mm height -->
<ambient>0.0 0.0 0.0 1</ambient>  <!-- BLACK -->
```

### Interior Walls (Spec 13.5-13.6)
- **Height**: 100mm (0.1m)
- **Outer colour**: BLACK
- **Inner colour**: BLACK
- **Top edge colour**: BLACK
- **Thickness**: Not defined
- **Distance from exterior walls**: Depends on round type (see Game Alternatives section)

**Note**: Interior wall configuration varies by challenge round.

---

## ✅ VERIFIED: Track Lines

### Orange Lines (Spec 13.9)
- **Color**: CMYK (0, 60, 100, 0)
  - RGB: `(255, 102, 0)`
  - Normalized: `(1.0, 0.4, 0.0)`
  - Hex: `#FF6600`
- **Thickness**: 20mm (0.02m)

**CMYK to RGB Conversion**:
```
C=0%: Red = 255 × (1 - 0) = 255
M=60%: Green = 255 × (1 - 0.60) = 102
Y=100%: Blue = 255 × (1 - 1.0) = 0
K=0%: No black adjustment
Result: RGB(255, 102, 0)
```

**Implementation**:
```xml
<box><size>2.0 0.02 0.001</size></box>  <!-- Length × 20mm × thin -->
<ambient>1.0 0.4 0.0 1</ambient>
<diffuse>1.0 0.4 0.0 1</diffuse>
```

### Blue Lines (Spec 13.9)
- **Color**: CMYK (100, 80, 0, 0)
  - RGB: `(0, 51, 255)`
  - Normalized: `(0.0, 0.2, 1.0)`
  - Hex: `#0033FF`
- **Thickness**: 20mm (0.02m)

**CMYK to RGB Conversion**:
```
C=100%: Red = 255 × (1 - 1.0) = 0
M=80%: Green = 255 × (1 - 0.80) = 51
Y=0%: Blue = 255 × (1 - 0) = 255
K=0%: No black adjustment
Result: RGB(0, 51, 255)
```

**Implementation**:
```xml
<box><size>2.0 0.02 0.001</size></box>
<ambient>0.0 0.2 1.0 1</ambient>
<diffuse>0.0 0.2 1.0 1</diffuse>
```

---

## ✅ VERIFIED: Starting Zones

### Starting Zone Specifications (Spec 13.10-13.11)
- **Size**: 200mm × 500mm (0.2m × 0.5m)
- **Dashed line color**: CMYK (0, 0, 0, 30)
  - RGB: `(179, 179, 179)` - Grey
  - Normalized: `(0.7, 0.7, 0.7)`
  - Hex: `#B3B3B3`
- **Line thickness**: 1mm (0.001m)

**CMYK to RGB Conversion** (with K component):
```
K=30%: Scale factor = 1 - 0.30 = 0.70
Red = 255 × 0.70 = 179
Green = 255 × 0.70 = 179
Blue = 255 × 0.70 = 179
Result: RGB(179, 179, 179)
```

**Implementation**:
```xml
<box><size>0.5 0.2 0.001</size></box>  <!-- 500mm × 200mm -->
<ambient>0.7 0.7 0.7 1</ambient>
```

---

## ✅ VERIFIED: Traffic Sign Seats

### Sign Seat Specifications (Spec 13.12-13.13)
- **Size**: 50mm × 50mm (0.05m × 0.05m) squares
- **Line thickness**: 1mm (0.001m)
- **Line color**: CMYK (0, 0, 0, 30)
  - RGB: `(179, 179, 179)` - Same grey as starting zone
  - Hex: `#B3B3B3`

**Implementation**:
```xml
<box><size>0.05 0.05 0.001</size></box>
<ambient>0.7 0.7 0.7 1</ambient>
```

### Circle Around Sign Seat (Spec 13.14-13.15)
- **Diameter**: 85mm (0.085m)
  - Radius: 42.5mm (0.0425m)
- **Line thickness**: 0.5mm (0.0005m)
- **Line color**: CMYK (20, 0, 100, 0)
  - RGB: `(204, 255, 0)` - Lime green
  - Normalized: `(0.8, 1.0, 0.0)`
  - Hex: `#CCFF00`

**CMYK to RGB Conversion**:
```
C=20%: Red = 255 × (1 - 0.20) = 204
M=0%: Green = 255 × (1 - 0) = 255
Y=100%: Blue = 255 × (1 - 1.0) = 0
K=0%: No black adjustment
Result: RGB(204, 255, 0)
```

**Implementation**:
```xml
<cylinder>
  <radius>0.0425</radius>  <!-- 42.5mm -->
  <length>0.001</length>
</cylinder>
<ambient>0.8 1.0 0.0 1</ambient>  <!-- Lime green -->
```

---

## ❓ UNVERIFIED: Need More Info

The following specifications are still needed from official documentation:

### Traffic Signs (Pillars)
- ❓ Exact dimensions (assumed: 6cm diameter × 30cm height)
- ❓ Official colors (assumed: pure red and pure green)
- ❓ Placement rules
- ❓ Number per round

### Obstacles
- ❓ Exact dimensions (assumed: 10cm × 10cm × 10cm cubes)
- ❓ Official colors (assumed: red and green matching pillars)
- ❓ Weight specifications
- ❓ Material properties

### Parking Zone
- ❓ Dimensions and location
- ❓ Magenta wall specifications
- ❓ Parking zone markings

### Interior Walls
- ❓ Exact placement by round type
- ❓ Opening/gap specifications
- ❓ Configuration variations

---

## Color Reference Table

| Element | CMYK | RGB | Hex | Normalized RGB |
|---------|------|-----|-----|----------------|
| Track surface | N/A | (255, 255, 255) | #FFFFFF | (1.0, 1.0, 1.0) |
| Walls | N/A | (0, 0, 0) | #000000 | (0.0, 0.0, 0.0) |
| Orange lines | (0, 60, 100, 0) | (255, 102, 0) | #FF6600 | (1.0, 0.4, 0.0) |
| Blue lines | (100, 80, 0, 0) | (0, 51, 255) | #0033FF | (0.0, 0.2, 1.0) |
| Dashed lines | (0, 0, 0, 30) | (179, 179, 179) | #B3B3B3 | (0.7, 0.7, 0.7) |
| Sign seat circles | (20, 0, 100, 0) | (204, 255, 0) | #CCFF00 | (0.8, 1.0, 0.0) |

---

## Dimension Reference Table

| Element | Metric | Simulation |
|---------|--------|------------|
| Game mat | 3200mm × 3200mm | 3.2m × 3.2m |
| Racetrack (inner) | 3000mm × 3000mm | 3.0m × 3.0m |
| Wall height | 100mm | 0.1m |
| Orange/blue line thickness | 20mm | 0.02m |
| Starting zone | 200mm × 500mm | 0.2m × 0.5m |
| Sign seat | 50mm × 50mm | 0.05m × 0.05m |
| Sign seat circle diameter | 85mm | 0.085m (radius 0.0425m) |
| Dashed line thickness | 1mm | 0.001m |
| Circle line thickness | 0.5mm | 0.0005m |

---

## Implementation Status

### ✅ Implemented in Simulation
- [x] White track surface (3200mm × 3200mm)
- [x] Black exterior walls (100mm height)
- [x] Orange lines (20mm thick, official color)
- [x] Blue lines (20mm thick, official color)
- [x] Starting zone (200mm × 500mm, grey dashed lines)
- [x] Traffic sign seats (50mm × 50mm squares)
- [x] Circles around sign seats (85mm diameter, lime green)

### ⏳ Pending Official Specs
- [ ] Traffic pillar exact specifications
- [ ] Obstacle exact specifications
- [ ] Interior wall placement
- [ ] Parking zone specifications
- [ ] Specific line patterns/layouts

---

## Notes for Simulation

### Tolerance
- Official specs include ±5mm tolerance on track dimensions
- Simulation uses exact values (no tolerance modeling)

### Line Placement
- Orange/blue line positions not specified in provided rules
- Current implementation uses example positions
- **TODO**: Get official line pattern diagram

### Visual Rendering
- Gazebo may render colors slightly differently than physical materials
- Domain randomization helps account for visual variations
- Test with real camera for color calibration

---

## Questions to Answer

Please provide additional WRO documentation for:

1. **Traffic Signs (Pillars)**:
   - What are the exact dimensions?
   - What are the official RGB/CMYK colors?
   - How many per round?
   - Placement rules?

2. **Obstacles**:
   - Exact dimensions?
   - Official colors?
   - Weight and material?

3. **Track Layout**:
   - Where exactly are orange/blue lines placed?
   - What patterns do they form?
   - Diagram or map available?

4. **Interior Walls** (Spec 13.5-13.8):
   - What is the "Game Alternatives" section?
   - Interior wall configurations for each round type?

5. **Parking Zone**:
   - Dimensions and location?
   - Marking specifications?

---

**Document Status**: Partially Complete
**Needs**: Additional specifications from official WRO rules (sections on traffic signs, obstacles, parking zones)

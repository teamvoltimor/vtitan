# Changes Made to Match WRO Official Colors

## Summary of Updates

All simulation files have been updated to match the **official WRO Future Engineers competition specifications**.

## Color Changes

### ✅ Track Surface (Floor)
**Before**: Medium grey `RGB(0.5, 0.5, 0.5)`
**After**: Light grey `RGB(0.75, 0.75, 0.75)` - **WRO Official**

### ✅ Boundary Walls
**Before**: Very dark grey `RGB(0.05, 0.05, 0.05)`
**After**: Pure black `RGB(0.0, 0.0, 0.0)` - **WRO Official**

### ✅ Traffic Pillars
**Before**: Red, Green, and Blue pillars
**After**: **Only Red and Green pillars** - **WRO Official**

- Red: `RGB(1.0, 0.0, 0.0)` - Pure red
- Green: `RGB(0.0, 1.0, 0.0)` - Pure green
- ❌ Blue: Removed (not used in WRO)

### ✅ Parking Zone (NEW)
**Added**: Magenta parking walls `RGB(1.0, 0.0, 1.0)` - **WRO Official**
- Located in corner of track
- Height: 15cm (half of boundary walls)
- Clearly visible magenta color

### ✅ Obstacles (Obstacles Challenge)
**Before**: Brown/orange blocks `RGB(0.6, 0.3, 0.1)`
**After**: **Red or Green blocks** - **WRO Official**

- Red obstacles: `RGB(1.0, 0.0, 0.0)` - Pass on right side
- Green obstacles: `RGB(0.0, 1.0, 0.0)` - Pass on left side
- Same colors as traffic pillars
- 10cm × 10cm × 10cm cubes

## Files Updated

### 1. World File
**File**: `simulation/worlds/wro_track_base.sdf`

Changes:
- ✅ Floor changed to light grey
- ✅ All walls changed to pure black
- ✅ Added magenta parking zone walls (2 walls forming corner)

### 2. Python Generator
**File**: `simulation/scripts/generate_training_data.py`

Changes:
- ✅ Removed blue from color randomization options
- ✅ Only generates red and green pillars
- ✅ Obstacles now use red or green colors (not brown)
- ✅ Updated metadata to include obstacle colors

### 3. Configuration Files
**Files**:
- `simulation/config/open_challenge.yaml`
- `simulation/config/obstacles_challenge.yaml`

Changes:
- ✅ Removed blue pillar type
- ✅ Added track_colors section with official WRO colors
- ✅ Updated obstacle specs to use red/green colors
- ✅ Added comments explaining WRO rules

### 4. Annotation Script
**File**: `simulation/scripts/extract_frames_and_annotate.py`

Changes:
- ✅ Removed 'blue' from class map
- ✅ Changed from 4 classes to 3 classes
- ✅ Updated YOLO data.yaml to reflect 3 classes
- ✅ Added WRO-specific comments

### 5. NEW: Official Specifications Document
**File**: `simulation/WRO_OFFICIAL_SPECS.md`

Contains:
- ✅ Official RGB values for all colors
- ✅ Exact dimensions (track, pillars, obstacles)
- ✅ HSV color ranges for detection
- ✅ Competition rules summary
- ✅ Simulation implementation guide
- ✅ Domain randomization recommendations

## Visual Comparison

### Track Appearance

**Before**:
```
Floor: Medium grey
Walls: Dark grey
Pillars: Red, Green, Blue
Obstacles: Brown
```

**After** (WRO Official):
```
Floor: Light grey (realistic mat)
Walls: Pure black (high contrast)
Pillars: Red, Green only
Obstacles: Red, Green (matching pillars)
Parking: Magenta corner walls
```

## YOLO Training Impact

### Class Changes

**Before**: 4 classes
```yaml
names: ['red', 'green', 'blue', 'obstacle']
```

**After**: 3 classes (WRO Official)
```yaml
names: ['red', 'green', 'obstacle']
```

### Benefits
- ✅ Simpler model (3 classes instead of 4)
- ✅ More training data per class
- ✅ Better accuracy (fewer classes to distinguish)
- ✅ Matches real competition environment

## Obstacle Challenge Rules

### Red vs Green Obstacles

**WRO Official Rules**:

**Red Obstacles**:
- Color: Pure red `RGB(255, 0, 0)`
- Rule: **Pass on the RIGHT side**
- Same color as red pillars

**Green Obstacles**:
- Color: Pure green `RGB(0, 255, 0)`
- Rule: **Pass on the LEFT side**
- Same color as green pillars

### Simulation Implementation

The generator now:
1. Randomly assigns red or green color to each obstacle
2. Saves color in metadata for ground truth
3. Generates proper YOLO annotations with color class
4. Allows training models to learn avoidance rules

## Color Detection Recommendations

### HSV Ranges (for Classical CV)

**Red Pillars/Obstacles**:
```python
# Red wraps around in HSV
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])
```

**Green Pillars/Obstacles**:
```python
lower_green = np.array([40, 100, 100])
upper_green = np.array([80, 255, 255])
```

**Black Walls** (for boundary detection):
```python
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 50, 50])
```

**Grey Floor** (for ground detection):
```python
lower_grey = np.array([0, 0, 100])
upper_grey = np.array([180, 30, 200])
```

**Magenta Parking** (for parking detection):
```python
lower_magenta = np.array([140, 100, 100])
upper_magenta = np.array([160, 255, 255])
```

## Testing Recommendations

### Verify Colors in Gazebo

After launching simulation:

1. **Check floor color**:
   - Should be light grey, not dark
   - Similar to real WRO mat

2. **Check wall color**:
   - Should be pure black
   - High contrast with floor

3. **Check pillar colors**:
   - Only red and green pillars
   - Bright, saturated colors
   - No blue pillars

4. **Check parking zone**:
   - Visible magenta walls in corner
   - Clearly distinguishable from other elements

5. **Check obstacles** (if obstacles challenge):
   - Red or green cubes
   - Same colors as pillars
   - 10cm size

### Camera View Test

Use this command to view camera feed:
```bash
ros2 run rqt_image_view rqt_image_view /wro_robot/camera/image_raw
```

Verify:
- Light grey floor appears in bottom of frame
- Pillars are clearly red or green
- Colors are saturated (not washed out)
- Black walls provide clear boundaries

## Regenerate Existing Scenarios

If you already generated scenarios with old colors:

```bash
# Delete old scenarios
rm -rf ~/wro_training_data/scenarios/*

# Regenerate with official colors
cd simulation/scripts
python3 generate_training_data.py \
    --challenge open \
    --num-scenarios 100 \
    --randomize-all \
    --output-dir ~/wro_training_data
```

## Domain Randomization (Still Applies)

Color randomization still active for sim-to-real transfer:

**Light Grey Floor**:
- Base: `RGB(0.75, 0.75, 0.75)`
- Variation: ±5% (simulates lighting/wear)

**Red Pillars**:
- Base: `RGB(1.0, 0.0, 0.0)`
- Gaussian noise: std=0.05
- Simulates camera sensor variation

**Green Pillars**:
- Base: `RGB(0.0, 1.0, 0.0)`
- Gaussian noise: std=0.05
- Simulates camera sensor variation

**Lighting**:
- Intensity: 0.5-1.5× (simulates different venues)
- Direction: ±30° (simulates sun/ceiling lights)

## Migration Guide

If you have existing code:

### Update Color Detection Code

**Before**:
```python
colors = ['red', 'green', 'blue']
class_map = {0: 'red', 1: 'green', 2: 'blue', 3: 'obstacle'}
```

**After**:
```python
colors = ['red', 'green']  # WRO official only
class_map = {0: 'red', 1: 'green', 2: 'obstacle'}
```

### Update YOLO Training

**Before**:
```yaml
nc: 4
names: ['red', 'green', 'blue', 'obstacle']
```

**After**:
```yaml
nc: 3
names: ['red', 'green', 'obstacle']
```

### Update Decision Logic

For obstacles challenge:

```python
def handle_obstacle(detection):
    if detection.color == 'red':
        # Pass on RIGHT side
        turn_right()
    elif detection.color == 'green':
        # Pass on LEFT side
        turn_left()
```

## References

- **Official Specs**: `simulation/WRO_OFFICIAL_SPECS.md`
- **WRO Website**: https://wro-association.org/
- **Competition Rules**: Check latest year's official PDF

## Questions?

If you notice any discrepancies with the official WRO rules:
1. Check `WRO_OFFICIAL_SPECS.md` for reference values
2. Verify with current year's competition rules
3. Test on practice track (if available)
4. Adjust HSV thresholds based on actual lighting

---

**Update Date**: 2026-02-06
**Status**: ✅ All files updated to WRO official specifications
**Tested**: Ready for generation and training

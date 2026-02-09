# 🎬 New Features: Dynamic Lighting + Robot Movement

## ✨ What's New

### 1. **Dynamic Lighting Scenarios** 💡

Six realistic lighting conditions for diverse training data:

| Scenario | Description | Use Case |
|----------|-------------|----------|
| **Direct Sunlight** | Bright, harsh shadows, high contrast | Outdoor competitions |
| **Cloudy** | Diffuse, soft shadows | Overcast conditions |
| **Indoor Bright** | Bright artificial, minimal shadows | Well-lit venues |
| **Indoor Dim** | Dimmer lighting | Less optimal venues |
| **Evening** | Warm tones, low-angle light | Dawn/dusk conditions |
| **Mixed** | Sun + indoor lights | Semi-outdoor venues |

### 2. **Autonomous Robot Movement** 🚗

- Robot now **drives around the track** autonomously
- Simple forward + turn behavior
- Respects **clockwise/counterclockwise** direction
- Camera captures **realistic motion** from robot POV

---

## 🎥 What You'll See in Videos

### Before
- ❌ Static camera view
- ❌ Single lighting condition
- ❌ No motion dynamics

### Now ✅
- ✅ **Moving robot POV**
- ✅ **6 different lighting scenarios** (random per video)
- ✅ **Realistic driving experience**
- ✅ **Motion blur and dynamics**
- ✅ **Varied shadows and contrast**

---

## 🚀 Usage

### Generate and Record with New Features

```bash
cd simulation/scripts

# Source ROS2
source /opt/ros/humble/setup.bash

# Generate scenarios with FULL randomization (includes lighting)
python3 record_scenario_videos.py \
    --challenge open \
    --num-scenarios 10 \
    --duration 30 \
    --randomize-all
```

**Important**: Use `--randomize-all` flag to enable dynamic lighting!

### What Happens

1. **Generates scenario** with random lighting scenario
2. **Launches Gazebo** with the track
3. **Launches robot driver** - robot starts moving
4. **Records 30 seconds** of robot driving around track
5. **Saves 720p HD video** with motion and varied lighting

---

## 📊 Lighting Scenarios Details

### Direct Sunlight
```python
Sun Intensity: 90-100%
Ambient: 30-40%
Shadows: Yes (harsh)
Angle: Varied
```
**Best for**: Training in bright outdoor conditions

### Cloudy
```python
Sun Intensity: 60-75%
Ambient: 50-60%
Shadows: Yes (soft)
Angle: Overhead
```
**Best for**: Overcast day simulation

### Indoor Bright
```python
Sun Intensity: 70-85%
Ambient: 60-70%
Shadows: No
Angle: Overhead (artificial lights)
```
**Best for**: Well-lit competition venues

### Indoor Dim
```python
Sun Intensity: 50-65%
Ambient: 40-50%
Shadows: No
Angle: Overhead
```
**Best for**: Challenging lighting conditions

### Evening
```python
Sun Intensity: 60-80%
Ambient: 30-40%
Shadows: Yes
Angle: Low (warm tones)
```
**Best for**: Dawn/dusk conditions

### Mixed
```python
Sun Intensity: 70-90%
Ambient: 50-65%
Shadows: Yes
Angle: Varied (sun + artificial)
```
**Best for**: Semi-outdoor or windowed venues

---

## 🤖 Robot Movement Details

### Simple Autonomous Driver

The robot uses a simple state machine:

```
State: FORWARD (3 seconds)
  → Drive straight at 0.3 m/s

State: TURNING (1.5 seconds)
  → Turn (right for clockwise, left for counterclockwise)
  → Slower speed (0.15 m/s)

Repeat...
```

### Robot Specifications
- **Size**: 20cm × 15cm × 8cm (WRO compliant)
- **Color**: Blue (visible in videos)
- **Speed**: 0.3 m/s forward, 0.5 rad/s turning
- **Control**: Simple differential drive

### Camera View
- **Positioned**: At robot center, 12cm height
- **Orientation**: Faces direction of travel
- **FOV**: 110 degrees (realistic)
- **Captures**: Track ahead, walls, signs as robot approaches

---

## 📈 Training Data Quality

### Diversity Improvements

| Feature | Before | Now | Improvement |
|---------|--------|-----|-------------|
| **Lighting** | Single | 6 scenarios | **6x variety** |
| **Motion** | Static | Moving | **Realistic dynamics** |
| **Shadows** | Fixed | Varied | **Better generalization** |
| **Contrast** | Same | Varied | **Robust to conditions** |
| **Total variety** | 1 | 36 | **36x combinations** |

*36 = 6 lighting scenarios × 6 track layouts (corridor widths, positions, etc.)*

---

## 🎯 Best Practices

### For Maximum Variety

```bash
# Generate large dataset with all randomization
python3 record_scenario_videos.py \
    --challenge obstacles \
    --num-scenarios 100 \
    --duration 45 \
    --randomize-all \
    --extract-frames
```

This creates:
- ✅ **100 videos** with varied lighting
- ✅ **Robot movement** in all videos
- ✅ **Different track layouts** (corridor widths)
- ✅ **Different starting positions** (6 possible per corridor)
- ✅ **Traffic signs** (obstacles only, 36 scenarios)
- ✅ **Sample frames** for YOLO training

### Output Size Estimate

- **Per video** (30s, 720p, moving robot): ~8-12 MB
- **100 videos**: ~1 GB
- **With frames** (20 frames/video): ~2 GB total

---

## 🔧 Customization

### Adjust Robot Speed

Edit `simple_robot_driver.py`:

```python
self.forward_speed = 0.5  # Faster (50 cm/s)
self.angular_speed = 0.8  # Faster turning
```

### Change Driving Pattern

Edit state machine in `control_loop()`:

```python
if self.state == 'forward':
    # Change duration
    if self.state_timer >= 5.0:  # Drive longer
        ...
```

### Add More Lighting Scenarios

Edit `randomize_lighting()` in `generate_training_data.py`:

```python
scenarios = ['direct_sunlight', 'cloudy', ..., 'your_new_scenario']
```

---

## 🐛 Troubleshooting

### Robot Not Moving

**Check 1**: Is robot in the world?
```bash
gz topic -l | grep cmd_vel
# Should see: /wro_robot/cmd_vel
```

**Check 2**: Is driver running?
```bash
ps aux | grep simple_robot_driver
```

### Lighting Looks Same

**Check**: Did you use `--randomize-all`?
```bash
# Wrong (no lighting randomization)
python3 record_scenario_videos.py --challenge open --num-scenarios 10

# Correct (with lighting)
python3 record_scenario_videos.py --challenge open --num-scenarios 10 --randomize-all
```

### Video Quality Low

**Solution**: Already set to 720p HD (1280×720)

To increase further, edit `generate_training_data.py`:
```python
ET.SubElement(image_elem, 'width').text = '1920'  # Full HD
ET.SubElement(image_elem, 'height').text = '1080'
```

---

## 📝 Example Session

```bash
cd /mnt/c/Users/ralva/.../simulation/scripts

# Source ROS2
source /opt/ros/humble/setup.bash

# Generate 20 varied scenarios
python3 record_scenario_videos.py \
    --challenge open \
    --num-scenarios 20 \
    --duration 30 \
    --randomize-all

# Expected output:
# 20 videos with:
#   - Different lighting (randomly selected from 6 scenarios)
#   - Robot driving around track
#   - Different starting positions/directions
#   - Different corridor layouts
```

---

## 🎓 Next Steps

1. **Test single video**: Generate 1 scenario, check video quality
2. **Generate small batch**: 10-20 scenarios to verify variety
3. **Generate full dataset**: 100-500 scenarios for training
4. **Train your model**: Use videos or extracted frames

---

## 📊 Feature Matrix

| Feature | Basic Mode | Randomize-All Mode |
|---------|------------|-------------------|
| Lighting | Fixed | ✅ 6 scenarios |
| Robot Motion | ✅ Yes | ✅ Yes |
| Corridor Width | Fixed | ✅ Random |
| Starting Position | Center | ✅ 6 positions |
| Direction | Fixed | ✅ Random |
| Shadows | Fixed | ✅ Varied |
| **Training Value** | Good | **Excellent** |

---

**Created**: 2026-02-09
**Version**: 2.0.0 (Dynamic Lighting + Robot Movement)
**Status**: ✅ Ready for production use

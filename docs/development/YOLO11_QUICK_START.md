# YOLO11 Quick Start Guide

## TL;DR

Train YOLO11 on your laptop (RTX 4050) → Export to ONNX → Test in ROS2 → Deploy to RPi5 with Hailo 8L

**Performance:**
- Laptop (ONNX + GPU): 200+ FPS ⚡
- RPi5 (ONNX + CPU): 10-15 FPS 🐌
- **RPi5 (Hailo 8L): 30-60 FPS** 🚀

---

## Why YOLO11?

**YOLO11** (released late 2024 by Ultralytics) is the **latest and fastest** YOLO model:

- ✅ **20-30% faster** than YOLOv8
- ✅ **Better accuracy** with improved architecture
- ✅ **Smaller models** - YOLO11n (nano) perfect for edge
- ✅ **Hailo 8L support** - Via ONNX → HEF conversion
- ✅ **Same API** as YOLOv8 (drop-in replacement)

**Note:** There is no "YOLOv26" as of early 2025. The progression is:
- YOLOv5 → YOLOv6 → YOLOv7 → YOLOv8 → YOLOv9 → YOLOv10 → **YOLO11** (latest)

---

## Installation (5 minutes)

```bash
# Install Ultralytics (includes YOLO11)
pip3 install ultralytics

# Install ONNX Runtime for testing
pip3 install onnxruntime-gpu  # GPU version
# OR: pip3 install onnxruntime  # CPU only

# Verify
yolo version
# Should show: Ultralytics YOLO11...
```

---

## Quick Training (30 minutes on RTX 4050)

### Step 1: Prepare Dataset (10 min)

```bash
# Create directory structure
mkdir -p ~/teamsteelbot_ws/datasets/traffic_signs/{images,labels}/{train,val}

# Generate synthetic data (fastest way to start)
python3 docs/development/generate_synthetic_dataset.py
# Creates 1000 training images with labels

# OR use Roboflow to label real images
# https://roboflow.com (free tier)
```

### Step 2: Create data.yaml (1 min)

```yaml
# ~/teamsteelbot_ws/datasets/traffic_signs/data.yaml
path: /home/your_user/teamsteelbot_ws/datasets/traffic_signs
train: images/train
val: images/val

nc: 3
names: ['red_sign', 'green_sign', 'blue_sign']
```

### Step 3: Train YOLO11-Nano (20 min)

```bash
# One command to train!
yolo detect train \
  data=~/teamsteelbot_ws/datasets/traffic_signs/data.yaml \
  model=yolo11n.pt \
  epochs=50 \
  imgsz=640 \
  batch=16 \
  device=0 \
  project=~/teamsteelbot_ws/models \
  name=signs_yolo11n

# Model saved to:
# ~/teamsteelbot_ws/models/signs_yolo11n/weights/best.pt
```

**Training parameters:**
- `yolo11n.pt` = Nano model (fastest, ~2.6M parameters)
- `epochs=50` = Good for small datasets
- `imgsz=640` = Hailo-optimized size
- `device=0` = Use GPU 0 (RTX 4050)

### Step 4: Export to ONNX (1 min)

```bash
yolo export \
  model=~/teamsteelbot_ws/models/signs_yolo11n/weights/best.pt \
  format=onnx \
  imgsz=640 \
  simplify=True

# Creates: best.onnx
```

---

## Testing on Laptop (ROS2 + ONNX)

### Step 1: Copy ROS2 Node

Copy the YOLO11 detector code from `docs/development/yolo11-hailo-guide.md` section 3.1:

```bash
# File location:
# ~/teamsteelbot_ws/src/teamsteelbot_vision/teamsteelbot_vision/sign_detector_yolo11.py
```

### Step 2: Update setup.py

```python
# ~/teamsteelbot_ws/src/teamsteelbot_vision/setup.py

entry_points={
    'console_scripts': [
        'sign_detector_classic = teamsteelbot_vision.sign_detector_classic:main',
        'sign_detector_yolo11 = teamsteelbot_vision.sign_detector_yolo11:main',  # Add this
    ],
},
```

### Step 3: Build and Run

```bash
# Build
cd ~/teamsteelbot_ws
colcon build --packages-select teamsteelbot_vision --symlink-install
source install/setup.bash

# Terminal 1: Mock camera
ros2 run teamsteelbot_simulation mock_camera_node

# Terminal 2: YOLO11 detector
ros2 run teamsteelbot_vision sign_detector_yolo11 --ros-args \
  -p model_path:=~/teamsteelbot_ws/models/signs_yolo11n/weights/best.onnx \
  -p use_hailo:=false

# Terminal 3: Monitor
ros2 topic hz /detections  # Check FPS (should be 30+ Hz with ONNX)
ros2 topic echo /detections  # See detections

# Terminal 4: Visualize
rviz2
# Add Image display → /detections/debug_image
```

**Expected performance on laptop:** 200+ FPS with GPU, 30-50 FPS with CPU

---

## Deployment to RPi5 + Hailo 8L

### Step 1: Convert ONNX to Hailo HEF

**Option A: Use Hailo Dataflow Compiler (on laptop)**

```bash
# Install from: https://hailo.ai/developer-zone/
# Then:
hailo parser onnx best.onnx
hailo compile --hw-arch hailo8l best.har -o best.hef
```

**Option B: Use Hailo Model Zoo (easier)**

```bash
git clone https://github.com/hailo-ai/hailo_model_zoo.git
cd hailo_model_zoo

# Follow their YOLO conversion guide
python hailo_model_zoo/main.py compile \
  --hw-arch hailo8l \
  --ckpt best.onnx \
  --output-model best.hef
```

### Step 2: Transfer to RPi5

```bash
# From laptop
scp best.hef pi@raspberrypi.local:~/teamsteelbot_ws/models/

# OR use git to sync entire workspace
```

### Step 3: Run on RPi5 with Hailo

```bash
# On RPi5
ros2 run teamsteelbot_vision sign_detector_yolo11 --ros-args \
  -p model_path:=~/teamsteelbot_ws/models/best.hef \
  -p use_hailo:=true

# Should achieve 30-60 FPS! 🚀
```

---

## Performance Comparison

| Environment | Backend | FPS | Latency | Power |
|-------------|---------|-----|---------|-------|
| Laptop (RTX 4050) | ONNX + GPU | 200+ | ~5ms | ~50W |
| RPi5 | ONNX + CPU | 10-15 | ~70ms | ~5W |
| **RPi5 + Hailo 8L** | **Hailo** | **30-60** | **~20ms** | **~9W** |

**Hailo 8L = 3-4x speedup vs CPU at similar power consumption!**

---

## Development Workflow

```
┌──────────────────────────────────┐
│  Laptop: Train & Test            │
│  • Train YOLO11n (30 min)        │
│  • Export to ONNX                │
│  • Test at 200+ FPS              │
│  • Develop ROS2 logic            │
└──────────────────────────────────┘
            ↓ (Git sync)
┌──────────────────────────────────┐
│  RPi5: Deploy & Run              │
│  • Convert ONNX to HEF           │
│  • Run at 30-60 FPS on Hailo     │
│  • Integrate with real sensors   │
└──────────────────────────────────┘
```

**Key insight:** Same code runs on both laptop and RPi5! Just change `use_hailo` parameter.

---

## Comparison: YOLO11 vs Classical CV

| Feature | YOLO11 | Classical CV |
|---------|--------|--------------|
| **Accuracy** | 92-95% | 85-90% |
| **Robustness** | ⭐⭐⭐⭐⭐ (lighting, occlusion) | ⭐⭐⭐ (sensitive to lighting) |
| **Speed (Hailo)** | 30-60 FPS | 60-120 FPS |
| **Development time** | 2-3 weeks | 1 week |
| **Training needed** | Yes (30 min) | No |
| **Complexity** | Medium | Low |
| **Professional appeal** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |

**Recommendation:** Start with Classical CV (Week 2), add YOLO11 (Week 3-4), use hybrid approach for competition!

---

## Hybrid Approach (Best Strategy) 🏆

```python
# In decision_node.py

if yolo11_confidence > 0.7:
    # High confidence - trust YOLO11
    sign = yolo11_detection

elif classical_cv_confidence > 0.8:
    # YOLO uncertain, but classical CV is confident
    sign = classical_detection

elif yolo11_detection == classical_detection:
    # Both agree, even if confidence is medium
    sign = yolo11_detection

else:
    # Uncertainty - slow down and retry
    reduce_speed()
    sign = None

# Optional: Physical validation with TCS34725
if sign and tcs34725_enabled:
    if not validate_with_color_sensor(sign):
        log_warning("Physical validation failed!")
        sign = None
```

**This gives you the best of both worlds:**
- ✅ YOLO11 for robust detection
- ✅ Classical CV as fast fallback
- ✅ TCS34725 for physical validation (unique!)
- ✅ Multi-sensor voting for reliability

---

## Troubleshooting

### Issue: "yolo: command not found"
```bash
pip3 install --upgrade ultralytics
```

### Issue: Training is slow
```bash
# Check if GPU is being used
yolo detect train data=data.yaml model=yolo11n.pt device=0
# device=0 = GPU 0
# device=cpu = force CPU (for testing)
```

### Issue: ONNX export fails
```bash
# Use older opset
yolo export model=best.pt format=onnx opset=11
```

### Issue: Hailo conversion fails
```bash
# Simplify ONNX first
yolo export model=best.pt format=onnx simplify=True

# Check Hailo Model Zoo docs for supported ops
# Some custom layers may need conversion
```

### Issue: Different results on Hailo vs ONNX
- **Normal!** Hailo uses INT8 quantization
- Solution: Quantization-aware training (advanced)
- OR: Accept small accuracy difference (usually <2%)

---

## Resources

- **YOLO11 Docs:** https://docs.ultralytics.com/
- **Hailo Model Zoo:** https://github.com/hailo-ai/hailo_model_zoo
- **Hailo Developer Zone:** https://hailo.ai/developer-zone/
- **Full Integration Guide:** `docs/development/yolo11-hailo-guide.md`
- **ROS2 Integration:** `docs/development/starter-code-examples.md`

---

## Quick Commands Cheat Sheet

```bash
# Train
yolo detect train data=data.yaml model=yolo11n.pt epochs=50

# Validate
yolo detect val model=best.pt data=data.yaml

# Predict
yolo detect predict model=best.pt source=image.jpg

# Export
yolo export model=best.pt format=onnx

# ROS2 run (laptop)
ros2 run teamsteelbot_vision sign_detector_yolo11 --ros-args -p use_hailo:=false

# ROS2 run (RPi5)
ros2 run teamsteelbot_vision sign_detector_yolo11 --ros-args -p use_hailo:=true
```

---

## Next Steps

1. ✅ Install Ultralytics: `pip3 install ultralytics`
2. ✅ Generate synthetic dataset (or collect real images)
3. ✅ Train YOLO11n (30 minutes)
4. ✅ Export to ONNX
5. ✅ Test in ROS2 with mock camera
6. ✅ Develop full pipeline on laptop
7. 🎯 Port to RPi5 and convert to Hailo HEF
8. 🏁 Competition!

---

**YOLO11 + Hailo 8L = Professional-grade edge AI at 30-60 FPS! 🚀**

Start training now - your RTX 4050 will have a model ready in 30 minutes!

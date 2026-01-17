# Hailo Model Calibration & GPU Optimization Guide

**Last Updated:** 2026-01-17

This document explains how to calibrate and optimize models for your **Hailo AI HAT+ 26 TOPS** accelerator using GPU acceleration, based on the latest Hailo documentation and community resources.

---

## 🎯 What is Model Calibration?

**Calibration** is the process of converting a trained floating-point model (FP32) to an optimized integer model (INT8) that runs efficiently on the Hailo accelerator.

### Why Calibration is Needed

- **Speed:** INT8 models run 4× faster than FP32
- **Memory:** INT8 models use 4× less memory
- **Power:** Lower precision = lower power consumption
- **Hailo Requirement:** Hailo accelerators run INT8 models exclusively

### The Process

```
Trained Model (FP32, PyTorch/TensorFlow)
    ↓
Export to ONNX (FP32)
    ↓
Hailo Dataflow Compiler
    ├── Parse (understand model structure)
    ├── Optimize (graph optimizations)
    ├── **Quantize (FP32 → INT8)** ← CALIBRATION HAPPENS HERE
    └── Compile (generate HEF file)
    ↓
Hailo HEF (INT8, runs on AI HAT+)
```

---

## 🖥️ GPU Requirements for Calibration

### Recommended Hardware

Based on Hailo documentation and community feedback:

| Component | Minimum | Recommended | Optimal |
|-----------|---------|-------------|---------|
| **GPU** | NVIDIA CUDA-capable | NVIDIA RTX 3060+ | NVIDIA RTX 4090 |
| **RAM** | 16 GB | **32 GB** ⭐ | 64 GB |
| **Storage** | 50 GB free | 100 GB SSD | 500 GB NVMe |
| **OS** | Ubuntu 20.04+ | Ubuntu 22.04 | Ubuntu 24.04 |

**⚠️ IMPORTANT:** It is **strongly recommended** to run calibration on a **GPU machine** with at least **32 GB RAM**.

**Why GPU matters:**
- Calibration processes 1000+ images
- Each image passes through the model
- GPU accelerates this 10-50× vs CPU
- Larger batch sizes = better quantization accuracy

**Native vs Docker:**
- **Native installation** (recommended) - Better GPU access, more reliable
- **Docker** - Works but may have GPU passthrough issues

---

## 📊 Calibration Dataset Requirements

### Dataset Size

**Minimum:** 100 images (for quick testing)
**Recommended:** **1,000+ images** ⭐
**Optimal:** 5,000-10,000 images

From Hailo documentation:
> "The quantization phase requires several hundreds to thousands of data samples, ideally a subset from the training data."

### Dataset Quality

**What you need:**
- ✅ Representative samples from your use case (WRO track images)
- ✅ Diverse conditions (lighting, angles, distances)
- ✅ **Images ONLY** (no annotations/labels required!)
- ✅ Same resolution as training (e.g., 640×640 for YOLO)

**What to include for WRO:**
- Different track configurations
- Various sign positions (left, right, center)
- Lighting variations (bright, dim, shadows)
- Camera angles (straight, tilted)
- Motion blur (simulated or real)

**Example dataset structure:**
```
calibration_data/
├── track_run_001/
│   ├── frame_0001.jpg
│   ├── frame_0002.jpg
│   └── ...
├── track_run_002/
│   └── ...
└── track_run_010/
    └── ...

Total: 1,000-5,000 images
```

### How to Collect Calibration Data

**Option 1: Manual Driving**
```bash
# Record images while driving robot manually
ros2 bag record /camera/image_raw

# Extract images from bag
ros2 bag play recorded_bag.db3 &
python extract_images.py --output calibration_data/
```

**Option 2: Simulation (Gazebo)**
```python
# Generate diverse images in Gazebo
for i in range(1000):
    # Randomize robot position, lighting
    randomize_world()

    # Capture image
    img = capture_camera()
    cv2.imwrite(f'calibration_data/sim_{i:04d}.jpg', img)
```

**Option 3: Combine Real + Synthetic**
- 70% real images (manual driving)
- 30% synthetic (Gazebo with domain randomization)
- **Total: 1,000-2,000 images**

---

## 🔧 Hailo Dataflow Compiler Setup

### Installation (Ubuntu 22.04)

```bash
# Install Hailo Dataflow Compiler
# Download from: https://hailo.ai/developer-zone/

# Example (check latest version):
wget https://hailo.ai/downloads/hailo_dataflow_compiler_3.27.0.tar.gz
tar -xzf hailo_dataflow_compiler_3.27.0.tar.gz
cd hailo_dataflow_compiler_3.27.0

# Install (native - recommended)
sudo ./install.sh

# Verify installation
hailo -v
```

### GPU Setup (CUDA)

```bash
# Install NVIDIA drivers
sudo ubuntu-drivers autoinstall

# Install CUDA Toolkit (version compatible with your GPU)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get install cuda

# Verify GPU
nvidia-smi  # Should show your GPU

# Verify CUDA
nvcc --version
```

### Python Environment

```bash
# Create virtual environment
python3 -m venv hailo_env
source hailo_env/bin/activate

# Install dependencies
pip install --upgrade pip
pip install hailo_sdk_client
pip install onnx
pip install opencv-python
pip install numpy
```

---

## ⚙️ Calibration Process

### Step 1: Prepare Your Model

Export your trained model to ONNX:

```python
# Example: Export YOLOv8 to ONNX
from ultralytics import YOLO

model = YOLO('traffic_signs.pt')  # Your trained model
model.export(format='onnx', imgsz=640, simplify=True)
# Outputs: traffic_signs.onnx
```

### Step 2: Create Calibration Configuration

Create `calibration_config.yaml`:

```yaml
# Hailo Model Zoo configuration
# Based on: https://github.com/hailo-ai/hailo_model_zoo

model_name: yolov8n_traffic_signs
input_shape: [1, 3, 640, 640]

# Calibration dataset
calib_set:
  path: /path/to/calibration_data/
  num_calib_batches: 128  # 128 batches × 8 images = 1024 images
  batch_size: 8  # Adjust based on GPU memory

# Quantization settings
quantization:
  precision: int8
  calib_method: mse  # Options: mse, percentile, max
  per_channel: true

# Optimization (optional)
optimization:
  adaround: true  # Better quantization (slower, needs GPU)
  bias_correction: true
```

**Batch Size Guidelines:**
- **8** (default) - For GPUs with 8+ GB VRAM (RTX 3060+)
- **4** - For GPUs with 4-6 GB VRAM (GTX 1660)
- **2** - Minimum (slower but works on smaller GPUs)

From Hailo docs:
> "It is best to use the maximum batch size supported by your GPU for each model to maintain quantization accuracy."

### Step 3: Run Calibration

```bash
# Activate environment
source hailo_env/bin/activate

# Run Hailo Dataflow Compiler
hailo parser onnx \
    --onnx traffic_signs.onnx \
    --hw-arch hailo8l  # Or hailo8 for AI HAT+
    # Output: traffic_signs.har

hailo optimize \
    --har traffic_signs.har \
    --calib-data-path /path/to/calibration_data/ \
    --calib-set-size 1024 \
    --batch-size 8 \
    # Output: traffic_signs_optimized.har

hailo compiler \
    --har traffic_signs_optimized.har \
    --hw-arch hailo8l \
    # Output: traffic_signs.hef (READY FOR DEPLOYMENT!)
```

**Expected Time (GPU):**
- Parsing: ~1 minute
- Optimization/Quantization: **5-30 minutes** (depends on model size, dataset size)
- Compilation: ~5 minutes
- **Total: 10-40 minutes** with GPU

**Expected Time (CPU only):**
- Optimization: **2-6 hours** (10-50× slower!)

### Step 4: Verify Quantized Model

```python
# Test HEF file accuracy
from hailo_platform import HailoScheduler, InferModel

# Load model
model = InferModel('/path/to/traffic_signs.hef')

# Test on validation set
for img_path in validation_images:
    img = preprocess(cv2.imread(img_path))
    output = model.run(img)
    # Compare with original ONNX model output

# Measure accuracy drop (should be <2%)
print(f"Original accuracy: {original_acc}%")
print(f"Quantized accuracy: {quantized_acc}%")
print(f"Accuracy drop: {original_acc - quantized_acc}%")
```

**Acceptable accuracy drop:** <2%
**Good quantization:** <1% accuracy drop

If accuracy drop is >2%, you may need:
- More calibration images (increase from 1k → 5k)
- Better calibration method (try `percentile` instead of `mse`)
- Enable advanced optimizations (`adaround`, `bias_correction`)

---

## 🚀 GPU Acceleration Tips

### 1. Use Maximum Batch Size

```yaml
# Start high, reduce if OOM (out of memory)
batch_size: 16  # Try first
batch_size: 8   # If OOM, reduce to 8
batch_size: 4   # If still OOM, reduce to 4
```

Monitor GPU memory:
```bash
watch -n 1 nvidia-smi  # Watch GPU usage in real-time
```

### 2. Enable NVIDIA DALI (Optional)

For even faster calibration:

```bash
# Install NVIDIA DALI
pip install nvidia-dali-cuda120  # Match your CUDA version

# Enable in Hailo config
# Add to calibration_config.yaml:
use_dali: true
```

**Benefits:**
- 2-3× faster data loading
- GPU-accelerated preprocessing

### 3. Use Native Installation (Not Docker)

From community feedback:
> "Native installation provides better GPU access and is generally more reliable."

**Docker issues:**
- GPU passthrough can fail
- Lower performance
- Complex setup

**Solution:** Install Hailo tools natively on Ubuntu.

---

## 📁 Full Workflow Example: YOLOv8 for WRO

### Step-by-Step

```bash
# 1. Train YOLOv8 (on desktop GPU)
cd ~/wro_ws
python train_yolo.py --data wro_signs.yaml --epochs 100

# 2. Export to ONNX
python export_onnx.py --model runs/train/weights/best.pt

# 3. Collect calibration data
ros2 bag record /camera/image_raw  # Drive manually 10 laps
python extract_images.py --bag recorded.db3 --output calib_data/
# Result: 1,200 images in calib_data/

# 4. Run Hailo calibration (GPU machine)
hailo parse onnx best.onnx --hw-arch hailo8l --har best.har

hailo optimize \
    --har best.har \
    --calib-data-path calib_data/ \
    --calib-set-size 1024 \
    --batch-size 8 \
    --har best_optimized.har

hailo compile --har best_optimized.har --hef best.hef

# 5. Deploy to Raspberry Pi AI HAT+
scp best.hef pi@raspberrypi:/home/pi/wro_ws/models/

# 6. Test on Pi5
ssh pi@raspberrypi
cd ~/wro_ws
python test_hailo.py --model models/best.hef
```

**Total time:** ~2-4 hours (including data collection)

---

## 🔍 Troubleshooting

### Issue 1: "Out of Memory" during calibration

**Symptoms:**
```
RuntimeError: CUDA out of memory
```

**Solutions:**
1. Reduce batch size: `batch_size: 8` → `batch_size: 4`
2. Use fewer calibration images: `calib_set_size: 1024` → `calib_set_size: 512`
3. Close other GPU applications
4. Use a GPU with more VRAM (8GB+ recommended)

### Issue 2: Large accuracy drop (>5%)

**Symptoms:**
Quantized model has much worse accuracy than original.

**Solutions:**
1. **Increase calibration data:** 1k → 5k images
2. **Try different calibration method:**
   ```yaml
   calib_method: percentile  # Instead of mse
   ```
3. **Enable advanced optimizations:**
   ```yaml
   optimization:
     adaround: true
     bias_correction: true
   ```
4. **Check data quality:** Ensure calibration data is representative

### Issue 3: GPU not detected

**Symptoms:**
```
No GPU available, using CPU
```

**Solutions:**
```bash
# Check NVIDIA driver
nvidia-smi

# Install CUDA
sudo apt install nvidia-cuda-toolkit

# Verify PyTorch sees GPU
python -c "import torch; print(torch.cuda.is_available())"
```

### Issue 4: Hailo Compiler errors

**Common errors:**
- `Unsupported ONNX opset` → Update ONNX export: `opset_version=11`
- `Dynamic shapes not supported` → Fix input shape: `imgsz=640` (static)
- `Unsupported layer` → Simplify model: `simplify=True` in export

**Check Hailo logs:**
```bash
cat ~/.hailo/logs/latest.log
```

---

## 📊 Performance Benchmarks

### Expected Performance on Your Hardware

**Your setup:** Raspberry Pi 5 16GB + **Hailo AI HAT+ 26 TOPS**

| Model | Input Size | Hailo FPS | Accuracy | Latency |
|-------|-----------|-----------|----------|---------|
| **YOLOv8-Nano** | 640×640 | 40-60 FPS | ~94% | 16-25 ms |
| **YOLOv5s** | 640×640 | 30-45 FPS | ~95% | 22-33 ms |
| **YOLOv8-Small** | 640×640 | 25-35 FPS | ~96% | 28-40 ms |
| **ViT-Small** | 224×224 | 25-35 FPS | ~93% | 28-40 ms |

**Notes:**
- AI HAT+ 26 TOPS is 2× better than Hailo-8L (13 TOPS)
- You'll get upper end of FPS ranges!

---

## 🎯 Recommendations for Your Project

### For Proposal 4 (ROS2 + YOLO) - RECOMMENDED

**Calibration Plan:**

1. **Collect Data (Week 5):**
   - Manual driving: 10 laps → ~1,200 images
   - Gazebo simulation: 500 synthetic images
   - **Total: 1,700 images**

2. **Train YOLO (Week 5):**
   - YOLOv8-Nano on desktop GPU
   - 100 epochs (~2 hours on RTX 3060)

3. **Calibrate with Hailo (Week 6):**
   - Use desktop GPU (RTX 3060+)
   - Batch size: 8
   - Expected time: 20-30 minutes
   - Expected accuracy: ~94%

4. **Deploy (Week 6):**
   - Transfer HEF to Pi5
   - Test: Should get 40-60 FPS!

**GPU Requirements:**
- **Training:** RTX 3060+ (6GB+ VRAM)
- **Calibration:** Same GPU (32GB RAM recommended)

### For Proposal 3 (Vision Transformer)

**Calibration Plan:**

1. **Collect Data (Weeks 3-5):**
   - Manual driving: 50+ hours → ~180,000 images (training)
   - Use subset for calibration: 5,000 images

2. **Train ViT (Weeks 6-8):**
   - PyTorch on desktop GPU
   - 1M RL steps in Gazebo (~3 days)

3. **Calibrate with Hailo (Week 9):**
   - Use desktop GPU
   - Batch size: 4-8 (ViT is larger)
   - Expected time: 45-60 minutes
   - Expected accuracy: ~92-93%

4. **Deploy (Week 9):**
   - Transfer HEF to Pi5
   - Test: Should get 25-35 FPS

**GPU Requirements:**
- **Training:** RTX 4080+ (16GB+ VRAM for ViT training)
- **Calibration:** RTX 3060+ (8GB+ VRAM, 32GB RAM)

---

## 📚 Resources

### Official Documentation
- [Hailo Dataflow Compiler User Guide (v3.27.0)](https://mmmsk.ai.kr/Projects/Embedded-AI/files/hailo_dataflow_compiler_v3.27.0_user_guide.pdf)
- [Hailo Model Zoo (GitHub)](https://github.com/hailo-ai/hailo_model_zoo)
- [Hailo Community Forum](https://community.hailo.ai/)

### Tutorials
- [Convert ONNX Models to Hailo8L: Step-by-Step Guide](https://www.ridgerun.ai/post/convert-onnx-model-to-hailo8l)
- [YOLOv11n to Hailo-8 HEF Compilation Guide](https://common.rosecityrobotics.com/YOLO_ObjectDetection/YOLOv11n_to_Hailo8_Guide.html)
- [Accelerating MediaPipe models with Hailo-8](https://medium.com/@grouby177/accelerating-the-mediapipe-models-with-hailo-8-69dc24719c9f)

### Community Discussions
- [Hailo Calibration - Quantization process](https://community.hailo.ai/t/hailo-calibration-quantization-process/2152)
- [GPU trained model compile error](https://community.hailo.ai/t/gpu-trained-model-compile-error/13625)

---

## ✅ Calibration Checklist

Before deploying to Pi5:

- [ ] Desktop/laptop with NVIDIA GPU (8GB+ VRAM)
- [ ] 32 GB+ RAM
- [ ] Ubuntu 22.04+ installed
- [ ] CUDA and NVIDIA drivers installed
- [ ] Hailo Dataflow Compiler installed (native, not Docker)
- [ ] Collected 1,000+ calibration images
- [ ] Trained model exported to ONNX
- [ ] Calibration config created
- [ ] Ran calibration (hailo optimize)
- [ ] Compiled to HEF (hailo compile)
- [ ] Verified accuracy drop <2%
- [ ] Tested HEF on Pi5 + AI HAT+
- [ ] Measured FPS (should be 30-60 for YOLO)

---

## 🏁 Summary

**Key Takeaways:**

1. **Use GPU for calibration** - 10-50× faster than CPU
2. **Recommended:** NVIDIA RTX 3060+ with 32GB RAM
3. **Calibration dataset:** 1,000-5,000 images (no labels needed)
4. **Expected time:** 10-40 minutes with GPU (vs 2-6 hours CPU)
5. **Accuracy drop:** Should be <1-2%
6. **Your AI HAT+ 26 TOPS:** Excellent hardware! Will run YOLO at 40-60 FPS

**Next Steps:**

1. Set up desktop GPU machine (Ubuntu + CUDA)
2. Install Hailo Dataflow Compiler (native)
3. Collect calibration data (1k+ images)
4. Train & export model to ONNX
5. Calibrate with GPU
6. Deploy HEF to Pi5 + AI HAT+
7. Enjoy fast AI inference! 🚀

---

**Good luck with your Hailo calibration!**

## Sources

- [Hailo Calibration - Quantization process - Hailo Community](https://community.hailo.ai/t/hailo-calibration-quantization-process/2152)
- [Hailo Model Zoo - Optimization Docs](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/OPTIMIZATION.rst)
- [Convert ONNX Models to Hailo8L - RidgeRun](https://www.ridgerun.ai/post/convert-onnx-model-to-hailo8l)
- [YOLOv11n to Hailo-8 HEF Compilation Guide - Rose City Robotics](https://common.rosecityrobotics.com/YOLO_ObjectDetection/YOLOv11n_to_Hailo8_Guide.html)
- [Hailo Dataflow Compiler User Guide v3.27.0](https://mmmsk.ai.kr/Projects/Embedded-AI/files/hailo_dataflow_compiler_v3.27.0_user_guide.pdf)
- [Raspberry Pi AI HAT+ 2 announcement - CNX Software](https://www.cnx-software.com/2026/01/15/raspberry-pi-ai-hat-2-targets-generative-ai-llm-vlm-with-hailo-10h-accelerator/)

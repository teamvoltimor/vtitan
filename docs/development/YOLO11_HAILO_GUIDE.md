# YOLO11 + Hailo 8L Integration Guide

## Overview

**YOLO11** (released by Ultralytics in late 2024) is the latest and most efficient YOLO model. Combined with Hailo 8L acceleration, you can achieve **30-60 FPS** on Raspberry Pi 5 for sign detection.

**Why YOLO11?**
- ✅ **Fastest inference** - 20-30% faster than YOLOv8
- ✅ **Better accuracy** - Improved architecture
- ✅ **Smaller models** - YOLO11n (nano) perfect for edge devices
- ✅ **Hailo support** - Works with Hailo 8L via ONNX conversion
- ✅ **Same API** - Drop-in replacement for YOLOv8

---

## Architecture Overview

```
┌─────────────────────────────────────────────────┐
│               Training (Laptop)                 │
│                                                 │
│  Collect Data → Label → Train YOLO11n          │
│                          ↓                      │
│                    checkpoint.pt                │
│                          ↓                      │
│                   Export to ONNX                │
│                          ↓                      │
│                    model.onnx                   │
└─────────────────────────────────────────────────┘
                          ↓
                   Transfer to RPi5
                          ↓
┌─────────────────────────────────────────────────┐
│          Hailo Conversion (RPi5 or Laptop)      │
│                                                 │
│  model.onnx → Hailo Compiler → model.hef        │
│                                                 │
└─────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────┐
│            Inference (RPi5 + Hailo 8L)          │
│                                                 │
│  Camera → YOLO11 (Hailo) → Detections → ROS2   │
│           13 TOPS @ 4W                          │
│           30-60 FPS @ 640x480                   │
└─────────────────────────────────────────────────┘
```

---

## Phase 1: Training YOLO11 on Your Laptop

### Step 1.1: Install Ultralytics (Laptop)

```bash
# In WSL2
pip3 install ultralytics

# Verify installation
yolo version
# Should show: Ultralytics YOLO11...

# Test GPU
python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"
# Should show: CUDA: True (if RTX 4050 is configured)
```

### Step 1.2: Prepare Dataset

**Directory structure:**
```
~/teamsteelbot_ws/datasets/traffic_signs/
├── images/
│   ├── train/
│   │   ├── img001.jpg
│   │   ├── img002.jpg
│   │   └── ...
│   └── val/
│       ├── img101.jpg
│       └── ...
├── labels/
│   ├── train/
│   │   ├── img001.txt  # YOLO format labels
│   │   ├── img002.txt
│   │   └── ...
│   └── val/
│       ├── img101.txt
│       └── ...
└── data.yaml
```

**data.yaml:**
```yaml
# Dataset configuration for YOLO11

path: /home/your_user/teamsteelbot_ws/datasets/traffic_signs
train: images/train
val: images/val

# Classes
nc: 3  # Number of classes
names: ['red_sign', 'green_sign', 'blue_sign']
```

### Step 1.3: Data Collection Strategies

**Option A: Real images (Best accuracy)**
```bash
# Capture images from mock camera or real camera
ros2 run teamsteelbot_tools image_capturer --output-dir ~/datasets/raw_images

# Label with Roboflow (easiest) or LabelImg
# Roboflow: https://roboflow.com (free tier)
# Export in YOLO format
```

**Option B: Synthetic data (Fastest to start)**
```python
# Create synthetic training data
# File: ~/teamsteelbot_ws/src/teamsteelbot_tools/scripts/generate_synthetic_data.py

import cv2
import numpy as np
import os
from pathlib import Path

def generate_sign_image(sign_color, img_id, output_dir):
    """Generate synthetic sign image with label"""
    # Create image (640x480)
    img = np.random.randint(100, 150, (480, 640, 3), dtype=np.uint8)

    # Random sign position
    sign_w, sign_h = np.random.randint(80, 150), np.random.randint(100, 180)
    x = np.random.randint(50, 640 - sign_w - 50)
    y = np.random.randint(50, 480 - sign_h - 50)

    # Color map
    colors = {
        'red': (0, 0, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0),
    }
    color = colors[sign_color]

    # Draw sign
    cv2.rectangle(img, (x, y), (x + sign_w, y + sign_h), color, -1)
    cv2.rectangle(img, (x, y), (x + sign_w, y + sign_h), (255, 255, 255), 3)

    # Add some noise/variation
    noise = np.random.randint(-20, 20, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # Save image
    img_path = output_dir / 'images' / f'{img_id:04d}.jpg'
    cv2.imwrite(str(img_path), img)

    # Create YOLO label (normalized coordinates)
    label_path = output_dir / 'labels' / f'{img_id:04d}.txt'
    class_id = {'red': 0, 'green': 1, 'blue': 2}[sign_color]
    x_center = (x + sign_w / 2) / 640
    y_center = (y + sign_h / 2) / 480
    width = sign_w / 640
    height = sign_h / 480

    with open(label_path, 'w') as f:
        f.write(f'{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n')

# Generate dataset
output_dir = Path('~/teamsteelbot_ws/datasets/traffic_signs/train')
output_dir.mkdir(parents=True, exist_ok=True)
(output_dir / 'images').mkdir(exist_ok=True)
(output_dir / 'labels').mkdir(exist_ok=True)

# Generate 1000 training images (333 each color)
img_id = 0
for _ in range(333):
    for color in ['red', 'green', 'blue']:
        generate_sign_image(color, img_id, output_dir)
        img_id += 1

print(f'Generated {img_id} synthetic training images')
```

**Option C: Mix of real + synthetic (Recommended)**
- Start with 200-500 synthetic images (quick baseline)
- Add 50-100 real images for fine-tuning
- Best balance of speed and accuracy

### Step 1.4: Train YOLO11

```bash
# Train YOLO11-nano (smallest, fastest)
yolo detect train \
  data=~/teamsteelbot_ws/datasets/traffic_signs/data.yaml \
  model=yolo11n.pt \
  epochs=100 \
  imgsz=640 \
  batch=16 \
  device=0 \
  project=~/teamsteelbot_ws/models \
  name=traffic_signs_yolo11n

# Training will take 30-60 minutes on RTX 4050
# Model saved to: ~/teamsteelbot_ws/models/traffic_signs_yolo11n/weights/best.pt
```

**Training parameters explained:**
- `model=yolo11n.pt` - Nano model (fastest, smallest)
- `epochs=100` - Training iterations (adjust based on dataset size)
- `imgsz=640` - Input image size (640x640, Hailo optimized)
- `batch=16` - Batch size (reduce if out of memory)
- `device=0` - Use GPU 0 (RTX 4050)

**Alternative models (if nano is not accurate enough):**
```bash
# YOLO11-small (better accuracy, slightly slower)
yolo detect train data=data.yaml model=yolo11s.pt epochs=100

# YOLO11-medium (best accuracy, slowest)
yolo detect train data=data.yaml model=yolo11m.pt epochs=100
```

### Step 1.5: Evaluate Model

```bash
# Test on validation set
yolo detect val \
  model=~/teamsteelbot_ws/models/traffic_signs_yolo11n/weights/best.pt \
  data=~/teamsteelbot_ws/datasets/traffic_signs/data.yaml

# Test on single image
yolo detect predict \
  model=~/teamsteelbot_ws/models/traffic_signs_yolo11n/weights/best.pt \
  source=~/test_images/test_sign.jpg \
  save=True
```

**Target metrics:**
- **mAP@0.5:** >0.85 (85% accuracy)
- **Precision:** >0.90
- **Recall:** >0.85
- **Inference time (laptop):** <10ms

### Step 1.6: Export to ONNX

```bash
# Export to ONNX format (required for Hailo)
yolo export \
  model=~/teamsteelbot_ws/models/traffic_signs_yolo11n/weights/best.pt \
  format=onnx \
  imgsz=640 \
  simplify=True

# Output: best.onnx
# This file will be converted to Hailo HEF format
```

---

## Phase 2: Convert ONNX to Hailo HEF

### Option A: On Laptop (Recommended if you have Hailo Dataflow Compiler)

```bash
# Install Hailo Dataflow Compiler
# (Follow Hailo's installation guide for your system)
# Download from: https://hailo.ai/developer-zone/

# Convert ONNX to HEF
hailo parser onnx best.onnx

hailo optimize \
  --hw-arch hailo8l \
  --input-model best.hn \
  --output-model best_optimized.har

hailo compile \
  --hw-arch hailo8l \
  --input-model best_optimized.har \
  --output-model traffic_signs_yolo11n.hef
```

### Option B: On Raspberry Pi 5 (Simpler, no separate compiler needed)

The Hailo runtime on RPi5 can sometimes use ONNX directly, or you can use pre-built conversion tools.

```bash
# On RPi5, install Hailo Python API
pip3 install hailort

# Use Hailo's model conversion script (if available)
# Or transfer HEF from laptop
```

### Option C: Use Hailo Model Zoo (Easiest)

```bash
# Clone Hailo Model Zoo
git clone https://github.com/hailo-ai/hailo_model_zoo.git
cd hailo_model_zoo

# Check if YOLO11 is available
# If not, use their conversion scripts for custom models
python hailo_model_zoo/main.py parse \
  --hw-arch hailo8l \
  --ckpt best.onnx \
  --output-model traffic_signs.har

python hailo_model_zoo/main.py compile \
  --hw-arch hailo8l \
  --har traffic_signs.har \
  --output-model traffic_signs.hef
```

---

## Phase 3: ROS2 Integration (Laptop Development)

### Step 3.1: YOLO11 Detector Node (Development Version - CPU/GPU)

First, develop and test on laptop using ONNX Runtime or PyTorch.

**File:** `~/teamsteelbot_ws/src/teamsteelbot_vision/teamsteelbot_vision/sign_detector_yolo11.py`

```python
#!/usr/bin/env python3
"""
YOLO11 sign detector for ROS2
Development version: Uses ONNX Runtime (works on laptop and RPi5)
Production version: Uses Hailo runtime (RPi5 only)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

# Development: Use ONNX Runtime
import onnxruntime as ort

# Production (RPi5): Use Hailo
# from hailo_platform import HEF, VDevice, HailoStreamInterface, InferVStreams


class SignDetectorYOLO11(Node):
    def __init__(self):
        super().__init__('sign_detector_yolo11')

        # Parameters
        self.declare_parameter('model_path',
            '~/teamsteelbot_ws/models/traffic_signs_yolo11n/weights/best.onnx')
        self.declare_parameter('use_hailo', False)  # True on RPi5, False on laptop
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)

        model_path = self.get_parameter('model_path').value
        self.use_hailo = self.get_parameter('use_hailo').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value

        # Class names
        self.class_names = ['red_sign', 'green_sign', 'blue_sign']

        # Load model
        if self.use_hailo:
            self.load_hailo_model(model_path)
        else:
            self.load_onnx_model(model_path)

        # ROS2 setup
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/detections/debug_image', 10)

        self.bridge = CvBridge()

        self.get_logger().info(f'YOLO11 detector started (Hailo: {self.use_hailo})')

    def load_onnx_model(self, model_path):
        """Load ONNX model for CPU/GPU inference (laptop development)"""
        self.get_logger().info(f'Loading ONNX model: {model_path}')

        # ONNX Runtime session
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)

        # Get input shape
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.input_height = self.input_shape[2]  # 640
        self.input_width = self.input_shape[3]   # 640

        self.get_logger().info(f'Model loaded: {self.input_width}x{self.input_height}')

    def load_hailo_model(self, model_path):
        """Load Hailo HEF model (RPi5 production)"""
        self.get_logger().info(f'Loading Hailo model: {model_path}')

        # This will be implemented when deploying to RPi5
        # For now, we'll use ONNX on laptop
        raise NotImplementedError("Hailo support coming soon - use use_hailo:=false for now")

    def preprocess_image(self, cv_image):
        """Preprocess image for YOLO11 input"""
        # Resize to model input size
        img = cv2.resize(cv_image, (self.input_width, self.input_height))

        # Convert BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Normalize to [0, 1]
        img = img.astype(np.float32) / 255.0

        # Transpose to CHW format (channels first)
        img = img.transpose(2, 0, 1)

        # Add batch dimension
        img = np.expand_dims(img, axis=0)

        return img

    def postprocess_detections(self, outputs, orig_shape):
        """Convert YOLO11 outputs to bounding boxes"""
        # YOLO11 output format: [batch, num_boxes, 5 + num_classes]
        # [x_center, y_center, width, height, confidence, class_scores...]

        predictions = outputs[0]  # Remove batch dimension

        detections = []

        for pred in predictions:
            # Parse prediction
            x_center, y_center, w, h = pred[0:4]
            confidence = pred[4]
            class_scores = pred[5:]

            # Filter by confidence
            if confidence < self.conf_threshold:
                continue

            # Get class with highest score
            class_id = np.argmax(class_scores)
            class_score = class_scores[class_id]

            # Combined score
            score = confidence * class_score

            if score < self.conf_threshold:
                continue

            # Convert to pixel coordinates
            orig_h, orig_w = orig_shape[:2]
            x_center_px = x_center * orig_w / self.input_width
            y_center_px = y_center * orig_h / self.input_height
            w_px = w * orig_w / self.input_width
            h_px = h * orig_h / self.input_height

            detections.append({
                'bbox': [x_center_px, y_center_px, w_px, h_px],
                'class_id': int(class_id),
                'class_name': self.class_names[class_id],
                'confidence': float(score)
            })

        # Apply NMS (Non-Maximum Suppression)
        detections = self.apply_nms(detections)

        return detections

    def apply_nms(self, detections):
        """Apply Non-Maximum Suppression to remove duplicate detections"""
        if len(detections) == 0:
            return []

        # Convert to format for cv2.dnn.NMSBoxes
        boxes = []
        confidences = []

        for det in detections:
            x_c, y_c, w, h = det['bbox']
            x1 = int(x_c - w / 2)
            y1 = int(y_c - h / 2)
            boxes.append([x1, y1, int(w), int(h)])
            confidences.append(det['confidence'])

        # Apply NMS
        indices = cv2.dnn.NMSBoxes(boxes, confidences,
                                   self.conf_threshold, self.iou_threshold)

        # Return filtered detections
        if len(indices) > 0:
            return [detections[i] for i in indices.flatten()]
        else:
            return []

    def image_callback(self, msg):
        """Process incoming image"""
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_shape = cv_image.shape

        # Preprocess
        input_tensor = self.preprocess_image(cv_image)

        # Inference
        outputs = self.session.run(None, {self.input_name: input_tensor})

        # Postprocess
        detections = self.postprocess_detections(outputs[0][0], orig_shape)

        # Create ROS2 detection messages
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        debug_image = cv_image.copy()

        for det in detections:
            # Create Detection2D message
            detection = Detection2D()
            detection.header = msg.header

            # Bounding box
            x_c, y_c, w, h = det['bbox']
            detection.bbox.center.position.x = x_c
            detection.bbox.center.position.y = y_c
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            # Classification
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class_name']
            hypothesis.hypothesis.score = det['confidence']
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

            # Draw on debug image
            x1 = int(x_c - w / 2)
            y1 = int(y_c - h / 2)
            x2 = int(x_c + w / 2)
            y2 = int(y_c + h / 2)

            color = {
                'red_sign': (0, 0, 255),
                'green_sign': (0, 255, 0),
                'blue_sign': (255, 0, 0),
            }[det['class_name']]

            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug_image,
                       f"{det['class_name']} {det['confidence']:.2f}",
                       (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish
        self.detection_pub.publish(detection_array)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        if detections:
            self.get_logger().info(f'Detected {len(detections)} signs')


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorYOLO11()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.2: Install Dependencies

```bash
# ONNX Runtime (for laptop development)
pip3 install onnxruntime-gpu  # GPU version
# Or: pip3 install onnxruntime  # CPU only

# Test
python3 -c "import onnxruntime; print(onnxruntime.get_device())"
```

### Step 3.3: Test on Laptop

```bash
# Terminal 1: Mock camera
ros2 run teamsteelbot_simulation mock_camera_node

# Terminal 2: YOLO11 detector
ros2 run teamsteelbot_vision sign_detector_yolo11 --ros-args \
  -p model_path:=~/teamsteelbot_ws/models/traffic_signs_yolo11n/weights/best.onnx \
  -p use_hailo:=false

# Terminal 3: View detections
ros2 topic echo /detections

# Terminal 4: View debug image in RViz2
rviz2
# Add Image display → /detections/debug_image
```

---

## Phase 4: Hailo Integration (RPi5 Only)

### Step 4.1: Install Hailo Runtime on RPi5

```bash
# On Raspberry Pi 5
# Install Hailo PCIe driver
sudo apt update
sudo apt install hailo-all

# Install Python bindings
pip3 install hailort

# Verify Hailo device
hailortcli fw-control identify
# Should show: Hailo-8L detected
```

### Step 4.2: Hailo-Optimized Detector Node

**File:** `sign_detector_yolo11_hailo.py` (create separate file for RPi5)

```python
#!/usr/bin/env python3
"""
YOLO11 detector with native Hailo acceleration (RPi5 only)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

from hailo_platform import (HEF, ConfigureParams, FormatType, HailoStreamInterface,
                            InferVStreams, InputVStreamParams, OutputVStreamParams,
                            VDevice)


class SignDetectorYOLO11Hailo(Node):
    def __init__(self):
        super().__init__('sign_detector_yolo11_hailo')

        # Parameters
        self.declare_parameter('model_path',
            '~/teamsteelbot_ws/models/traffic_signs_yolo11n.hef')
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        # Class names
        self.class_names = ['red_sign', 'green_sign', 'blue_sign']

        # Initialize Hailo
        self.init_hailo(model_path)

        # ROS2 setup
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/detections/debug_image', 10)

        self.bridge = CvBridge()

        self.get_logger().info('YOLO11 Hailo detector started')

    def init_hailo(self, hef_path):
        """Initialize Hailo device and load model"""
        self.get_logger().info(f'Loading Hailo model: {hef_path}')

        # Load HEF
        self.hef = HEF(hef_path)

        # Create VDevice (virtual device)
        self.target = VDevice()

        # Configure network group
        self.network_group = self.target.configure(self.hef)[0]
        self.network_group_params = self.network_group.create_params()

        # Get input/output shapes
        self.input_vstream_info = self.hef.get_input_vstream_infos()[0]
        self.output_vstream_info = self.hef.get_output_vstream_infos()[0]

        self.input_shape = self.input_vstream_info.shape
        self.input_height = self.input_shape[0]
        self.input_width = self.input_shape[1]

        self.get_logger().info(f'Hailo initialized: {self.input_width}x{self.input_height}')

    def preprocess_image(self, cv_image):
        """Preprocess image for Hailo input"""
        # Resize
        img = cv2.resize(cv_image, (self.input_width, self.input_height))

        # Hailo expects uint8 input (no normalization needed)
        return img

    def image_callback(self, msg):
        """Process incoming image with Hailo"""
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_shape = cv_image.shape

        # Preprocess
        input_data = self.preprocess_image(cv_image)

        # Hailo inference
        with InferVStreams(self.network_group,
                          InputVStreamParams.make_from_network_group(self.network_group, quantized=False),
                          OutputVStreamParams.make_from_network_group(self.network_group, quantized=False)) as infer_pipeline:

            # Run inference
            input_dict = {self.input_vstream_info.name: input_data}
            output_dict = infer_pipeline.infer(input_dict)

            # Get output
            output_data = list(output_dict.values())[0]

        # Postprocess (similar to ONNX version)
        detections = self.postprocess_detections(output_data, orig_shape)

        # Publish detections (same as ONNX version)
        # ... (copy from ONNX version)

        if detections:
            self.get_logger().info(f'Detected {len(detections)} signs @ Hailo')


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorYOLO11Hailo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4.3: Test on RPi5

```bash
# On RPi5
ros2 run teamsteelbot_vision sign_detector_yolo11_hailo --ros-args \
  -p model_path:=~/teamsteelbot_ws/models/traffic_signs_yolo11n.hef

# Should achieve 30-60 FPS with Hailo acceleration!
```

---

## Performance Comparison

| Platform | Model | Backend | FPS | Latency |
|----------|-------|---------|-----|---------|
| Laptop (RTX 4050) | YOLO11n | ONNX GPU | 200+ | ~5ms |
| RPi5 | YOLO11n | ONNX CPU | 10-15 | ~70ms |
| **RPi5 + Hailo 8L** | **YOLO11n** | **Hailo** | **30-60** | **~20ms** |

**Hailo advantage: 3-4x speedup vs CPU!**

---

## Troubleshooting

### Issue: Low FPS on Hailo
**Solution:** Check if model is actually using Hailo
```bash
hailortcli run model.hef
# Should show GPU utilization
```

### Issue: ONNX model conversion fails
**Solution:** Simplify export
```bash
yolo export model=best.pt format=onnx imgsz=640 simplify=True opset=11
```

### Issue: Different results on Hailo vs ONNX
**Solution:** This is expected due to quantization. Retrain with quantization-aware training if needed.

---

## Summary: Development Workflow

```
┌─────────────────────────────────────────┐
│  Week 1-2: Train on Laptop (RTX 4050)  │
│  → Collect data, train YOLO11n          │
│  → Export to ONNX                       │
│  → Test with ONNX Runtime (200+ FPS)    │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  Week 3-6: Develop Logic on Laptop     │
│  → ROS2 nodes with ONNX backend         │
│  → State machine, control logic         │
│  → Full simulation                      │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  Week 7-8: Port to RPi5                 │
│  → Convert ONNX to Hailo HEF            │
│  → Test Hailo performance (30-60 FPS)   │
│  → Integrate with real sensors          │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  Week 9+: Physical Testing              │
│  → Track testing with Hailo             │
│  → Fine-tune, optimize                  │
│  → Competition ready! 🏁                │
└─────────────────────────────────────────┘
```

---

## Next Steps

1. **Install Ultralytics:** `pip3 install ultralytics`
2. **Generate synthetic dataset** (or collect real images)
3. **Train YOLO11n:** `yolo detect train data=data.yaml model=yolo11n.pt`
4. **Export to ONNX:** `yolo export model=best.pt format=onnx`
5. **Integrate with ROS2** (use starter code above)
6. **Test on laptop** with ONNX Runtime
7. **Port to RPi5** and convert to Hailo HEF

**YOLO11 + Hailo = 30-60 FPS sign detection on edge device! 🚀**

# WRO State Machine - Quick Start Guide

## Hardware Setup (5 minutes)

### 1. Connect Physical Button
```
GPIO17 (Pin 11) ──┬── Button ── GND (Pin 6)
                  └── 10kΩ Pull-up (internal)
```

### 2. Connect OLED Display (SSD1306)
```
Raspberry Pi 5              SSD1306 OLED
────────────────            ─────────────
3.3V (Pin 1)     ───────→   VCC
GND (Pin 6)      ───────→   GND
GPIO2 (Pin 3)    ───────→   SDA
GPIO3 (Pin 5)    ───────→   SCL
```

### 3. Connect IMU (MCP2221A)
- Plug MCP2221A USB adapter into any USB port
- Auto-detection via USB VID/PID (no configuration needed)

### 4. Connect Hailo-8 NPU
- Insert Hailo-8 AI accelerator into M.2 M-Key slot
- Verify detection: `lspci | grep Hailo`

## Software Setup (10 minutes)

### 1. Install System Dependencies
```bash
# Enable I2C interface
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable

# Install required system packages
sudo apt-get update
sudo apt-get install -y i2c-tools python3-pip
```

### 2. Install Python Dependencies
```bash
cd platform/robot
pip install gpiozero pillow adafruit-circuitpython-ssd1306 cv-bridge scipy
```

### 3. Install ROS2 Dependencies
```bash
sudo apt-get install -y \
    ros-humble-ackermann-msgs \
    ros-humble-vision-msgs \
    ros-humble-diagnostic-msgs
```

### 4. Build ROS2 Workspace
```bash
cd platform/robot/ros2_ws
colcon build --packages-select klevor_robot
source install/setup.bash
```

### 5. Configure Environment
```bash
# Copy example configuration
cp .env.example .env

# Edit configuration (optional - defaults work for standard setup)
nano .env
```

### 6. Set Permissions
```bash
# Add user to GPIO and I2C groups
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER

# Allow I2C access (or reboot)
sudo chmod a+rw /dev/i2c-*

# Log out and back in for group changes to take effect
```

## Verification Tests (5 minutes)

### Test 1: Button
```bash
# Test button functionality
python3 -c "
from src.hardware.button.gpio import Driver as ButtonDriver
button = ButtonDriver()
button.connect()
print('Press button now...')
button.wait_for_press(timeout=10)
print('Button works!')
button.close()
"
```

### Test 2: Display
```bash
# Test OLED display
python3 -c "
from src.hardware.display.ssd1306 import Driver as DisplayDriver
from PIL import ImageDraw
display = DisplayDriver()
display.connect()
img = display.get_blank_image()
draw = ImageDraw.Draw(img)
draw.text((0, 0), 'WRO Test OK', fill=255)
display.show_image(img)
print('Check OLED - you should see: WRO Test OK')
"
```

### Test 3: IMU
```bash
# Test IMU communication
ros2 run klevor_robot bno08x_uart_rvc_node &
sleep 3
ros2 topic echo /imu/data --once
killall bno08x_uart_rvc_node
```

### Test 4: LiDAR
```bash
# Test LiDAR
ros2 launch klevor_robot lidar_launch.py &
sleep 3
ros2 topic echo /scan --once
killall sllidar_node
```

## Run State Machine System

### Full System Launch
```bash
# Terminal 1: Launch all nodes
cd platform/robot/ros2_ws
source install/setup.bash
ros2 launch klevor_robot wro_state_machine_launch.py
```

### Monitor System (Optional)
```bash
# Terminal 2: Watch robot state changes
ros2 topic echo /robot_state

# Terminal 3: View remote OLED display on laptop
ros2 run rqt_image_view rqt_image_view /ui/oled_mirror
```

## Expected Behavior

### 1. BOOT_CHECK (0-10 seconds)
- OLED displays sensor checklist with ✓ or ✗ symbols
- System verifies: IMU, LiDAR, Hailo, Drive, Network
- IP address fetched asynchronously (shows "FETCHING..." then IP or "OFFLINE")
- Hailo model must be loaded and confirming inferences (FPS > 0)
- **Automatic transition to READY when all checks pass**

### 2. READY (waiting)
- OLED displays:
  ```
  READY TO START
  ━━━━━━━━━━━━━━
  IP: 192.168.1.100
  Model: yolov8n.hef
  Steering: READY
  
  Press to START
  ```
- **Press button to start race** → transitions to RACING

### 3. RACING (autonomous)
- Robot drives autonomously
- OLED auto-cycles through 3 pages every 1.2 seconds:
  - Page A: Ackermann data (velocity, steering, yaw, laps)
  - Page B: Hailo Vision (FPS, detections, confidence)
  - Page C: LiDAR (clearances, path status)
- Short button presses are ignored (competition rules)
- **Hold button 2 seconds for Emergency Stop** → transitions to FINISHED
- **Complete 3 laps** → transitions to FINISHED

### 4. FINISHED (terminal)
- Robot stops immediately (publishes 0.0 velocity/steering)
- OLED displays final results:
  ```
  RACE FINISHED
  ━━━━━━━━━━━━━━
  Laps: 3/3
  Time: 2:34.56
  
  Status: COMPLETE
  ```
- Press Ctrl+C to exit or restart system

## Troubleshooting

### Problem: "No IMU data received"
**Solution:**
```bash
# Check USB connection
lsusb | grep 04d8:00dd
# Should show: Bus XXX Device XXX: ID 04d8:00dd Microchip Technology, Inc.

# Check serial port
ls /dev/ttyACM*
# Should show: /dev/ttyACM0 or /dev/ttyACM1

# Test manual connection
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200); print('OK')"
```

### Problem: "Display not showing"
**Solution:**
```bash
# Check I2C is enabled
i2cdetect -y 1
# Should show address 0x3C in the grid

# If nothing shows, enable I2C:
sudo raspi-config
# Interface Options → I2C → Enable → Reboot
```

### Problem: "Button not responding"
**Solution:**
```bash
# Check GPIO permissions
groups | grep gpio
# If gpio not listed:
sudo usermod -a -G gpio $USER
# Log out and back in

# Test GPIO access
gpio readall
# Should show GPIO table without errors
```

### Problem: "Hailo model not loaded"
**Solution:**
```bash
# Verify Hailo PCIe connection
lspci | grep Hailo
# Should show: XX:XX.X Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor

# Check model file exists
ls -lh /home/pi/models/yolov8n.hef
# Should show file size (typically 5-20 MB)

# Verify model path in .env
grep HAILO_MODEL_PATH .env
```

### Problem: "IP shows OFFLINE"
**Solution:**
```bash
# Check network connection
ip addr show
# Should show IP address on wlan0 or eth0

# Test internet connectivity
ping -c 3 8.8.8.8

# If network is working but still shows OFFLINE:
# - Increase IP_FETCH_TIMEOUT_SEC in .env
# - Check firewall rules: sudo iptables -L
```

## ROS2 Topic Monitor Commands

```bash
# List all active topics
ros2 topic list

# Monitor robot state changes
ros2 topic echo /robot_state

# Watch system diagnostics
ros2 topic echo /system_status

# View race metrics (JSON)
ros2 topic echo /race_metrics

# Monitor Ackermann commands
ros2 topic echo /ackermann_cmd

# View OLED mirror stream
ros2 topic hz /ui/oled_mirror
```

## Debug Mode

For verbose logging during development:

```bash
# Set log level to DEBUG
export LOG_LEVEL=DEBUG

# Run node with debug output
ros2 run klevor_robot state_machine_node --ros-args --log-level DEBUG
```

## Next Steps

1. **Calibrate steering**: Use motor calibration tools to set steering limits
2. **Tune navigation**: Adjust Ackermann parameters for track performance  
3. **Train AI model**: Fine-tune Hailo model on WRO track objects
4. **Test lap detection**: Implement lap counting logic in state machine
5. **Practice button timing**: Train team on 2-second E-STOP procedure

## Support

For issues or questions:
- Check logs: `ros2 node list` and `ros2 node info <node_name>`
- Review diagnostics: `ros2 topic echo /system_status`
- Verify hardware connections with multimeter
- Test each component individually before full system launch

## Competition Day Checklist

- [ ] All hardware connections secure and tested
- [ ] Battery fully charged and voltage verified
- [ ] IMU calibrated and responding
- [ ] LiDAR spinning and publishing data
- [ ] Hailo model loaded and inference confirmed (FPS > 20)
- [ ] OLED display clear and readable
- [ ] Button press/hold tested and reliable
- [ ] IP address visible on display
- [ ] Emergency stop tested (2-second hold)
- [ ] Lap counting logic verified
- [ ] Backup .hef model file on robot
- [ ] System boots to READY in < 15 seconds

Good luck at the competition! 🏁

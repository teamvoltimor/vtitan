# WRO State Machine System

## Overview

This is a complete 4-stage state machine system for WRO (World Robot Olympiad) competition control. The system manages robot states, hardware verification, and autonomous racing operations in strict compliance with WRO competition rules.

## System Architecture

### State Machine States

1. **BOOT_CHECK** - Hardware verification phase
   - Verifies IMU, LiDAR, Ackermann drive, and Hailo NPU
   - Performs non-blocking network IP address fetch
   - Blocks robot startup until Hailo .hef model is loaded and confirming inferences
   - Fails gracefully to OFFLINE status if no network

2. **READY** - Pre-race standby
   - All hardware verified and operational
   - Displays IP address and AI model name
   - Waits for single physical button press to start race

3. **RACING** - Autonomous operation
   - Robot operates fully autonomously
   - Short button presses are ignored (competition compliance)
   - OLED display auto-cycles through 3 pages every 1.2 seconds:
     - Page A: Ackermann (velocity, steering, gyro yaw)
     - Page B: Hailo Vision (NPU FPS, detections, confidence)
     - Page C: LiDAR (spatial clearances, path status)

4. **FINISHED** - Race completion or E-STOP
   - Triggered by completing 3 laps OR 2-second button hold
   - Immediately publishes zero velocity/steering command
   - Displays final race results and lap count

### Hardware Components

- **Physical Button**: GPIO-based with debouncing and long-press detection (2s threshold)
- **OLED Display**: SSD1306 128x64 I2C display with live ROS2 mirroring
- **IMU**: BNO08x via MCP2221A (UART RVC mode)
- **LiDAR**: Slamtec RPLiDAR C1 (via sllidar_ros2)
- **Hailo NPU**: Hailo-8 AI accelerator for object detection

## Installation

### Dependencies

```bash
# Python dependencies
pip install gpiozero pillow adafruit-circuitpython-ssd1306 cv-bridge

# ROS2 dependencies
sudo apt-get install ros-humble-ackermann-msgs ros-humble-vision-msgs
```

### Build ROS2 Workspace

```bash
cd ros2_ws
colcon build --packages-select klevor_robot
source install/setup.bash
```

## Configuration

### Environment Variables

Create a `.env` file or export these variables:

```bash
# Button Configuration
export BUTTON_GPIO_PIN=17                    # GPIO pin for physical button
export BUTTON_PULL_UP=True                   # Use internal pull-up resistor
export BUTTON_DEBOUNCE_MS=50                 # Debounce delay in milliseconds
export BUTTON_LONG_PRESS_SEC=2.0             # Long press threshold (E-STOP)

# Display Configuration
export DISPLAY_WIDTH=128                     # Display width in pixels
export DISPLAY_HEIGHT=64                     # Display height in pixels
export DISPLAY_I2C_ADDRESS=0x3C              # I2C address of SSD1306
export DISPLAY_I2C_BUS=1                     # I2C bus number (/dev/i2c-1)

# IMU Configuration
export IMU_UART_RVC_PORT=/dev/ttyACM0        # Serial port for MCP2221 (auto-detect if empty)
export IMU_UART_RVC_BAUDRATE=115200          # Baud rate for UART
export IMU_UART_RVC_POLL_RATE=100            # Polling rate in Hz

# Hailo Configuration
export HAILO_MODEL_PATH=/home/pi/models/yolov8n.hef  # Path to .hef model file
```

### Hardware Wiring

**Button:**
- Connect button between GPIO17 (Pin 11) and GND
- Internal pull-up resistor is enabled by default

**OLED Display (I2C):**
- VCC → 3.3V (Pin 1)
- GND → GND (Pin 6)
- SDA → GPIO2 (Pin 3)
- SCL → GPIO3 (Pin 5)

**IMU (MCP2221A via USB):**
- Connect MCP2221A to USB port
- Auto-detection via VID/PID (0x04D8:0x00DD)

## Usage

### Start All Nodes

```bash
# Launch complete system
ros2 launch klevor_robot wro_state_machine_launch.py

# View display remotely on laptop
ros2 run rqt_image_view rqt_image_view /ui/oled_mirror
```

### Individual Node Testing

```bash
# Test state machine only
ros2 run klevor_robot state_machine_node

# Test OLED display only
ros2 run klevor_robot oled_display_node

# Test IMU only
ros2 run klevor_robot bno08x_uart_rvc_node
```

### Monitor Robot State

```bash
# Watch current state
ros2 topic echo /robot_state

# Monitor system diagnostics
ros2 topic echo /system_status

# View race metrics
ros2 topic echo /race_metrics
```

## ROS2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/robot_state` | `std_msgs/String` | Current state machine state |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Drive commands (speed, steering) |
| `/system_status` | `diagnostic_msgs/DiagnosticArray` | Hardware diagnostics |
| `/race_metrics` | `std_msgs/String` | Race metrics (JSON format) |
| `/ui/oled_mirror` | `sensor_msgs/Image` | Live OLED display mirror |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | IMU orientation and acceleration |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/hailo/fps` | `std_msgs/Float32` | Hailo inference FPS |
| `/hailo/detections` | `vision_msgs/Detection2DArray` | AI object detections |

## Display Pages

### BOOT_CHECK
```
BOOT CHECK
━━━━━━━━━━━━━━
✓ IMU
✓ LiDAR
✓ Hailo
✓ Drive
✓ IP:192.168.1.100
```

### READY
```
READY TO START
━━━━━━━━━━━━━━
IP: 192.168.1.100
Model: yolov8n.hef
Steering: READY

Press to START
```

### RACING - Page A (Ackermann)
```
ACKERMANN
━━━━━━━━━━━━━━
Vel: 1.50 m/s
Steer: 15.0 deg
Yaw: 45.2 deg
Laps: 1/3
```

### RACING - Page B (Hailo Vision)
```
HAILO VISION
━━━━━━━━━━━━━━
NPU: 25.5 FPS
Target: LOCKED
Conf: 0.87
Dist: 2.3 m
```

### RACING - Page C (LiDAR)
```
LIDAR
━━━━━━━━━━━━━━
Front: 120 cm
Left:  45 cm
Right: 50 cm
Path: CLEAR
```

### FINISHED
```
RACE FINISHED
━━━━━━━━━━━━━━
Laps: 3/3
Time: 2:34.56

Status: COMPLETE
```

## Competition Rules Compliance

✅ **Single button control**: One physical button manages all state transitions
✅ **E-STOP safety**: 2-second hold immediately stops robot at any time
✅ **No manual intervention during race**: Short presses ignored in RACING state
✅ **Hardware pre-flight checks**: Robot cannot start until all systems verified
✅ **AI model verification**: Hailo must be loaded and confirming inferences before start
✅ **Network fault tolerance**: Non-blocking IP fetch with graceful OFFLINE handling

## Troubleshooting

### Button not responding
- Check GPIO pin configuration (default: GPIO17)
- Verify button is wired between GPIO and GND
- Check permissions: `sudo usermod -a -G gpio $USER`

### Display not showing
- Verify I2C is enabled: `sudo raspi-config` → Interface Options → I2C
- Check I2C address: `i2cdetect -y 1` (should show 0x3C)
- Verify wiring on SDA/SCL pins

### IMU not detected
- Check USB connection for MCP2221A
- Verify port: `ls /dev/ttyACM*`
- Check VID/PID: `lsusb | grep 04d8:00dd`

### Hailo model not loading
- Verify .hef file path in environment variable
- Check Hailo PCIe driver: `lspci | grep Hailo`
- Monitor Hailo node logs for inference confirmation

### Remote display mirror not showing
- Verify cv_bridge is installed: `pip install cv-bridge`
- Check topic is publishing: `ros2 topic hz /ui/oled_mirror`
- Open rqt_image_view: `ros2 run rqt_image_view rqt_image_view`

## Performance Characteristics

- **UI Refresh Rate**: 10Hz (0.1s) - Fast data updates
- **Page Cycle Interval**: 1.2s - Readable page transitions during RACING
- **Button Check Rate**: 20Hz (0.05s) - Responsive button handling
- **State Machine Loop**: 10Hz - Regular state monitoring
- **IP Fetch**: Non-blocking async with 2s timeout
- **Sensor Timeout**: 3s - Hardware considered unavailable after this delay

## File Structure

```
platform/robot/
├── src/
│   ├── hardware/
│   │   ├── button/
│   │   │   ├── base.py                    # Abstract button interface
│   │   │   └── gpio/
│   │   │       └── driver.py              # GPIO button driver
│   │   └── display/
│   │       ├── base.py                    # Abstract display interface
│   │       └── ssd1306/
│   │           └── driver.py              # SSD1306 OLED driver
│   └── state_machine/
│       ├── types.py                       # State definitions & data structures
│       └── core.py                        # State machine logic
├── ros2_ws/src/klevor_robot/
│   ├── klevor_robot/
│   │   ├── state_machine_node.py          # State machine ROS2 node
│   │   └── oled_display_node.py           # OLED display ROS2 node
│   ├── launch/
│   │   └── wro_state_machine_launch.py    # Launch file for all nodes
│   └── setup.py                           # ROS2 package configuration
```

## Development Notes

### Adding New Sensors

To integrate additional sensors into the BOOT_CHECK process:

1. Add sensor status field to `SystemStatus` in `src/state_machine/types.py`
2. Implement sensor check in `_check_system_status()` in `state_machine_node.py`
3. Add sensor status display to `_render_boot_check()` in `oled_display_node.py`

### Custom Display Pages

To add new OLED display pages:

1. Add page name to `self.racing_pages` list in `OLEDDisplayNode.__init__()`
2. Create rendering function (e.g., `_render_custom_page()`)
3. Add page case to `_update_display()` conditional logic

### State Transition Customization

Valid state transitions are defined in `StateMachine._is_valid_transition()`. Modify the `valid_transitions` dictionary to add/remove allowed transitions.

## License

WRO 2026 Future Engineers Competition - Team Klevor

## Authors

- Ramón Álvarez - State Machine Architecture & Implementation

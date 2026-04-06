# Ackermann Motor Controller

## Overview

The Ackermann Motor Controller is a ROS2 node that bridges Ackermann drive commands (`/ackermann_cmd`) to physical motor control via the LEGO Build HAT. It provides:

- **Steering offset calibration** to compensate for mechanical misalignment
- **Drive direction reversal** for backwards-mounted motors
- **Safety limits** for speed and steering angle
- **Watchdog timer** to stop motors if commands cease
- **Real-time feedback** publishing motor position and speed

## Architecture

```
┌─────────────────────┐
│  State Machine /    │
│  Navigation Node    │
└──────────┬──────────┘
           │ /ackermann_cmd
           │ (velocity m/s, steering radians)
           ▼
┌─────────────────────────────────────────────┐
│     Ackermann Motor Controller Node         │
│  ┌─────────────────────────────────────┐   │
│  │ 1. Convert velocity → motor speed   │   │
│  │ 2. Apply steering offset            │   │
│  │ 3. Apply drive direction reversal   │   │
│  │ 4. Clamp to safety limits           │   │
│  │ 5. Send to Build HAT driver         │   │
│  └─────────────────────────────────────┘   │
└──────────┬──────────────────┬───────────────┘
           │                  │
           ▼                  ▼
    ┌──────────┐       ┌──────────┐
    │ Steering │       │  Drive   │
    │  Motor   │       │  Motor   │
    └──────────┘       └──────────┘
```

## Hardware Connection

The node runs on the **Raspberry Pi Zero** which has the Build HAT attached. The Pi Zero connects to the Pi 5 over the network via ROS2.

**Build HAT Port Assignments:**
- Port A: Steering motor (configurable via `MOTOR_STEERING_PORT`)
- Port B: Drive motor (configurable via `MOTOR_DRIVE_PORT`)

## Configuration

### Environment Variables

All configuration is done via environment variables in `.env`:

#### Port Configuration
```bash
MOTOR_STEERING_PORT=A          # Build HAT port for steering
MOTOR_DRIVE_PORT=B             # Build HAT port for drive
```

#### Steering Calibration
```bash
MOTOR_STEERING_OFFSET=0.0      # Offset in degrees
```

**How to calibrate:**
1. Set `MOTOR_STEERING_OFFSET=0.0`
2. Run node and send 0° steering command
3. Observe actual wheel alignment
4. If wheels point right, use **negative** offset (e.g., `-2.5`)
5. If wheels point left, use **positive** offset (e.g., `+2.5`)
6. Iterate until 0° command produces straight-ahead alignment

**Example scenarios:**
- Wheels point 3° to the right when commanded 0° → Use `MOTOR_STEERING_OFFSET=-3.0`
- Wheels point 2° to the left when commanded 0° → Use `MOTOR_STEERING_OFFSET=2.0`

#### Drive Direction Reversal
```bash
MOTOR_REVERSE_DRIVE=False      # Set to True if motor is backwards
```

**When to use:**
- Motor shaft points towards front of robot → `False`
- Motor shaft points towards rear of robot → `True`
- Positive velocity should move robot forward

**Quick test:**
```bash
# Publish a forward command
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.5, steering_angle: 0.0}}"

# If robot moves backward, set MOTOR_REVERSE_DRIVE=True
```

#### Safety Limits
```bash
MOTOR_MAX_SPEED=50              # Maximum speed (0-100)
MOTOR_MAX_STEERING_ANGLE=45.0   # Maximum steering (degrees)
```

**Tuning guidance:**
- Start with conservative limits (e.g., 30-40 speed)
- Increase gradually during testing
- WRO competition: 60-80 speed typical
- Never exceed mechanical steering limits

#### Velocity Scaling
```bash
MOTOR_SPEED_SCALE=30.0          # m/s to motor percentage
```

**How it works:**
```
motor_speed = velocity (m/s) × MOTOR_SPEED_SCALE
```

**Calibration procedure:**
1. Measure wheel diameter: `D` (meters)
2. Measure gear ratio: `R` (output/input)
3. Desired max velocity: `V_max` (m/s)
4. Target motor speed: `S_target` (0-100)
5. Calculate: `MOTOR_SPEED_SCALE = S_target / V_max`

**Example:**
- Wheel diameter: 5.6 cm = 0.056 m
- Gear ratio: 1:1
- Want 1.0 m/s → motor speed 30
- **MOTOR_SPEED_SCALE = 30.0**

## ROS2 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Ackermann drive commands from navigation |

**Message structure:**
```yaml
drive:
  speed: 1.0           # Linear velocity in m/s
  steering_angle: 0.5  # Steering angle in radians (+ = left, - = right)
```

### Published Topics

| Topic | Type | Description | Rate |
|-------|------|-------------|------|
| `/motor/steering_position` | `std_msgs/Float32` | Current steering position (degrees) | 20 Hz |
| `/motor/drive_speed` | `std_msgs/Float32` | Current drive speed (degrees/s) | 20 Hz |
| `/motor/status` | `diagnostic_msgs/DiagnosticStatus` | Motor diagnostics & health | 20 Hz |

**Status message includes:**
- Steering position (actual)
- Drive speed (actual)
- Commanded speed
- Commanded steering
- Steering offset setting
- Drive reversal setting

## Usage

### Launch with Full System
```bash
# Included in main launch file
ros2 launch klevor_robot wro_state_machine_launch.py
```

### Launch Standalone (for testing)
```bash
# On Pi Zero with Build HAT
ros2 run klevor_robot ackermann_motor_node
```

### Test Motor Control

#### Test Steering
```bash
# Center steering
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"

# Turn left 30°
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.524}}"  # 30° in radians

# Turn right 30°
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: -0.524}}"  # -30° in radians
```

#### Test Drive
```bash
# Drive forward slowly
ros2 topic pub --rate 10 /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.5, steering_angle: 0.0}}"

# Stop (Ctrl+C to stop publishing, watchdog will stop motors)

# Drive in reverse
ros2 topic pub --rate 10 /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: -0.5, steering_angle: 0.0}}"
```

#### Combined Test (drive in circle)
```bash
ros2 topic pub --rate 10 /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.8, steering_angle: 0.785}}"  # 45° left at 0.8 m/s
```

### Monitor Motor Feedback
```bash
# Watch steering position
ros2 topic echo /motor/steering_position

# Watch drive speed
ros2 topic echo /motor/drive_speed

# Watch diagnostics
ros2 topic echo /motor/status
```

## Safety Features

### 1. Watchdog Timer
- **Function**: Stops motors if no command received for 1 second
- **Purpose**: Prevents runaway robot if network drops
- **Behavior**: Automatically calls `stop_drive()` and logs warning

### 2. Speed Clamping
- **Function**: Limits motor speed to `±MOTOR_MAX_SPEED`
- **Purpose**: Prevents mechanical damage and battery drain
- **Behavior**: Silently clamps, logs warning if exceeded

### 3. Steering Clamping
- **Function**: Limits steering to `±MOTOR_MAX_STEERING_ANGLE`
- **Purpose**: Prevents mechanical damage to steering mechanism
- **Behavior**: Clamps to limit, logs warning if exceeded

### 4. Graceful Shutdown
- **Function**: Stops motors and centers steering on node exit
- **Purpose**: Leaves robot in safe, predictable state
- **Behavior**: Triggered by Ctrl+C or node crash

## Calibration Guide

### Step 1: Find Mechanical Center
```bash
# 1. Manually center wheels physically
# 2. Run node
ros2 run klevor_robot ackermann_motor_node

# 3. Send 0° command
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"

# 4. Observe wheel position
# 5. Adjust MOTOR_STEERING_OFFSET in .env
```

### Step 2: Test Drive Direction
```bash
# 1. Send forward command
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.5, steering_angle: 0.0}}"

# 2. If robot moves backward, set MOTOR_REVERSE_DRIVE=True in .env
# 3. Restart node and retest
```

### Step 3: Calibrate Speed Scaling
```bash
# 1. Mark a 1-meter distance on floor
# 2. Send 1.0 m/s command for 10 seconds
ros2 topic pub --rate 10 /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}"

# 3. Measure actual distance traveled
# 4. If robot travels:
#    - Too fast: decrease MOTOR_SPEED_SCALE
#    - Too slow: increase MOTOR_SPEED_SCALE
# 5. Formula: new_scale = current_scale × (desired_distance / actual_distance)
```

### Step 4: Find Maximum Safe Steering
```bash
# 1. Gradually increase steering angle
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.785}}"  # 45° = 0.785 rad

# 2. Watch for mechanical binding or stress
# 3. Note maximum comfortable angle
# 4. Set MOTOR_MAX_STEERING_ANGLE to 80% of maximum
#    (e.g., if max is 50°, use 40°)
```

## Troubleshooting

### Problem: Steering drifts to one side
**Solution:** Adjust `MOTOR_STEERING_OFFSET`
```bash
# If drifting right: MOTOR_STEERING_OFFSET=-2.0
# If drifting left: MOTOR_STEERING_OFFSET=2.0
```

### Problem: Robot drives backward when commanded forward
**Solution:** Enable drive reversal
```bash
MOTOR_REVERSE_DRIVE=True
```

### Problem: Robot moves too fast/slow
**Solution:** Adjust `MOTOR_SPEED_SCALE`
```bash
# Too fast: lower value (e.g., 25.0)
# Too slow: higher value (e.g., 35.0)
```

### Problem: Motors stop unexpectedly
**Check watchdog:**
```bash
# Monitor command rate
ros2 topic hz /ackermann_cmd

# Should be >1 Hz to prevent watchdog timeout
```

### Problem: Steering hits mechanical limit
**Solution:** Reduce `MOTOR_MAX_STEERING_ANGLE`
```bash
# Start conservative
MOTOR_MAX_STEERING_ANGLE=35.0

# Test with extreme commands
ros2 topic pub --once /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 1.57}}"  # 90° - should clamp

# Check logs for clamping warnings
```

### Problem: Build HAT not detected
**Check connection:**
```bash
# Verify Build HAT is recognized
lsusb | grep -i "LEGO"

# Check motor ports
python3 -c "from buildhat import Motor; m = Motor('A'); print(m.get_aposition())"
```

## Performance Characteristics

- **Command latency**: <50ms (network + processing)
- **Feedback rate**: 20Hz (steering position, drive speed)
- **Watchdog timeout**: 1 second (configurable in code)
- **Steering speed**: 30 units (Build HAT scale, ~fast)
- **Control loop**: Event-driven (no polling overhead)

## Integration with State Machine

The motor controller integrates with the state machine:

**BOOT_CHECK state:**
- State machine verifies `/motor/status` topic is publishing
- Checks that motors are responsive

**READY → RACING transition:**
- Motors are already initialized and ready
- First `/ackermann_cmd` immediately controls robot

**RACING state:**
- Navigation node publishes continuous `/ackermann_cmd`
- Motor controller executes commands in real-time

**FINISHED / E-STOP:**
- State machine publishes `{speed: 0.0, steering_angle: 0.0}`
- Motor controller immediately stops drive and centers steering
- Watchdog ensures motors stay stopped

## Advanced Topics

### Custom Speed Curves
For non-linear speed response, modify `_ackermann_callback()`:
```python
# Square-root scaling for gentler acceleration
motor_speed = int(math.sqrt(abs(velocity)) * self.speed_scale * math.copysign(1, velocity))
```

### Steering Rate Limiting
To prevent jerky steering motions:
```python
# Limit steering change rate
max_steering_delta = 10.0  # degrees per command
delta = clamped_steering - self.last_steering
limited_delta = max(-max_steering_delta, min(max_steering_delta, delta))
clamped_steering = self.last_steering + limited_delta
```

### Odometry Publishing
Add wheel encoder integration for dead reckoning (future enhancement).

## Testing Checklist

Before competition:
- [ ] Steering centers correctly with 0° command
- [ ] Positive velocity drives forward
- [ ] Negative velocity drives reverse  
- [ ] Maximum steering does not bind
- [ ] Speed scaling produces expected velocity
- [ ] Watchdog stops motors when commands cease
- [ ] Node recovers gracefully from crashes (respawn)
- [ ] Feedback topics publish at 20Hz
- [ ] Network latency <100ms Pi Zero ↔ Pi 5

## API Reference

### Main Callback
```python
def _ackermann_callback(self, msg: AckermannDriveStamped) -> None:
    """Process Ackermann command and control motors."""
```

**Processing pipeline:**
1. Extract `velocity` (m/s) and `steering_angle` (rad)
2. Convert steering to degrees
3. Apply `steering_offset`
4. Clamp to `±max_steering_angle`
5. Convert velocity to motor speed via `speed_scale`
6. Apply `reverse_drive` if enabled
7. Clamp to `±max_speed`
8. Send to Build HAT driver

## License

WRO 2026 Future Engineers Competition - Team Klevor

## Support

For calibration assistance or motor control issues:
1. Check diagnostics: `ros2 topic echo /motor/status`
2. Verify configuration: `env | grep MOTOR_`
3. Test Build HAT directly: `python3 -c "from buildhat import Motor; ..."`
4. Review node logs: `ros2 node list` → `ros2 node info ackermann_motors`

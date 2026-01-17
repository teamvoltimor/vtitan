# Build HAT + LEGO Technic Motor Option Analysis

**NEW OPTION: Raspberry Pi Build HAT + LEGO Technic Large Motor (88013)**

---

## 🎯 Overview

This document analyzes using the **Raspberry Pi Build HAT** with **LEGO Technic Large Motor (88013)** as an alternative to your current INJORA motor setup and the LEGO SPIKE Prime motor option from the motor comparison document.

**Key Advantage:** Much simpler integration than custom LEGO SPIKE adapter board!

---

## 🔧 Hardware Specifications

### Raspberry Pi Build HAT

**Connectivity:**
- Mounts directly on Raspberry Pi 40-pin GPIO header
- Compatible with all Raspberry Pi models with 40-pin header (including your Pi 5!)
- 4× LPF2 connectors for LEGO Powered Up motors/sensors
- Onboard RP2040 microcontroller handles low-level motor control

**Power:**
- Requires separate 8V ±10% DC power supply (2.1mm barrel jack, center positive)
- Can use 7.5V battery pack OR official Build HAT Power Supply
- Powers motors independently from Raspberry Pi

**Supported Devices:**
- All LEGO Technic devices from SPIKE Portfolio
- LEGO Mindstorms Robot Inventor devices
- All LEGO Powered Up (LPF2) compatible motors/sensors

**Software:**
- Official Python library: `buildhat`
- Easy installation: `pip install buildhat`
- Well-documented API
- Works alongside other Pi peripherals (camera, etc.)

**Cost:** ~$25-35

---

### LEGO Technic Large Motor (88013)

**Official Specifications** (from LEGO Education):

| Condition | Speed | Torque | Current |
|-----------|-------|--------|---------|
| **No-Load** | 175 RPM ± 15% | 0 Ncm | 135 mA ± 15% |
| **Running Load** | 135 RPM ± 15% | 8 Ncm | 430 mA ± 15% |
| **Stall** | 0 RPM | 25 Ncm | 1400 mA ± 15% |

*All specs at 7.2V power supply*

**Encoder Specifications:**
- Resolution: 360 counts per revolution
- Accuracy: ≤ ±3 degrees
- Update Rate: 100 Hz
- Type: Absolute encoder (within one rotation)

**Features:**
- Integrated rotation sensor
- Absolute positioning for accurate speed/position control
- LEGO Technic mounting holes (easy integration)
- Includes connecting wire with LPF2 connector

**Cost:** ~$20-30

**Total System Cost:** ~$45-65 (Build HAT + Motor)

---

## 📊 Comparison with Other Options

### vs. Your Current INJORA 180 Motor

| Feature | INJORA 180 | Build HAT + LEGO 88013 |
|---------|------------|------------------------|
| **Encoder** | ❌ None | ✅ 360 CPR, absolute |
| **Max Speed** | 20,500 RPM (no load) | 175 RPM (no load) |
| **Torque** | Unknown | 25 Ncm stall |
| **Position Control** | ❌ Time-based only | ✅ Absolute position |
| **Integration Effort** | ✅ Already done | ⚠️ Need Build HAT |
| **Cost** | $0 (have it) | $45-65 |
| **ROS2 Odometry** | ❌ No encoder | ✅ Perfect |
| **Mounting** | Custom | LEGO Technic |

**Note:** INJORA is MUCH faster but lacks encoder feedback!

---

### vs. LEGO SPIKE Prime Motor (from doc 06)

| Feature | SPIKE Prime (Custom) | Build HAT + 88013 |
|---------|---------------------|-------------------|
| **Motor** | LEGO Large Angular Motor | LEGO 88013 (same specs!) |
| **Speed** | 175 RPM | 175 RPM |
| **Torque** | 25 Ncm | 25 Ncm |
| **Encoder** | 360 CPR | 360 CPR |
| **Integration** | ❌ Custom adapter board needed | ✅ Build HAT (plug & play) |
| **Wiring** | Complex (level shifters, H-bridge) | ✅ Simple (LPF2 cable) |
| **Software** | Custom driver on Pico 2W | ✅ Python library ready |
| **Cost** | $30-40 + adapter (~$10) = $40-50 | $45-65 |
| **Development Time** | High (design adapter PCB) | Low (ready to use) |

**Verdict:** Build HAT is MUCH easier to integrate!

---

### vs. INJORA + PMW3901 Optical Flow (Recommended in doc 06)

| Feature | INJORA + PMW3901 | Build HAT + 88013 |
|---------|------------------|-------------------|
| **Odometry Type** | Optical flow (ground) | Wheel encoder |
| **Slip Immunity** | ✅ Yes (optical) | ❌ Wheel slip possible |
| **Innovation Factor** | ⭐⭐⭐ High | ⭐⭐ Medium |
| **Surface Dependency** | ⚠️ Needs texture | ✅ Any surface |
| **Cost** | $30 (PMW3901) | $45-65 |
| **Motor Speed** | 20,500 RPM | 175 RPM |
| **Integration** | Moderate (SPI sensor) | Moderate (Build HAT) |
| **ROS2 Compatibility** | ✅ Works well | ✅ Works well |

**Trade-off:** Optical flow is more innovative and slip-immune, but Build HAT gives traditional encoder odometry.

---

## 🏗️ Integration with Your Setup

### Current Setup (Klevor)
- Raspberry Pi 5 (16GB)
- Raspberry Pi Pico 2 WH (motor controller)
- USB-CDC communication (Pico ↔ Pi)
- INJORA motor + ESC

### With Build HAT Integration

**Option A: Replace Pico 2W for Motors (Simplest)**
```
┌─────────────────────────────────────────┐
│ Raspberry Pi 5                          │
│ ┌─────────────────────────────────────┐ │
│ │ ROS2 Nodes (Python)                 │ │
│ │ - Vision (Camera Module 3)          │ │
│ │ - YOLO on AI HAT+                   │ │
│ │ - LiDAR (USB)                       │ │
│ │ - IMU (I2C)                         │ │
│ │ - Motor Control via Build HAT       │ │
│ └─────────────────────────────────────┘ │
│           │                              │
│           ├─ GPIO Header                 │
│           │                              │
│   ┌───────▼──────────┐                  │
│   │ Build HAT        │                  │
│   │ (RP2040 onboard) │                  │
│   └──────────────────┘                  │
│    │    │    │    │                     │
└────┼────┼────┼────┼─────────────────────┘
     │    │    │    │
   Port1 Port2 Port3 Port4
     │    │    │    │
  LEGO  LEGO Optional Optional
  Motor Servo Sensor  Sensor
  88013
```

**Advantages:**
- ✅ Simplifies architecture (one less microcontroller)
- ✅ Direct Python control from Pi 5
- ✅ No USB-CDC protocol needed
- ✅ Build HAT handles low-level motor control

**Trade-offs:**
- ⚠️ Can't reuse your INJORA motor (Build HAT only for LEGO)
- ⚠️ Your Pico 2W becomes unused (or repurpose for other tasks)

---

**Option B: Keep Pico 2W + Add Build HAT (Hybrid)**
```
┌─────────────────────────────────────────┐
│ Raspberry Pi 5                          │
│ - Build HAT for LEGO motors             │
│ - USB-CDC to Pico 2W for INJORA         │
└─────────────────────────────────────────┘
       │                    │
   Build HAT             USB-CDC
       │                    │
   LEGO 88013          Pico 2W
                           │
                      INJORA Motor
```

**Advantages:**
- ✅ Can use BOTH motor systems
- ✅ Flexibility for testing

**Disadvantages:**
- ❌ Overly complex
- ❌ Not recommended

---

## 💻 Software Integration (Option A - Recommended)

### Python Code Example with Build HAT

```python
#!/usr/bin/env python3
from buildhat import Motor
import time

# Initialize LEGO motor on port A
drive_motor = Motor('A')

# --- Basic Control ---

# Set speed (-100 to 100)
drive_motor.start(50)  # 50% speed forward
time.sleep(2)
drive_motor.stop()

# --- Position Control (Odometry!) ---

# Get current position (encoder counts)
current_pos = drive_motor.get_aposition()
print(f"Position: {current_pos} degrees")

# Move to absolute position
drive_motor.run_to_position(360, 50)  # Go to 360°, 50% speed
drive_motor.wait_for_arrival()

# Move relative distance
drive_motor.run_for_degrees(180, 50)  # Rotate 180° forward
drive_motor.wait_for_arrival()

# --- Speed Control with PID ---
drive_motor.set_default_speed(50)
drive_motor.start()

# --- Get telemetry ---
position = drive_motor.get_aposition()  # Absolute position
speed = drive_motor.get_speed()  # Current speed
```

### ROS2 Integration Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from buildhat import Motor
import math

class BuildHATMotorNode(Node):
    def __init__(self):
        super().__init__('buildhat_motor_node')

        # Initialize LEGO motors
        self.left_motor = Motor('A')
        self.right_motor = Motor('B')

        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ROS2 publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Odometry timer (100 Hz - matches encoder update rate)
        self.create_timer(0.01, self.publish_odometry)

        # Robot parameters
        self.wheel_diameter = 0.056  # meters (56mm LEGO wheel)
        self.wheel_base = 0.15  # meters (distance between wheels)

        # Previous encoder positions
        self.prev_left_pos = 0
        self.prev_right_pos = 0

        self.get_logger().info('Build HAT Motor Node started')

    def cmd_vel_callback(self, msg):
        """Convert Twist to differential drive commands"""
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s

        # Differential drive kinematics
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2)

        # Convert m/s to motor speed (-100 to 100)
        left_speed = self.velocity_to_motor_speed(left_vel)
        right_speed = self.velocity_to_motor_speed(right_vel)

        # Command motors
        self.left_motor.start(left_speed)
        self.right_motor.start(right_speed)

    def velocity_to_motor_speed(self, vel):
        """Convert m/s to motor speed percentage"""
        # LEGO motor: 175 RPM @ 100% = 2.92 rev/s
        # Wheel circumference = π * diameter
        max_vel = 2.92 * math.pi * self.wheel_diameter  # ~0.51 m/s

        motor_speed = (vel / max_vel) * 100
        return max(-100, min(100, motor_speed))  # Clamp to [-100, 100]

    def publish_odometry(self):
        """Publish odometry from encoder data"""
        # Get current encoder positions (degrees)
        left_pos = self.left_motor.get_aposition()
        right_pos = self.right_motor.get_aposition()

        # Calculate change in position
        delta_left = math.radians(left_pos - self.prev_left_pos)
        delta_right = math.radians(right_pos - self.prev_right_pos)

        # Convert to linear distance
        left_dist = delta_left * (self.wheel_diameter / 2)
        right_dist = delta_right * (self.wheel_diameter / 2)

        # Calculate robot motion
        delta_dist = (left_dist + right_dist) / 2
        delta_theta = (right_dist - left_dist) / self.wheel_base

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Velocity (distance / dt)
        odom.twist.twist.linear.x = delta_dist / 0.01
        odom.twist.twist.angular.z = delta_theta / 0.01

        self.odom_pub.publish(odom)

        # Update previous positions
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

def main(args=None):
    rclpy.init(args=args)
    node = BuildHATMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## ⚡ Power Considerations

### Power Requirements

**Build HAT:**
- Input: 8V ±10% DC (7.2V - 8.8V)
- Recommended: 7.5V battery pack (6× AA or 2S LiPo)
- Current: Depends on motors (up to 1.4A per motor at stall)
- Max total: ~5-6A (4 motors stalled)

**Your Current Battery:**
- URGENEX 7.4V Battery (3000 mAh, 2S LiPo)
- ✅ **PERFECT for Build HAT!** (7.4V is within 8V ±10% range)

**System Power Architecture:**

```
┌─────────────────────┐
│ Shargeek Storm 2    │ 25600 mAh, 100W
│ (Main battery)      │
└──────┬──────────────┘
       │
       ├─ USB-C → Raspberry Pi 5 (5V, 5A)
       │          ├─ Camera Module 3
       │          ├─ AI HAT+ 26 TOPS
       │          ├─ Build HAT (via GPIO)
       │          │
       │          └─ USB peripherals
       │              ├─ RPLiDAR C1
       │              └─ Other sensors
       │
       └─ (Optional) Can charge URGENEX battery

┌─────────────────────┐
│ URGENEX 7.4V        │ 3000 mAh, 2S LiPo
│ (Motor power)       │
└──────┬──────────────┘
       │
       └─ Build HAT Power Input (2.1mm barrel jack)
          └─ LEGO Motors (via LPF2 ports)
```

**Advantages:**
- ✅ Can reuse your URGENEX 7.4V battery for Build HAT!
- ✅ Clean power separation (logic vs motors)
- ✅ No additional battery needed

---

## 🎯 Recommendation for Your Setup

### For Proposal 4 (ROS2 Edge Racer)

**Choose Build HAT + LEGO 88013 if:**
- ✅ You want the **easiest integration** with encoders
- ✅ You want **professional, tested hardware** (LEGO Education)
- ✅ You prioritize **development speed** (Python library ready)
- ✅ You can afford **$45-65** for the combo
- ✅ You don't mind **slower max speed** (175 RPM vs 20,500 RPM)
- ✅ You want to **simplify** your setup (no Pico 2W needed)

**Choose INJORA + PMW3901 Optical Flow if:**
- ✅ You want **maximum innovation** (unique approach)
- ✅ You want **slip-immune odometry**
- ✅ You need **high speed** (20,500 RPM)
- ✅ You want to **save money** ($30 vs $45-65)
- ✅ You're OK with **more integration work** (SPI sensor + calibration)
- ✅ You want to **reuse existing motor**

**Choose Keep INJORA Only if:**
- ✅ You want **zero new cost**
- ✅ You accept **IMU/LiDAR-only odometry**
- ✅ You need **high speed** capability

---

## 🏆 Final Verdict

### Build HAT + LEGO 88013: **★★★★☆ (4/5 stars)**

**Pros:**
- ✅ Easiest encoder integration (no custom PCB!)
- ✅ Professional hardware (LEGO Education standard)
- ✅ Official Python library (well-documented)
- ✅ Perfect for ROS2 Nav2 (encoder odometry)
- ✅ Can reuse URGENEX 7.4V battery
- ✅ Absolute encoder (±3° accuracy)
- ✅ LEGO mounting (easy mechanical integration)
- ✅ Simplifies architecture (no Pico 2W needed)

**Cons:**
- ❌ More expensive than optical flow ($45-65 vs $30)
- ❌ Much slower than INJORA (175 RPM vs 20,500 RPM)
- ❌ Can't reuse existing INJORA motor
- ❌ Less innovative than optical flow approach
- ⚠️ Your Pico 2W becomes unused (unless repurposed)

**Best Use Case:**
- **Proposal 4 (ROS2 Edge Racer)** - if you prioritize development speed and proven hardware
- Great for beginners with LEGO integration
- Perfect if WRO track doesn't require high speed

---

## 🤔 Speed Consideration: Is 175 RPM Enough?

**CRITICAL QUESTION:** Is LEGO motor too slow for WRO?

**LEGO 88013 with typical wheel:**
- Motor: 175 RPM no-load, 135 RPM loaded
- Wheel: 56mm diameter LEGO wheel
- Speed calculation:
  ```
  Circumference = π × 0.056m = 0.176m
  Speed @ 135 RPM = 135 × 0.176m = 23.7 m/min = 0.40 m/s
  Speed @ 175 RPM (no-load) = 175 × 0.176m = 30.8 m/min = 0.51 m/s
  ```

**INJORA 180 with gearing:**
- Motor: 20,500 RPM no-load
- Gear ratio: 48:1
- Output: 427 RPM
- With 56mm wheel: 7.5 m/min = **1.25 m/s** (3× faster!)

**For WRO Racing:**
- ⚠️ LEGO motor (0.4 m/s) might be **too slow** for competitive times
- ✅ INJORA (1.25 m/s) is **much better for racing**

**Recommendation:**
- If WRO requires **speed**: Stick with INJORA + PMW3901 optical flow
- If WRO prioritizes **precision**: Build HAT + LEGO 88013 works

---

## 📋 Updated Cost Summary

| Option | Hardware Cost | Integration Effort | Speed | Encoder | Innovation |
|--------|--------------|-------------------|-------|---------|------------|
| **Keep INJORA** | $0 | ✅ Done | ⭐⭐⭐ | ❌ | ⭐ |
| **INJORA + PMW3901** | $30 | Medium | ⭐⭐⭐ | ✅ Optical | ⭐⭐⭐ |
| **Build HAT + 88013** | $45-65 | Medium | ⭐ | ✅ Wheel | ⭐⭐ |
| **LEGO SPIKE (custom)** | $40-50 | High | ⭐ | ✅ Wheel | ⭐⭐ |

---

## 🚀 Implementation Timeline

### If Choosing Build HAT + LEGO 88013:

**Week 1:**
- [ ] Order Raspberry Pi Build HAT (~$25-35)
- [ ] Order LEGO Technic Large Motor 88013 (~$20-30)
- [ ] Optional: Order LEGO wheels/mounting parts
- [ ] Plan mechanical integration (LEGO chassis?)

**Week 2:**
- [ ] Install Build HAT on Pi 5
- [ ] Connect URGENEX 7.4V battery to Build HAT power
- [ ] Install Python library: `pip install buildhat`
- [ ] Test motor control (basic speed/position)

**Week 3:**
- [ ] Implement ROS2 motor node (cmd_vel subscriber)
- [ ] Implement ROS2 odometry publisher
- [ ] Test with Nav2 (basic movement commands)

**Week 4:**
- [ ] Integrate with YOLO vision system
- [ ] Test complete system
- [ ] Tune PID controllers

**Total:** 4 weeks to full integration

---

## 📚 Resources & Documentation

### Official Documentation:
- [Raspberry Pi Build HAT Documentation](https://www.raspberrypi.com/documentation/accessories/build-hat.html)
- [Build HAT Python Library](https://buildhat.readthedocs.io/en/latest/buildhat/index.html)
- [Build HAT GitHub Repository](https://github.com/RaspberryPiFoundation/python-build-hat)
- [LEGO Education SPIKE Prime Technical Specifications](https://le-www-live-s.legocdn.com/sc/media/files/support/spike-prime/techspecs_techniclargeangularmotor-1b79e2f4fbb292aaf40c97fec0c31fff.pdf)

### Community Resources:
- [Meet the Raspberry Pi Build HAT (Raspberry Pi News)](https://www.raspberrypi.com/news/raspberry-pi-build-hat-lego-education/)
- [Raspberry Pi Build HAT Review (Tom's Hardware)](https://www.tomshardware.com/reviews/raspberry-pi-build-hat-review-combine-lego-with-pi)
- [PyBricks Motor Documentation](https://docs.pybricks.com/en/stable/pupdevices/motor.html)

### Purchase Links:
- [Raspberry Pi Build HAT - Official Store](https://www.raspberrypi.com/products/build-hat/)
- [Adafruit Build HAT](https://www.adafruit.com/product/5287)
- [LEGO Technic Large Motor 88013](https://www.lego.com/en-us/product/technic-large-motor-88013)

---

## ✅ Summary

**Build HAT + LEGO 88013 is EXCELLENT if:**
- Development speed > Cost savings
- Precision > Raw speed
- Proven hardware > Innovation points
- You want simplest encoder integration

**Stick with INJORA + PMW3901 if:**
- Speed is critical for WRO
- Budget is tight ($30 vs $45-65)
- You want innovation points
- Slip-immune odometry matters

**My recommendation:**
1. **First choice:** INJORA + PMW3901 (better speed, innovation, cost)
2. **Second choice:** Build HAT + LEGO 88013 (easier integration, if speed OK)

**Test this:** Check WRO track dimensions and calculate required speed. If 0.4 m/s is sufficient, Build HAT is great. If you need 1+ m/s, stick with INJORA.

---

**Last updated:** 2026-01-16

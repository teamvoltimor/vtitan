# Dual LEGO Motor Setup with Build HAT

**Using Your Bugatti Bolide Technic Parts + Build HAT + 2 LEGO Motors**

---

## 🎯 Your Situation

**What You Have:**
- ✅ LEGO Technic Bugatti Bolide #42151 (905 parts: gears, beams, axles, connectors)
- ✅ Raspberry Pi 5 (16GB)
- ✅ AI HAT+ 26 TOPS
- ✅ All sensors (Camera, LiDAR, IMU)
- ✅ INJORA 180 Motor + Servo (current setup)

**What You're Considering:**
- Buy 2× LEGO Technic Large Motors (88013)
- Buy 1× Raspberry Pi Build HAT
- Use LEGO for BOTH drive and steering
- Build chassis from Bugatti Bolide parts

---

## 🔧 Key Finding: LEGO Motors CAN Do Steering!

**Modern LEGO Powered Up motors have built-in encoders and can do position control!**

From research:
> "As a result of the L and XL motors being able to calibrate to become steering motors, there is no dedicated servo motor, as there is no need for one in the newer Powered Up and Control+ systems."

**This means:**
- ✅ LEGO Large/Medium motors CAN control steering (with encoders!)
- ✅ No need for separate servo motor
- ✅ All-LEGO solution is viable!

---

## 🚗 Two Steering Approaches

### Option A: Differential Steering (Tank/Skid Steering) ⭐ **RECOMMENDED**

**Setup:**
- 2× LEGO Large Motors (88013)
- Left motor drives left wheel(s)
- Right motor drives right wheel(s)
- Steering by running motors at different speeds

**How It Works:**
```
Turn Left:  Left motor slow, Right motor fast
Turn Right: Left motor fast, Right motor slow
Forward:    Both motors same speed forward
Backward:   Both motors same speed backward
Spin:       Left forward, Right backward (or vice versa)
```

**Advantages:**
- ✅ **Simplest setup** (only 2 motors needed!)
- ✅ **No steering mechanism** (no Ackermann linkage)
- ✅ **Zero turning radius** (can spin in place)
- ✅ **Perfect for WRO** (tight spaces, precise positioning)
- ✅ **Easier to build** with LEGO parts
- ✅ **More reliable** (fewer moving parts)
- ✅ **Cheaper** (2 motors vs 3)

**Disadvantages:**
- ⚠️ **Wheel slip** when turning (skids sideways)
- ⚠️ **Not car-like** (but WRO doesn't require car steering)
- ⚠️ **Depends on friction** (works best on rubber mats)

**Cost:**
- 2× LEGO Large Motor 88013: $40-60
- 1× Build HAT: $25-35
- **Total: $65-95**

**ROS2 Integration:**
```python
# Differential drive is BUILT-IN to ROS2!
from buildhat import Motor

left_motor = Motor('A')
right_motor = Motor('B')

def cmd_vel_callback(msg):
    """ROS2 Twist message → Differential drive"""
    linear = msg.linear.x   # Forward/backward speed
    angular = msg.angular.z # Turning rate

    # Differential drive equations
    left_speed = linear - angular * wheel_base / 2
    right_speed = linear + angular * wheel_base / 2

    left_motor.start(left_speed * 100)
    right_motor.start(right_speed * 100)
```

**ROS2 Support:**
- ✅ `diff_drive_controller` package (standard!)
- ✅ Nav2 fully supports differential drive
- ✅ Most tutorials use differential drive
- ✅ Easiest to tune

---

### Option B: Ackermann Steering (Car-Like Steering)

**Setup:**
- 1× LEGO Large Motor (88013) for drive (rear wheels)
- 1× LEGO Medium Motor (45603) for steering (front wheels)
- Ackermann linkage (LEGO gears/beams)

**How It Works:**
```
Drive motor:    Powers rear wheels
Steering motor: Turns front wheels via Ackermann linkage
                (like a real car)
```

**Advantages:**
- ✅ **Car-like steering** (more realistic)
- ✅ **Less wheel slip** (front wheels pivot)
- ✅ **Works on any surface** (less friction dependent)

**Disadvantages:**
- ❌ **More complex** (need Ackermann linkage)
- ❌ **Harder to build** (steering geometry critical)
- ❌ **Needs 3 motors** (2 drive + 1 steer, or 1 drive + 1 steer)
- ❌ **Larger turning radius** (can't spin in place)
- ❌ **More tuning** needed (steering calibration)
- ⚠️ **Overkill for WRO** (differential is sufficient)

**Cost:**
- 1× LEGO Large Motor 88013: $20-30 (drive)
- 1× LEGO Medium Motor 45603: $15-25 (steering)
- 1× Build HAT: $25-35
- **Total: $60-90**

**ROS2 Integration:**
```python
# Need Ackermann controller
from buildhat import Motor

drive_motor = Motor('A')
steer_motor = Motor('B')

def cmd_vel_callback(msg):
    """ROS2 Twist → Ackermann steering"""
    linear = msg.linear.x
    angular = msg.angular.z

    # Calculate steering angle from angular velocity
    steering_angle = atan(angular * wheelbase / linear)

    # Position control for steering
    steer_motor.run_to_position(steering_angle * 180/pi, 50)

    # Speed control for drive
    drive_motor.start(linear * 100)
```

**ROS2 Support:**
- ⚠️ `ackermann_steering_controller` available
- ⚠️ More complex tuning
- ⚠️ Fewer tutorials
- ⚠️ Not necessary for WRO

---

## 📊 Comparison: Differential vs Ackermann

| Factor | Differential (2 motors) | Ackermann (2-3 motors) |
|--------|------------------------|------------------------|
| **Motors Needed** | 2 (drive only) ⭐ | 2-3 (drive + steer) |
| **LEGO Build Complexity** | ⭐⭐⭐⭐⭐ Easy | ⭐⭐ Complex |
| **Turning Radius** | Zero (spin in place) ⭐ | Larger |
| **Wheel Slip** | More (skids) | Less |
| **ROS2 Support** | ⭐⭐⭐⭐⭐ Excellent | ⭐⭐⭐ Good |
| **WRO Suitability** | ⭐⭐⭐⭐⭐ Perfect | ⭐⭐⭐ Adequate |
| **Cost** | $65-95 | $60-90 (similar) |
| **Setup Time** | Fast | Slow |
| **Reliability** | ⭐⭐⭐⭐⭐ High | ⭐⭐⭐ Medium |

**Recommendation:** **Differential steering** for WRO!

---

## 🏆 Full System Options

### Option 1: Full LEGO System (Differential) ⭐⭐⭐⭐⭐ **BEST CHOICE**

**Hardware:**
- 2× LEGO Large Motor 88013 ($40-60)
- 1× Raspberry Pi Build HAT ($25-35)
- 1× TCS34725 color sensor ($8)
- Chassis from Bugatti Bolide parts (FREE!)
- **Total: $73-103**

**What You Get:**
- ✅ **All-LEGO solution** (proven, reliable)
- ✅ **Simplest setup** (differential steering)
- ✅ **Plug & play** (Build HAT Python library)
- ✅ **Perfect for WRO** (tight spaces, zero turning radius)
- ✅ **Encoder odometry** (360 CPR per motor)
- ✅ **ROS2 compatible** (diff_drive_controller)
- ✅ **No Pico 2W needed** (eliminates complexity)
- ✅ **Easy mechanical integration** (LEGO mounting)

**What You Lose:**
- ❌ Speed (0.4 m/s max vs 1.25 m/s with INJORA)
- ❌ Optical flow innovation (using wheel encoders instead)

**Speed Calculation:**
```
2 motors in differential drive:
- Each motor: 135 RPM loaded
- Wheel: 56mm diameter (typical LEGO)
- Speed: 135 RPM × π × 0.056m = 0.40 m/s per motor

With differential drive, max forward speed = 0.40 m/s
(Both motors forward at same speed)
```

**Is 0.4 m/s enough for WRO?**
- ⚠️ Slower than INJORA setup (1.25 m/s)
- ✅ BUT: Tight turns, zero turning radius compensates
- ✅ WRO tracks often prioritize precision over raw speed
- ✅ Can navigate tight spaces better than Ackermann

---

### Option 2: Hybrid - INJORA Drive + LEGO Steering

**Hardware:**
- Keep INJORA 180 motor for drive (via Pico 2W)
- 1× LEGO Medium Motor 45603 for steering ($15-25)
- 1× Raspberry Pi Build HAT ($25-35)
- 1× PMW3901 optical flow ($30)
- 1× TCS34725 color sensor ($8)
- **Total: $78-98**

**What You Get:**
- ✅ **Fast drive** (1.25 m/s with INJORA)
- ✅ **LEGO steering** (position control, no servo calibration)
- ✅ **Optical flow** innovation
- ✅ **Best of both worlds**

**What You Lose:**
- ⚠️ More complex (Pico 2W + Build HAT)
- ⚠️ Need Ackermann steering mechanism
- ⚠️ More wiring/integration work

---

### Option 3: Keep Current Setup (INJORA + Servo)

**Hardware:**
- Keep INJORA 180 motor + servo
- 1× PMW3901 optical flow ($30)
- 1× TCS34725 color sensor ($8)
- **Total: $38**

**What You Get:**
- ✅ **Cheapest** option
- ✅ **Fastest** (1.25 m/s)
- ✅ **Already working** (mechanical integration done)
- ✅ **Optical flow** innovation

**What You Lose:**
- ❌ No wheel encoder odometry
- ❌ Keep Pico 2W complexity

---

## 🎯 Recommendation Based on Your Requirements

### **Go with Option 1: Full LEGO Differential System!** ⭐

**Why:**

1. **✅ Configuration Flexibility (Your #1 Requirement!)**
   - Build HAT + ROS2 = JSON/YAML config
   - Differential drive = simplest ROS2 integration
   - Change mission logic in seconds
   - Perfect for surprise rules

2. **✅ Uses Your Bugatti Bolide Parts**
   - 905 LEGO Technic parts ready to use!
   - No need to design custom chassis
   - LEGO mounting = easy assembly/modification
   - Professional look

3. **✅ Simplest System**
   - Only 2 motors + Build HAT
   - No Pico 2W (eliminates complexity)
   - Proven LEGO Education hardware
   - Plug & play Python library

4. **✅ Perfect for WRO**
   - Zero turning radius (spin in place!)
   - Precise positioning
   - Reliable mechanical design
   - Most robots use differential drive

5. **✅ Best Encoder Odometry**
   - 2× encoders (one per motor)
   - Absolute position tracking
   - ROS2 diff_drive_controller built-in
   - Easy EKF fusion with IMU

6. **✅ Proven in Education**
   - LEGO Education standard
   - Used in FLL, WRO, educational robotics
   - Extensive community support
   - Many examples to learn from

**Trade-off:**
- ⚠️ Slower (0.4 m/s vs 1.25 m/s)
- ✅ BUT: Tight turning compensates
- ✅ And WRO often values precision > speed

---

## 🏗️ Build HAT Architecture (Option 1)

```
┌─────────────────────────────────────────────────┐
│ Raspberry Pi 5 (16GB)                           │
│ ┌─────────────────────────────────────────────┐ │
│ │ ROS2 Humble                                 │ │
│ │ - YOLO vision (AI HAT+)                     │ │
│ │ - Navigation (Nav2)                         │ │
│ │ - diff_drive_controller                     │ │
│ │ - Mission config (YAML)                     │ │
│ └─────────────────────────────────────────────┘ │
│              │                                   │
│              ├─ GPIO Header                      │
│              │                                   │
│   ┌──────────▼────────────────┐                 │
│   │ Raspberry Pi Build HAT    │                 │
│   │ (RP2040 onboard)          │                 │
│   └───────────────────────────┘                 │
│    │      │      │      │                       │
└────┼──────┼──────┼──────┼───────────────────────┘
     │      │      │      │
   Port A Port B Port C Port D
     │      │      │      │
  LEGO   LEGO   Color  Distance
  Motor  Motor  Sensor Sensor
  Left   Right  (opt)  (opt)
```

**Clean Architecture:**
- Everything connects to Pi 5
- One power supply for Build HAT (URGENEX 7.4V)
- One power supply for Pi 5 (Shargeek)
- Simple wiring, reliable

---

## 📐 LEGO Chassis Design from Bugatti Bolide

**You Already Have:**
- ✅ LEGO beams (structure)
- ✅ LEGO axles (wheel mounting)
- ✅ LEGO gears (power transmission)
- ✅ LEGO connectors (assembly)
- ✅ 905 parts total!

**Simple Differential Chassis Design:**

```
Top View:
┌─────────────────────────────────┐
│  [Camera Module 3]              │
│                                 │
│  [Raspberry Pi 5]               │
│  [AI HAT+]                      │
│                                 │
│  [RPLiDAR C1]                   │
│                                 │
│  ┌──────────────────────┐       │
│  │   Build HAT          │       │
│  └──────────────────────┘       │
│                                 │
│  [Battery: Shargeek]            │
│                                 │
├─[Motor L]──────────[Motor R]───┤
│     │                  │        │
│   Wheel              Wheel      │
│                                 │
│   [Wheel]          [Wheel]      │ (Rear casters/omni)
└─────────────────────────────────┘

Side View:
        ┌─────────┐
        │ Pi5+HAT+│
        │ Sensors │
        └────┬────┘
             │
   ┌─────────┴─────────┐
   │  LEGO Frame       │
   │   (Beams)         │
   └─┬───────────────┬─┘
     │               │
  [Motor]         [Motor]
     │               │
   Wheel           Wheel
```

**Build Steps:**
1. Create base frame (LEGO beams)
2. Mount motors on sides (differential drive)
3. Add rear caster wheels (LEGO omni wheels)
4. Mount Pi 5 + Build HAT on top
5. Add sensors (Camera, LiDAR, IMU)
6. Wire everything up
7. Done!

**Estimated Build Time:** 4-6 hours

---

## 💰 Cost Comparison (All Options)

| Option | Cost | Speed | Complexity | Best For |
|--------|------|-------|------------|----------|
| **1. Full LEGO (Differential)** | $73-103 | 0.4 m/s | ⭐ Easy | **WRO** |
| **2. Hybrid INJORA+LEGO** | $78-98 | 1.25 m/s | ⭐⭐ Medium | Speed tracks |
| **3. Keep INJORA+Servo** | $38 | 1.25 m/s | ⭐⭐ Medium | Budget + Speed |
| 4. Full LEGO (Ackermann) | $60-90 | 0.4 m/s | ⭐⭐⭐ Hard | Car-like |

---

## ✅ My Final Recommendation

### **Option 1: Full LEGO Differential System** 🏆

**Purchase List:**
- [ ] 2× LEGO Technic Large Motor 88013 ($40-60)
- [ ] 1× Raspberry Pi Build HAT ($25-35)
- [ ] 1× TCS34725 color sensor ($8)
- **Total: $73-103**

**Use What You Have:**
- ✅ Bugatti Bolide 905 LEGO Technic parts
- ✅ Raspberry Pi 5 (16GB)
- ✅ AI HAT+ 26 TOPS
- ✅ Camera, LiDAR, IMU, batteries

**Why This Wins:**

1. **Configuration Flexibility** ✅
   - JSON/YAML configs (your #1 requirement!)
   - ROS2 diff_drive_controller
   - Behavior trees
   - Perfect for surprise rules

2. **Simplicity** ✅
   - Simplest mechanical design
   - Simplest ROS2 integration
   - Plug & play Build HAT
   - No Pico 2W needed

3. **Reliability** ✅
   - LEGO Education standard
   - Proven hardware
   - Community support
   - Easy to debug

4. **WRO Performance** ✅
   - Zero turning radius (best!)
   - Precise positioning
   - Good enough speed for WRO
   - Tight space navigation

5. **Build Experience** ✅
   - Use your Bugatti Bolide parts!
   - LEGO = easy assembly/modification
   - Professional appearance
   - Fun to build!

**Trade-offs Accepted:**
- ⚠️ Slower than INJORA (0.4 vs 1.25 m/s)
- ✅ Compensated by zero turning radius
- ✅ WRO values precision over raw speed
- ✅ Can still complete courses competitively

---

## 🚀 Implementation Plan

### Week 1: Order & Design
- [ ] Order 2× LEGO Large Motor 88013
- [ ] Order 1× Build HAT
- [ ] Order 1× TCS34725
- [ ] Design chassis layout (sketch)

### Week 2: Mechanical Build
- [ ] Build differential chassis from Bugatti parts
- [ ] Mount motors
- [ ] Add caster wheels
- [ ] Mount Pi 5 + Build HAT
- [ ] Basic assembly complete

### Week 3: Electronics & Software
- [ ] Install Build HAT library (`pip install buildhat`)
- [ ] Test motor control (basic)
- [ ] Implement differential drive node (ROS2)
- [ ] Test Nav2 integration

### Week 4: Integration & Testing
- [ ] Add all sensors (Camera, LiDAR, IMU)
- [ ] Implement odometry fusion (EKF)
- [ ] Test YOLO + navigation
- [ ] Create mission config YAML

### Week 5-11: Competition Prep
- [ ] Fine-tune navigation
- [ ] Test on WRO-like courses
- [ ] Optimize performance
- [ ] Document everything

---

## 📊 Speed Analysis: Is 0.4 m/s Enough?

**LEGO Setup Speed:**
- Max speed: 0.4 m/s (24 m/min)
- Typical WRO course: 3m × 3m = 9 sqm
- Course perimeter: ~12m

**Time Estimates:**

| Task | INJORA (1.25 m/s) | LEGO (0.4 m/s) |
|------|------------------|----------------|
| **Straight line (3m)** | 2.4 sec | 7.5 sec |
| **Turn in place (180°)** | 1 sec | 1 sec ⭐ |
| **Navigate obstacles** | 15 sec | 20 sec |
| **Complete lap** | ~30 sec | ~45 sec |

**Analysis:**
- ⚠️ 50% slower on straights
- ✅ Same turning time (zero radius!)
- ✅ Better precision = fewer corrections
- ✅ Typical WRO run: 2-3 minutes
- ✅ 15-second difference won't matter if reliable!

**Bottom Line:**
- LEGO is slower but more reliable
- **Reliability > Raw Speed** for competitions
- Fewer retries due to errors = better final time
- **Speed is acceptable for WRO!**

---

## 🔗 Resources

### Build HAT:
- [Raspberry Pi Build HAT Documentation](https://www.raspberrypi.com/documentation/accessories/build-hat.html)
- [Build HAT Python Library](https://buildhat.readthedocs.io/en/latest/buildhat/index.html)
- [Meet the Build HAT (Raspberry Pi News)](https://www.raspberrypi.com/news/raspberry-pi-build-hat-lego-education/)

### LEGO Motors:
- [LEGO Technic Large Motor 88013](https://www.lego.com/en-us/product/technic-large-motor-88013)
- [LEGO Technic Medium Motor 45603](https://education.lego.com/en-us/products/lego-technic-medium-angular-motor/45603/)

### Differential Steering:
- [ROS2 diff_drive_controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)

### Community:
- [LEGO Technic Motors Guide (ZENE)](https://www.zenebricks.com/blogs/parts/lego-technic-motors-and-mechanical-sets)
- [LEGO Robot Car Tutorial (Code Club)](https://projects.raspberrypi.org/en/projects/lego-robot-car)

---

## ✅ Summary

**Question:** Should I use 2 LEGO Large Motors (one for drive, one for steering)?

**Answer:** **YES, but use differential steering (both for drive)!**

**Recommended Setup:**
- ✅ 2× LEGO Large Motor 88013 (differential drive)
- ✅ 1× Raspberry Pi Build HAT
- ✅ Chassis from Bugatti Bolide parts
- ✅ ROS2 + diff_drive_controller
- ✅ JSON/YAML configuration
- ✅ Total: $73-103

**Why It's Best:**
1. ✅ Simplest setup (no Ackermann linkage)
2. ✅ Perfect for WRO (zero turning radius)
3. ✅ Uses your LEGO parts
4. ✅ Meets configuration requirement
5. ✅ Reliable and proven
6. ✅ Good enough speed (0.4 m/s)

**Alternative (If Speed Critical):**
- Keep INJORA motor (1.25 m/s)
- Add PMW3901 optical flow ($30)
- Total: $38
- Trade-off: More complex, no LEGO integration

**Winner for YOU: Full LEGO Differential System!** 🏆

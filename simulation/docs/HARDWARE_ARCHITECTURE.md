# Hardware Architecture - Klevor V2 Robot Platform

**WRO 2026 Future Engineers - Team Steel Bot**
**Last Updated:** 2026-02-12
**Status:** Current Hardware Revision

This document describes the complete hardware architecture of the Klevor V2 autonomous robot, including all compute boards, sensors, actuators, communication buses, and mechanical specifications.

---

## 1. System Overview

The Klevor V2 uses a dual-processor architecture. A Raspberry Pi 5 serves as the primary perception and decision-making unit, while a Raspberry Pi Zero 2W handles low-level motor control through the LEGO Build HAT. The two boards communicate over a USB OTG link running Zenoh-based ROS 2 middleware.

### 1.1 High-Level System Diagram

```
+===========================================================================+
|                        KLEVOR V2 - SYSTEM ARCHITECTURE                    |
+===========================================================================+

  +-----------------------------------------------------------------------+
  |                     RASPBERRY PI 5 (Primary Compute)                  |
  |                        Quad Cortex-A76 @ 2.4 GHz                     |
  |                                                                       |
  |  +----------------+    +------------------+    +-------------------+  |
  |  |  RPi Camera 3  |    |   Hailo 8L NPU   |    |  Slamtec C1 LIDAR | |
  |  |     Wide       |    |    13 TOPS        |    |    360 deg        | |
  |  |                |    |                   |    |                   | |
  |  |  CSI Ribbon    |    |   M.2 Key E      |    |   USB 2.0         | |
  |  +-------+--------+    +--------+---------+    +--------+----------+ |
  |          |                       |                       |            |
  |          v                       v                       v            |
  |  +-------+--------+    +--------+---------+    +--------+----------+ |
  |  | Camera Node    |    | Detection Node   |    |  LIDAR Node       | |
  |  | (image_pub)    |    | (hailo_inference)|    |  (sllidar_ros2)   | |
  |  +----------------+    +------------------+    +-------------------+ |
  |                                                                       |
  |  +-------------------------------+                                    |
  |  |  BNO085 IMU (9-DOF)          |                                    |
  |  |  via MCP2221A USB-I2C Bridge  |                                    |
  |  |  USB 2.0 --> I2C @ 400 kHz   |                                    |
  |  +---------------+---------------+                                    |
  |                   |                                                   |
  |                   v                                                   |
  |  +---------------+---------------+                                    |
  |  |  IMU Node (imu_pub)          |                                    |
  |  +-------------------------------+                                    |
  |                                                                       |
  |         ROS 2 Kilted  /  rmw_zenoh_cpp (DDS-free)                     |
  |                          |                                            |
  +--------------------------|--------------------------------------------+
                             |
                     USB OTG Cable
                  (Data + 5V Power)
                             |
  +--------------------------|--------------------------------------------+
  |                          v                                            |
  |                RASPBERRY PI ZERO 2W (Motor Controller)                |
  |                   Quad Cortex-A53 @ 1.0 GHz                          |
  |                                                                       |
  |         ROS 2 Kilted  /  rmw_zenoh_cpp (DDS-free)                     |
  |                                                                       |
  |  +-------------------------------+                                    |
  |  |       LEGO Build HAT          |                                    |
  |  |     (Serial UART @ 115200)    |                                    |
  |  +-------+--------------+--------+                                    |
  |           |              |                                            |
  |           v              v                                            |
  |  +--------+---+  +------+------+                                      |
  |  | Port A     |  | Port B      |                                      |
  |  | Drive Motor|  | Steer Motor |                                      |
  |  | (Rear Axle)|  | (Front Axle)|                                      |
  |  +------------+  +-------------+                                      |
  |                                                                       |
  +-----------------------------------------------------------------------+
```

### 1.2 Data Flow Summary

```
Camera ----CSI----> RPi5 --frame--> Hailo 8L NPU --detections--> Navigation Node
LIDAR  ----USB----> RPi5 --/scan--> Navigation Node
IMU  ------USB----> RPi5 --/imu---> Navigation Node
                              |
                     Zenoh (USB OTG)
                              |
                              v
                    RPi Zero 2W --/cmd_vel--> Build HAT --> Motors
```

---

## 2. Compute Boards

### 2.1 Raspberry Pi 5 -- Primary Compute

| Parameter          | Value                                    |
|--------------------|------------------------------------------|
| **SoC**            | Broadcom BCM2712, Quad Cortex-A76 2.4GHz |
| **RAM**            | 8 GB LPDDR4X-4267                        |
| **Storage**        | 32 GB microSD (Class A2 U3)              |
| **USB Ports Used** | USB 2.0: LIDAR, MCP2221A (IMU)          |
| **CSI Port**       | RPi Camera 3 Wide                        |
| **M.2 HAT+**      | Hailo 8L NPU (Key E)                    |
| **OS**             | Ubuntu 24.04 LTS (arm64)                 |
| **ROS 2**          | Kilted Kaiju (rmw_zenoh_cpp)             |
| **Role**           | Perception, planning, decision-making    |

### 2.2 Raspberry Pi Zero 2W -- Motor Controller

| Parameter          | Value                                     |
|--------------------|-------------------------------------------|
| **SoC**            | Broadcom BCM2710A1, Quad Cortex-A53 1GHz  |
| **RAM**            | 512 MB LPDDR2                             |
| **Storage**        | 16 GB microSD (Class A2 U3)               |
| **USB Port**       | Micro-USB OTG (data + power from RPi5)    |
| **GPIO HAT**       | LEGO Build HAT (UART serial)              |
| **OS**             | Ubuntu 24.04 LTS (arm64, server)          |
| **ROS 2**          | Kilted Kaiju (rmw_zenoh_cpp)              |
| **Role**           | Motor control, Build HAT interface         |

---

## 3. Sensor Specifications

### 3.1 Slamtec C1 LIDAR

The Slamtec C1 is a 360-degree 2D laser scanner mounted on top of the robot chassis. It provides the primary obstacle detection and wall-following data for navigation.

| Parameter              | Value                          |
|------------------------|--------------------------------|
| **Range**              | 0.05 m -- 12.0 m              |
| **Scan Rate**          | 5--10 Hz (typical 7--8 Hz)    |
| **Samples per Scan**   | ~500 points                    |
| **Angular Coverage**   | 360 degrees                    |
| **Angular Resolution** | ~0.72 deg (360/500)            |
| **Laser Wavelength**   | 785 nm (infrared)              |
| **Interface**          | USB 2.0 (internal UART bridge) |
| **Body Radius**        | 27.8 mm                        |
| **Body Height**        | 41.3 mm                        |
| **Weight**             | 110 g                          |
| **ROS 2 Driver**       | `sllidar_ros2`                 |
| **ROS 2 Topic**        | `/scan` (sensor_msgs/LaserScan)|

### 3.2 Raspberry Pi Camera Module 3 Wide

The wide-angle camera is forward-facing, mounted at the front of the chassis. It is used for traffic sign detection (red/green pillars) and parking lot identification.

| Parameter              | Value                                 |
|------------------------|---------------------------------------|
| **Sensor**             | Sony IMX708                           |
| **Horizontal FOV**     | 102 degrees                           |
| **Resolution (used)**  | 1536 x 864 pixels                     |
| **Frame Rate**         | 30 fps                                |
| **Interface**          | MIPI CSI-2 (15-pin ribbon cable)      |
| **Autofocus**          | Phase Detection (PDAF)                |
| **ROS 2 Driver**       | `camera_ros`                          |
| **ROS 2 Topic**        | `/image_raw` (sensor_msgs/Image)      |
| **Processing**         | Frames forwarded to Hailo 8L NPU      |

### 3.3 Bosch BNO085 IMU (9-DOF)

The BNO085 provides fused orientation data using its internal sensor fusion engine. It is connected to the RPi5 through a Microchip MCP2221A USB-to-I2C bridge adapter, since the RPi5 I2C bus is occupied by the camera and HAT peripherals.

| Parameter              | Value                                      |
|------------------------|--------------------------------------------|
| **Degrees of Freedom** | 9-DOF (accel + gyro + magnetometer)        |
| **Gyroscope Noise**    | 0.054 rad/s (std. deviation)               |
| **Accelerometer Noise**| 0.3 m/s^2 (std. deviation)                 |
| **Sensor Fusion**      | On-chip ARVR stabilization (quaternion)    |
| **Dimensions**         | 25.6 x 22.7 x 4.6 mm (breakout board)     |
| **Weight**             | 2.5 g                                      |
| **I2C Address**        | 0x4A (default)                             |
| **I2C Bus Speed**      | 400 kHz (Fast Mode)                        |
| **USB Bridge**         | Microchip MCP2221A (USB HID to I2C)        |
| **Interface to RPi5**  | USB 2.0 (via MCP2221A adapter)             |
| **ROS 2 Topic**        | `/imu` (sensor_msgs/Imu)                   |

#### MCP2221A USB-I2C Bridge Detail

```
RPi5 USB 2.0 Port
       |
       v
+------+--------+
| MCP2221A      |
| USB HID Class |
| (no driver    |
|  install req.)|
+------+--------+
       |  I2C SDA/SCL (400 kHz)
       v
+------+--------+
| BNO085 IMU    |
| Addr: 0x4A    |
+---------------+
```

The MCP2221A presents as a standard USB HID device and requires no kernel driver. The `bno085_driver` ROS 2 node communicates through the `smbus2` or `hidapi` Python library to read fused orientation and raw inertial data from the BNO085.

### 3.4 Hailo 8L Neural Processing Unit

The Hailo 8L is an M.2 AI accelerator module installed on the Raspberry Pi 5 via the official M.2 HAT+. It handles real-time object detection (traffic sign classification: red/green pillars, parking lot markers).

| Parameter              | Value                                     |
|------------------------|-------------------------------------------|
| **Performance**        | 13 TOPS (INT8)                            |
| **Form Factor**        | M.2 2242 (Key E)                          |
| **Interface**          | PCIe Gen3 x1 (via RPi5 M.2 HAT+)        |
| **Power Consumption**  | ~1.5 W (typical inference)                |
| **Supported Frameworks**| TensorFlow Lite, ONNX (via Hailo DFC)   |
| **SDK**                | HailoRT + TAPPAS (Hailo Application Suite)|
| **Model Format**       | `.hef` (Hailo Executable Format)          |
| **Typical Model**      | YOLOv8n / YOLOv5s (quantized INT8)       |
| **Inference Latency**  | ~8--15 ms per frame (model-dependent)     |

---

## 4. Actuators

### 4.1 LEGO Technic Large Angular Motors

The robot uses two LEGO Technic Large Angular Motors (Item No. 88017), each connected to the LEGO Build HAT on the Raspberry Pi Zero 2W. The Ackermann steering geometry is mechanically implemented through the LEGO Technic gear train.

| Parameter               | Value                                    |
|--------------------------|------------------------------------------|
| **Motor Type**           | LEGO Technic Large Angular Motor (88017) |
| **Quantity**             | 2                                        |
| **Encoder**              | Integrated absolute encoder (1 deg res.) |
| **Interface**            | LEGO Powered Up connector (6-pin)        |
| **Controlled By**        | LEGO Build HAT (serial UART to RPi Zero) |
| **Voltage**              | 7.2 V (regulated by Build HAT)           |

#### Motor Assignments

| Motor | Build HAT Port | Function          | Mechanical Coupling               |
|-------|----------------|-------------------|------------------------------------|
| **Drive Motor**  | Port A | Rear-wheel drive  | Gear train to rear axle (both rear wheels) |
| **Steer Motor**  | Port B | Front-axle steering | Gear train to front steering linkage (Ackermann geometry) |

- **Drive Motor (Port A):** Connected to the rear axle through a LEGO Technic gear reduction. Powers both rear wheels simultaneously. Speed is controlled via PWM duty cycle commands from the Build HAT.
- **Steer Motor (Port B):** Connected to the front axle steering linkage. The motor's absolute encoder provides closed-loop position control for precise steering angle targeting. The mechanical linkage enforces Ackermann steering geometry, meaning the inner wheel turns at a larger angle than the outer wheel during cornering.

### 4.2 LEGO Build HAT

| Parameter          | Value                                       |
|--------------------|---------------------------------------------|
| **Processor**      | Raspberry Pi RP2040                         |
| **Motor Ports**    | 4x LPF2 (LEGO Powered Up) connectors       |
| **Ports Used**     | Port A (drive), Port B (steer)              |
| **Interface**      | Serial UART @ 115200 baud (to RPi Zero 2W) |
| **Power Input**    | 7.5 V barrel jack (external battery pack)   |
| **Power Output**   | Regulated 5V to RPi Zero 2W via GPIO        |

---

## 5. Robot Mechanical Specifications

### 5.1 Chassis

The robot is built on a modified **LEGO Technic Bugatti Bolide** (Set 42151) chassis. The original chassis has been adapted to mount all compute boards, sensors, and the LIDAR turret.

| Parameter              | Value                |
|------------------------|----------------------|
| **Chassis Base**       | LEGO Technic Bugatti Bolide (42151) |
| **Overall Dimensions** | 280 x 150 x 100 mm (L x W x H)    |
| **Total Weight**       | ~0.8 kg (with all electronics)      |
| **Drive Configuration**| Rear-wheel drive, front-axle steer  |
| **Steering Type**      | Ackermann geometry (mechanical)     |

### 5.2 Ackermann Steering Geometry

The front axle uses a physical Ackermann linkage built from LEGO Technic components. This ensures that during turns, the inner front wheel steers at a larger angle than the outer front wheel, reducing tire scrub and improving turning accuracy.

```
                     Front of Robot
                          |
              +-----------+-----------+
              |                       |
         Left Wheel             Right Wheel
          (delta_L)              (delta_R)
              |                       |
              +---+    Tie Rod    +---+
                  |     Linkage   |
                  +--+---------+--+
                     | Steering |
                     | Motor    |
                     +----------+

                    <-- 105 mm -->
                     (Track Width)

                          ^
                          |  170 mm (Wheelbase)
                          |
                          v

              +-----------+-----------+
              |                       |
         Left Rear              Right Rear
          (fixed)                (fixed)
```

#### Ackermann Parameters

| Parameter                | Value              |
|--------------------------|--------------------|
| **Wheelbase (L)**        | 170 mm             |
| **Track Width (T)**      | 105 mm             |
| **Max Steering Angle**   | 30 degrees         |
| **Min Turning Radius**   | ~340 mm (calculated at max steer) |
| **Steering Ratio**       | Motor-to-wheel via gear linkage   |

#### Ackermann Equations

For a commanded center steering angle `delta`:

```
Inner wheel angle:  delta_inner = atan( L / (L/tan(delta) - T/2) )
Outer wheel angle:  delta_outer = atan( L / (L/tan(delta) + T/2) )
Turning radius (R): R = L / tan(delta)
```

Where:
- `L` = 170 mm (wheelbase, center of rear axle to center of front axle)
- `T` = 105 mm (track width, center-to-center of front wheels)
- `delta` = commanded steering angle (max 30 deg)

At maximum steering (30 degrees):
- Inner wheel: ~34.7 degrees
- Outer wheel: ~26.0 degrees
- Turning radius: ~294 mm (center of rear axle to ICR)

---

## 6. Communication Architecture

### 6.1 USB Gadget Mode (RPi5 to RPi Zero 2W)

The Raspberry Pi Zero 2W connects to the RPi5 via its micro-USB OTG port, configured in **USB Gadget Mode**. This single USB cable provides both data communication and 5V power delivery to the Zero 2W.

#### Configuration

On the **RPi Zero 2W** (gadget/device side):

1. Enable the `dwc2` USB OTG driver overlay in `/boot/firmware/config.txt`:
   ```
   dtoverlay=dwc2
   ```

2. Load the USB Ethernet gadget kernel module via `/boot/firmware/cmdline.txt`:
   ```
   ... modules-load=dwc2,g_ether
   ```

3. After boot, the Zero 2W presents itself as a USB Ethernet device (`usb0`) to the RPi5.

On the **RPi5** (host side):

1. The RPi5 automatically detects the Zero 2W as a USB Ethernet adapter.
2. Assign a static IP to the `usb0` interface via `netplan`:
   ```yaml
   # /etc/netplan/99-usb-gadget.yaml
   network:
     version: 2
     ethernets:
       usb0:
         addresses:
           - 10.0.0.1/24
         dhcp4: false
   ```

3. On the Zero 2W, configure the matching static IP:
   ```yaml
   # /etc/netplan/99-usb-gadget.yaml
   network:
     version: 2
     ethernets:
       usb0:
         addresses:
           - 10.0.0.2/24
         dhcp4: false
   ```

#### USB Gadget Network Summary

| Parameter        | RPi5 (Host)     | RPi Zero 2W (Gadget) |
|------------------|------------------|-----------------------|
| **IP Address**   | 10.0.0.1         | 10.0.0.2              |
| **Subnet**       | /24 (255.255.255.0) | /24 (255.255.255.0) |
| **Interface**    | usb0             | usb0                  |
| **USB Role**     | Host             | Device (Gadget)       |
| **Power**        | Supplies 5V      | Receives 5V           |
| **Data Link**    | USB 2.0 (480 Mbps max) | USB 2.0          |

### 6.2 Zenoh Middleware (rmw_zenoh_cpp)

The robot runs ROS 2 Kilted Kaiju with `rmw_zenoh_cpp` as the RMW (ROS Middleware) implementation instead of the default DDS-based middleware. Zenoh provides lower latency, smaller memory footprint, and simpler network configuration -- all critical for the resource-constrained RPi Zero 2W.

#### Why Zenoh Instead of DDS

| Concern                  | DDS (default)               | Zenoh (rmw_zenoh_cpp)          |
|--------------------------|-----------------------------|--------------------------------|
| **Discovery**            | Multicast (unreliable over USB gadget) | TCP peer-to-peer (reliable) |
| **Memory Footprint**     | ~80--120 MB per process     | ~15--30 MB per process         |
| **Startup Time**         | 5--10 seconds               | < 1 second                     |
| **Configuration**        | Complex XML profiles        | Single TOML/JSON config        |
| **Multi-host Support**   | Requires DDS router/bridge  | Native peer-to-peer            |

#### Installation (Both Boards)

```bash
# Install ROS 2 Kilted (Ubuntu 24.04 arm64)
sudo apt install ros-kilted-desktop  # RPi5
sudo apt install ros-kilted-ros-base # RPi Zero 2W (minimal)

# Install rmw_zenoh_cpp
sudo apt install ros-kilted-rmw-zenoh-cpp

# Set as default RMW implementation
echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc
source ~/.bashrc
```

#### Zenoh Router Configuration

A Zenoh router runs on the RPi5 to bridge communication between the two boards.

```bash
# Start Zenoh router on RPi5 (listens for Zero 2W connections)
ros2 run rmw_zenoh_cpp rmw_zenohd
```

The Zenoh router default config listens on `tcp/0.0.0.0:7447`. The RPi Zero 2W's Zenoh session connects to the RPi5 router at `tcp/10.0.0.1:7447`.

**RPi Zero 2W Zenoh session config** (`~/.config/zenoh/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`):

```json5
{
  mode: "client",
  connect: {
    endpoints: [
      "tcp/10.0.0.1:7447"
    ]
  }
}
```

**RPi5 Zenoh session config** (`~/.config/zenoh/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`):

```json5
{
  mode: "client",
  connect: {
    endpoints: [
      "tcp/localhost:7447"
    ]
  }
}
```

#### Verifying Zenoh Communication

```bash
# On RPi5: list all topics (should see topics from both boards)
ros2 topic list

# On RPi5: echo motor commands being sent to Zero 2W
ros2 topic echo /cmd_vel

# On RPi Zero 2W: echo LIDAR data coming from RPi5
ros2 topic echo /scan --no-arr
```

---

## 7. ROS 2 Topic Map

The following table lists all ROS 2 topics exchanged between the two compute boards.

| Topic              | Message Type              | Publisher    | Subscriber    | Rate     |
|--------------------|---------------------------|-------------|---------------|----------|
| `/scan`            | `sensor_msgs/LaserScan`   | RPi5        | RPi5 (nav)    | ~8 Hz    |
| `/image_raw`       | `sensor_msgs/Image`       | RPi5        | RPi5 (hailo)  | 30 Hz    |
| `/imu`             | `sensor_msgs/Imu`         | RPi5        | RPi5 (nav)    | 100 Hz   |
| `/detections`      | `vision_msgs/Detection2DArray` | RPi5   | RPi5 (nav)    | 30 Hz    |
| `/cmd_vel`         | `geometry_msgs/Twist`     | RPi5 (nav)  | Zero 2W       | 20 Hz    |
| `/odom`            | `nav_msgs/Odometry`       | Zero 2W     | RPi5 (nav)    | 20 Hz    |
| `/joint_states`    | `sensor_msgs/JointState`  | Zero 2W     | RPi5 (diag)   | 20 Hz    |

Topics crossing the USB gadget link (via Zenoh): `/cmd_vel`, `/odom`, `/joint_states`.

---

## 8. Power Architecture

```
+-------------------+
| 7.4V LiPo Battery |
| (2S, 1500 mAh)    |
+--------+----------+
         |
         v
+--------+----------+
| LEGO Build HAT    |  --> 7.2V to LEGO Motors (Port A, Port B)
| (barrel jack in)  |  --> 5V regulated to RPi Zero 2W (via GPIO)
+--------+----------+
         |
         | (5V via GPIO header)
         v
+--------+----------+
| RPi Zero 2W       |
| (powered by HAT)  |
+--------+----------+
         |
         | USB OTG cable (5V passthrough to RPi5 NOT used;
         | RPi5 has its own power supply)
         v
+--------+----------+     +-------------------+
| RPi5              |<----| 5V 5A USB-C PSU   |
| (separate supply) |     | (or 5V BEC from   |
+-------------------+     |  main battery)     |
                          +-------------------+
```

| Component             | Voltage | Typical Current | Power Source              |
|-----------------------|---------|-----------------|---------------------------|
| RPi5 + Hailo 8L      | 5V      | 2.5 A           | USB-C PD or 5V BEC        |
| RPi Zero 2W          | 5V      | 0.4 A           | Build HAT GPIO 5V rail    |
| Build HAT + Motors    | 7.4V    | 1.0 A (peak 2A) | LiPo battery (2S)         |
| Slamtec C1 LIDAR     | 5V      | 0.25 A          | RPi5 USB port              |
| MCP2221A + BNO085     | 5V/3.3V | 0.05 A          | RPi5 USB port              |

---

## 9. Physical Layout

### 9.1 Top-Down View

```
         FRONT (Camera faces this direction)
    +--------------------------------------+
    |  [RPi Camera 3 Wide]   (front edge)  |
    |                                      |
    |  +----------+     +----------+       |
    |  |  RPi5    |     | Hailo 8L |       |
    |  |          |     | (under   |       |
    |  |          |     |  M.2 HAT)|       |
    |  +----------+     +----------+       |
    |                                      |
    |         +----------------+           |
    |         | Slamtec C1     |           |
    |         | LIDAR (top)    |           |
    |         +----------------+           |
    |                                      |
    |  +----------+     +----------+       |
    |  | RPi Zero |     | Build HAT|       |
    |  | 2W       |     | (under   |       |
    |  |          |     |  Zero)   |       |
    |  +----------+     +----------+       |
    |                                      |
    |  [LiPo Battery]     [BNO085+MCP2221] |
    +--------------------------------------+
         REAR (Drive axle)

    <------------- 280 mm --------------->
    ^
    |  150 mm
    v
```

### 9.2 WRO Size Compliance

Per WRO 2026 Future Engineers rules, the robot must fit within a **300 x 200 x 300 mm** bounding box.

| Dimension | Robot   | WRO Limit | Margin |
|-----------|---------|-----------|--------|
| Length    | 280 mm  | 300 mm    | 20 mm  |
| Width     | 150 mm  | 200 mm    | 50 mm  |
| Height    | 100 mm  | 300 mm    | 200 mm |
| Weight    | 0.8 kg  | 1.5 kg    | 0.7 kg |

---

## 10. Summary of Interfaces

```
+--------+-------------+-----------+------------------------------+
| Bus    | From        | To        | Purpose                      |
+--------+-------------+-----------+------------------------------+
| CSI    | Camera 3    | RPi5      | Video frames (30 fps)        |
| PCIe   | Hailo 8L    | RPi5      | NPU inference (M.2 HAT+)    |
| USB    | Slamtec C1  | RPi5      | LIDAR scan data              |
| USB    | MCP2221A    | RPi5      | I2C bridge for BNO085 IMU    |
| I2C    | BNO085      | MCP2221A  | IMU data (400 kHz)           |
| USB OTG| RPi5        | Zero 2W   | Zenoh transport + 5V power   |
| UART   | Zero 2W     | Build HAT | Motor commands (115200 baud) |
| LPF2   | Build HAT   | Motor A   | Drive motor (rear axle)      |
| LPF2   | Build HAT   | Motor B   | Steer motor (front axle)     |
+--------+-------------+-----------+------------------------------+
```

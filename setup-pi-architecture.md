# Voldemorbot Pi Setup & Startup Services — Specification

> **⚠️ Historical design spec — partially superseded.** This predates the
> actual implementation and has drifted from it in several places: systemd
> units here launch via raw `ros2 launch` (the real units source
> `ros2_ws/install/setup.bash` via a pixi task instead), `User=pi`/`/home/pi`
> paths are now parameterized per-account, and the Taskfile provisioning
> tasks shown below were deleted (superseded by `rpi:provision:zero` /
> `rpi:provision:pi5` in the root `Taskfile.yml`). For the current, accurate
> setup reference use `docs/pi-setup.md` and `scripts/README.md`. Pin
> assignments and hardware architecture below are still accurate.

## 1. Hardware Architecture

```
Pi 5 (16GB — compute, USB-heavy)              Pi Zero 2W (GPIO — real-time control)
┌─────────────────────────────────┐           ┌──────────────────────────────────────┐
│ BNO055 IMU   → USB (MCP2221A)   │           │ Servo       ← GPIO12 (PWM1)          │
│ RPLiDAR C1   → USB (ttyUSB0)    │           │ H-bridge:   ← GPIO13 (PWM0)          │
│ AI HAT+      → PCIe/HAT         │  USB      │   IN3       ← GPIO5                  │
│ Camera Mod 3 → CSI              │  Gadget   │   IN4       ← GPIO6                  │
│                                 │◄────────► │ Encoder C1  ← GPIO16                 │
│                                 │ g_ether   │ Encoder C2  ← GPIO20                 │
│                                 │192.168.250.x│ Button       ← GPIO4 (pull-up)      │
│                                 │           │ OLED (I2C)   ← GPIO2 (SDA), 3 (SCL)  │
└─────────────────────────────────┘           └──────────────────────────────────────┘

Steering mechanism: single servo moves all 4 wheels to same side (crab/parallel steering)
```

## 2. Pin Assignment (Pi Zero 2W)

| Signal | GPIO | Physical | Notes |
|---|---|---|---|
| Push button | 4 | 7 | Start/E-stop, pull-up, GND=press |
| Servo PWM | 12 | 32 | PWM1, 50 Hz, 500-2500 µs |
| Motor PWM (H-bridge) | 13 | 33 | PWM0, speed control |
| H-bridge IN3 | 5 | 29 | Direction (L298N channel B) |
| H-bridge IN4 | 6 | 31 | Direction (L298N channel B) |
| Encoder C1 | 16 | 36 | Quadrature channel A |
| Encoder C2 | 20 | 38 | Quadrature channel B |
| OLED SDA | 2 | 3 | I2C data |
| OLED SCL | 3 | 5 | I2C clock |

## 3. Node Distribution

| Node | Runs on | Subscribes | Publishes |
|---|---|---|---|
| `ackermann_motor_node` | Pi Zero | `/ackermann_cmd` | `/motor/steering_position`, `/motor/drive_speed`, `/motor/status`, `/motor/odometry` |
| `button_node` | Pi Zero | — | `/button/event` |
| `oled_display_node` | Pi Zero | `/robot_state`, `/system_status`, `/race_metrics`, `/imu/data`, `/hailo/fps` | `/ui/oled_mirror` |
| `state_machine_node` | Pi 5 | `/imu/data`, `/scan`, `/hailo/detections`, `/hailo/fps`, `/button/event`, `/motor/status` | `/robot_state`, `/ackermann_cmd`, `/system_status`, `/race_metrics` |
| `telemetry_bridge_node` | Pi 5 | `/scan`, `/odom`, `/imu/data`, `/robot_state`, `/hailo/detections` | HTTP→backend |
| `bno08x_uart_rvc_node` | Pi 5 | — | `/imu/data` |
| `vision_node` | Pi 5 | — | `/hailo/detections`, `/hailo/fps` |
| `lidar_launch` | Pi 5 | — | `/scan` |

## 4. Files to Create

```
scripts/
├── setup_common.sh
├── setup_pi_zero.sh
└── setup_pi_5.sh

platform/robot/
├── systemd/
│   ├── voldemorbot-pi5.service
│   ├── voldemorbot-pi-zero.service
│   ├── voldemorbot-lidar.service
│   └── voldemorbot-backend.service
│
├── udev/
│   └── 99-voldemorbot-gpio.rules
│
├── ros2_ws/src/voldemorbot_robot/
│   ├── setup.py                               # add button_node entry point
│   ├── launch/
│   │   ├── rpi_zero_nodes.launch.py           # add button_node + oled_display_node
│   │   └── rpi5_nodes.launch.py               # add IMU + vision nodes
│   └── voldemorbot_robot/
│       ├── button_node.py                     # NEW
│       └── motors/
│           └── ackermann_motor_node.py         # rewrite: servo + dc_encoder
│
└── src/hardware/motors/
    └── servo/                                 # NEW PACKAGE
        ├── __init__.py
        ├── driver.py
        └── config.py
```

## 5. New Files: Content Specifications

### 5a. `servo/driver.py`

```python
"""SteeringDriver for RC servo via PWM (pigpio/gpiozero)."""

from src.hardware.motors.base import SteeringDriver

class Driver(SteeringDriver):
    def __init__(self, pin: int = 12,
                 min_pulse_us: float = 500,
                 max_pulse_us: float = 2500,
                 range_deg: float = 180,
                 center_pulse_us: float = 1500):
        # GPIO pin, 50 Hz PWM
        # Position = 0 is center, negative = left, positive = right

    def move_steering_to(self, position: float, speed: int = 20) -> None:
        # Map position (deg) to pulse width, set PWM
        # pulse = center_pulse_us + (position / range_deg * (max_pulse_us - min_pulse_us))

    def center_steering(self) -> None:
        self.move_steering_to(0.0)

    def get_steering_position(self) -> float:
        # Return last commanded position (servo has no position feedback)
```

### 5b. `button_node.py`

```python
"""Publishes GPIO button events to /button/event for remote state machine."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from src.hardware.button.gpio import Driver as ButtonDriver

class ButtonNode(Node):
    def __init__(self):
        super().__init__("button_node")
        self.pub = self.create_publisher(String, "/button/event", 10)
        self.driver = ButtonDriver()
        self.driver.connect()
        self.timer = self.create_timer(0.05, self._check)  # 20 Hz

    def _check(self):
        state = self.driver.get_state()
        if state.last_event:
            msg = String()
            msg.data = state.last_event.value
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ButtonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    rclpy.shutdown()
```

### 5c. `setup.py` — add entry point

```python
entry_points={
    "console_scripts": [
        # ... existing entries ...
        "button_node = voldemorbot_robot.button_node:main",
    ],
},
```

## 6. Modified Files

### 6a. `ackermann_motor_node.py` — Backend swap

```python
# BEFORE (Build HAT):
from src.hardware.motors.build_hat import Driver as MotorDriver
self.motor_driver = MotorDriver()
self.motor_driver.connect()
self.motor_driver.center_steering()

# AFTER (servo + DC encoder):
from src.hardware.motors.servo.driver import Driver as SteeringDriver
from src.hardware.motors.dc_encoder.driver import Driver as EncodedDriveDriver

self.steering = SteeringDriver(pin=12)
self.drive = EncodedDriveDriver(
    pwm_pin=13, dir_a_pin=5, dir_b_pin=6,
    standby_pin=None,               # L298N, no standby pin
    encoder_a_pin=16, encoder_b_pin=20,
)
self.steering.center_steering()
self.drive.connect()
```

The `/ackermann_cmd` callback stays identical — `steering_angle` drives the servo, `speed` drives the DC motor.

### 6b. `state_machine_node.py` — Button decoupling

```python
# REMOVE:
from src.hardware.button.gpio import Driver as ButtonDriver
self.button_driver = ButtonDriver()
self.button_driver.connect()
self.button_timer = self.create_timer(0.05, self._button_check_loop)

# ADD:
self.button_sub = self.create_subscription(
    String, "/button/event", self._button_event_callback, 10)

def _button_event_callback(self, msg: String) -> None:
    if msg.data == "short_press" and self.state_machine.current_state == RobotState.READY:
        self.state_machine.transition_to(RobotState.RACING, StateTransitionReason.BUTTON_PRESSED)
        self.race_start_time = time.time()
        self.laps_completed = 0
    elif msg.data == "long_press" and self.state_machine.current_state == RobotState.RACING:
        self.state_machine.transition_to(RobotState.FINISHED, StateTransitionReason.EMERGENCY_STOP)
        self._publish_stop_command()
```

### 6c. `rpi_zero_nodes.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="voldemorbot_robot", executable="ackermann_motor_node",
             name="ackermann_motors", output="screen",
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
             respawn=True, respawn_delay=2.0),
        Node(package="voldemorbot_robot", executable="button_node",
             name="button", output="screen",
             respawn=True, respawn_delay=2.0),
        Node(package="voldemorbot_robot", executable="oled_display_node",
             name="oled_display", output="screen",
             respawn=True, respawn_delay=2.0),
    ])
```

### 6d. `rpi5_nodes.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="voldemorbot_robot", executable="state_machine_node",
             name="state_machine", output="screen"),
        Node(package="voldemorbot_robot", executable="telemetry_bridge_node",
             name="telemetry_bridge", output="screen"),
        Node(package="voldemorbot_robot", executable="bno08x_uart_rvc_node",
             name="imu", output="screen"),
        Node(package="voldemorbot_robot", executable="vision_node",
             name="vision", output="screen"),
    ])
```

## 7. Systemd Service Files

### 7a. `voldemorbot-pi-zero.service`

```ini
[Unit]
Description=Voldemorbot Pi Zero — Motors, Button, OLED
Wants=network-online.target dev-gpiochip0.device
After=network-online.target dev-gpiochip0.device

[Service]
Type=exec
User=pi
Group=pi
WorkingDirectory=/home/pi/voldemorbot/platform/robot
Environment=PATH=/home/pi/.pixi/bin:/usr/local/bin:/usr/bin:/bin
EnvironmentFile=-/home/pi/voldemorbot/platform/robot/.env
Environment=ROS_DOMAIN_ID=0
ExecStart=/home/pi/.pixi/bin/pixi run -e dev ros2 launch voldemorbot_robot rpi_zero_nodes.launch.py
Restart=on-failure
RestartSec=3
StartLimitBurst=5
StartLimitIntervalSec=30
ExecStop=/usr/bin/kill -SIGINT $MAINPID
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
```

### 7b. `voldemorbot-pi5.service`

```ini
[Unit]
Description=Voldemorbot Pi 5 — State Machine, Vision, IMU, LiDAR, Bridge
Wants=network-online.target hailort.service voldemorbot-lidar.service
After=network-online.target hailort.service voldemorbot-lidar.service

[Service]
Type=exec
User=pi
Group=pi
WorkingDirectory=/home/pi/voldemorbot/platform/robot
Environment=PATH=/home/pi/.pixi/bin:/usr/local/bin:/usr/bin:/bin
EnvironmentFile=-/home/pi/voldemorbot/platform/robot/.env
Environment=ROS_DOMAIN_ID=0
ExecStart=/home/pi/.pixi/bin/pixi run -e vision ros2 launch voldemorbot_robot rpi5_nodes.launch.py
Restart=on-failure
RestartSec=5
StartLimitBurst=3
StartLimitIntervalSec=60
ExecStop=/usr/bin/kill -SIGINT $MAINPID
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
```

### 7c. `voldemorbot-lidar.service`

```ini
[Unit]
Description=Voldemorbot — RPLiDAR C1
Wants=dev-ttyUSB0.device
After=dev-ttyUSB0.device

[Service]
Type=exec
User=pi
Group=pi
Environment=PATH=/home/pi/.pixi/bin:/usr/local/bin:/usr/bin:/bin
Environment=ROS_DOMAIN_ID=0
ExecStart=/home/pi/.pixi/bin/pixi run -e vision ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0
Restart=on-failure
RestartSec=3

[Install]
WantedBy=voldemorbot-pi5.service
```

### 7d. `voldemorbot-backend.service` (optional, runs on dev machine or Pi 5)

```ini
[Unit]
Description=Voldemorbot — Go Telemetry Backend
Wants=network-online.target
After=network-online.target

[Service]
Type=exec
User=pi
Group=pi
WorkingDirectory=/home/pi/voldemorbot/platform/backend
ExecStart=/home/pi/voldemorbot/platform/backend/bin/server
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

## 8. Udev Rules (`99-voldemorbot-gpio.rules`)

```udev
# GPIO — group gpio
SUBSYSTEM=="gpio*", KERNEL=="gpiochip*", ACTION=="add", \
    PROGRAM="/bin/sh -c 'chown root:gpio /dev/gpiochip* && chmod 660 /dev/gpiochip*'"

# PWM for servo + motor
SUBSYSTEM=="pwm*", ACTION=="add", \
    PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm'"

# I2C for OLED
SUBSYSTEM=="i2c-dev", MODE="0666"

# LiDAR serial
KERNEL=="ttyUSB[0-9]*", MODE="0660", GROUP="dialout"
```

## 9. `.env` Additions (Pi Zero)

```ini
# ============================================================
# PUSH BUTTON (GPIO)
# ============================================================
BUTTON_GPIO_PIN=4
BUTTON_PULL_UP=True
BUTTON_DEBOUNCE_MS=50
BUTTON_LONG_PRESS_SEC=2.0

# ============================================================
# SERVO STEERING (parallel/crab mechanism, single servo)
# ============================================================
SERVO_GPIO_PIN=12
SERVO_MIN_PULSE_US=500
SERVO_MAX_PULSE_US=2500
SERVO_RANGE_DEG=180
SERVO_CENTER_PULSE_US=1500
SERVO_REVERSED=false

# ============================================================
# DC ENCODER MOTOR (INJORA 180 + L298N + quadrature encoder)
# ============================================================
MOTOR_PWM_PIN=13
MOTOR_IN3_PIN=5
MOTOR_IN4_PIN=6
MOTOR_ENCODER_A_PIN=16
MOTOR_ENCODER_B_PIN=20
MOTOR_COUNTS_PER_REV=194.0
MOTOR_WHEEL_DIAMETER_M=0.056
MOTOR_MAX_RPM=1590.0
MOTOR_REVERSED=false

# ============================================================
# OLED DISPLAY (I2C on Pi Zero)
# ============================================================
DISPLAY_WIDTH=128
DISPLAY_HEIGHT=64
DISPLAY_I2C_ADDRESS=0x3C
DISPLAY_I2C_BUS=1
```

## 10. Provisioning Scripts

### `setup_pi_zero.sh`

```
create --device /dev/sdX [--wifi-ssid "..." --wifi-password "..."]:
  1. Flash RPi OS Lite (32-bit Bookworm) to SD
  2. /boot/config.txt:
       dtparam=i2c_arm=on
       dtoverlay=dwc2
       enable_uart=1
       dtoverlay=pwm-2chan
  3. /boot/cmdline.txt: modules-load=dwc2,g_ether
  4. touch /boot/ssh, hostname=ralvarezdev-raspberrypi-zero
  5. wpa_supplicant.conf (if --wifi-ssid)
  6. Copy firstboot.sh to /boot/
  7. Eject, boot Pi Zero

provision (firstboot.sh, runs once via SSH):
  1. raspi-config --expand-rootfs
  2. apt update && apt full-upgrade -y
  3. usermod -aG gpio,i2c,spi,dialout pi
  4. Install: pixi (pixi.sh), git, build-essential, pigpio, i2c-tools, python3-pip
  5. Verify PWM: ls /sys/class/pwm/pwmchip0/
  6. git clone ~/voldemorbot
  7. cd ~/voldemorbot/platform/robot && pixi install && pixi run -e dev build-ws
  8. cp .env.example .env
  9. cp systemd/voldemorbot-pi-zero.service /etc/systemd/system/
  10. cp udev/99-voldemorbot-gpio.rules /etc/udev/rules.d/
  11. systemctl enable voldemorbot-pi-zero.service
  12. udevadm control --reload-rules && udevadm trigger
  13. rm /boot/firstboot.sh && reboot
```

### `setup_pi_5.sh`

```
provision [--hailo-deb /path/to/hailort.deb]:
  1. apt update && apt full-upgrade -y
  2. raspi-config nonint: do_camera 0, do_i2c 0, do_spi 0, do_serial 2
  3. usermod -aG gpio,i2c,spi,dialout pi
  4. Install: pixi, git, build-essential, i2c-tools, network-manager, rpi-usb-gadget
  5. dpkg -i hailort*.deb && pip install libs/linux_aarch64/hailort-*.whl
     systemctl enable hailort
  6. nmcli con add type ethernet ifname usb0 ipv4.method manual \
       ipv4.addresses 192.168.250.2/24 connection.id usb-gadget
  7. git clone ~/voldemorbot
  8. cd ~/voldemorbot/platform/robot && pixi install && pixi run -e dev build-ws
  9. cp .env.example .env
  10. cp systemd/voldemorbot-pi5.service /etc/systemd/system/
      cp systemd/voldemorbot-lidar.service /etc/systemd/system/
  11. systemctl enable voldemorbot-lidar.service voldemorbot-pi5.service
  12. reboot
```

## 11. Boot Chain

```
Power On
  │
  ├── Pi Zero (~5s to ready)
  │    1. Kernel: GPIO, I2C, PWM
  │    2. g_ether: usb0 @ 192.168.250.1
  │    3. voldemorbot-pi-zero.service
  │       ├── button_node         → /button/event
  │       ├── ackermann_motor     → /motor/*  (sub: /ackermann_cmd)
  │       └── oled_display        → /ui/oled_mirror (sub: /robot_state, etc.)
  │
  └── Pi 5 (~15s to ready)
       1. Kernel + USB: ttyUSB0, ttyACM0
       2. g_ether: usb0 @ 192.168.250.2
       3. hailort.service (firmware load)
       4. voldemorbot-lidar.service → /scan
       5. voldemorbot-pi5.service
          ├── bno08x_uart_rvc    → /imu/data
          ├── vision_node        → /hailo/detections, /hailo/fps
          ├── state_machine      → /robot_state, /ackermann_cmd, /system_status
          └── telemetry_bridge   → HTTP/gRPC to backend

ROS2 DDS Domain 0 (auto-discovery over usb0):
  Pi 5 publishes:   /ackermann_cmd, /robot_state, /system_status, /race_metrics,
                    /imu/data, /scan, /hailo/detections, /hailo/fps
  Pi Zero publishes: /button/event, /motor/steering_position, /motor/drive_speed,
                     /motor/status, /motor/odometry, /ui/oled_mirror
```

## 12. Taskfile Integration

Add to root `Taskfile.yml`:

```yaml
# ——— Provisioning ———

rpi:setup:pi-zero:create:
  desc: "Flash SD card for Pi Zero (DEVICE=/dev/sdX, WIFI_SSID=..., WIFI_PASS=...)"
  cmds:
    - bash scripts/setup_pi_zero.sh create --device {{.DEVICE}} --wifi-ssid "{{.WIFI_SSID}}" --wifi-password "{{.WIFI_PASS}}"
  requires:
    vars: [DEVICE]

rpi:setup:pi-zero:provision:
  desc: "Run first-boot provisioning on Pi Zero via SSH"
  cmds:
    - scp scripts/setup_pi_zero.sh {{.RPI_ZERO_SSH}}:/tmp/
    - ssh {{.RPI_ZERO_SSH}} "bash /tmp/setup_pi_zero.sh provision && rm /tmp/setup_pi_zero.sh"

rpi:setup:pi5:
  desc: "Provision Pi 5 via SSH (SSH_HOST=rpi-5-local)"
  cmds:
    - scp scripts/setup_pi_5.sh {{.SSH_HOST}}:/tmp/
    - ssh {{.SSH_HOST}} "bash /tmp/setup_pi_5.sh && rm /tmp/setup_pi_5.sh"

# ——— Services ———

rpi:services:install:
  desc: "Copy systemd + udev files to target Pi (SSH_HOST= alias)"
  cmds:
    - scp platform/robot/systemd/*.service {{.SSH_HOST}}:/tmp/
    - scp platform/robot/udev/99-voldemorbot-gpio.rules {{.SSH_HOST}}:/tmp/
    - ssh {{.SSH_HOST}} "sudo mv /tmp/*.service /etc/systemd/system/
          && sudo mv /tmp/*.rules /etc/udev/rules.d/
          && sudo systemctl daemon-reload
          && sudo udevadm control --reload-rules && sudo udevadm trigger"

rpi:services:enable:
  cmds:
    - ssh {{.SSH_HOST}} "sudo systemctl enable voldemorbot-pi5.service voldemorbot-lidar.service"

rpi-zero:services:enable:
  cmds:
    - ssh {{.RPI_ZERO_SSH}} "sudo systemctl enable voldemorbot-pi-zero.service"

rpi:services:status:
  cmds:
    - ssh {{.SSH_HOST}} "for s in voldemorbot-pi5 voldemorbot-lidar hailort; do echo === \$s ===; systemctl status \$s --no-pager -l | head -8; echo; done"

rpi-zero:services:status:
  cmds:
    - ssh {{.RPI_ZERO_SSH}} "systemctl status voldemorbot-pi-zero --no-pager -l | head -15"

rpi:services:logs:
  desc: "Tail logs for a service (SERVICE=name)"
  cmds:
    - ssh {{.SSH_HOST}} "journalctl -u {{.SERVICE}} -n 50 --no-pager -f"

# ——— Startup ———

rpi:startup:
  desc: "Full startup: provision both Pis, install/enable services"
  cmds:
    - task: rpi:setup:pi5
    - task: rpi:setup:pi-zero:provision
    - echo "Both Pis provisioned. Reboot both to start services automatically."
```

## 13. Summary of All Changes

| Category | File | Action |
|---|---|---|
| **New** | `scripts/setup_common.sh` | Create — shared helpers |
| **New** | `scripts/setup_pi_zero.sh` | Create — SD creation + provisioning |
| **New** | `scripts/setup_pi_5.sh` | Create — Pi 5 provisioning |
| **New** | `platform/robot/systemd/voldemorbot-pi5.service` | Create |
| **New** | `platform/robot/systemd/voldemorbot-pi-zero.service` | Create |
| **New** | `platform/robot/systemd/voldemorbot-lidar.service` | Create |
| **New** | `platform/robot/systemd/voldemorbot-backend.service` | Create (optional) |
| **New** | `platform/robot/udev/99-voldemorbot-gpio.rules` | Create |
| **New** | `platform/robot/src/hardware/motors/servo/__init__.py` | Create |
| **New** | `platform/robot/src/hardware/motors/servo/driver.py` | Create |
| **New** | `platform/robot/src/hardware/motors/servo/config.py` | Create |
| **New** | `ros2_ws/.../voldemorbot_robot/button_node.py` | Create |
| **Modify** | `ros2_ws/.../setup.py` | Add `button_node` entry point |
| **Modify** | `ros2_ws/.../launch/rpi_zero_nodes.launch.py` | Add button_node + oled_display_node |
| **Modify** | `ros2_ws/.../launch/rpi5_nodes.launch.py` | Add IMU + vision, remove button |
| **Modify** | `ros2_ws/.../motors/ackermann_motor_node.py` | Swap Build HAT → servo + dc_encoder |
| **Modify** | `ros2_ws/.../state_machine_node.py` | Sub `/button/event` instead of GPIO |
| **Modify** | `platform/robot/.env.example` | Add servo + motor GPIO vars |
| **Modify** | `Taskfile.yml` | Add setup/service tasks |
| **Modify** | `fleet/Taskfile.yml` | Add provision shortcuts |
| **Unchanged** | `src/hardware/motors/dc_encoder/driver.py` | Already correct — pass `standby_pin=None` |
| **Unchanged** | `src/hardware/motors/dc_encoder/control.py` | Already correct |

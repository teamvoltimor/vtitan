# Sensor / Actuator Wiring Reference

What is physically wired where: each sensor and actuator, its board, its bus or pin,
and the ROS topic it lands on. The verification order, the bugs that were found, and
the validation methods are not repeated here.

Rationale lives in the ADRs: `adr:0082-wiring-harness-as-code` (the harness is defined
as code), `adr:0080-lidar-mount-and-scan-plane` (LIDAR mount and scan plane),
`adr:0069-config-governance` (TOML values and schemas). The full netlist is
`schemes/wiring/tscircuit/harness.netlist.txt`, rendered to
`schemes/wiring/harness.schematic.svg` / `.png`; motor-specific wiring is in
`src/python/docs/bts7960-ibt2-wiring.md` and `adr:0076-drivetrain-and-steering-hardware`.

## Devices and topics

| Device | Board | Interface | ROS topic(s) | Notes |
|---|---|---|---|---|
| Slamtec RPLIDAR C1 | Pi 5 | USB via the bundled Slamtec adapter | `/scan` | `/dev/ttyUSB0`, 460800 baud, Standard mode; mounted inverted |
| BNO08x IMU | Pi 5 | MCP2221A USB to UART bridge | `/imu/data` | `/dev/ttyACM0`, 115200 baud, 100 Hz |
| Camera Module 3 Wide | Pi 5 | CSI | `/camera/image_raw`, `/vision/detections` | |
| Hailo-8 AI HAT+ 26 TOPS | Pi 5 | 26-pin header | `/vision/detections` | no `/hailo/fps` topic is published |
| Push button | Pi Zero | GPIO4 | `/button/event`, `/button/hold` | |
| SSD1306 OLED | Pi Zero | I2C1 (SDA1/SCL1), address 0x3C | `/ui/oled_mirror`, `/ui/telemetry_summary` | |
| Challenge-mode jumper | Pi Zero | GPIO23 | `/challenge_mode/jumper_inserted`, `/challenge_mode/active` | |
| Steering servo (Hiwonder HPS-3527SG) | Pi Zero | GPIO12, hardware PWM0 | `/motor/steering_position` | |
| Drive motor (REV HD Hex 6000rpm) + encoder | Pi Zero | BTS7960/IBT-2, quadrature encoder | `/motor/drive_speed`, `/motor/status` | see pin table |
| Pi Zero to Pi 5 link | both | USB gadget (`usb0`, `192.168.250.1`) | cross-board ROS2 graph | |
| Battery / power switch | both | 3S LiPo, switch SW1 | - | see power rails |

## Pi Zero GPIO map

| Pin | Signal |
|---|---|
| GPIO4 | Button signal |
| GPIO5 | BTS7960 `L_EN` (held permanently HIGH) |
| GPIO6 | BTS7960 `R_EN` (held permanently HIGH) |
| GPIO12 | Servo signal (hardware PWM0) |
| GPIO13 | BTS7960 `RPWM`, forward (hardware PWM1) |
| GPIO16 | Motor encoder A |
| GPIO20 | Motor encoder B |
| GPIO23 | Challenge-mode jumper signal |
| GPIO26 | BTS7960 `LPWM`, reverse (software PWM) |
| SDA1 / SCL1 | OLED I2C |

## Pi 5 connections

| Port | Device |
|---|---|
| USB (Zero) | Pi Zero USB-gadget link |
| USB | RPLIDAR C1 via the Slamtec adapter |
| USB | MCP2221A IMU bridge |
| CSI | Camera Module 3 Wide |
| 26-pin header | Hailo-8 AI HAT+ |

## Power rails

| Rail | Source | Load |
|---|---|---|
| Battery | Ovonic 3S LiPo 11.1V | switch SW1 |
| 5V / 5A | KL89576 DCDC | Pi 5 (USB-C power) |
| 5V (Pi USB) | Pi 5 USB | Pi Zero, BTS7960 `VCC`, level-shifter HV |
| 3V3 | Pi Zero 3V3 | level-shifter LV, OLED `VCC`, motor encoder `ENC_VCC` |
| Servo rail | Mini-560 Pro regulator | servo `VCC` |
| Drive rail | battery via SW1 | BTS7960 `B+` |

## Operational notes

- The LIDAR is mounted inverted (`lidar.inverted = true`); the mount facts and the
  angle correction are owned by `adr:0080-lidar-mount-and-scan-plane`.
- Self-occlusion from the robot's own chassis reads as small clusters concentrated
  around the rear, measured spanning about +117 to +172 deg on one side and -122 to
  -177 deg on the other. Mask the whole rearward extent with margin
  (`abs(angle) > 115 deg`) rather than masking each cluster, since the elements are
  thin (wiring, brackets, screw heads) and clip differently on a re-scan.

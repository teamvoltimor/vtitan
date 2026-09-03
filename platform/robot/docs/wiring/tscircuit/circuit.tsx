// vtitan robot wiring harness, ported from the WireViz draft.
// Pin/signal sources (do not restate literals elsewhere -- update these files
// instead and re-run `tsci build`):
//   config/hardware/motors/servo.toml         -> servo PWM: GPIO12
//   config/hardware/motors/bts7960.toml       -> RPWM GPIO13, LPWM GPIO26, R_EN GPIO6, L_EN GPIO5
//   config/hardware/motors/encoder.toml       -> quadrature pin_a GPIO16, pin_b GPIO20
//   config/hardware/button/gpio.toml          -> button GPIO4, pull-up, GND-to-pin
//   config/hardware/challenge_mode.toml       -> challenge-mode jumper GPIO23, pull-up,
//                                                 GND-to-pin, ON THE ZERO
//   config/hardware/display/oled_node.toml    -> SSD1306 over I2C1 (blinka backend)
//   docs/bts7960-ibt2-wiring.md               -> RPWM/LPWM independence rationale,
//                                                 R_EN/L_EN held permanently HIGH,
//                                                 R_IS/L_IS deliberately unwired (no ADC)
//   docs/challenge-mode-jumper-spec.md        -> jumper polarity + boot-time read
//
// CONFLICT, resolved in favour of the TOML: challenge-mode-jumper-spec.md says
// the GPIO23 jumper is on the "Pi 5"; challenge_mode.toml says "Wired to the PI
// ZERO; this file matters on the Zero's checkout, not Pi 5's". The TOML is the
// narrower and later statement, and every other GPIO in this drawing is the
// Zero's, so JUMPER hangs off PI below. Worth a physical confirmation.
//
// Power tree (as described by the user, not derived from repo docs -- the
// repo has no power/harness documentation of its own):
//   Ovonic 3S LiPo (11.1V nominal, 50C; 2200mAh and 3000mAh packs both used,
//   interchangeably -- BATT below is whichever is in the robot that day)
//     -> SW1 (inline power switch)
//     -> SPLIT_MAIN (Y-split)
//          -> DCDC_5V5A (12/24V -> 5V@5A buck, USB-C output) -> Pi 5
//          -> SPLIT_DRIVE (Y-split)
//               -> U_REG (Mini-560 Pro, 5V) -> SERVO (Hiwonder)
//               -> U_MOTOR (BTS7960) B_POS/B_NEG (motor supply)
//
// Board split: the Pi Zero 2 W owns motor/servo/button/encoder/OLED. The
// Pi 5 owns the Hailo AI+ 26 TOPS module (its header/PCIe slot), the camera
// (Pi Camera Module 3 Wide, CSI), the LIDAR (RPLIDAR Slamtec C1, via a
// USB-to-UART adapter -- NOT direct USB), the MCP2221A/BNO085 IMU bridge,
// and its own 5V@5A USB-C power feed.
//
// Encoder runs entirely at 3.3V off the Zero: ENC_VCC on the Zero's 3V3 rail,
// ENC_A/ENC_B straight to GPIO16/20. Nothing on this leg crosses a voltage
// boundary, so it stays clear of the level converter. (Stated by the user
// 2026-09-03; the repo doesn't record the encoder's supply voltage --
// config/hardware/motors/encoder.toml is pins and PID gains only.)
//
// Pi 5 <-> Pi Zero link: USB gadget Ethernet (g_ether/cdc_ether, usb0,
// 192.168.250.1), confirmed in docs/sensor-verification.md and
// docs/dds-shm-transport-disabled.md -- carries all inter-board ROS2 traffic.
// That one cable also POWERS the Zero: sensor-verification.md's 2026-07-25
// USB-link investigation notes the Zero "drawing its power through the same
// micro-USB port carrying the data" (vcgencmd get_throttled = 0x0, no
// undervoltage). So it is broken out below into its three electrically
// distinct conductors -- VBUS, GND, and the D+/D- data pair as one logical
// line -- rather than a single abstract link, because VBUS is what sources
// the Zero's whole 5V rail (U_MOTOR VCC, U_LVL HV, OLED, encoder).
//
// OPEN QUESTION, deliberately not resolved here: this is the DEV power path.
// Nothing in the repo documents how the Zero is powered during a competition
// run, when it is not tethered to a laptop-side Pi 5 arrangement. If the Zero
// gets its own regulator off the battery for racing, that is a second, as-yet
// undrawn feed into PI's 5V rail.
//
// BTS7960 IN/EN pins are 5V logic; Pi Zero GPIO is 3.3V -- every signal that
// crosses that boundary routes through the level converter (U_LVL), not
// directly to GPIO.
//
// Regulator grounds: DCDC_5V5A and U_REG are both non-isolated buck
// converters, so VIN_NEG and VOUT_NEG are the same physical node inside each
// module. That tie is drawn explicitly below -- without it the servo's GPIO12
// signal has no return path to the Zero, and the Pi 5 has no reference to the
// battery at all.
//
// NOT modeled: GPIO pull-up/pull-down config from /boot/firmware/config.txt
// (e.g. the `gpio=5,6,26=op,dl` boot-float fix), the button's
// `pull_up = true` in gpio.toml, and the jumper's internal pull-up. Those are
// internal Broadcom SoC pad configuration, not physical components or wires
// -- there's nothing to draw. The GND-to-pin wiring of the button and the
// jumper (both shown) is the only part of that story that's a real physical
// connection.
//
// R_IS/L_IS on the H-bridge are left unconnected ON PURPOSE, not by omission:
// they are analog current-sense outputs, the Pi has no ADC, and no external
// converter is on hand (docs/bts7960-ibt2-wiring.md, "Not wired").

export default () => (
  <board width="320mm" height="260mm">
    {/* ---- Power ---- */}
    {/* Ovonic 3S LiPo, 11.1V nominal, 50C. Both the 2200mAh and 3000mAh
        packs are used interchangeably -- same connector, same wiring. */}
    <chip
      name="BATT"
      manufacturerPartNumber="Ovonic 3S LiPo 11.1V (2200mAh or 3000mAh)"
      footprint="pinrow2"
      pinLabels={{ pin1: "BATT_POS", pin2: "BATT_NEG" }}
      schX={-18}
      schY={0}
    />

    {/* Inline power switch on the positive lead, ahead of the first split. */}
    <chip
      name="SW1"
      footprint="pinrow2"
      pinLabels={{ pin1: "IN", pin2: "OUT" }}
      schX={-14}
      schY={0}
    />

    {/* Y-split: one leg to the Pi 5's 5V buck, the other to a second split
        feeding the servo regulator and the drive H-bridge. */}
    <chip
      name="SPLIT_MAIN"
      footprint="pinrow6"
      pinLabels={{
        pin1: "IN_POS",
        pin2: "IN_NEG",
        pin3: "OUT1_POS",
        pin4: "OUT1_NEG",
        pin5: "OUT2_POS",
        pin6: "OUT2_NEG",
      }}
      schX={-9}
      schY={0}
    />

    {/* 12/24V -> 5V@5A buck, USB-C output, dedicated to the Pi 5. */}
    <chip
      name="DCDC_5V5A"
      manufacturerPartNumber="KL89576"
      footprint="pinrow4"
      pinLabels={{ pin1: "VIN_POS", pin2: "VIN_NEG", pin3: "VOUT_POS", pin4: "VOUT_NEG" }}
      schX={-9}
      schY={8}
    />

    {/* Second Y-split: servo regulator + drive H-bridge share the battery rail. */}
    <chip
      name="SPLIT_DRIVE"
      footprint="pinrow6"
      pinLabels={{
        pin1: "IN_POS",
        pin2: "IN_NEG",
        pin3: "OUT1_POS",
        pin4: "OUT1_NEG",
        pin5: "OUT2_POS",
        pin6: "OUT2_NEG",
      }}
      schX={-4}
      schY={-5}
    />

    {/* Buck regulator feeding the steering servo's own supply rail --
        keeps servo current spikes off the Pi's 5V rail. */}
    <chip
      name="U_REG"
      manufacturerPartNumber="Mini-560 Pro"
      footprint="pinrow4"
      pinLabels={{ pin1: "VIN_POS", pin2: "VIN_NEG", pin3: "VOUT_POS", pin4: "VOUT_NEG" }}
      schX={1}
      schY={-2}
    />

    <chip
      name="PI"
      manufacturerPartNumber="Raspberry Pi Zero 2 W"
      footprint="pinrow16"
      pinLabels={{
        // Header 5V pin. Physically the SAME rail as USB_OTG_VBUS below --
        // the Zero has no separate regulator between them -- but kept as its
        // own pin because they are different physical points to probe or
        // tap: VBUS is the micro-USB connector, 5V is header pin 2/4.
        pin1: "5V",
        pin2: "3V3",
        pin3: "GND",
        pin4: "GPIO4",
        pin5: "GPIO5",
        pin6: "GPIO6",
        pin7: "GPIO12",
        pin8: "GPIO13",
        pin9: "GPIO16",
        pin10: "GPIO20",
        pin11: "GPIO23",
        pin12: "GPIO26",
        pin13: "SDA1",
        pin14: "SCL1",
        // The Zero's USB/OTG micro-USB connector, split into its two roles:
        // the gadget-Ethernet data pair, and the VBUS conductor that actually
        // powers the board. The Zero's OTHER micro-USB port (PWR IN) is
        // unused in this build -- see the race-power open question above.
        pin15: "USB_OTG_D",
        pin16: "USB_OTG_VBUS",
      }}
      schX={0}
      schY={-11}
    />

    {/* Pi 5: Hailo AI+ 26 TOPS on the header/PCIe slot, camera on CSI, LIDAR
        via a USB-to-UART adapter, MCP2221A/IMU bridge on its own USB port,
        link to the Zero over a fourth USB port (gadget Ethernet, also
        bus-powers the Zero during dev), and its own 5V@5A USB-C feed. */}
    <chip
      name="PI5"
      manufacturerPartNumber="Raspberry Pi 5"
      footprint="pinrow9"
      pinLabels={{
        pin1: "USB_ZERO_D",
        pin2: "USB_ZERO_VBUS",
        // One pin for the Pi 5's USB ground: all four ports share it on the
        // board, and it is what references the LIDAR adapter, the IMU bridge
        // and the whole Zero side to the battery.
        pin3: "USB_GND",
        pin4: "USB_LIDAR_ADAPTER",
        pin5: "USB_IMU_BRIDGE",
        pin6: "CSI",
        pin7: "HEADER",
        pin8: "PWR_USBC_POS",
        pin9: "PWR_USBC_NEG",
      }}
      schX={-9}
      schY={16}
    />

    {/* Hailo-8 AI+ 26 TOPS module on the Pi 5's header/PCIe slot. */}
    <chip
      name="HAILO"
      manufacturerPartNumber="Hailo-8 (AI HAT+ 26 TOPS)"
      footprint="pinrow2"
      pinLabels={{ pin1: "HEADER", pin2: "NC" }}
      schX={-18}
      schY={16}
    />

    <chip
      name="CAMERA"
      manufacturerPartNumber="Raspberry Pi Camera Module 3 Wide"
      footprint="pinrow2"
      pinLabels={{ pin1: "CSI", pin2: "NC" }}
      schX={-9}
      schY={23}
    />

    {/* USB-to-UART adapter between the Pi 5 and the LIDAR -- the C1 is NOT
        wired to USB directly in this build. This is the adapter SHIPPED WITH
        the C1, not a generic breakout, so its regulator is rated for the
        scanner's motor current by construction. */}
    <chip
      name="ADAPTER"
      manufacturerPartNumber="Slamtec USB adapter (bundled with RPLIDAR C1)"
      footprint="pinrow5"
      pinLabels={{ pin1: "USB", pin2: "VCC", pin3: "GND", pin4: "TX", pin5: "RX" }}
      schX={0}
      schY={16}
    />

    <chip
      name="LIDAR"
      manufacturerPartNumber="Slamtec RPLIDAR C1"
      footprint="pinrow4"
      pinLabels={{ pin1: "VCC", pin2: "GND", pin3: "RX", pin4: "TX" }}
      schX={0}
      schY={23}
    />

    {/* Bidirectional logic-level converter: BTS7960 IN/EN pins are 5V,
        Pi GPIO is 3.3V. LV side -> Pi, HV side -> BTS7960. 4 channels used:
        RPWM, LPWM, R_EN, L_EN. */}
    {/* Real part: a single 4-channel bidirectional logic-level converter
        breakout (4x HV + 4x LV, the MOSFET/BSS138 style with no output-enable
        pin -- NOT a TXS0108E, which is 8-channel and needs OE pulled high).
        ONE board on the car, and all four channels are spoken for: RPWM,
        LPWM, R_EN, L_EN. There is no spare channel here -- anything new that
        crosses the 3.3V/5V boundary needs a second converter. */}
    <chip
      name="U_LVL"
      footprint="pinrow11"
      pinLabels={{
        pin1: "HV",
        pin2: "LV",
        pin3: "GND",
        pin4: "HV1",
        pin5: "HV2",
        pin6: "HV3",
        pin7: "HV4",
        pin8: "LV1",
        pin9: "LV2",
        pin10: "LV3",
        pin11: "LV4",
      }}
      schX={10}
      schY={-11}
    />

    {/* Drive H-bridge. Pin roles and the RPWM/LPWM independence rationale
        come from config/hardware/motors/bts7960.toml + the wiring doc. */}
    <chip
      name="U_MOTOR"
      manufacturerPartNumber="BTS7960 / IBT-2"
      footprint="pinrow12"
      pinLabels={{
        pin1: "RPWM",
        pin2: "LPWM",
        pin3: "R_EN",
        pin4: "L_EN",
        pin5: "R_IS",
        pin6: "L_IS",
        pin7: "VCC",
        pin8: "GND",
        pin9: "B_POS",
        pin10: "B_NEG",
        pin11: "M_POS",
        pin12: "M_NEG",
      }}
      schX={17}
      schY={-11}
    />

    {/* Drive motor with integrated quadrature encoder. CORRECTED 2026-09-03:
        this said "JGB37-520", which is the RETIRED motor. The active hardware
        profile is rev-hd-hex-motor-6000rpm, whose encoder.toml opens "the REV
        HD Hex 6000rpm drive motor, in service since the 2026-08-27 motor
        swap". Calibration (counts_per_rev 60, max_rpm 348) lives there, not
        in the base config/hardware/motors/encoder.toml. */}
    <chip
      name="MOTOR"
      manufacturerPartNumber="REV HD Hex 6000rpm"
      footprint="pinrow6"
      pinLabels={{
        pin1: "M_POS",
        pin2: "M_NEG",
        pin3: "ENC_VCC",
        pin4: "ENC_GND",
        pin5: "ENC_A",
        pin6: "ENC_B",
      }}
      schX={24}
      schY={-11}
    />

    {/* Steering servo, 270deg / 35kg-cm. Hardware profile
        270deg-hiwonder-35kg sets range_deg=270 but leaves the pulse span at
        the base file's 180deg values (500-2500us) with a TODO -- whether this
        part keeps that span is unverified against its datasheet. */}
    <chip
      name="SERVO"
      manufacturerPartNumber="Hiwonder HPS-3527SG"
      footprint="pinrow3"
      pinLabels={{ pin1: "SIGNAL", pin2: "VCC", pin3: "GND" }}
      schX={10}
      schY={-2}
    />

    <chip
      name="BUTTON"
      footprint="pinrow2"
      pinLabels={{ pin1: "SIGNAL", pin2: "GND" }}
      schX={-9}
      schY={-11}
    />

    {/* Challenge-mode select: a bare 2-pin jumper cap, GPIO23 to an adjacent
        GND pin (physical 16 to 14 or 20). Inserted = shorted LOW = Obstacle
        Challenge; absent = internal pull-up HIGH = Open Challenge. Both
        states are determinate by construction -- see
        docs/challenge-mode-jumper-spec.md. Read once at BOOT_CHECK. */}
    <chip
      name="JUMPER"
      footprint="pinrow2"
      pinLabels={{ pin1: "SIGNAL", pin2: "GND" }}
      schX={-9}
      schY={-16}
    />

    {/* 128x64 mono OLED, I2C1 @ 0x3C, per config/hardware/display/
        ssd1306.toml. Runs off the Zero's 3V3 rail. */}
    <chip
      name="OLED"
      manufacturerPartNumber="SSD1306 128x64"
      footprint="pinrow4"
      pinLabels={{ pin1: "VCC", pin2: "GND", pin3: "SDA", pin4: "SCL" }}
      schX={-18}
      schY={-11}
    />

    {/* USB-to-UART bridge for the IMU, hosted off the Pi 5's USB, not the
        Zero's. Backend confirmed by config/hardware/imu/
        bno08x_mcp2221_uart_rvc.toml (VID 0x04D8 / PID 0x00DD), which is the
        actively deployed IMU path over the two I2C alternatives. */}
    <chip
      name="MCP2221A"
      manufacturerPartNumber="MCP2221A"
      footprint="pinrow5"
      pinLabels={{ pin1: "USB", pin2: "VCC", pin3: "GND", pin4: "TX", pin5: "RX" }}
      schX={10}
      schY={16}
    />

    {/* 9-DoF IMU, run in UART-RVC mode (115200 baud) off the bridge above. */}
    <chip
      name="BNO085"
      manufacturerPartNumber="BNO085"
      footprint="pinrow4"
      pinLabels={{ pin1: "VCC", pin2: "GND", pin3: "RX", pin4: "TX" }}
      schX={10}
      schY={23}
    />

    {/* ---- Power distribution ---- */}
    <trace from=".BATT > .BATT_POS" to=".SW1 > .IN" />
    <trace from=".SW1 > .OUT" to=".SPLIT_MAIN > .IN_POS" />
    <trace from=".BATT > .BATT_NEG" to=".SPLIT_MAIN > .IN_NEG" />

    {/* Both SPLIT_* parts are Y-cable splices, not active devices: each
        IN conductor is physically the SAME node as both of its OUT
        conductors. Drawn explicitly, because without it the battery rails
        stop dead at the splitters and every downstream "net" is orphaned.
        SW1 is the one part whose IN/OUT are deliberately NOT tied -- that
        open pair IS the switch contact. */}
    <trace from=".SPLIT_MAIN > .IN_POS" to=".SPLIT_MAIN > .OUT1_POS" />
    <trace from=".SPLIT_MAIN > .IN_POS" to=".SPLIT_MAIN > .OUT2_POS" />
    <trace from=".SPLIT_MAIN > .IN_NEG" to=".SPLIT_MAIN > .OUT1_NEG" />
    <trace from=".SPLIT_MAIN > .IN_NEG" to=".SPLIT_MAIN > .OUT2_NEG" />

    <trace from=".SPLIT_DRIVE > .IN_POS" to=".SPLIT_DRIVE > .OUT1_POS" />
    <trace from=".SPLIT_DRIVE > .IN_POS" to=".SPLIT_DRIVE > .OUT2_POS" />
    <trace from=".SPLIT_DRIVE > .IN_NEG" to=".SPLIT_DRIVE > .OUT1_NEG" />
    <trace from=".SPLIT_DRIVE > .IN_NEG" to=".SPLIT_DRIVE > .OUT2_NEG" />

    <trace from=".SPLIT_MAIN > .OUT1_POS" to=".DCDC_5V5A > .VIN_POS" />
    <trace from=".SPLIT_MAIN > .OUT1_NEG" to=".DCDC_5V5A > .VIN_NEG" />
    <trace from=".DCDC_5V5A > .VOUT_POS" to=".PI5 > .PWR_USBC_POS" />
    <trace from=".DCDC_5V5A > .VOUT_NEG" to=".PI5 > .PWR_USBC_NEG" />
    {/* Non-isolated buck: input and output ground are one node internally.
        This is what references the Pi 5 to the battery at all. */}
    <trace from=".DCDC_5V5A > .VIN_NEG" to=".DCDC_5V5A > .VOUT_NEG" />

    <trace from=".SPLIT_MAIN > .OUT2_POS" to=".SPLIT_DRIVE > .IN_POS" />
    <trace from=".SPLIT_MAIN > .OUT2_NEG" to=".SPLIT_DRIVE > .IN_NEG" />

    <trace from=".SPLIT_DRIVE > .OUT1_POS" to=".U_REG > .VIN_POS" />
    <trace from=".SPLIT_DRIVE > .OUT1_NEG" to=".U_REG > .VIN_NEG" />
    <trace from=".U_REG > .VOUT_POS" to=".SERVO > .VCC" />
    <trace from=".U_REG > .VOUT_NEG" to=".SERVO > .GND" />
    {/* Mini-560 Pro is likewise non-isolated. Without this tie the servo's
        GPIO12 signal has no return path to the Zero. */}
    <trace from=".U_REG > .VIN_NEG" to=".U_REG > .VOUT_NEG" />

    <trace from=".SPLIT_DRIVE > .OUT2_POS" to=".U_MOTOR > .B_POS" />
    <trace from=".SPLIT_DRIVE > .OUT2_NEG" to=".U_MOTOR > .B_NEG" />

    {/* The Zero's whole 5V rail is USB VBUS off the Pi 5 -- see the header
        note. This is the only thing sourcing PI's 5V pin. */}
    <trace from=".PI5 > .USB_ZERO_VBUS" to=".PI > .USB_OTG_VBUS" />
    {/* Internal to the Zero: the USB connector's VBUS and the header's 5V pin
        are one rail, no regulator between them. Drawn so the connector and
        the header pin stay separately identifiable. */}
    <trace from=".PI > .USB_OTG_VBUS" to=".PI > .5V" />
    <trace from=".PI5 > .USB_GND" to=".PI > .GND" />
    {/* Pi 5 board ground: its USB ground and its USB-C power return are the
        same node, which is what puts the Zero, the LIDAR adapter and the IMU
        bridge on the battery's ground rather than three floating islands. */}
    <trace from=".PI5 > .USB_GND" to=".PI5 > .PWR_USBC_NEG" />

    <trace from=".PI > .5V" to=".U_MOTOR > .VCC" />
    <trace from=".PI > .5V" to=".U_LVL > .HV" />
    <trace from=".PI > .3V3" to=".U_LVL > .LV" />
    {/* SSD1306 module runs off the Zero's 3V3 rail, not 5V. */}
    <trace from=".PI > .3V3" to=".OLED > .VCC" />
    {/* Encoder supply is 3V3, not 5V -- that is what keeps ENC_A/ENC_B safe
        to land directly on GPIO16/20 with no level shifting. */}
    <trace from=".PI > .3V3" to=".MOTOR > .ENC_VCC" />

    <trace from=".PI > .GND" to=".U_LVL > .GND" />
    {/* H-bridge LOGIC ground -- distinct from B_NEG, which is the motor
        supply return. Without this the level-shifted RPWM/LPWM/R_EN/L_EN
        signals and the 5V VCC feed have no reference to the Zero. */}
    <trace from=".PI > .GND" to=".U_MOTOR > .GND" />
    <trace from=".PI > .GND" to=".BUTTON > .GND" />
    <trace from=".PI > .GND" to=".JUMPER > .GND" />
    <trace from=".PI > .GND" to=".OLED > .GND" />
    <trace from=".PI > .GND" to=".MOTOR > .ENC_GND" />
    <trace from=".BATT > .BATT_NEG" to=".PI > .GND" />

    {/* ---- Motor control, through the level converter ---- */}
    <trace from=".PI > .GPIO13" to=".U_LVL > .LV1" />
    <trace from=".U_LVL > .HV1" to=".U_MOTOR > .RPWM" />

    <trace from=".PI > .GPIO26" to=".U_LVL > .LV2" />
    <trace from=".U_LVL > .HV2" to=".U_MOTOR > .LPWM" />

    <trace from=".PI > .GPIO6" to=".U_LVL > .LV3" />
    <trace from=".U_LVL > .HV3" to=".U_MOTOR > .R_EN" />

    <trace from=".PI > .GPIO5" to=".U_LVL > .LV4" />
    <trace from=".U_LVL > .HV4" to=".U_MOTOR > .L_EN" />

    <trace from=".U_MOTOR > .M_POS" to=".MOTOR > .M_POS" />
    <trace from=".U_MOTOR > .M_NEG" to=".MOTOR > .M_NEG" />

    {/* ---- Encoder, straight to GPIO (3.3V native, no level shift needed) ---- */}
    <trace from=".PI > .GPIO16" to=".MOTOR > .ENC_A" />
    <trace from=".PI > .GPIO20" to=".MOTOR > .ENC_B" />

    {/* ---- Servo signal, straight to GPIO12 hardware PWM ---- */}
    <trace from=".PI > .GPIO12" to=".SERVO > .SIGNAL" />

    {/* ---- Button, GND-to-pin, start/E-STOP signal (see gpio.toml) ---- */}
    <trace from=".PI > .GPIO4" to=".BUTTON > .SIGNAL" />

    {/* ---- Challenge-mode jumper, GND-to-pin (see challenge_mode.toml) ---- */}
    <trace from=".PI > .GPIO23" to=".JUMPER > .SIGNAL" />

    {/* ---- OLED, I2C1 ---- */}
    <trace from=".PI > .SDA1" to=".OLED > .SDA" />
    <trace from=".PI > .SCL1" to=".OLED > .SCL" />

    {/* ---- Pi 5 peripherals ---- */}
    {/* Data half of the gadget-Ethernet cable; VBUS/GND halves of the same
        cable are drawn up in the power section. */}
    <trace from=".PI > .USB_OTG_D" to=".PI5 > .USB_ZERO_D" />
    <trace from=".PI5 > .HEADER" to=".HAILO > .HEADER" />
    <trace from=".PI5 > .CSI" to=".CAMERA > .CSI" />

    {/* ---- LIDAR, via USB-to-UART adapter (not direct USB) ---- */}
    <trace from=".PI5 > .USB_LIDAR_ADAPTER" to=".ADAPTER > .USB" />
    {/* The adapter's ground is the Pi 5's USB ground -- the LIDAR's return
        path runs back through the adapter to the host, not to a local net. */}
    <trace from=".PI5 > .USB_GND" to=".ADAPTER > .GND" />
    <trace from=".ADAPTER > .VCC" to=".LIDAR > .VCC" />
    <trace from=".ADAPTER > .GND" to=".LIDAR > .GND" />
    <trace from=".ADAPTER > .TX" to=".LIDAR > .RX" />
    <trace from=".ADAPTER > .RX" to=".LIDAR > .TX" />

    {/* ---- IMU bridge, hosted off the Pi 5's USB. VCC/GND/UART for the IMU
        all come from the bridge board itself, not straight off either Pi. ---- */}
    <trace from=".PI5 > .USB_IMU_BRIDGE" to=".MCP2221A > .USB" />
    <trace from=".PI5 > .USB_GND" to=".MCP2221A > .GND" />
    <trace from=".MCP2221A > .VCC" to=".BNO085 > .VCC" />
    <trace from=".MCP2221A > .GND" to=".BNO085 > .GND" />
    <trace from=".MCP2221A > .TX" to=".BNO085 > .RX" />
    <trace from=".MCP2221A > .RX" to=".BNO085 > .TX" />
  </board>
)

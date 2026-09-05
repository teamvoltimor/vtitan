// vtitan robot wiring harness, ported from the WireViz draft.
//
// THIS FILE IS THE ONLY TRACKED SOURCE. harness.netlist.txt, .svg and .png are
// gitignored build output -- regenerate all three with `npm run artifacts`
// (see package.json) after any change here, or they silently go stale and
// `git status` will NOT warn you, because git cannot see them.
//
// Pin/signal sources (do not restate literals elsewhere -- update these files
// instead and re-run `npm run build`):
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
// CONFIRMED by the user: the GPIO23 jumper is on the PI ZERO, matching
// challenge_mode.toml. docs/challenge-mode-jumper-spec.md used to say "Pi 5";
// that was the wrong board and has been corrected. Consistent with the rest of
// this drawing, where every other GPIO is the Zero's -- and with the spec's own
// no-conflict list (button=4, servo=12, motor PWM=13, IN3/IN4=5/6,
// encoders=16/20), all of which are Zero pins. JUMPER hangs off PI below.
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
// Parts identification is COMPLETE as of 2026-09-05. Every component that has
// a model number carries it as a manufacturerPartNumber prop. The three that
// do NOT -- SW1, U_LVL and BUTTON -- are generic unbranded parts with no model
// number to record (confirmed by the user); their blank prop is a finding, not
// a gap still being worked. Do not go looking for those three again.
//
// SW1 is rated 10 A / 125 V AC (user, 2026-09-05). Note the AC: this is an
// AC-rated switch used on an 11.1 V DC circuit, and an AC rating does not
// transfer to DC unchanged. The usual reason is arc quenching -- AC current
// crosses zero 100-120x/second and self-extinguishes the arc, DC does not, so
// AC-rated switches are conventionally derated hard for DC.
//
// That conventional derate is MILDER here than the rule of thumb implies,
// because it is mostly a high-voltage effect: sustaining an arc across
// separating contacts needs roughly 12-15 V, and this pack is 11.1 V nominal
// (12.6 V charged). The circuit sits right around the threshold, so break arcs
// should be weak-to-absent rather than the sustained arc that destroys an
// AC-rated switch at 125 V DC. Low voltage is doing the work here, not the
// switch's rating.
//
// So the real exposure is THERMAL CARRY and CONTACT EROSION, not arc-over:
//   - 10 A is a continuous-carry figure, and drive draw is ~10 A average at
//     max_duty = 0.5 with ~20 A instantaneous peaks -- at, or over, rating.
//   - Switch-on inrush into DCDC_5V5A's input capacitance is a make-arc /
//     contact-pitting event on every power-up, independent of drive current.
// None of this is settled: 125 V AC is the only rating on the part, and its DC
// carry rating is unknown. Treat the above as reasoning, not measurement.
//
// The 10 A figure makes SW1 the LOWEST-rated element
// in the power path by a wide margin: the BTS7960 is a 43 A part and a 3S 50C
// 2200mAh pack can deliver ~110 A, so the switch, not the H-bridge or the
// battery, is the weakest link in series with the drive branch.
//
// Whether 10 A is sufficient depends on drive-motor stall current, which THIS
// REPO DOES NOT RECORD anywhere -- there is no current figure in
// config/hardware/motors/ at all, only pins, duty and PID gains. The REV HD Hex
// datasheet figure is ~20 A stall at 12 V; that is from the part's datasheet,
// NOT from this repo, and should be confirmed against the actual motor before
// anyone relies on it. If it holds, then:
//   - max_duty = 0.5 (config/hardware/motors/encoder.toml:27) caps AVERAGE
//     current at roughly half stall, i.e. ~10 A -- right at the switch rating,
//     with instantaneous peaks at the full ~20 A each PWM cycle.
//   - Stall is a REAL operating condition on this robot, not a theoretical
//     one: wedging against a wall is a known, repeatedly observed failure mode.
// So the margin is thin-to-negative under a sustained wall push. This is NOT a
// call to change anything -- it is the number to have in hand if SW1 ever runs
// warm, pits or arcs on make/break, or the robot browns out under hard
// acceleration. Failure stays confined to SW1 (contacts welding closed, or the
// housing melting); it does not propagate into the rest of the tree.
//
// If it ever DOES need replacing, the spec to buy against is a DC rating at or
// above ~20 A at 12 V DC -- an automotive/marine-style switch quotes DC
// directly and removes the AC-to-DC guesswork above.
//
// SCHEMATIC LAYOUT: schX/schY are hand-placed in three bands -- Pi 5 and its
// sensors on top, the battery spine through the middle in supply order, the
// Zero and everything it drives along the bottom. Placement is driven by NET
// LABELS, not by the component boxes: tscircuit renders a label box on every
// pin, on both sides of a chip, so two chips in the same row need roughly 8
// units between centres or their labels collide while the boxes themselves sit
// nowhere near each other. Checking for box overlap alone will pass a drawing
// that is unreadable. Budget for the labels, and re-render to check.
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
// SETTLED by the user: USB VBUS off the Pi 5 is the Zero's ONLY 5V feed, at
// race time as well as during development -- there is no second regulator off
// the battery, so node 5 has exactly the one source drawn below. The Zero's
// other micro-USB port (PWR IN) stays unused. Note the consequence: the whole
// Zero side (U_MOTOR logic VCC, U_LVL HV, OLED, encoder) is powered through the
// Pi 5's USB port, and its budget is therefore that port's, not the battery's.
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
      schX={-20}
      schY={1}
    />

    {/* Inline power switch on the positive lead, ahead of the first split.
        GENERIC, unbranded part -- manufacturerPartNumber is left off on
        purpose, not by omission; there is no model number to record.
        Position matters more than identity here: SW1 is upstream of
        SPLIT_MAIN, so it is in series with BOTH the 5V@5A buck and the whole
        drive branch, and therefore carries drive-motor stall current -- the
        largest current in the build, and well above the running draw. Rated
        10 A / 125 V AC -- an AC rating on a DC circuit, and the lowest rating
        anywhere in the power path; see the
        margin analysis in the header block. */}
    <chip
      name="SW1"
      footprint="pinrow2"
      pinLabels={{ pin1: "IN", pin2: "OUT" }}
      schX={-14}
      schY={1}
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
      schX={-8}
      schY={1}
    />

    {/* 12/24V -> 5V@5A buck, USB-C output, dedicated to the Pi 5. */}
    <chip
      name="DCDC_5V5A"
      manufacturerPartNumber="KL89576"
      footprint="pinrow4"
      pinLabels={{ pin1: "VIN_POS", pin2: "VIN_NEG", pin3: "VOUT_POS", pin4: "VOUT_NEG" }}
      schX={-8}
      schY={4}
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
      schX={0}
      schY={1}
    />

    {/* Buck regulator feeding the steering servo's own supply rail --
        keeps servo current spikes off the Pi's 5V rail. */}
    <chip
      name="U_REG"
      manufacturerPartNumber="Mini-560 Pro"
      footprint="pinrow4"
      pinLabels={{ pin1: "VIN_POS", pin2: "VIN_NEG", pin3: "VOUT_POS", pin4: "VOUT_NEG" }}
      schX={8}
      schY={1}
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
        // unused in this build, at race time too -- see the power tree above.
        pin15: "USB_OTG_D",
        pin16: "USB_OTG_VBUS",
      }}
      schX={-12}
      schY={-7}
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
      schX={-13}
      schY={7}
    />

    {/* Hailo-8 AI+ 26 TOPS module on the Pi 5's header/PCIe slot. */}
    <chip
      name="HAILO"
      manufacturerPartNumber="Hailo-8 (AI HAT+ 26 TOPS)"
      footprint="pinrow2"
      pinLabels={{ pin1: "HEADER", pin2: "NC" }}
      schX={-20}
      schY={7}
    />

    <chip
      name="CAMERA"
      manufacturerPartNumber="Raspberry Pi Camera Module 3 Wide"
      footprint="pinrow2"
      pinLabels={{ pin1: "CSI", pin2: "NC" }}
      schX={-20}
      schY={12}
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
      schX={-4}
      schY={12}
    />

    <chip
      name="LIDAR"
      manufacturerPartNumber="Slamtec RPLIDAR C1"
      footprint="pinrow4"
      pinLabels={{ pin1: "VCC", pin2: "GND", pin3: "RX", pin4: "TX" }}
      schX={4}
      schY={12}
    />

    {/* Bidirectional logic-level converter: BTS7960 IN/EN pins are 5V,
        Pi GPIO is 3.3V. LV side -> Pi, HV side -> BTS7960. 4 channels used:
        RPWM, LPWM, R_EN, L_EN. */}
    {/* Real part: a single 4-channel bidirectional logic-level converter
        breakout (4x HV + 4x LV, the MOSFET/BSS138 style with no output-enable
        pin -- NOT a TXS0108E, which is 8-channel and needs OE pulled high).
        GENERIC, unbranded part -- manufacturerPartNumber is left off on
        purpose, not by omission; there is no model number to record. The
        MOSFET style is the slow one, and motor PWM (GPIO13 -> RPWM) crosses
        it, so if PWM ever looks rounded or weak at the H-bridge this part is
        a candidate before the H-bridge itself is suspected.
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
      schX={-3}
      schY={-7}
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
      schX={5}
      schY={-7}
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
      schX={13}
      schY={-7}
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
      schX={5}
      schY={-2}
    />

    <chip
      name="BUTTON"
      footprint="pinrow2"
      pinLabels={{ pin1: "SIGNAL", pin2: "GND" }}
      schX={-20}
      schY={-8}
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
      schX={-20}
      schY={-11}
    />

    {/* 128x64 mono OLED, I2C1 @ 0x3C, per config/hardware/display/
        ssd1306.toml. Runs off the Zero's 3V3 rail. */}
    <chip
      name="OLED"
      manufacturerPartNumber="SSD1306 128x64"
      footprint="pinrow4"
      pinLabels={{ pin1: "VCC", pin2: "GND", pin3: "SDA", pin4: "SCL" }}
      schX={-20}
      schY={-4}
    />

    {/* USB-to-UART bridge for the IMU, hosted off the Pi 5's USB, not the
        Zero's. Backend confirmed by config/hardware/imu/
        bno08x_mcp2221_uart_rvc.toml (VID 0x04D8 / PID 0x00DD), which is the
        actively deployed IMU path over the two I2C alternatives. */}
    <chip
      name="MCP2221A"
      manufacturerPartNumber="MCP2221A"
      footprint="pinrow5"
      pinLabels={{ pin1: "USB", pin2: "3V3", pin3: "GND", pin4: "TX", pin5: "RX" }}
      schX={-4}
      schY={7}
    />

    {/* 9-DoF IMU, run in UART-RVC mode (115200 baud) off the bridge above. */}
    <chip
      name="BNO085"
      manufacturerPartNumber="BNO085"
      footprint="pinrow4"
      pinLabels={{ pin1: "VCC", pin2: "GND", pin3: "RX", pin4: "TX" }}
      schX={4}
      schY={7}
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
        all come from the bridge board itself, not straight off either Pi.
        Supply is the bridge's 3V3 output and its GND (user, 2026-09-05) --
        so the IMU leg is entirely 3.3V, bus-powered off the Pi 5's USB via
        the MCP2221A's own regulator, and never touches the battery side.
        Nothing on this leg crosses a voltage boundary, so like the encoder
        it stays clear of the level converter. ---- */}
    <trace from=".PI5 > .USB_IMU_BRIDGE" to=".MCP2221A > .USB" />
    <trace from=".PI5 > .USB_GND" to=".MCP2221A > .GND" />
    <trace from=".MCP2221A > .3V3" to=".BNO085 > .VCC" />
    <trace from=".MCP2221A > .GND" to=".BNO085 > .GND" />
    <trace from=".MCP2221A > .TX" to=".BNO085 > .RX" />
    <trace from=".MCP2221A > .RX" to=".BNO085 > .TX" />
  </board>
)

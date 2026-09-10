# BTS7960/IBT-2 Wiring: Independent RPWM/LPWM

`src/hardware/motors/bts7960/driver.py` wires genuinely independent PWM
signals into the IBT-2 module's `RPWM` and `LPWM` inputs -- matching every
documented reference wiring for this chip -- and holds `R_EN`/`L_EN`
permanently HIGH. Direction is selected entirely by which PWM channel
carries a nonzero duty; the other is held at 0.

## Two earlier revisions, both wrong, for different reasons

1. An external 2:1 demux (two AND gates or a 74HC157) synthesizing
   `RPWM`/`LPWM` from a direction bit, with both enables tied permanently
   high. That hardware was never built for this board -- `connect()`
   succeeded, duty writes succeeded, and the motor just never turned.
2. ONE shared PWM signal into both `RPWM`/`LPWM`, direction selected by
   toggling `R_EN`/`L_EN` instead. This assumed a disabled side's
   half-bridge ignores its own PWM input. It doesn't: per the chip's own
   truth table, `RPWM=LPWM=HIGH` is **Fast Brake** (motor terminals
   shorted) and `RPWM=LPWM=LOW` is **Coast**. Tying them together means the
   chip alternates between exactly those two *braking* states every PWM
   cycle, regardless of `R_EN`/`L_EN` -- never actual drive. The module was
   very likely fine the whole time; the wiring scheme itself could never
   have worked.

## Why independent channels need two PWM sources

BTS7960 exposes two independent PWM inputs (`RPWM`/`LPWM`, one per
direction, each gated by its own `R_EN`/`L_EN`, which the module needs held
HIGH regardless of anything direction-related). The Pi Zero 2 W's SoC has
exactly 2 hardware PWM engines, one already claimed by the steering servo
-- so genuinely independent `RPWM`/`LPWM` means one of them has to be
software PWM.

Forward (`RPWM`) gets the Pi's one free **hardware** PWM engine: it's the
overwhelmingly more frequent, performance-critical direction. Reverse
(`LPWM`) rides **software** PWM via `gpiozero` (the `lgpio` pin factory this
board uses): measured jitter is real but mild, and reverse is only ever
used for parking/recovery at low duty, tolerant of it in a way the steering
servo's absolute-position hold is not (see `servo/driver.py`'s docstring --
that's exactly why the servo stays on hardware PWM and never moves to
software PWM).

## Wiring

| Signal | Pi pin | Role |
|---|---|---|
| Servo PWM | GPIO12 (hw PWM0) | Unchanged from the L298N setup |
| `RPWM` (forward) | GPIO13 (hw PWM1) | Same channel the L298N's `ENA` used; hardware PWM |
| `LPWM` (reverse) | GPIO26 | Software PWM (`gpiozero`); repurposed from the abandoned demux `dir_select` wire |
| `R_EN` | GPIO6 | Held permanently HIGH -- gates protection, not direction |
| `L_EN` | GPIO5 | Held permanently HIGH -- gates protection, not direction |

Defaults live on `Bts7960PwmConfig` (`config/hardware/motors/bts7960.toml`)
-- override there or via `BTS7960_PWM_*` env vars if wired differently.

## Not wired: current-sense (`R_IS`/`L_IS`)

The IBT-2 module also exposes `R_IS`/`L_IS` -- analog voltages proportional
to each half-bridge's motor current, useful for stall/overcurrent detection.
These are **not connected or read** by this driver: they're analog, and the
Pi has no built-in ADC, so reading them needs an external ADC chip (e.g. an
MCP3008 over SPI, or ADS1115 over I2C) between the module and the Pi. No such
converter is on hand as of this writing -- if one is added later, it's a new
sensor input path (and a new ADC driver module), not a `Bts7960PwmConfig`
field.

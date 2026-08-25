# BTS7960/IBT-2 Wiring: External PWM Demux

`src/hardware/motors/bts7960/driver.py` assumes an external 2:1 demux
(two AND gates, or a single 74HC157) sits between the Pi and the IBT-2
module. This is a hardware-side assumption the Python code cannot see or
verify -- if the demux is missing or miswired, the driver will still run,
duty writes will still succeed, and the motor simply won't respond correctly.

## Why

BTS7960 exposes two independent PWM inputs (`RPWM`/`LPWM`, one per direction,
each gated by its own `R_EN`/`L_EN`). Driving both directly would need 3
simultaneous hardware-PWM channels on the Pi (servo + `RPWM` + `LPWM`), but
the Pi Zero 2 W's SoC has exactly 2 hardware PWM engines (`PWM0` on GPIO12 or
GPIO18, `PWM1` on GPIO13 or GPIO19 -- GPIO12/18 are the *same* engine, not
independent channels). There is no way to get a 3rd simultaneous
hardware-PWM output on this board.

Since only one direction is ever active at a time, one real PWM signal + one
digital direction bit is enough, synthesizing both `RPWM`/`LPWM` through the
demux:

```
RPWM = PWM AND dir
LPWM = PWM AND NOT dir
```

## Wiring

| Signal | Pi pin | Role |
|---|---|---|
| Servo PWM | GPIO12 (hw PWM0) | Unchanged from the L298N setup |
| Shared drive PWM | GPIO13 (hw PWM1) | Same channel the L298N's `ENA` used; now feeds the demux instead of the H-bridge directly |
| Direction-select | GPIO26 | Demux's `dir` input |
| `R_EN` | GPIO5 | Tied `HIGH` for the driver's lifetime (see `Driver.connect`) |
| `L_EN` | GPIO6 | Tied `HIGH` for the driver's lifetime |
| Demux `RPWM` out | IBT-2 `RPWM` | |
| Demux `LPWM` out | IBT-2 `LPWM` | |

`R_EN`/`L_EN` are wired directly to the Pi (not through the demux) and simply
held high -- the demux is what actually gates which direction receives PWM at
any given moment, not these enable pins. Defaults live on `Bts7960PwmConfig`
(`config/hardware/motors/bts7960.toml`) -- override there or via
`BTS7960_PWM_*` env vars if wired differently.

## Not wired: current-sense (`R_IS`/`L_IS`)

The IBT-2 module also exposes `R_IS`/`L_IS` -- analog voltages proportional
to each half-bridge's motor current, useful for stall/overcurrent detection.
These are **not connected or read** by this driver: they're analog, and the
Pi has no built-in ADC, so reading them needs an external ADC chip (e.g. an
MCP3008 over SPI, or ADS1115 over I2C) between the module and the Pi. No such
converter is on hand as of this writing -- if one is added later, it's a new
sensor input path (and a new ADC driver module), not a `Bts7960PwmConfig`
field.

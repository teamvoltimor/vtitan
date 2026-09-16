# 0083. The OLED has a raw-I2C fallback and the button thresholds escalate

- Status: accepted
- Date: 2026-09-15

## Context

The SSD1306 OLED hung on hardware under the Adafruit CircuitPython (Blinka)
stack: smbus2 stress tests (100+ back-to-back 32-byte writes, about 3.2 ms each,
zero hangs) showed the bus, wiring and kernel driver are reliable, so the fault
was specific to that library stack. The single panel button must start a race,
stop one, and power the robot down safely without a laptop or SSH.

## Options considered

- (a) One button threshold; a single OLED backend.
- (b) Distinguish start from E-STOP from shutdown, and keep a dependency-free
      raw-I2C OLED backend as a fallback.

## Decision

(b). The OLED keeps two backends: `blinka` (the Adafruit stack, the committed
default) and `raw_i2c`, which talks to `/dev/i2c-N` directly through the
`I2C_SLAVE` ioctl in 32-byte blocks with no Blinka or smbus2 dependency. `raw_i2c`
exists alongside Blinka, not as a replacement, and is the override to reach for if
the hang recurs.

The button is on `button_gpio_pin = 4` (GPIO4, physical pin 7), with the internal
pull-up and `debounce_ms = 50`. Hold events fire while the button is held, not on
release, because an E-STOP that waits for the operator to let go is useless to
someone gripping it in a panic. `long_press_threshold_sec = 3.0` is E-STOP while
racing or reset to BOOT_CHECK once finished; `shutdown_press_threshold_sec = 10.0`
powers the robot down cleanly so it can be unplugged without ext4 damage. The
thresholds escalate rather than replace: 10 s passes 3 s first, so a panicked grip
can only ever stop the robot, never skip straight to power-off; the 7 s gap is the
margin for that.

## Consequences

- A library-level I2C hang has a dependency-free fallback.
- The panel shows the hold countdown, with the thresholds sourced from the button
  node rather than hardcoded into the render function, so there is one source of
  truth.

## History

- 18331b87 2026-07-08: add the raw-I2C driver and the backend enum.
- 836b785b 2026-07-27: raise the long-press threshold 2.0 to 3.0 s (a firm start
  press measured 2.13 s, 130 ms from E-STOPping instead of starting).
- 8a05c4fd 2026-07-28: centralize the hardware config into TOML.
- 4f059664 2026-07-30: lowercase the TOML keys, keep SHOUT_CASE env aliases.
- 0b0980c6 2026-08-01: fire on the press, not the release; add hold-to-power-off.
- 36ce1a9a 2026-08-01: show the hold countdown, thresholds from the button node.
- 3b6456d5 and b982e109 2026-09-13: schemas and rationale.
- `ui_refresh_rate_hz = 10.0` is the OLED's own redraw cadence, and the telemetry
  bridge's `ui_summary_rate_hz` is matched to it; anything lower made every other
  redraw show stale numbers, so the two rates stay tied.
- The latched state/diagnostic QoS ships BEST_EFFORT, not RELIABLE: the Pi Zero's
  OLED was measured stalling for 30+ seconds under its own CPU/memory contention,
  and RELIABLE's flow control holds a writer's `publish()` until the matched
  reader acks, so a RELIABLE `/robot_state` publisher would block the Pi 5's whole
  single-threaded executor for the same duration. A dropped sample is corrected on
  the next tick.

## Refuted

- The 2.0 s threshold; E-STOP classified on release; the claim `raw_i2c` replaces
  Blinka (it is an alternate backend, and the committed default is `blinka`).

## Cross-references

- 0073 owns the challenge jumper read on the same Pi Zero.

# Servo comparison: 180° (current) vs 270° (servo270 profile)

Reference sheet for the two steering servo candidates. The 180° unit is the
currently-built hardware (`platform/robot/config/hardware/motors/servo.toml`);
the 270° unit (Hiwonder HPS-3527SG) is the part backing the `servo270`
hardware profile scaffolded in
`platform/robot/config/hardware/motors/profiles/servo270/servo.toml` (see
`docs/internal/plans/2026-08-11-servo-hardware-profiles.md`).

## Datasheet specs

| Spec | 180° — INJORA INJS035 | 270° — Hiwonder HPS-3527SG |
|---|---|---|
| Part | INJORA INJS035 35KG Digital Servo | Hiwonder HPS-3527SG 35KG Coreless Servo |
| Rotation range | 180° (switchable to 270°, mode unconfirmed) | 0–270° |
| PWM pulse width | 500–2500 µs → 180° | 500–2500 µs → 270° |
| Neutral position | 1500 µs / 330 Hz | not specified |
| Dead band | 4 µs | not specified |
| Control accuracy | not specified | 2 µs |
| Motor | Core (brushed) motor | Coreless motor |
| Gear | Reinforced metal gears | Stainless steel gears, ratio 342 |
| Stall torque | 35 kg·cm (voltage unspecified) | 29 kg·cm@5V / 30@7.4V / 35@8.4V |
| Speed | not specified | 0.13 s/60° @7.4V (~0.585 s/270°) |
| Voltage range | 4.8–6.0V | 5–8.4V |
| Bearing | 2BB | not specified |
| Standby current | not specified | 5 mA |
| Stall current | not specified | 2.2–3.2 A |
| Weight | 57g / 60g (mode-dependent) | 60g |
| Size | 40.5×20×40.5 mm | 40×20×38 mm |
| Connector cable length | 260 mm (JR) | 300 mm |
| Spline | Φ5.9, 25T | not specified |

## Config constants (this repo)

| Field | Base (`servo.toml`, 180°) | `servo270` overlay | Status |
|---|---|---|---|
| `range_deg` | 180.0 | 270.0 | set (direct spec fact) |
| `min_pulse_us` | 500.0 | *(commented out — inherits 500.0)* | `TODO(servo270)`: verify pulse span matches the Hiwonder datasheet before uncommenting |
| `max_pulse_us` | 2500.0 | *(commented out — inherits 2500.0)* | same as above |
| `center_pulse_us` | 1500.0 | *(commented out — inherits 1500.0)* | same as above |
| `gpio_pin` / `pwmchip` / `pwm_channel` / `reversed` | 12 / 0 / 0 / false | not overridden | wiring-level, doesn't vary by servo model |

Datasheet pulse span (500–2500 µs) matches on both parts, so the commented-out
inheritance in the overlay is plausible — but it's still unconfirmed against
a real bench sweep of the physical Hiwonder unit, not just its printed spec.

## Downstream geometry (not yet bench-validated)

| Field | Base (180°, bench-measured) | `servo270` (proposed, per `2026-08-11-servo-hardware-profiles.md`) |
|---|---|---|
| `servo_max_angle_deg` (center-to-lock) | 90.0 (180°/2) | 135.0 (270°/2 — geometric fact) |
| `max_wheel_angle_deg` | 55.0 (bench-confirmed 2026-08-10, see memory `steering_trim_and_understeer_2026_08_09`) | 85.0 (user estimate — **needs protractor confirmation**) |

`max_wheel_angle_deg` does not scale 1:1 with `servo_max_angle_deg` because
of the steering linkage ratio (`linkage_ratio ≈ 0.611` measured for the
current build) — the 85.0° figure for servo270 is a rough estimate, not a
derived value, until the new servo is on the bench.

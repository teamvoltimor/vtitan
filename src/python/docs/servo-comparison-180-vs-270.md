# Servo comparison: 180° (retired) vs 270° (270deg-hiwonder-35kg profile, CURRENT BUILD)

Reference sheet for the two steering servo candidates. The 180° unit was the
originally-built hardware (`src/config/hardware/motors/servo.toml`);
the 270° unit (Hiwonder HPS-3527SG) is the current build, backed by the
`270deg-hiwonder-35kg` hardware profile at
`src/config/hardware/motors/profiles/270deg-hiwonder-35kg/servo.toml`
(see `docs/internal/plans/2026-08-11-servo-hardware-profiles.md`). That
folder was named `servo270` until 2026-08-27, which didn't match the active
profile name and meant its `range_deg=270` override silently never applied
on hardware -- see `src/config/hardware/motors/profiles/
270deg-hiwonder-35kg/servo.toml`'s own comment for the mechanism.

## Profile activation is a runtime setting, not a config edit (2026-08-20)

Editing a value inside `src/shared/config/profiles/<name>/robot.toml`
(or the matching `src/config/hardware/motors/profiles/<name>/servo.toml`)
does **not** put that value into effect on its own. Every profile is an
overlay that only applies when its name is listed in the
`VTITAN_HARDWARE_PROFILE` env var at process start (`shared.config.
hardware_profile.active_profiles()`, read fresh by
`RobotConstants.load_default()` / `NavigationTuning.load_default()` - no
separate build/codegen step on the Python side). Empty/unset means every
node runs on the base, unmodified config, full stop.

Confirmed on hardware 2026-08-20: `VTITAN_HARDWARE_PROFILE=` was blank in
the Pi 5's `.env` the entire time a motor-speed change was being tested -
the change lived in the retired `fastwide`/`fastonly` profile overlays
(`max_speed_mps = 0.234`, vs the base `robot.toml`'s `0.156`), so every
drive test that night ran against the **unmodified base ceiling**, not the
intended higher one. Same applies to `270deg-hiwonder-35kg`: its geometry
overrides (`servo_max_angle_deg`, `max_wheel_angle_deg`, etc. - see below)
are inert on a robot with a blank/different `VTITAN_HARDWARE_PROFILE`, even
with the physical 270° servo already wired in.

To activate one or more profiles: set
`VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm`
(one profile per physical component, comma-separated, ordered - later names
win on any key both set) in `src/.env` on the Pi Zero (this is
where `ackermann_motor_node` actually runs), then restart
`vtitan-pi-zero.service` so the new nodes pick it up. There is no way to
tell a profile is active from the robot's behavior alone without checking
`.env` directly - a silently inactive (or, as found 2026-08-27, silently
*misnamed*) profile looks identical to a robot correctly running on
unmodified defaults.

## Datasheet specs

| Spec | 180° - INJORA INJS035 | 270° - Hiwonder HPS-3527SG |
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

| Field | Base (`servo.toml`, 180°) | `270deg-hiwonder-35kg` overlay | Status |
|---|---|---|---|
| `range_deg` | 180.0 | 270.0 | set (direct spec fact) |
| `min_pulse_us` | 500.0 | *(commented out - inherits 500.0)* | `TODO(270deg-hiwonder-35kg)`: verify pulse span matches the Hiwonder datasheet before uncommenting |
| `max_pulse_us` | 2500.0 | *(commented out - inherits 2500.0)* | same as above |
| `center_pulse_us` | 1500.0 | *(commented out - inherits 1500.0)* | same as above |
| `gpio_pin` / `pwmchip` / `pwm_channel` / `reversed` | 12 / 0 / 0 / false | not overridden | wiring-level, doesn't vary by servo model |

Datasheet pulse span (500–2500 µs) matches on both parts, so the commented-out
inheritance in the overlay is plausible - but it's still unconfirmed against
a real bench sweep of the physical Hiwonder unit, not just its printed spec.

## Downstream geometry (not yet bench-validated)

| Field | Base (180°, retired, bench-measured) | `270deg-hiwonder-35kg` (CURRENT BUILD) |
|---|---|---|
| `servo_max_angle_deg` (center-to-lock) | 90.0 (180°/2) | 135.0 (270°/2 - geometric fact) |
| `max_wheel_angle_deg` | 55.0 (bench-confirmed 2026-08-10, see memory `steering_trim_and_understeer_2026_08_09`) | 85.0 (user estimate - **still needs protractor confirmation**) |

`max_wheel_angle_deg` does not scale 1:1 with `servo_max_angle_deg` because
of the steering linkage ratio (`linkage_ratio ≈ 0.611` measured for the
retired 180° build) - the 85.0° figure for the current 270° servo is a
rough estimate, not a derived value, until it's bench-measured directly.
The folder backing this overlay was misnamed `servo270` until 2026-08-27,
so this override never actually applied on hardware until now.

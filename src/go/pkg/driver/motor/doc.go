// Package motor provides a BTS7960/IBT-2 H-bridge drive-motor driver.
//
// Unlike a sensor, a motor driver has nothing to "Read" in the sense
// driver.Driver[T] (src/go/pkg/driver) models - it is
// commanded, not sampled. This package therefore does not implement
// driver.Driver[T]; it exposes its own narrow Actuator interface instead
// (Connect/SetSpeed/Close), defined in actuator.go next to its only
// implementation. See adr:0068-go-parallel-track-single-cutover's note that
// Driver[T] is a sensor-shaped default, not a mandate - forcing Read onto an
// actuator would just mean a Read method nobody calls.
//
// Ported from src/python/src/hardware/motors/bts7960/driver.py - see
// src/python/docs/bts7960-ibt2-wiring.md for the wiring rationale
// (independent RPWM/LPWM, R_EN/L_EN held permanently HIGH, why
// RPWM=LPWM=HIGH - "Fast Brake" - must never happen) and commit f0fc617b
// ("assert BTS7960 R_EN/L_EN only after PWM channels confirm 0 duty") for
// the safety-critical connect-ordering bug hbridge.Controller.Connect
// preserves: both PWM channels must be confirmed at 0 duty before R_EN/L_EN
// are ever driven HIGH, or the bridge can kick the motor to full speed for
// the brief window between enabling and the PWM channels settling.
//
// Package layout mirrors pkg/driver/imu: the pure, hardware-independent
// logic layer (the duty split and hbridge.Controller - unit-testable with
// fakes, no real GPIO/PWM I/O) lives in pkg/portable/hbridge, shared with
// the Pico 2 firmware, and this package keeps only the thin Linux hardware
// layer (sysfs_pwm.go, soft_pwm.go, gpio_enable.go, driver.go) that wires
// real /sys/class/pwm and github.com/warthog618/go-gpiocdev access behind
// hbridge's DutyWriter/EnableWriter interfaces.
package motor

// Package motor provides a BTS7960/IBT-2 H-bridge drive-motor driver.
//
// Unlike a sensor, a motor driver has nothing to "Read" in the sense
// driver.Driver[T] (platform/robot-go/internal/driver) models — it is
// commanded, not sampled. This package therefore does not implement
// driver.Driver[T]; it exposes its own narrow Actuator interface instead
// (Connect/SetSpeed/Close), defined in controller.go next to its only
// implementation. See go-migration-plan.md's note that Driver[T] is a
// sensor-shaped default, not a mandate — forcing Read onto an actuator
// would just mean a Read method nobody calls.
//
// Ported from platform/robot/src/hardware/motors/bts7960/driver.py — see
// platform/robot/docs/bts7960-ibt2-wiring.md for the wiring rationale
// (independent RPWM/LPWM, R_EN/L_EN held permanently HIGH, why
// RPWM=LPWM=HIGH — "Fast Brake" — must never happen) and commit f0fc617b
// ("assert BTS7960 R_EN/L_EN only after PWM channels confirm 0 duty") for
// the safety-critical connect-ordering bug this package's Controller.Connect
// preserves: both PWM channels must be confirmed at 0 duty before R_EN/L_EN
// are ever driven HIGH, or the bridge can kick the motor to full speed for
// the brief window between enabling and the PWM channels settling.
//
// Package layout mirrors internal/driver/imu: a pure, hardware-independent
// logic layer (duty.go, controller.go — unit-testable with fakes, no real
// GPIO/PWM I/O) and a thin hardware layer (sysfs_pwm.go, soft_pwm.go,
// gpio_enable.go, driver.go) that wires real /sys/class/pwm and
// github.com/warthog618/go-gpiocdev access behind the same small interfaces
// the logic layer already defines.
package motor

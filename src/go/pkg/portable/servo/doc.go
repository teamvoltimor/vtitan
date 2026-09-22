// Package servo is the pure pulse mapping of the steering servo: a servo
// angle to a pulse width in microseconds (PulseUS), and the 1 us dedupe
// that decides whether a new pulse is worth writing at all (NeedsWrite).
//
// It holds no carrier and writes nothing: the Pi Zero's Linux driver
// (pkg/driver/servo, /sys/class/pwm) and the Pico 2 firmware (TinyGo
// machine.PWM) each own the hardware and call into this package, so both
// boards turn the same angle into the same pulse.
//
// Like every package under pkg/portable it must compile under TinyGo: no
// os, net, syscall, reflect, unsafe, log/slog, Linux or transport deps, and
// nothing from src/go/internal. See
// adr:0098-pico-actuation-board-and-portable-cores.
package servo

// Package hbridge is the pure, hardware-independent BTS7960/IBT-2 H-bridge
// drive logic: the safety-critical connect ordering (both PWM channels
// confirmed at 0 duty before R_EN/L_EN are ever driven HIGH, commit
// f0fc617b) and the signed-duty-to-channel-duty split that makes the
// Fast-Brake state (RPWM and LPWM both nonzero) structurally impossible.
//
// It knows nothing about how a duty fraction or an enable level reaches a
// pin: Controller writes through the small DutyWriter/EnableWriter
// interfaces. It is shared by the two actuation boards: the Pi Zero's Linux
// backend (pkg/driver/motor, /sys/class/pwm plus go-gpiocdev) and the Pico 2
// firmware (TinyGo machine.PWM/machine.Pin), so the ordering invariants are
// written, and tested, exactly once.
//
// Like every package under pkg/portable it must compile under TinyGo: no
// os, net, syscall, reflect, unsafe, log/slog, Linux or transport deps, and
// nothing from src/go/internal. See
// adr:0098-pico-actuation-board-and-portable-cores.
package hbridge

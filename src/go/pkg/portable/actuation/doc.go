// Package actuation is the transport-free core of the actuation board's
// control loop: an AckermannCmd's wheel angle to a servo angle
// (SteeringConfig, SteeringToServoDeg), its speed to a signed duty fraction
// (SpeedToNormalized), the rule that a non-finite command is rejected whole
// (FiniteCommand), and the command-deadline watchdog as a clock-injected
// state machine (Watchdog).
//
// It is shared by the two actuation boards: internal/node/motor on the Pi
// Zero wraps it in NATS/protobuf transport and Linux drivers, and the Pico 2
// firmware wraps it in its own link and TinyGo drivers, so a fix to the
// safety-critical watchdog or the steering conversion lands in both at once.
//
// Like every package under pkg/portable it must compile under TinyGo: no
// os, net, syscall, reflect, unsafe, log/slog, Linux or transport deps, and
// nothing from src/go/internal. See
// adr:0098-pico-actuation-board-and-portable-cores.
package actuation

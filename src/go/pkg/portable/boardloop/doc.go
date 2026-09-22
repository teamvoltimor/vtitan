// Package boardloop is the actuation board's control loop for a
// microcontroller board (the Pico 2, adr:0098-pico-actuation-board-and-portable-cores):
// the boardlink session with the Pi 5 and the command failsafe, with every
// piece of hardware behind a small interface so the whole loop runs, and is
// tested, on the host.
//
// It is the microcontroller counterpart of internal/node/motor's Loop on the
// Pi Zero, and applies a command by the same rules, through the same
// pkg/portable code: a non-finite speed or steering angle rejects the whole
// command as missing (actuation.FiniteCommand), steering is written before
// drive, and a command stream silent for Config.CommandTimeoutMS stops the
// drive and centers the steering once per staleness episode
// (actuation.Watchdog).
//
// The firmware owns no timing of its own: it calls Loop.Step in a tight loop
// with the time since boot, and the Loop decides what to read, apply and
// send. Tests drive the same Step with a hand-advanced clock.
//
// # Session
//
// The board is configured if and only if the last Config it received was
// valid and the drive connected. Until then it:
//
//   - repeats Hello every HelloInterval, carrying Options.BootFaults (for
//     example boardlink.FaultWatchdogReset) plus boardlink.FaultActuator if
//     the drive failed to connect;
//   - never writes the servo, because it does not know the servo's pulse
//     range yet: the firmware leaves the servo pin without pulses, and a
//     hobby servo without pulses holds or goes limp, it does not move;
//   - never connects the drive, so the H-bridge enables stay low;
//   - drops every Command, counted in Counters.CommandsIgnored and not
//     flagged in any fault bit (nothing was acted on, and Status is not sent
//     before configuration);
//   - answers Ping with Pong, so the host can measure the clock offset
//     before it configures.
//
// A valid Config connects the drive on first use (hbridge.Controller.Connect,
// zeroing both PWM channels before the enables), then stops the drive,
// centers the servo with the new pulse calibration, re-arms the command
// watchdog in its stopped state, and sends a Status at once. A later valid
// Config does the same: the host sends one when it restarts, and a new
// session starts from rest.
//
// An invalid Config (ValidateConfig) is not applied at all. A configured
// board that receives one safety-stops with the calibration it already had
// and drops back to unconfigured, so it resumes Hello and the host sees that
// its Config was refused. Keeping the old configuration instead would leave
// the board driving on a profile the host no longer has.
//
// # Status
//
// Status goes out every StatusIntervalMS, and immediately on configuration
// and on a watchdog stop. State is Fault while the last actuator write
// failed, else Running while the commanded duty is nonzero, else Idle. Duty
// is the commanded signed duty before InvertDrive, as the Zero reports it;
// ServoAngleDeg is the servo angle after the travel clamp and before
// Reversed; CommandAgeMS is the time since the last accepted command, or
// since configuration if none has been accepted. Faults carries the boot
// faults on every Status of the boot, FaultActuator while the last write
// failed, and FaultRejectedCommand if a non-finite command was dropped since
// the previous Status (the bit is cleared once reported).
//
// Odometry goes out every OdometryIntervalMS when the board has an encoder
// and the interval is nonzero.
//
// # Button
//
// A board with a Button sends a boardlink.Button message whenever the raw
// reading changes, and once at boot so the host knows the initial state. No
// debounce or hold threshold runs on the board: the host evaluates the
// edges with the Zero's own evaluator, so the timing policy has one home.
//
// # Drive inversion
//
// Config.InvertDrive is applied here, by negating the signed duty before
// Drive.SetSpeed, because the Pico learns it from the host after its drive
// is built. The firmware must therefore construct its hbridge.Controller
// with invert false. On the Zero the same flag goes into
// hbridge.NewController instead; either way it is applied exactly once.
//
// # Allocation
//
// Step allocates nothing in steady state: the decoder, the receive buffer,
// the packets and the transmit buffer are all fixed fields of Loop.
//
// Like every package under pkg/portable it must compile under TinyGo: no
// os, net, syscall, reflect, unsafe, log/slog, Linux or transport deps, and
// nothing from src/go/internal.
package boardloop

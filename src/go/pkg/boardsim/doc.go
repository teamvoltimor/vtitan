// Package boardsim is a virtual actuation board: any board that runs
// pkg/portable/boardloop and speaks pkg/portable/boardlink to the host,
// with its hardware in memory instead of GPIO and PWM.
//
// It is deliberately not a model of a particular board. The Pico 2
// (firmware/pico2, adr:0098-pico-actuation-board-and-portable-cores) is
// today's only implementation of the boardloop hardware interfaces, but the
// contract is the five interfaces (Link, Drive, Servo, Encoder, Button) and
// the boardlink protocol, and a different microcontroller only needs its own
// adapter layer. The Pi Zero does not speak boardlink at all (it runs
// internal/node/motor behind NATS), so it is out of this package's scope.
//
// What is real here is everything above the adapters: the session, the
// Config validation, the command conversion, the failsafe and the odometry
// cadence are the same boardloop code a board's firmware runs. What is not
// covered is each board's adapter layer (pin setup, PWM, its hardware
// watchdog, encoder interrupts), which still needs the bench.
//
// The Link reads from any io.ReadWriter (a net.Pipe end in tests, a pty or
// a socket for a longer-running virtual board). A background goroutine
// drains it into a buffer, because boardloop requires a Read that never
// blocks.
//
// The Encoder can follow a simple wheel model: with
// Options.CountsPerSecondAtFullDuty set, every Step advances the count by
// the duty the Loop actually wrote to the Drive (after InvertDrive), so a
// forward command produces odometry without the test touching the encoder.
// It is a kinematic stand-in, not a motor model: no inertia, no load.
package boardsim

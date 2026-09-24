//go:build tinygo

package main

import "machine"

// Pin map. PROVISIONAL: no Pico 2 has been wired yet, so every pin below is
// a choice made on paper and must be confirmed on the bench before the
// firmware drives a motor. Keep every pin number in this file.
//
// The servo and the H-bridge sit on separate PWM slices, because a slice
// has one period and they need different ones (50 Hz and 1 kHz). A slice
// drives the GPIO pair 2n, 2n+1 (channel A on the even pin, B on the odd),
// so the two H-bridge inputs share slice 1 at the same carrier.
const (
	// pinServo is the steering servo's signal: GP16, PWM slice 0 channel A.
	pinServo = machine.GP16
	// pinRPWM is the BTS7960's RPWM (forward): GP18, PWM slice 1 channel A.
	pinRPWM = machine.GP18
	// pinLPWM is the BTS7960's LPWM (reverse): GP19, PWM slice 1 channel B.
	// Keep the LPWM pull-down resistor fitted (adr:0098): it covers the
	// window between power-on and this firmware driving the pin.
	pinLPWM = machine.GP19
	// pinREN and pinLEN are the BTS7960's R_EN and L_EN, plain outputs held
	// low until the drive connects.
	pinREN = machine.GP20
	pinLEN = machine.GP21
	// pinButton is the start/stop button: GP17, pulled up, pressed shorts it
	// to ground. PROVISIONAL like the rest of the map. The board reports the
	// raw edges; the host runs the debounce and hold evaluator.
	pinButton = machine.GP17

	// pinEncoderA and pinEncoderB are the wheel encoder's quadrature
	// channels, decoded on GPIO edge interrupts by encoderPins (hw.go).
	// TODO(encoder-pio): adr:0098 asked for the RP2350's hardware PIO to
	// count quadrature; TinyGo 0.42.0 ships no PIO API (no program loader,
	// no register wrapper) and this module carries no PIO driver
	// dependency, so this is the interrupt-driven fallback the toolchain
	// supports today, not the hardware path the ADR measured against.
	// Revisit if bench numbers show the CPU-driven decode losing edges
	// under load, or once a PIO driver becomes available.
	pinEncoderA = machine.GP14
	pinEncoderB = machine.GP15
)

// PWM carriers.
const (
	// servoPeriodNS is the standard 50 Hz hobby-servo frame.
	servoPeriodNS = 20_000_000
	// driveFrequencyHz matches pkg/driver/motor.DefaultFrequencyHz, the
	// carrier the Zero drives the same BTS7960 at.
	driveFrequencyHz = 1000
	drivePeriodNS    = 1_000_000_000 / driveFrequencyHz
)

// servoSlice and driveSlice are the PWM slices of the pins above.
var (
	servoSlice = machine.PWM0
	driveSlice = machine.PWM1
)

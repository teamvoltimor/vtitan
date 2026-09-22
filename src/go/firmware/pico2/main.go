//go:build tinygo

// Command pico2 is the Pico 2 actuation-board firmware
// (adr:0098-pico-actuation-board-and-portable-cores). It is wiring only: the
// session, the command failsafe and every conversion live in
// pkg/portable/boardloop and the packages it uses, which are tested on the
// host. This file brings the pins up in a safe order, arms the RP2350
// hardware watchdog, and calls boardloop.Loop.Step in a tight loop.
package main

import (
	"context"
	"device/rp"
	"machine"
	"runtime"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/hbridge"
)

// brokenDrive stands in for a drive whose PWM failed to come up, so the
// board still speaks the protocol and reports FaultActuator in Hello
// instead of going silent.
type brokenDrive struct{ err error }

// brokenServo is brokenDrive for the servo.
type brokenServo struct{ err error }

const (
	// bootWatchdogMS arms the hardware watchdog from boot until a Config
	// supplies HardwareWatchdogMS, and again whenever the board drops back
	// to unconfigured.
	bootWatchdogMS = 1000
	// refClockMHz is the RP2350 reference clock (the 12 MHz crystal) the
	// watchdog tick divides down to 1 us.
	refClockMHz = 12
)

func (d brokenDrive) Connect(context.Context) error           { return d.err }
func (d brokenDrive) SetSpeed(context.Context, float64) error { return d.err }
func (s brokenServo) SetPulseUS(float64) error                { return s.err }

func main() {
	// Read before anything can reset it: the reset cause decides
	// FaultWatchdogReset for the whole boot.
	var bootFaults boardlink.Faults
	if rp.WATCHDOG.GetREASON_TIMER() != 0 {
		bootFaults |= boardlink.FaultWatchdogReset
	}
	startWatchdogTick()
	armed := uint16(bootWatchdogMS)
	armWatchdog(armed)

	loop, err := boardloop.New(boardloop.Hardware{
		Link:    usbLink{serial: machine.Serial},
		Drive:   setupDrive(),
		Servo:   setupServo(),
		Encoder: nil, // TODO(encoder): see pinEncoderA in board.go.
		Button:  setupButton(),
	}, boardloop.Options{BootID: bootID(), BootFaults: bootFaults})
	if err != nil {
		// Unreachable: every Hardware field above is non-nil. Stop feeding
		// the watchdog so the board resets rather than idling silently.
		for {
			runtime.Gosched()
		}
	}

	boot := time.Now()
	for {
		loop.Step(time.Since(boot))

		want := uint16(bootWatchdogMS)
		if cfg, ok := loop.Config(); ok {
			want = cfg.HardwareWatchdogMS
		}
		if want != armed {
			armWatchdog(want)
			armed = want
		}
		machine.Watchdog.Update()
		runtime.Gosched()
	}
}

// setupDrive brings the H-bridge pins up safe: R_EN and L_EN driven low
// first, then both PWM inputs routed to a slice already at zero duty. The
// bridge is enabled later, by boardloop through hbridge.Controller.Connect,
// on the first valid Config. The Controller is built with invert false:
// boardloop applies Config.InvertDrive itself.
func setupDrive() boardloop.Drive {
	for _, pin := range [...]machine.Pin{pinREN, pinLEN} {
		pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
		pin.Low()
	}
	if err := driveSlice.Configure(machine.PWMConfig{Period: drivePeriodNS}); err != nil {
		return brokenDrive{err: err}
	}
	rCh, err := driveSlice.Channel(pinRPWM)
	if err != nil {
		return brokenDrive{err: err}
	}
	lCh, err := driveSlice.Channel(pinLPWM)
	if err != nil {
		return brokenDrive{err: err}
	}
	driveSlice.Set(rCh, 0)
	driveSlice.Set(lCh, 0)
	return hbridge.NewController(
		&pwmDuty{slice: driveSlice, channel: rCh},
		&pwmDuty{slice: driveSlice, channel: lCh},
		enablePin{pin: pinREN},
		enablePin{pin: pinLEN},
		false,
	)
}

// setupServo starts the 50 Hz frame at zero width: no pulses, so the servo
// does not move until boardloop writes the center pulse of a valid Config.
func setupServo() boardloop.Servo {
	ch, err := newPWMChannel(servoSlice, servoPeriodNS, pinServo)
	if err != nil {
		return brokenServo{err: err}
	}
	return &servoPWM{slice: servoSlice, channel: ch}
}

// setupButton brings the start button up as a pulled-up input. Pressing it
// shorts the line to ground; boardloop reports the raw edges and the host
// applies the debounce and hold thresholds.
func setupButton() boardloop.Button {
	pinButton.Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	return buttonPin{pin: pinButton}
}

// startWatchdogTick sets the RP2350 watchdog tick to 1 us. TinyGo enables
// the tick generator but leaves its divider at the reset value.
func startWatchdogTick() {
	rp.TICKS.SetWATCHDOG_CTRL_ENABLE(0)
	rp.TICKS.SetWATCHDOG_CYCLES(refClockMHz)
	rp.TICKS.SetWATCHDOG_CTRL_ENABLE(1)
}

// armWatchdog (re)starts the hardware watchdog at ms. TinyGo 0.42's rp2
// watchdog loads twice the ticks asked for, a workaround for RP2040
// erratum E1 that the RP2350 does not have, so it is asked for half.
func armWatchdog(ms uint16) {
	half := max(1, uint32(ms)/2)
	_ = machine.Watchdog.Configure(machine.WatchdogConfig{TimeoutMillis: half})
	_ = machine.Watchdog.Start()
}

// bootID is random per boot, from the ring-oscillator RNG, falling back to
// the boot-time clock if the RNG fails.
func bootID() uint32 {
	if v, err := machine.GetRNG(); err == nil {
		return v
	}
	return uint32(time.Now().UnixNano())
}

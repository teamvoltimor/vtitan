//go:build tinygo

package main

import (
	"context"
	"errors"
	"machine"
)

// pwmSlice is the part of TinyGo's RP2 PWM slice (an unexported type) the
// adapters use.
type pwmSlice interface {
	Configure(config machine.PWMConfig) error
	Channel(pin machine.Pin) (uint8, error)
	Top() uint32
	Set(channel uint8, value uint32)
}

// pwmDuty is one H-bridge PWM input as an hbridge.DutyWriter.
type pwmDuty struct {
	slice   pwmSlice
	channel uint8
}

// enablePin is R_EN or L_EN as an hbridge.EnableWriter.
type enablePin struct {
	pin machine.Pin
}

// servoPWM is the steering servo as a boardloop.Servo: a pulse width in
// microseconds becomes a compare level against the slice's 20 ms frame.
type servoPWM struct {
	slice   pwmSlice
	channel uint8
}

// buttonPin is the start button as a boardloop.Button. The line is pulled up
// and the switch shorts it to ground, so a low reading is a press.
type buttonPin struct {
	pin machine.Pin
}

// Pressed reports the raw button state, true while it is down. The host
// debounces it, so this is deliberately the un-filtered reading.
func (b buttonPin) Pressed() bool { return !b.pin.Get() }

// usbLink is USB CDC serial as a boardloop.Link.
type usbLink struct {
	serial machine.Serialer
}

const nsPerUS = 1000

var errDutyRange = errors.New("pico2: duty fraction outside [0, 1]")

// SetDuty writes fraction of the carrier, in TinyGo's Set(channel, Top)
// convention.
func (d *pwmDuty) SetDuty(_ context.Context, fraction float64) error {
	if !(fraction >= 0 && fraction <= 1) {
		return errDutyRange
	}
	d.slice.Set(d.channel, uint32(fraction*float64(d.slice.Top())))
	return nil
}

// SetHigh drives the enable line.
func (e enablePin) SetHigh(_ context.Context, high bool) error {
	e.pin.Set(high)
	return nil
}

// SetPulseUS sets the servo pulse. boardloop only passes pulses already
// clamped to the Config's validated range, well inside the frame.
func (s *servoPWM) SetPulseUS(pulseUS float64) error {
	level := pulseUS * nsPerUS / servoPeriodNS * float64(s.slice.Top())
	s.slice.Set(s.channel, uint32(level))
	return nil
}

// Read copies what the CDC receive buffer already holds, without waiting.
func (l usbLink) Read(p []byte) (int, error) {
	n := 0
	for n < len(p) && l.serial.Buffered() > 0 {
		b, err := l.serial.ReadByte()
		if err != nil {
			break
		}
		p[n] = b
		n++
	}
	return n, nil
}

// Write queues one frame. TinyGo drops it when no host has the port open.
// With the port open and the host not reading, TinyGo's CDC Write waits for
// ring space; the hardware watchdog is what bounds that wait.
func (l usbLink) Write(p []byte) (int, error) {
	return l.serial.Write(p)
}

// newPWMChannel configures slice at periodNS with both levels at zero, and
// only then routes pin to it, so the pin never carries a pulse it was not
// given.
func newPWMChannel(slice pwmSlice, periodNS uint64, pin machine.Pin) (uint8, error) {
	if err := slice.Configure(machine.PWMConfig{Period: periodNS}); err != nil {
		return 0, err
	}
	ch, err := slice.Channel(pin)
	if err != nil {
		return 0, err
	}
	slice.Set(ch, 0)
	return ch, nil
}

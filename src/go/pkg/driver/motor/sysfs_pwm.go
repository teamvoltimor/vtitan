//go:build linux

// This file, and the rest of the hardware adapter layer (gpio_enable.go,
// soft_pwm.go, driver.go), only builds on linux -- /sys/class/pwm is a
// Linux-kernel-specific interface, and github.com/warthog618/go-gpiocdev
// (imported by the other three) wraps the Linux GPIO character-device ABI,
// which has no Windows equivalent. This is a real platform boundary, not an
// oversight: the pure logic layer (pkg/portable/hbridge) has no such
// constraint and builds/tests on every platform, including the Windows dev
// machine this was written on -- see doc.go. Build/vet/test/lint this
// package's hardware layer with GOOS=linux GOARCH=arm64 (the actual
// deployment target).
package motor

import (
	"context"
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/internal/sysfspwm"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/hbridge"
)

// sysfsPWMChannel is the real hbridge.DutyWriter for RPWM
// (forward): the Pi's one free hardware PWM engine, driven directly through
// the kernel's /sys/class/pwm sysfs interface (export/period/duty_cycle/
// enable files) rather than a library, per
// adr:0068-go-parallel-track-single-cutover's stated preference for this pin.
//
// The sysfs mechanics (export-wait, init order) live in
// pkg/driver/internal/sysfspwm, shared with pkg/driver/servo; this type only
// adapts them to hbridge.DutyWriter's duty-fraction contract.
type sysfsPWMChannel struct {
	ch *sysfspwm.Channel
}

const (
	sysfsPWMRoot = sysfspwm.DefaultRoot

	// pwmOverlayHint is the config.txt line Export names when pwmchip is
	// missing: RPWM shares the two-channel overlay with the steering servo.
	pwmOverlayHint = "'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4'"
)

var _ hbridge.DutyWriter = (*sysfsPWMChannel)(nil)

// newSysfsPWMChannel describes (without touching the filesystem) the PWM
// channel at root/pwmchip<chip>/pwm<channel>, running at frequencyHz.
func newSysfsPWMChannel(root string, chip, channel, frequencyHz int) *sysfsPWMChannel {
	return &sysfsPWMChannel{ch: sysfspwm.New(root, chip, channel, frequencyHz, pwmOverlayHint)}
}

// Export claims the channel from the kernel (writing to the chip's "export"
// file, tolerating EBUSY if some earlier process already exported it) and
// waits for its files to become writable.
func (c *sysfsPWMChannel) Export(ctx context.Context) error {
	if err := c.ch.Export(ctx); err != nil {
		return fmt.Errorf("motor: %w", err)
	}
	return nil
}

// Init zeroes the channel's duty, sets its period, and enables the PWM
// output -- in that order (see sysfspwm.Channel.Init for why the order is
// load-bearing).
func (c *sysfsPWMChannel) Init(_ context.Context) error {
	if err := c.ch.Init(); err != nil {
		return fmt.Errorf("motor: %w", err)
	}
	return nil
}

// SetDuty writes a duty fraction in [0, 1], converted to nanoseconds
// against the channel's configured period.
func (c *sysfsPWMChannel) SetDuty(_ context.Context, fraction float64) error {
	dutyNS := int64(fraction * float64(c.ch.PeriodNS()))
	if err := c.ch.WriteDutyNS(dutyNS); err != nil {
		return fmt.Errorf("motor: %w", err)
	}
	return nil
}

// Disable turns off the PWM channel's output. It does not unexport the
// channel -- matching the Python driver, which leaves the channel exported
// across reconnects.
func (c *sysfsPWMChannel) Disable() error {
	if err := c.ch.Disable(); err != nil {
		return fmt.Errorf("motor: %w", err)
	}
	return nil
}

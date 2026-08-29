//go:build linux

// This file, and the rest of the hardware adapter layer (gpio_enable.go,
// soft_pwm.go, driver.go), only builds on linux -- /sys/class/pwm is a
// Linux-kernel-specific interface, and github.com/warthog618/go-gpiocdev
// (imported by the other three) wraps the Linux GPIO character-device ABI,
// which has no Windows equivalent. This is a real platform boundary, not an
// oversight: the pure logic layer (duty.go, controller.go) has no such
// constraint and builds/tests on every platform, including the Windows dev
// machine this was written on -- see doc.go. Build/vet/test/lint this
// package's hardware layer with GOOS=linux GOARCH=arm64 (the actual
// deployment target).
package motor

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"time"
)

const (
	sysfsPWMRoot = "/sys/class/pwm"

	// pwmExportTimeout is how long to wait for the kernel + udev to create
	// and chgrp the channel directory after export -- exporting a channel
	// is asynchronous, matching pwm_sysfs.py's EXPORT_TIMEOUT_S.
	pwmExportTimeout = 2 * time.Second
	pwmPollInterval  = 50 * time.Millisecond

	pwmFilePerm = 0o200 // write-only: matches the sysfs files' own permissions
)

// sysfsPWMChannel is the real dutyWriter (controller.go) for RPWM
// (forward): the Pi's one free hardware PWM engine, driven directly through
// the kernel's /sys/class/pwm sysfs interface (export/period/duty_cycle/
// enable files) rather than a library, per go-migration-plan.md's stated
// preference for this pin.
type sysfsPWMChannel struct {
	chipDir    string
	channelDir string
	periodNS   int64
	channel    int
}

var _ dutyWriter = (*sysfsPWMChannel)(nil)

// newSysfsPWMChannel describes (without touching the filesystem) the PWM
// channel at root/pwmchip<chip>/pwm<channel>, running at frequencyHz.
func newSysfsPWMChannel(root string, chip, channel, frequencyHz int) *sysfsPWMChannel {
	chipDir := filepath.Join(root, fmt.Sprintf("pwmchip%d", chip))
	return &sysfsPWMChannel{
		chipDir:    chipDir,
		channelDir: filepath.Join(chipDir, fmt.Sprintf("pwm%d", channel)),
		periodNS:   time.Second.Nanoseconds() / int64(frequencyHz),
		channel:    channel,
	}
}

// Export claims the channel from the kernel (writing to the chip's "export"
// file, tolerating EBUSY if some earlier process already exported it) and
// waits for its files to become writable.
func (c *sysfsPWMChannel) Export(ctx context.Context) error {
	if _, err := os.Stat(c.chipDir); err != nil {
		return fmt.Errorf(
			"motor: %s not present -- add 'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' "+
				"to /boot/firmware/config.txt and reboot: %w: %w",
			c.chipDir, errPWMOverlayMissing, err,
		)
	}

	if _, err := os.Stat(c.channelDir); err != nil {
		exportPath := filepath.Join(c.chipDir, "export")
		if writeErr := os.WriteFile(exportPath, []byte(strconv.Itoa(c.channel)), pwmFilePerm); writeErr != nil {
			if _, statErr := os.Stat(c.channelDir); statErr != nil {
				return fmt.Errorf("motor: exporting PWM channel via %s: %w", exportPath, writeErr)
			}
		}
	}

	return c.waitWritable(ctx)
}

func (c *sysfsPWMChannel) waitWritable(ctx context.Context) error {
	dutyPath := filepath.Join(c.channelDir, "duty_cycle")
	deadline := time.Now().Add(pwmExportTimeout)
	for time.Now().Before(deadline) {
		if writable(dutyPath) {
			return nil
		}
		select {
		case <-ctx.Done():
			return fmt.Errorf("motor: waiting for %s to become writable: %w", dutyPath, ctx.Err())
		case <-time.After(pwmPollInterval):
		}
	}
	return fmt.Errorf(
		"motor: %s did not become writable within %s (is the service user in the 'gpio' group?)",
		dutyPath, pwmExportTimeout,
	)
}

// writable reports whether path can be opened for writing, mirroring
// pwm_sysfs.py's os.access(path, os.W_OK) check.
func writable(path string) bool {
	f, err := os.OpenFile(path, os.O_WRONLY, 0)
	if err != nil {
		return false
	}
	_ = f.Close()
	return true
}

// Init zeroes the channel's duty, sets its period, and enables the PWM
// output -- in that order. Order matters: duty_cycle may never exceed
// period, so a stale larger duty left over from a previous run would make
// the period write fail if it happened first. This mirrors the Python
// driver's connect() comment on the same three writes.
func (c *sysfsPWMChannel) Init(_ context.Context) error {
	if err := c.writeFile("duty_cycle", "0"); err != nil {
		return fmt.Errorf("motor: zeroing PWM duty on init: %w", err)
	}
	if err := c.writeFile("period", strconv.FormatInt(c.periodNS, 10)); err != nil {
		return fmt.Errorf("motor: setting PWM period on init: %w", err)
	}
	if err := c.writeFile("enable", "1"); err != nil {
		return fmt.Errorf("motor: enabling PWM channel on init: %w", err)
	}
	return nil
}

// SetDuty writes a duty fraction in [0, 1], converted to nanoseconds
// against the channel's configured period.
func (c *sysfsPWMChannel) SetDuty(_ context.Context, fraction float64) error {
	dutyNS := int64(fraction * float64(c.periodNS))
	if err := c.writeFile("duty_cycle", strconv.FormatInt(dutyNS, 10)); err != nil {
		return fmt.Errorf("motor: writing PWM duty_cycle: %w", err)
	}
	return nil
}

// Disable turns off the PWM channel's output. It does not unexport the
// channel -- matching the Python driver, which leaves the channel exported
// across reconnects.
func (c *sysfsPWMChannel) Disable() error {
	if err := c.writeFile("enable", "0"); err != nil {
		return fmt.Errorf("motor: disabling PWM channel: %w", err)
	}
	return nil
}

func (c *sysfsPWMChannel) writeFile(name, value string) error {
	if err := os.WriteFile(filepath.Join(c.channelDir, name), []byte(value), pwmFilePerm); err != nil {
		return fmt.Errorf("motor: writing %s/%s: %w", c.channelDir, name, err)
	}
	return nil
}

// errPWMOverlayMissing documents the config.txt fix for the most common
// Export failure -- kept as a sentinel so callers/tests can errors.Is
// against "overlay missing" specifically if that ever becomes necessary.
var errPWMOverlayMissing = errors.New("motor: hardware PWM overlay missing")

// Package sysfspwm drives one channel of the kernel's hardware PWM
// peripheral through /sys/class/pwm. It is the Go form of
// src/python/src/hardware/motors/pwm_sysfs.py, shared by pkg/driver/motor
// (BTS7960 RPWM) and pkg/driver/servo (steering) so the two cannot drift
// apart on the export-wait loop or the init write order - the same reason
// the Python helper was extracted.
//
// It lives under pkg/driver/internal because it is an implementation detail
// of those drivers, not public surface: consumers configure a driver, not a
// PWM channel.
package sysfspwm

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"time"
)

// Channel is one PWM channel at <root>/pwmchip<chip>/pwm<channel>, running
// at a fixed carrier period. Build it with New; nothing touches the
// filesystem until Export.
type Channel struct {
	chipDir     string
	channelDir  string
	overlayHint string
	periodNS    int64
	channel     int

	// write is os-backed in production; the in-package tests swap it to
	// record the order of writes, which the filesystem alone cannot show.
	write func(path, value string) error
}

const (
	// DefaultRoot is the kernel's sysfs PWM class directory
	// (pwm_sysfs.py:22 SYSFS_PWM_ROOT).
	DefaultRoot = "/sys/class/pwm"

	// ExportTimeout is how long Export waits for the kernel + udev to
	// create and chgrp the channel directory: exporting is asynchronous
	// (pwm_sysfs.py:24 EXPORT_TIMEOUT_S).
	ExportTimeout = 2 * time.Second

	// pollInterval matches pwm_sysfs.py:40's time.sleep(0.05).
	pollInterval = 50 * time.Millisecond

	// filePerm is only used if a write creates the file, which sysfs never
	// does; it matches the sysfs attribute files' own write-only mode.
	filePerm = 0o200
)

// ErrOverlayMissing is returned (wrapped) by Export when the pwmchip
// directory does not exist: the dtoverlay that maps the PWM engine onto the
// pin is not loaded. On a dev machine with no /sys/class/pwm this is the
// error every Export returns.
var ErrOverlayMissing = errors.New("sysfspwm: hardware PWM overlay missing")

// New describes the channel at root/pwmchip<chip>/pwm<channel> running at
// frequencyHz. overlayHint is the full config.txt line (quotes included)
// Export names when the chip is missing, e.g.
// "'dtoverlay=pwm,pin=12,func=4'" - pwm_sysfs.py:47's overlay_hint.
// frequencyHz must be positive; callers validate their config first.
func New(root string, chip, channel, frequencyHz int, overlayHint string) *Channel {
	chipDir := filepath.Join(root, fmt.Sprintf("pwmchip%d", chip))
	return &Channel{
		chipDir:     chipDir,
		channelDir:  filepath.Join(chipDir, fmt.Sprintf("pwm%d", channel)),
		overlayHint: overlayHint,
		periodNS:    time.Second.Nanoseconds() / int64(frequencyHz),
		channel:     channel,
		write:       writeSysfs,
	}
}

// PeriodNS is one carrier frame in nanoseconds (20,000,000 at 50 Hz).
func (c *Channel) PeriodNS() int64 { return c.periodNS }

// Dir is the channel's sysfs directory, for log and error messages.
func (c *Channel) Dir() string { return c.channelDir }

// Export claims the channel from the kernel and waits for its files to
// become writable (pwm_sysfs.py:44-82 export_pwm_channel). An export write
// that fails because the channel already exists (EBUSY) is tolerated.
func (c *Channel) Export(ctx context.Context) error {
	if _, err := os.Stat(c.chipDir); err != nil {
		return fmt.Errorf(
			"sysfspwm: %s not present - add %s to /boot/firmware/config.txt and reboot: %w: %w",
			c.chipDir, c.overlayHint, ErrOverlayMissing, err,
		)
	}

	if _, err := os.Stat(c.channelDir); err != nil {
		exportPath := filepath.Join(c.chipDir, "export")
		if writeErr := c.write(exportPath, strconv.Itoa(c.channel)); writeErr != nil {
			if _, statErr := os.Stat(c.channelDir); statErr != nil {
				return fmt.Errorf("sysfspwm: exporting PWM channel via %s: %w", exportPath, writeErr)
			}
		}
	}

	return c.waitWritable(ctx)
}

// Init zeroes the duty, sets the period, and enables the output - in that
// order. duty_cycle may never exceed period, so a stale larger duty left by
// a previous run would make the period write fail if it came first
// (servo/driver.py:118-123).
func (c *Channel) Init() error {
	if err := c.writeAttr("duty_cycle", "0"); err != nil {
		return fmt.Errorf("sysfspwm: zeroing duty on init: %w", err)
	}
	if err := c.writeAttr("period", strconv.FormatInt(c.periodNS, 10)); err != nil {
		return fmt.Errorf("sysfspwm: setting period on init: %w", err)
	}
	if err := c.writeAttr("enable", "1"); err != nil {
		return fmt.Errorf("sysfspwm: enabling channel on init: %w", err)
	}
	return nil
}

// WriteDutyNS writes a high time in nanoseconds. The caller keeps it within
// [0, PeriodNS]; the kernel rejects anything larger.
func (c *Channel) WriteDutyNS(dutyNS int64) error {
	if err := c.writeAttr("duty_cycle", strconv.FormatInt(dutyNS, 10)); err != nil {
		return fmt.Errorf("sysfspwm: writing duty_cycle: %w", err)
	}
	return nil
}

// Disable turns the output off. It does not unexport the channel, matching
// the Python drivers, which leave it exported across reconnects.
func (c *Channel) Disable() error {
	if err := c.writeAttr("enable", "0"); err != nil {
		return fmt.Errorf("sysfspwm: disabling channel: %w", err)
	}
	return nil
}

func (c *Channel) waitWritable(ctx context.Context) error {
	dutyPath := filepath.Join(c.channelDir, "duty_cycle")
	deadline := time.Now().Add(ExportTimeout)
	for time.Now().Before(deadline) {
		if writable(dutyPath) {
			return nil
		}
		select {
		case <-ctx.Done():
			return fmt.Errorf("sysfspwm: waiting for %s to become writable: %w", dutyPath, ctx.Err())
		case <-time.After(pollInterval):
		}
	}
	return fmt.Errorf(
		"sysfspwm: %s did not become writable within %s (is the service user in the 'gpio' group?)",
		dutyPath, ExportTimeout,
	)
}

func (c *Channel) writeAttr(name, value string) error {
	return c.write(filepath.Join(c.channelDir, name), value)
}

// writable reports whether path can be opened for writing, mirroring
// pwm_sysfs.py:38's os.access(path, os.W_OK).
func writable(path string) bool {
	f, err := os.OpenFile(path, os.O_WRONLY, 0)
	if err != nil {
		return false
	}
	_ = f.Close()
	return true
}

func writeSysfs(path, value string) error {
	if err := os.WriteFile(path, []byte(value), filePerm); err != nil {
		return fmt.Errorf("writing %s: %w", path, err)
	}
	return nil
}

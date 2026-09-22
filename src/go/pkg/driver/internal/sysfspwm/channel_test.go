package sysfspwm

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"slices"
	"testing"
)

// recordWrites swaps c's writer for one that logs "<attr>=<value>" in order.
func recordWrites(c *Channel) *[]string {
	var got []string
	c.write = func(path, value string) error {
		got = append(got, filepath.Base(path)+"="+value)
		return nil
	}
	return &got
}

// Init's order is the load-bearing part: duty 0 before period, period
// before enable.
func TestInit_WritesDutyThenPeriodThenEnable(t *testing.T) {
	t.Parallel()

	c := New(t.TempDir(), 0, 0, 50, "hint")
	got := recordWrites(c)
	if err := c.Init(); err != nil {
		t.Fatalf("Init: %v", err)
	}
	want := []string{"duty_cycle=0", "period=20000000", "enable=1"}
	if !slices.Equal(*got, want) {
		t.Fatalf("writes %v, want %v", *got, want)
	}
}

func TestInit_StopsAtTheFirstFailedWrite(t *testing.T) {
	t.Parallel()

	c := New(t.TempDir(), 0, 0, 50, "hint")
	var got []string
	c.write = func(path, _ string) error {
		got = append(got, filepath.Base(path))
		return errors.New("EINVAL")
	}
	if err := c.Init(); err == nil {
		t.Fatal("Init succeeded with every write failing")
	}
	if !slices.Equal(got, []string{"duty_cycle"}) {
		t.Fatalf("writes %v: Init went on after the duty write failed", got)
	}
}

func TestExport_MissingChipIsOverlayMissing(t *testing.T) {
	t.Parallel()

	c := New(t.TempDir(), 3, 0, 50, "'dtoverlay=test'")
	if err := c.Export(t.Context()); !errors.Is(err, ErrOverlayMissing) {
		t.Fatalf("Export = %v, want ErrOverlayMissing", err)
	}
}

// An already-exported channel is not re-exported.
func TestExport_AlreadyExportedSkipsTheExportWrite(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	dir := filepath.Join(root, "pwmchip0", "pwm1")
	if err := os.MkdirAll(dir, 0o750); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "duty_cycle"), nil, 0o600); err != nil {
		t.Fatal(err)
	}

	c := New(root, 0, 1, 1000, "hint")
	got := recordWrites(c)
	if err := c.Export(t.Context()); err != nil {
		t.Fatalf("Export: %v", err)
	}
	if len(*got) != 0 {
		t.Fatalf("Export wrote %v to an exported channel", *got)
	}
	if c.PeriodNS() != 1_000_000 {
		t.Errorf("PeriodNS() = %d at 1 kHz, want 1000000", c.PeriodNS())
	}
}

// A fresh channel is exported by writing its number to the chip's export
// file; the kernel then creates the directory (simulated here).
func TestExport_WritesTheChannelNumber(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	chip := filepath.Join(root, "pwmchip0")
	if err := os.MkdirAll(chip, 0o750); err != nil {
		t.Fatal(err)
	}
	c := New(root, 0, 1, 50, "hint")
	var exported string
	c.write = func(path, value string) error {
		exported = filepath.Base(path) + "=" + value
		dir := filepath.Join(chip, "pwm1")
		if err := os.MkdirAll(dir, 0o750); err != nil {
			return fmt.Errorf("simulating the kernel: %w", err)
		}
		if err := os.WriteFile(filepath.Join(dir, "duty_cycle"), nil, 0o600); err != nil {
			return fmt.Errorf("simulating the kernel: %w", err)
		}
		return nil
	}
	if err := c.Export(t.Context()); err != nil {
		t.Fatalf("Export: %v", err)
	}
	if exported != "export=1" {
		t.Fatalf("export write %q, want export=1", exported)
	}
}

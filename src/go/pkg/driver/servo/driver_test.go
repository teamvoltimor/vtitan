package servo

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"slices"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/internal/sysfspwm"
)

// recordingChannel records every call the driver makes, in order.
type recordingChannel struct {
	calls []string
	duty  []int64
}

const pulseTolerance = 1e-9

// servoTOML mirrors src/config/hardware/motors/servo.toml's base values.
func servoTOML() Config {
	return Config{
		GPIOPin:       12,
		PWMChip:       0,
		PWMChannel:    0,
		FrequencyHz:   50,
		MinPulseUS:    500,
		MaxPulseUS:    2500,
		CenterPulseUS: 1500,
		RangeDeg:      180,
	}
}

func (r *recordingChannel) Init() error {
	r.calls = append(r.calls, "init")
	return nil
}

func (r *recordingChannel) WriteDutyNS(dutyNS int64) error {
	r.calls = append(r.calls, "duty")
	r.duty = append(r.duty, dutyNS)
	return nil
}

func (r *recordingChannel) Disable() error {
	r.calls = append(r.calls, "disable")
	return nil
}

func connected(t *testing.T, cfg Config) (*Driver, *recordingChannel) {
	t.Helper()
	d, err := New(cfg)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	rec := &recordingChannel{}
	if err = d.connectChannel(rec); err != nil {
		t.Fatalf("connectChannel: %v", err)
	}
	return d, rec
}

func TestPulseUS(t *testing.T) {
	t.Parallel()

	reversed := servoTOML()
	reversed.Reversed = true
	wide := servoTOML()
	wide.RangeDeg = 270 // the 270deg-hiwonder-35kg profile overlay

	tests := []struct {
		name  string
		cfg   Config
		angle float64
		want  float64
	}{
		{name: "center", cfg: servoTOML(), angle: 0, want: 1500},
		{name: "positive", cfg: servoTOML(), angle: 45, want: 2000},
		{name: "negative", cfg: servoTOML(), angle: -45, want: 1000},
		{name: "reversed positive", cfg: reversed, angle: 45, want: 1000},
		{name: "reversed negative", cfg: reversed, angle: -45, want: 2000},
		{name: "clamped high", cfg: servoTOML(), angle: 200, want: 2500},
		{name: "clamped low", cfg: servoTOML(), angle: -200, want: 500},
		{name: "reversed clamps the flipped sign", cfg: reversed, angle: 200, want: 500},
		{name: "270 range full lock", cfg: wide, angle: 135, want: 2500},
		{name: "270 range", cfg: wide, angle: 27, want: 1700},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()
			if got := PulseUS(tt.cfg, tt.angle); math.Abs(got-tt.want) > pulseTolerance {
				t.Errorf("PulseUS(%v) = %v, want %v", tt.angle, got, tt.want)
			}
		})
	}
}

func TestNew_RejectsInvalidConfig(t *testing.T) {
	t.Parallel()

	for name, mutate := range map[string]func(*Config){
		"zero range":          func(c *Config) { c.RangeDeg = 0 },
		"zero frequency":      func(c *Config) { c.FrequencyHz = 0 },
		"max below min":       func(c *Config) { c.MaxPulseUS = 400 },
		"center outside span": func(c *Config) { c.CenterPulseUS = 3000 },
	} {
		cfg := servoTOML()
		mutate(&cfg)
		if _, err := New(cfg); err == nil {
			t.Errorf("%s: New accepted %+v", name, cfg)
		}
	}
}

// Connect must start the carrier before it writes a pulse, and its first
// pulse is the center.
func TestConnect_InitsThenCenters(t *testing.T) {
	t.Parallel()

	d, rec := connected(t, servoTOML())
	if want := []string{"init", "duty"}; !slices.Equal(rec.calls, want) {
		t.Fatalf("calls %v, want %v", rec.calls, want)
	}
	if rec.duty[0] != 1_500_000 {
		t.Errorf("first duty %d ns, want 1500000 (center)", rec.duty[0])
	}
	if d.Angle() != CenterDeg {
		t.Errorf("Angle() = %v, want center", d.Angle())
	}
}

func TestSetAngle_SkipsChangesUnderOneMicrosecond(t *testing.T) {
	t.Parallel()

	d, rec := connected(t, servoTOML())
	// 180 deg over 2000 us: 1 deg = 11.1 us, so 0.05 deg = 0.56 us.
	steps := []struct {
		angle     float64
		wantWrite bool
	}{
		{angle: 10, wantWrite: true},
		{angle: 10, wantWrite: false},
		{angle: 10.05, wantWrite: false},
		{angle: 10.2, wantWrite: true},
		{angle: 10.2 - 0.089, wantWrite: false}, // 0.989 us back
		{angle: 10.2 - 0.091, wantWrite: true},  // 1.011 us back: over the epsilon, written
	}
	for _, s := range steps {
		before := len(rec.duty)
		if err := d.SetAngle(s.angle); err != nil {
			t.Fatalf("SetAngle(%v): %v", s.angle, err)
		}
		if wrote := len(rec.duty) > before; wrote != s.wantWrite {
			t.Errorf("SetAngle(%v): wrote=%v, want %v", s.angle, wrote, s.wantWrite)
		}
		if d.Angle() != s.angle {
			t.Errorf("Angle() = %v after SetAngle(%v); a skipped write still updates it", d.Angle(), s.angle)
		}
	}
}

func TestSetAngle_NonFiniteIsRejectedAndNotWritten(t *testing.T) {
	t.Parallel()

	for _, angle := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		d, rec := connected(t, servoTOML())
		if err := d.SetAngle(20); err != nil {
			t.Fatalf("SetAngle(20): %v", err)
		}
		writes := len(rec.duty)

		err := d.SetAngle(angle)
		if !errors.Is(err, ErrNonFiniteAngle) {
			t.Errorf("SetAngle(%v) = %v, want ErrNonFiniteAngle", angle, err)
		}
		if len(rec.duty) != writes {
			t.Errorf("SetAngle(%v) wrote duty %d", angle, rec.duty[len(rec.duty)-1])
		}
		if d.Angle() != 20 {
			t.Errorf("SetAngle(%v) moved Angle() to %v", angle, d.Angle())
		}
	}
}

func TestSetAngle_BeforeConnectIsAnError(t *testing.T) {
	t.Parallel()

	d, err := New(servoTOML())
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	if err = d.SetAngle(0); !errors.Is(err, ErrNotConnected) {
		t.Fatalf("SetAngle before Connect = %v, want ErrNotConnected", err)
	}
}

// Close disables the carrier, and a reconnect writes its center even though
// it equals the last pulse written before the close.
func TestClose_DisablesAndForgetsThePulse(t *testing.T) {
	t.Parallel()

	d, rec := connected(t, servoTOML())
	if err := d.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	if last := rec.calls[len(rec.calls)-1]; last != "disable" {
		t.Fatalf("last call %q, want disable", last)
	}
	if err := d.SetAngle(0); !errors.Is(err, ErrNotConnected) {
		t.Fatalf("SetAngle after Close = %v, want ErrNotConnected", err)
	}
	if err := d.Close(); err != nil {
		t.Fatalf("second Close: %v", err)
	}

	again := &recordingChannel{}
	if err := d.connectChannel(again); err != nil {
		t.Fatalf("reconnect: %v", err)
	}
	if want := []string{"init", "duty"}; !slices.Equal(again.calls, want) {
		t.Fatalf("reconnect calls %v, want %v: the center write was skipped", again.calls, want)
	}
}

// fakeSysfs builds root/pwmchip0/pwm0 with writable attribute files, as the
// kernel leaves them after an export.
func fakeSysfs(t *testing.T) (root, channelDir string) {
	t.Helper()
	root = t.TempDir()
	channelDir = filepath.Join(root, "pwmchip0", "pwm0")
	if err := os.MkdirAll(channelDir, 0o750); err != nil {
		t.Fatal(err)
	}
	for _, name := range []string{"duty_cycle", "period", "enable"} {
		if err := os.WriteFile(filepath.Join(channelDir, name), []byte("stale"), 0o600); err != nil {
			t.Fatal(err)
		}
	}
	return root, channelDir
}

func readAttr(t *testing.T, dir, name string) string {
	t.Helper()
	b, err := os.ReadFile(filepath.Join(dir, name))
	if err != nil {
		t.Fatal(err)
	}
	return strings.TrimSpace(string(b))
}

// End to end through the real sysfs channel, against a temp-dir root.
func TestConnect_AgainstFakeSysfs(t *testing.T) {
	t.Parallel()

	root, dir := fakeSysfs(t)
	cfg := servoTOML()
	cfg.Root = root
	d, err := New(cfg)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	if err = d.Connect(t.Context()); err != nil {
		t.Fatalf("Connect: %v", err)
	}
	for name, want := range map[string]string{
		"period": "20000000", "enable": "1", "duty_cycle": "1500000",
	} {
		if got := readAttr(t, dir, name); got != want {
			t.Errorf("%s = %q after Connect, want %q", name, got, want)
		}
	}

	if err = d.SetAngle(-45); err != nil {
		t.Fatalf("SetAngle: %v", err)
	}
	if got := readAttr(t, dir, "duty_cycle"); got != "1000000" {
		t.Errorf("duty_cycle = %q after SetAngle(-45), want 1000000", got)
	}

	if err = d.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	if got := readAttr(t, dir, "enable"); got != "0" {
		t.Errorf("enable = %q after Close, want 0", got)
	}
}

// A dev machine has no pwmchip: Connect fails, naming the overlay.
func TestConnect_WithoutOverlayFails(t *testing.T) {
	t.Parallel()

	cfg := servoTOML()
	cfg.Root = t.TempDir()
	d, err := New(cfg)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	err = d.Connect(t.Context())
	if !errors.Is(err, sysfspwm.ErrOverlayMissing) {
		t.Fatalf("Connect = %v, want ErrOverlayMissing", err)
	}
	if !strings.Contains(err.Error(), "dtoverlay=pwm,pin=12,func=4") {
		t.Errorf("error %q does not name the overlay", err)
	}
}

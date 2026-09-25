package harness

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
)

var driveForward = controllers.DriveCommand{SpeedMPS: 0.4, SteeringNorm: 0.2}

func TestTransportConfig_AnyAndValidate(t *testing.T) {
	t.Parallel()

	if (TransportConfig{}).Any() {
		t.Error("the zero TransportConfig reports emulation on")
	}
	for name, tc := range map[string]struct {
		cfg   TransportConfig
		valid bool
	}{
		"zero":             {TransportConfig{}, true},
		"typical":          {TransportConfig{CommandDelayS: 0.05, CommandDropRate: 0.1, CommandTimeoutS: 0.5, ScanDelayS: 0.1}, true},
		"negative delay":   {TransportConfig{CommandDelayS: -0.1}, false},
		"negative scan":    {TransportConfig{ScanDelayS: -0.1}, false},
		"drop rate of one": {TransportConfig{CommandDropRate: 1}, false},
	} {
		if err := tc.cfg.Validate(); (err == nil) != tc.valid {
			t.Errorf("%s: Validate = %v, want valid %v", name, err, tc.valid)
		}
	}
}

// A command reaches the body CommandDelayS after it is published, not
// before.
func TestTransport_CommandDelay(t *testing.T) {
	t.Parallel()

	tr := newTransport(TransportConfig{CommandDelayS: 0.2}, 1)
	tr.publish(driveForward, 0)
	if got := tr.commandAt(controllers.DriveCommand{}, 0.15); got != (controllers.DriveCommand{}) {
		t.Errorf("command at 0.15 s = %+v, want still none", got)
	}
	if got := tr.commandAt(controllers.DriveCommand{}, 0.2); got != driveForward {
		t.Errorf("command at 0.2 s = %+v, want %+v", got, driveForward)
	}
}

// Drops happen at the configured rate, deterministically for a seed.
func TestTransport_CommandDropRate(t *testing.T) {
	t.Parallel()

	const n, rate = 20_000, 0.25
	run := func() TransportStats {
		tr := newTransport(TransportConfig{CommandDropRate: rate}, 7)
		for i := range n {
			tr.publish(driveForward, float64(i)*0.05)
		}
		return tr.stats
	}
	a, b := run(), run()
	if a != b {
		t.Errorf("same seed, different stats: %+v vs %+v", a, b)
	}
	if got := float64(a.CommandsDropped) / n; math.Abs(got-rate) > 0.01 {
		t.Errorf("drop fraction = %.4f, want %.2f +- 0.01", got, rate)
	}
}

// The emulated board watchdog stops the body after CommandTimeoutS without
// an arrival, once, and the next arrival resumes driving.
func TestTransport_WatchdogStopsAndResumes(t *testing.T) {
	t.Parallel()

	tr := newTransport(TransportConfig{CommandTimeoutS: 0.5}, 1)
	tr.publish(driveForward, 0)
	cur := tr.commandAt(controllers.DriveCommand{}, 0)
	if cur != driveForward {
		t.Fatalf("command at 0 = %+v, want %+v", cur, driveForward)
	}
	if cur = tr.commandAt(cur, 0.4); cur != driveForward {
		t.Errorf("command at 0.4 s = %+v, want still driving", cur)
	}
	if cur = tr.commandAt(cur, 0.5); cur != (controllers.DriveCommand{}) {
		t.Errorf("command at 0.5 s = %+v, want a watchdog stop", cur)
	}
	cur = tr.commandAt(cur, 0.6)
	if tr.stats.WatchdogStops != 1 {
		t.Errorf("WatchdogStops = %d, want 1", tr.stats.WatchdogStops)
	}
	tr.publish(driveForward, 0.7)
	if cur = tr.commandAt(cur, 0.7); cur != driveForward {
		t.Errorf("command after a new arrival = %+v, want driving again", cur)
	}
}

// The navigator reads the newest sweep at least ScanDelayS old.
func TestTransport_ScanDelay(t *testing.T) {
	t.Parallel()

	tr := newTransport(TransportConfig{ScanDelayS: 0.2}, 1)
	scanAtTime := func(s float64) controllers.LidarScan {
		return controllers.LidarScan{RangesM: []float64{s}}
	}
	for i := range 10 {
		at := float64(i) * 0.1
		tr.recordScan(scanAtTime(at), at)
	}
	got, ok := tr.scanAt(0.95)
	if !ok || math.Abs(got.RangesM[0]-0.7) > 1e-9 {
		t.Errorf("scan at 0.95 s = %v (ok %v), want the one cast at 0.7 s", got.RangesM, ok)
	}

	fresh := newTransport(TransportConfig{ScanDelayS: 0.2}, 1)
	fresh.recordScan(scanAtTime(0), 0)
	if _, ok = fresh.scanAt(0.1); ok {
		t.Error("a scan younger than ScanDelayS was returned")
	}
}

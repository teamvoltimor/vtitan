package harness

import (
	"fmt"
	"math/rand/v2"
	"slices"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
)

// TransportConfig emulates the time the real robot loses between deciding
// and acting, and between sensing and deciding, which the in-process sim
// otherwise treats as zero: the navigator's command reaches the body the
// same tick, and the scan it reads is the one cast this tick.
//
// Everything runs on the simulation clock, so a run stays deterministic for
// a given seed. The zero value is the previous behaviour exactly, and every
// existing corpus number was measured on it.
//
// Resolution is one control tick (1/ControlHz, 50 ms at 20 Hz): a command
// takes effect at the first tick at or after its arrival, so any delay in
// (0, dt] is one tick late, and the zero value means a command acts during
// the very tick it was computed, which no real robot does. A scan delay is
// likewise resolved against the sweeps the sim has cast.
type TransportConfig struct {
	// CommandDelayS is how long a published command takes to reach the
	// body: NATS, the board link and the board's loop together.
	CommandDelayS float64
	// CommandDropRate is the probability a published command never
	// arrives; the previous one keeps applying, as on the real board.
	CommandDropRate float64
	// CommandTimeoutS emulates the board's command watchdog: with no command
	// arriving for this long, the body is commanded to stop and centre.
	// Zero disables it, which is the previous behaviour.
	CommandTimeoutS float64
	// ScanDelayS is the age of the LIDAR sweep the navigator reads: the
	// sensor, driver and transport latency together. The scan-matcher, when
	// Localize is set, still sees the fresh sweep; pose latency is not
	// modelled here.
	ScanDelayS float64
	// DetectionDelayS is the camera-to-detection latency: a sign detection
	// the navigator receives now was seen from where the robot was this
	// long ago, and is placed in the world through where it is now, as the
	// real pipeline does. Measured on hardware at about 0.85 s. Applied by
	// the scenario runner's emulated camera, not by the gateway.
	DetectionDelayS float64
	// DetectionDropRate is the probability that a camera frame yields no
	// detections at all. The emulator reports a sign on 54.5% of ticks
	// against 11.6% on hardware, a drop of about 0.79 when a sign is in
	// view.
	DetectionDropRate float64
}

// TransportStats counts what the transport emulation did during a run.
type TransportStats struct {
	CommandsPublished uint64
	CommandsDropped   uint64
	WatchdogStops     uint64
}

// transport is the gateway's emulation state.
type transport struct {
	cfg  TransportConfig
	rand *rand.Rand

	inFlight []timedCommand
	// lastArrivalS is when the body last received a command, for the
	// emulated watchdog; stopped marks a watchdog stop already applied.
	lastArrivalS float64
	stopped      bool

	scans []timedScan

	stats TransportStats
}

// timedCommand is a command and when it reaches the body.
type timedCommand struct {
	command controllers.DriveCommand
	dueS    float64
}

// timedScan is a sweep and when it was cast.
type timedScan struct {
	scan controllers.LidarScan
	atS  float64
}

// TimeEpsilonS absorbs the rounding in sim time, which accumulates dt by
// addition: after eight 0.05 s ticks the clock reads 0.39999999999999997,
// so an exact "at least 0.3 s old" test would miss the sweep cast at 0.1 s
// and deliver everything one tick late at exact multiples of dt.
const TimeEpsilonS = 1e-9

// transportStreamSalt keeps the drop draws off the LIDAR and sensor-error
// streams, so switching transport emulation on cannot perturb the noise an
// unperturbed control run was measured on.
const transportStreamSalt = 0x5851_f42d_4c95_7f2d

// Any reports whether any emulation is switched on.
func (c TransportConfig) Any() bool {
	return c.CommandDelayS != 0 || c.CommandDropRate != 0 || c.CommandTimeoutS != 0 || c.ScanDelayS != 0 ||
		c.DetectionDelayS != 0 || c.DetectionDropRate != 0
}

// Validate reports a negative time or a drop rate outside [0, 1).
func (c TransportConfig) Validate() error {
	switch {
	case c.CommandDelayS < 0 || c.CommandTimeoutS < 0 || c.ScanDelayS < 0 || c.DetectionDelayS < 0:
		return fmt.Errorf("harness: transport delays and timeout must not be negative: %+v", c)
	case c.CommandDropRate < 0 || c.CommandDropRate >= 1:
		return fmt.Errorf("harness: command drop rate %v is outside [0, 1)", c.CommandDropRate)
	case c.DetectionDropRate < 0 || c.DetectionDropRate >= 1:
		return fmt.Errorf("harness: detection drop rate %v is outside [0, 1)", c.DetectionDropRate)
	}
	return nil
}

func newTransport(cfg TransportConfig, seed uint64) *transport {
	return &transport{cfg: cfg, rand: rand.New(rand.NewPCG(seed^transportStreamSalt, seed))}
}

// publish queues command, sent at nowS, unless it is dropped.
func (t *transport) publish(command controllers.DriveCommand, nowS float64) {
	t.stats.CommandsPublished++
	if t.cfg.CommandDropRate > 0 && t.rand.Float64() < t.cfg.CommandDropRate {
		t.stats.CommandsDropped++
		return
	}
	t.inFlight = append(t.inFlight, timedCommand{command: command, dueS: nowS + t.cfg.CommandDelayS})
}

// commandAt returns the command the body carries at nowS, given the one it
// carried before: the newest arrival, or a watchdog stop.
func (t *transport) commandAt(current controllers.DriveCommand, nowS float64) controllers.DriveCommand {
	arrived := 0
	for arrived < len(t.inFlight) && t.inFlight[arrived].dueS <= nowS+TimeEpsilonS {
		current = t.inFlight[arrived].command
		t.lastArrivalS = nowS
		t.stopped = false
		arrived++
	}
	t.inFlight = t.inFlight[arrived:]

	if t.cfg.CommandTimeoutS > 0 && !t.stopped && nowS-t.lastArrivalS >= t.cfg.CommandTimeoutS-TimeEpsilonS {
		t.stopped = true
		t.stats.WatchdogStops++
		return controllers.DriveCommand{}
	}
	if t.stopped {
		return controllers.DriveCommand{}
	}
	return current
}

// recordScan keeps scan, cast at nowS, for delayed reads.
func (t *transport) recordScan(scan controllers.LidarScan, nowS float64) {
	if t.cfg.ScanDelayS <= 0 {
		return
	}
	t.scans = append(t.scans, timedScan{scan: scan, atS: nowS})
	// Keep the newest sweep at least ScanDelayS old, and everything after.
	keep := 0
	for i, sc := range slices.Backward(t.scans) {
		if nowS-sc.atS >= t.cfg.ScanDelayS-TimeEpsilonS {
			keep = i
			break
		}
	}
	t.scans = t.scans[keep:]
}

// scanAt returns the newest sweep at least ScanDelayS old at nowS, and
// false while none is that old yet.
func (t *transport) scanAt(nowS float64) (controllers.LidarScan, bool) {
	for _, sc := range slices.Backward(t.scans) {
		if nowS-sc.atS >= t.cfg.ScanDelayS-TimeEpsilonS {
			return sc.scan, true
		}
	}
	return controllers.LidarScan{}, false
}

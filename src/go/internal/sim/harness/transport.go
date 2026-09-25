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

// transportStreamSalt keeps the drop draws off the LIDAR and sensor-error
// streams, so switching transport emulation on cannot perturb the noise an
// unperturbed control run was measured on.
const transportStreamSalt = 0x5851_f42d_4c95_7f2d

// Any reports whether any emulation is switched on.
func (c TransportConfig) Any() bool {
	return c.CommandDelayS != 0 || c.CommandDropRate != 0 || c.CommandTimeoutS != 0 || c.ScanDelayS != 0
}

// Validate reports a negative time or a drop rate outside [0, 1).
func (c TransportConfig) Validate() error {
	switch {
	case c.CommandDelayS < 0 || c.CommandTimeoutS < 0 || c.ScanDelayS < 0:
		return fmt.Errorf("harness: transport delays and timeout must not be negative: %+v", c)
	case c.CommandDropRate < 0 || c.CommandDropRate >= 1:
		return fmt.Errorf("harness: command drop rate %v is outside [0, 1)", c.CommandDropRate)
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
	for arrived < len(t.inFlight) && t.inFlight[arrived].dueS <= nowS {
		current = t.inFlight[arrived].command
		t.lastArrivalS = nowS
		t.stopped = false
		arrived++
	}
	t.inFlight = t.inFlight[arrived:]

	if t.cfg.CommandTimeoutS > 0 && !t.stopped && nowS-t.lastArrivalS >= t.cfg.CommandTimeoutS {
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
		if nowS-sc.atS >= t.cfg.ScanDelayS {
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
		if nowS-sc.atS >= t.cfg.ScanDelayS {
			return sc.scan, true
		}
	}
	return controllers.LidarScan{}, false
}
